use crate::bus::{BusFrame, CanBus, CanFrame};
use crate::device::MotorDevice;
use crate::error::{MotorError, Result};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PollingMode {
    Background,
    Manual,
}

type DeviceMap<T> = HashMap<u16, Arc<dyn MotorDevice<T>>>;

pub struct CoreController<T: BusFrame = CanFrame> {
    bus: Arc<dyn CanBus<T>>,
    devices: Arc<Mutex<DeviceMap<T>>>,
    polling_mode: PollingMode,
    recv_lock: Arc<Mutex<()>>,
    tx_gap_ns: Arc<AtomicU64>,
    tx_gap_env_override: bool,
    polling_active: Arc<AtomicBool>,
    polling_thread: Mutex<Option<JoinHandle<()>>>,
    polling_start_lock: Mutex<()>,
}

impl<T: BusFrame> CoreController<T> {
    // Pace broadcast-like per-motor operations (enable/disable) to avoid burst writes on a busy bus.
    const DEFAULT_BULK_OP_GAP_MS: u64 = 2;
    // Auto pacing for multi-motor continuous TX (small enough to avoid obvious loop-frequency impact).
    const AUTO_MULTI_MOTOR_TX_GAP_US: u64 = 120;

    pub fn new(bus: Arc<dyn CanBus<T>>) -> Self {
        Self::new_internal(bus, PollingMode::Background)
    }

    fn new_internal(bus: Arc<dyn CanBus<T>>, polling_mode: PollingMode) -> Self {
        let (initial_tx_gap_ns, tx_gap_env_override) = Self::resolve_tx_gap_from_env();
        let tx_gap_ns = Arc::new(AtomicU64::new(initial_tx_gap_ns));
        let bus: Arc<dyn CanBus<T>> = Arc::new(TxPacedBus::new(bus, Arc::clone(&tx_gap_ns)));

        Self {
            bus,
            devices: Arc::new(Mutex::new(HashMap::new())),
            polling_mode,
            recv_lock: Arc::new(Mutex::new(())),
            tx_gap_ns,
            tx_gap_env_override,
            polling_active: Arc::new(AtomicBool::new(false)),
            polling_thread: Mutex::new(None),
            polling_start_lock: Mutex::new(()),
        }
    }

    pub fn bus(&self) -> Arc<dyn CanBus<T>> {
        Arc::clone(&self.bus)
    }

    pub fn add_device(&self, device: Arc<dyn MotorDevice<T>>) -> Result<()> {
        let motor_id = device.motor_id();
        let device_count = {
            let mut devices = self
                .devices
                .lock()
                .map_err(|_| MotorError::Io("devices lock poisoned".to_string()))?;
            if devices.contains_key(&motor_id) {
                return Err(MotorError::InvalidArgument(format!(
                    "device with motor_id {motor_id} already exists"
                )));
            }
            devices.insert(motor_id, Arc::clone(&device));
            devices.len()
        };

        // If user did not pin TX gap via env, enable tiny pacing once controller becomes multi-motor.
        if !self.tx_gap_env_override && device_count >= 2 {
            self.tx_gap_ns.store(
                Duration::from_micros(Self::AUTO_MULTI_MOTOR_TX_GAP_US).as_nanos() as u64,
                Ordering::Release,
            );
        }

        self.start_polling_if_needed()?;
        Ok(())
    }

    pub fn poll_feedback_once(&self) -> Result<()> {
        self.poll_feedback_once_internal()
    }

    fn poll_feedback_once_internal(&self) -> Result<()> {
        let _recv_guard = self
            .recv_lock
            .lock()
            .map_err(|_| MotorError::Io("recv lock poisoned".to_string()))?;
        while let Some(frame) = self.bus.recv(Duration::from_millis(0))? {
            if !frame.is_rx() {
                continue;
            }
            let devices = self
                .devices
                .lock()
                .map_err(|_| MotorError::Io("devices lock poisoned".to_string()))?
                .values()
                .cloned()
                .collect::<Vec<_>>();
            for device in devices {
                if !device.accepts_frame(&frame) {
                    continue;
                }
                device.process_feedback_frame(frame)?;
                break;
            }
        }
        Ok(())
    }

    pub fn enable_all(&self) -> Result<()> {
        self.apply_bulk_device_op(|d| d.enable())
    }

    pub fn disable_all(&self) -> Result<()> {
        self.apply_bulk_device_op(|d| d.disable())
    }

    fn sorted_devices(&self) -> Result<Vec<Arc<dyn MotorDevice<T>>>> {
        let mut devices = self
            .devices
            .lock()
            .map_err(|_| MotorError::Io("devices lock poisoned".to_string()))?
            .values()
            .cloned()
            .collect::<Vec<_>>();
        devices.sort_by_key(|d| d.motor_id());
        Ok(devices)
    }

    fn apply_bulk_device_op<F>(&self, mut op: F) -> Result<()>
    where
        F: FnMut(&Arc<dyn MotorDevice<T>>) -> Result<()>,
    {
        let devices = self.sorted_devices()?;
        let gap = Self::bulk_op_gap();
        for (idx, device) in devices.iter().enumerate() {
            op(device)?;
            // Best-effort drain during bulk ops to keep RX queue fresh when many motors respond.
            let _ = self.poll_feedback_once_internal();
            if idx + 1 < devices.len() && !gap.is_zero() {
                std::thread::sleep(gap);
            }
        }
        Ok(())
    }

    fn bulk_op_gap() -> Duration {
        static GAP: OnceLock<Duration> = OnceLock::new();
        *GAP.get_or_init(|| {
            std::env::var("MOTORBRIDGE_BULK_OP_GAP_MS")
                .ok()
                .and_then(|s| s.parse::<u64>().ok())
                .map(Duration::from_millis)
                .unwrap_or(Duration::from_millis(Self::DEFAULT_BULK_OP_GAP_MS))
        })
    }

    fn resolve_tx_gap_from_env() -> (u64, bool) {
        match std::env::var("MOTORBRIDGE_TX_GAP_US") {
            Ok(v) => match v.parse::<u64>() {
                Ok(us) => (Duration::from_micros(us).as_nanos() as u64, true),
                Err(_) => (0, false),
            },
            Err(_) => (0, false),
        }
    }

    fn start_polling_if_needed(&self) -> Result<()> {
        if self.polling_mode == PollingMode::Manual {
            return Ok(());
        }
        let _start_guard = self
            .polling_start_lock
            .lock()
            .map_err(|_| MotorError::Io("polling start lock poisoned".to_string()))?;
        if self.polling_active.load(Ordering::Acquire) {
            return Ok(());
        }
        self.polling_active.store(true, Ordering::Release);
        let active = Arc::clone(&self.polling_active);
        let bus = Arc::clone(&self.bus);
        let devices = Arc::clone(&self.devices);
        let recv_lock = Arc::clone(&self.recv_lock);

        let handle = thread::spawn(move || {
            let idle_sleep = Duration::from_micros(200);
            while active.load(Ordering::Acquire) {
                let recv_result = {
                    match recv_lock.lock() {
                        Ok(_guard) => bus.recv(Duration::from_millis(0)),
                        Err(_) => {
                            active.store(false, Ordering::Release);
                            return;
                        }
                    }
                };
                match recv_result {
                    Ok(Some(frame)) => {
                        if frame.is_rx() {
                            let snapshot = match devices.lock() {
                                Ok(m) => m.values().cloned().collect::<Vec<_>>(),
                                Err(_) => {
                                    eprintln!(
                                        "[motor_core] polling worker: devices lock poisoned, stopping"
                                    );
                                    active.store(false, Ordering::Release);
                                    return;
                                }
                            };
                            for device in snapshot {
                                if !device.accepts_frame(&frame) {
                                    continue;
                                }
                                if let Err(err) = device.process_feedback_frame(frame) {
                                    eprintln!(
                                        "[motor_core] polling worker: process_feedback_frame failed: {err}"
                                    );
                                }
                                break;
                            }
                        }
                        // Fast path: while frames are flowing, keep draining without sleeping.
                        continue;
                    }
                    Ok(None) => {
                        // Idle path: queue is empty, briefly yield CPU.
                        std::thread::sleep(idle_sleep);
                    }
                    Err(_) => {
                        // Keep worker alive on transient bus errors so upper layers remain stable.
                        std::thread::sleep(idle_sleep);
                    }
                }
            }
        });

        self.polling_thread
            .lock()
            .map_err(|_| MotorError::Io("polling thread lock poisoned".to_string()))?
            .replace(handle);

        Ok(())
    }

    pub fn shutdown(&self) -> Result<()> {
        self.close_inner(true)
    }

    pub fn close_bus(&self) -> Result<()> {
        self.close_inner(false)
    }

    fn stop_polling_thread(&self) -> Result<()> {
        self.polling_active.store(false, Ordering::Release);
        if let Some(handle) = self
            .polling_thread
            .lock()
            .map_err(|_| MotorError::Io("polling thread lock poisoned".to_string()))?
            .take()
        {
            let _ = handle.join();
        }
        Ok(())
    }

    fn close_inner(&self, disable_devices: bool) -> Result<()> {
        self.stop_polling_thread()?;
        if disable_devices {
            let _ = self.disable_all();
        }
        self.bus.shutdown()
    }
}

impl<T: BusFrame> Drop for CoreController<T> {
    fn drop(&mut self) {
        let _ = self.stop_polling_thread();
    }
}

struct TxPacedBus<T> {
    inner: Arc<dyn CanBus<T>>,
    min_gap_ns: Arc<AtomicU64>,
    last_send: Mutex<Option<Instant>>,
}

impl<T> TxPacedBus<T> {
    fn new(inner: Arc<dyn CanBus<T>>, min_gap_ns: Arc<AtomicU64>) -> Self {
        Self {
            inner,
            min_gap_ns,
            last_send: Mutex::new(None),
        }
    }
}

impl<T> CanBus<T> for TxPacedBus<T> {
    fn send(&self, frame: T) -> Result<()> {
        let gap_ns = self.min_gap_ns.load(Ordering::Acquire);
        if gap_ns > 0 {
            let min_gap = Duration::from_nanos(gap_ns);
            let mut guard = self
                .last_send
                .lock()
                .map_err(|_| MotorError::Io("tx pacing lock poisoned".to_string()))?;
            if let Some(last) = *guard {
                let elapsed = last.elapsed();
                if elapsed < min_gap {
                    std::thread::sleep(min_gap - elapsed);
                }
            }
            *guard = Some(Instant::now());
        }
        self.inner.send(frame)
    }

    fn recv(&self, timeout: Duration) -> Result<Option<T>> {
        self.inner.recv(timeout)
    }

    fn shutdown(&self) -> Result<()> {
        self.inner.shutdown()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::CanFrame;
    use crate::device::MotorDevice;
    use crate::test_support::MockBus;
    use crate::vendor_controller::VendorController;
    use std::collections::VecDeque;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::mpsc;

    struct FakeDevice {
        id: u16,
        accepts_id: u32,
        enable_count: AtomicUsize,
        disable_count: AtomicUsize,
        processed_count: AtomicUsize,
    }

    impl FakeDevice {
        fn new(id: u16, accepts_id: u32) -> Self {
            Self {
                id,
                accepts_id,
                enable_count: AtomicUsize::new(0),
                disable_count: AtomicUsize::new(0),
                processed_count: AtomicUsize::new(0),
            }
        }
    }

    impl MotorDevice for FakeDevice {
        fn vendor(&self) -> &'static str {
            "fake"
        }

        fn model(&self) -> &str {
            "fake-model"
        }

        fn motor_id(&self) -> u16 {
            self.id
        }

        fn feedback_id(&self) -> u16 {
            self.accepts_id as u16
        }

        fn enable(&self) -> Result<()> {
            self.enable_count.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }

        fn disable(&self) -> Result<()> {
            self.disable_count.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }

        fn accepts_frame(&self, frame: &CanFrame) -> bool {
            frame.arbitration_id == self.accepts_id
        }

        fn process_feedback_frame(&self, _frame: CanFrame) -> Result<()> {
            self.processed_count.fetch_add(1, Ordering::SeqCst);
            Ok(())
        }
    }

    #[derive(Debug, PartialEq, Eq)]
    struct TestFrame {
        device_id: u16,
        data: Vec<u8>,
        is_rx: bool,
    }

    impl BusFrame for TestFrame {
        fn is_rx(&self) -> bool {
            self.is_rx
        }
    }

    #[derive(Default)]
    struct MockFrameBus {
        rx: Mutex<VecDeque<TestFrame>>,
        sent: Mutex<Vec<TestFrame>>,
    }

    impl CanBus<TestFrame> for MockFrameBus {
        fn send(&self, frame: TestFrame) -> Result<()> {
            self.sent.lock().unwrap().push(frame);
            Ok(())
        }

        fn recv(&self, _timeout: Duration) -> Result<Option<TestFrame>> {
            Ok(self.rx.lock().unwrap().pop_front())
        }

        fn shutdown(&self) -> Result<()> {
            Ok(())
        }
    }

    struct FakeFrameDevice {
        id: u16,
        feedback: mpsc::Sender<(u16, TestFrame)>,
    }

    impl MotorDevice<TestFrame> for FakeFrameDevice {
        fn vendor(&self) -> &'static str {
            "fake"
        }

        fn model(&self) -> &str {
            "fake-frame-model"
        }

        fn motor_id(&self) -> u16 {
            self.id
        }

        fn feedback_id(&self) -> u16 {
            self.id
        }

        fn enable(&self) -> Result<()> {
            Ok(())
        }

        fn disable(&self) -> Result<()> {
            Ok(())
        }

        fn accepts_frame(&self, frame: &TestFrame) -> bool {
            frame.device_id == self.id
        }

        fn process_feedback_frame(&self, frame: TestFrame) -> Result<()> {
            self.feedback.send((self.id, frame)).unwrap();
            Ok(())
        }
    }

    fn test_frame(device_id: u16, len: usize, is_rx: bool) -> TestFrame {
        TestFrame {
            device_id,
            data: (0..len).map(|i| i as u8).collect(),
            is_rx,
        }
    }

    fn rx_frame(id: u32) -> CanFrame {
        CanFrame {
            arbitration_id: id,
            data: [0; 8],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        }
    }

    #[test]
    fn add_device_rejects_duplicate_motor_id() {
        let bus = Arc::new(MockBus::new());
        let ctrl = CoreController::new(bus);
        let d1: Arc<dyn MotorDevice> = Arc::new(FakeDevice::new(1, 0x11));
        let d2: Arc<dyn MotorDevice> = Arc::new(FakeDevice::new(1, 0x12));
        ctrl.add_device(d1).expect("first add");
        assert!(ctrl.add_device(d2).is_err());
        ctrl.close_bus().expect("close");
    }

    #[test]
    fn poll_feedback_routes_to_accepting_device() {
        let bus = Arc::new(MockBus::new());
        bus.push_rx(rx_frame(0x12));
        bus.push_rx(rx_frame(0x11));

        let ctrl = CoreController::new_internal(bus, PollingMode::Manual);
        let d1 = Arc::new(FakeDevice::new(1, 0x11));
        let d2 = Arc::new(FakeDevice::new(2, 0x12));
        ctrl.add_device(d1.clone()).expect("add d1");
        ctrl.add_device(d2.clone()).expect("add d2");

        ctrl.poll_feedback_once().expect("poll");

        assert_eq!(d1.processed_count.load(Ordering::SeqCst), 1);
        assert_eq!(d2.processed_count.load(Ordering::SeqCst), 1);
        ctrl.close_bus().expect("close");
    }

    #[test]
    fn enable_and_disable_all_touch_each_device_once() {
        let bus = Arc::new(MockBus::new());
        let ctrl = CoreController::new(bus);
        let d1 = Arc::new(FakeDevice::new(1, 0x11));
        let d2 = Arc::new(FakeDevice::new(2, 0x12));
        ctrl.add_device(d1.clone()).expect("add d1");
        ctrl.add_device(d2.clone()).expect("add d2");

        ctrl.enable_all().expect("enable all");
        ctrl.disable_all().expect("disable all");

        assert_eq!(d1.enable_count.load(Ordering::SeqCst), 1);
        assert_eq!(d2.enable_count.load(Ordering::SeqCst), 1);
        assert_eq!(d1.disable_count.load(Ordering::SeqCst), 1);
        assert_eq!(d2.disable_count.load(Ordering::SeqCst), 1);
        ctrl.close_bus().expect("close");
    }

    #[test]
    fn shutdown_disables_devices_and_closes_bus() {
        let bus = Arc::new(MockBus::new());
        let ctrl = CoreController::new(bus.clone());
        let d1 = Arc::new(FakeDevice::new(1, 0x11));
        ctrl.add_device(d1.clone()).expect("add d1");

        ctrl.shutdown().expect("shutdown");

        assert_eq!(d1.disable_count.load(Ordering::SeqCst), 1);
        assert_eq!(bus.shutdown_count.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn poll_feedback_once_returns_bus_recv_error() {
        let bus = Arc::new(MockBus::new());
        let ctrl = CoreController::new_internal(bus.clone(), PollingMode::Manual);
        let d1: Arc<dyn MotorDevice> = Arc::new(FakeDevice::new(1, 0x11));
        ctrl.add_device(d1).expect("add d1");

        bus.set_fail_recv(true);
        let err = ctrl.poll_feedback_once().expect_err("recv should fail");
        assert!(err.to_string().contains("injected recv error"));
        ctrl.close_bus().expect("close");
    }

    #[test]
    fn poll_feedback_once_works_in_background_mode_without_errors() {
        let bus = Arc::new(MockBus::new());
        bus.push_rx(rx_frame(0x11));

        let ctrl = CoreController::new(bus.clone());
        let d1 = Arc::new(FakeDevice::new(1, 0x11));
        ctrl.add_device(d1.clone()).expect("add d1");

        ctrl.poll_feedback_once()
            .expect("manual poll should still work");
        assert!(d1.processed_count.load(Ordering::SeqCst) >= 1);
        ctrl.close_bus().expect("close");
    }

    #[test]
    fn drop_stops_background_polling_thread() {
        let bus = Arc::new(MockBus::new());
        {
            let ctrl = CoreController::new(bus.clone());
            let d1 = Arc::new(FakeDevice::new(1, 0x11));
            ctrl.add_device(d1).expect("add d1");
            std::thread::sleep(Duration::from_millis(5));
        }

        assert_eq!(Arc::strong_count(&bus), 1);
    }

    #[test]
    fn custom_frames_send_and_route_in_both_polling_modes() {
        for mode in [PollingMode::Manual, PollingMode::Background] {
            let bus = Arc::new(MockFrameBus::default());
            let core = Arc::new(CoreController::new_internal(bus.clone(), mode));
            let ctrl: VendorController<FakeFrameDevice, TestFrame> =
                VendorController::new_shared(core);
            let (feedback_tx, feedback_rx) = mpsc::channel();
            for id in 1..=2 {
                ctrl.add_motor_with(id, |_| {
                    Ok(FakeFrameDevice {
                        id,
                        feedback: feedback_tx.clone(),
                    })
                })
                .expect("add device");
            }

            ctrl.bus()
                .send(test_frame(1, 256, false))
                .expect("send frame");
            assert_eq!(
                bus.sent.lock().unwrap().as_slice(),
                &[test_frame(1, 256, false)]
            );

            bus.rx.lock().unwrap().extend([
                test_frame(1, 256, false),
                test_frame(3, 256, true),
                test_frame(2, 16, true),
                test_frame(1, 256, true),
            ]);
            if mode == PollingMode::Manual {
                ctrl.poll_feedback_once().expect("poll frames");
            }
            for expected in [test_frame(2, 16, true), test_frame(1, 256, true)] {
                let (device_id, actual) = feedback_rx
                    .recv_timeout(Duration::from_secs(1))
                    .expect("routed frame");
                assert_eq!(device_id, expected.device_id);
                assert_eq!(actual, expected);
            }
            ctrl.close_bus().expect("close controller");
            assert!(matches!(
                feedback_rx.try_recv(),
                Err(mpsc::TryRecvError::Empty)
            ));
        }
    }
}

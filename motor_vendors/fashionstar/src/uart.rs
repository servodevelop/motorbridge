use crate::motor::MotorFeedbackState;
use crate::protocol::{self, CAN_ID};
use motor_core::bus::{CanBus, UartFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::feedback::{FeedbackCache, FeedbackSnapshot};
use parking_lot::FairMutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

struct Pending {
    command: u8,
    address: Option<u8>,
    length: usize,
    reply: Option<(Vec<u8>, Instant)>,
}

pub(crate) struct UartMotor {
    motor_id: u16,
    bus: Arc<dyn CanBus<UartFrame>>,
    transactions: Arc<FairMutex<()>>,
    pending: Mutex<Option<Pending>>,
    ready: Condvar,
    previous: Mutex<Option<(i32, Instant)>>,
    state: FeedbackCache<MotorFeedbackState>,
    state_valid: AtomicBool,
}

// Keep the public FashionStar parameter IDs; translate only at the protocol boundary.
fn uart_register(rid: u8) -> Result<(u8, usize)> {
    match rid {
        0x01 => Ok((6, 2)),
        0x02 => Ok((7, 2)),
        0x03 | 0x04 => Ok((8, 4)),
        0x10 => Ok((1, 2)),
        0x11 => Ok((2, 2)),
        0x12 => Ok((3, 2)),
        0x13 => Ok((4, 2)),
        0x14 => Ok((5, 1)),
        0x80 => Ok((33, 1)),
        0x81 => Ok((34, 1)),
        0x82 => Ok((36, 1)),
        0x83 => Ok((37, 1)),
        0x84 => Ok((38, 2)),
        0x85 => Ok((39, 2)),
        0x86 => Ok((40, 2)),
        0x87 => Ok((41, 2)),
        0x88 => Ok((42, 2)),
        0x89 => Ok((43, 2)),
        0x8b => Ok((45, 1)),
        0x8c => Ok((46, 1)),
        0x8e => Ok((48, 1)),
        0x8f => Ok((49, 1)),
        0x90 => Ok((50, 2)),
        0x91 => Ok((51, 2)),
        0x92 => Ok((52, 2)),
        0x93 => Ok((53, 2)),
        _ => Err(MotorError::Unsupported(format!(
            "FashionStar UART does not support parameter 0x{rid:02X}"
        ))),
    }
}

impl UartMotor {
    pub(crate) fn new(
        motor_id: u16,
        _feedback_id: u16,
        model: &str,
        bus: Arc<dyn CanBus<UartFrame>>,
        transactions: Arc<FairMutex<()>>,
    ) -> Result<Self> {
        if model != "uart" || motor_id > 254 {
            return Err(MotorError::InvalidArgument(
                "FashionStar UART requires model=uart and ServoID 0..254".to_string(),
            ));
        }
        Ok(Self {
            motor_id,
            bus,
            transactions,
            pending: Mutex::new(None),
            ready: Condvar::new(),
            previous: Mutex::new(None),
            state: FeedbackCache::new(),
            state_valid: AtomicBool::new(false),
        })
    }

    fn send(&self, command: u8, fields: &[u8]) -> Result<()> {
        let mut data = vec![command, self.motor_id as u8];
        data.extend_from_slice(fields);
        self.bus.send(UartFrame { data, is_rx: false })
    }

    fn request(
        &self,
        command: u8,
        address: Option<u8>,
        length: usize,
        timeout: Duration,
    ) -> Result<(Vec<u8>, Instant)> {
        if timeout.is_zero() {
            return Err(MotorError::Timeout(
                "FashionStar UART query timeout".to_string(),
            ));
        }
        let mut pending = self
            .pending
            .lock()
            .map_err(|_| MotorError::Io("UART reply lock poisoned".to_string()))?;
        *pending = Some(Pending {
            command,
            address,
            length,
            reply: None,
        });
        drop(pending);
        let result = (|| {
            self.send(command, address.as_slice())?;
            let deadline = Instant::now().checked_add(timeout).ok_or_else(|| {
                MotorError::InvalidArgument("UART timeout is too large".to_string())
            })?;
            let mut pending = self
                .pending
                .lock()
                .map_err(|_| MotorError::Io("UART reply lock poisoned".to_string()))?;
            loop {
                if let Some(reply) = pending.as_mut().and_then(|p| p.reply.take()) {
                    return Ok(reply);
                }
                let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                    return Err(MotorError::Timeout(format!(
                        "FashionStar UART query timeout: ServoID={}, command=0x{command:02X}",
                        self.motor_id
                    )));
                };
                pending = self
                    .ready
                    .wait_timeout(pending, remaining)
                    .map_err(|_| MotorError::Io("UART reply lock poisoned".to_string()))?
                    .0;
            }
        })();
        self.pending
            .lock()
            .map_err(|_| MotorError::Io("UART reply lock poisoned".to_string()))?
            .take();
        result
    }

    pub(crate) fn get_register_u32(&self, rid: u8, timeout: Duration) -> Result<u32> {
        let _guard = self.transactions.lock();
        if (0x15..=0x18).contains(&rid) {
            let (command, length) = if rid == 0x18 { (0x0a, 4) } else { (0x10, 8) };
            let (data, _) = self.request(command, None, length, timeout)?;
            return Ok(match rid {
                0x15 => u32::from_le_bytes(data[2..6].try_into().expect("validated angle length")),
                0x16 => u32::from(u16::from_le_bytes([data[4], data[5]])),
                0x17 => u32::from(u16::from_le_bytes([data[6], data[7]])),
                _ => u32::from(u16::from_le_bytes([data[2], data[3]])),
            });
        }
        let (address, width) = uart_register(rid)?;
        let (data, _) = self.request(0x03, Some(address), width + 3, timeout)?;
        let mut bytes = [0; 4];
        bytes[..width].copy_from_slice(&data[3..]);
        let value = u32::from_le_bytes(bytes);
        Ok(if rid == 0x04 { value >> 16 } else { value })
    }

    pub(crate) fn write_register_u32(&self, rid: u8, value: u32) -> Result<()> {
        let (address, width) = uart_register(rid)?;
        let valid = rid >= 0x80
            && u64::from(value) < (1u64 << (width * 8))
            && match address {
                33 | 37 | 46 | 48 | 49 => value <= 1,
                34 => value <= 254,
                36 => (1..=8).contains(&value),
                41 => (1..4096).contains(&value),
                _ => true,
            };
        if !valid {
            return Err(MotorError::InvalidArgument(format!(
                "invalid FashionStar UART write: parameter=0x{rid:02X}, value={value}"
            )));
        }
        let _guard = self.transactions.lock();
        let mut fields = vec![address];
        fields.extend_from_slice(&value.to_le_bytes()[..width]);
        // Optional ACKs are disabled by default. Success means the packet was sent.
        self.send(0x04, &fields)
    }

    pub(crate) fn send_cmd_pos_vel(&self, target_position: f32, velocity_limit: f32) -> Result<()> {
        let target = protocol::angle_raw(target_position)?;
        let speed = protocol::speed_raw(velocity_limit)?;
        let mut fields = target.to_le_bytes().to_vec();
        fields.extend_from_slice(&speed.to_le_bytes());
        // The protocol requires at least 20 ms for each ramp; power=0 uses the configured limit.
        fields.extend_from_slice(&[20, 0, 20, 0, 0, 0]);
        let _guard = self.transactions.lock();
        self.send(0x0f, &fields)
    }

    pub(crate) fn ensure_control_mode(&self, mode: u32, _timeout: Duration) -> Result<()> {
        if mode == 2 {
            Ok(())
        } else {
            Err(MotorError::Unsupported(
                "FashionStar UART supports mode 2(POS_VEL)".to_string(),
            ))
        }
    }

    pub(crate) fn enable(&self) -> Result<()> {
        self.stop()
    }

    pub(crate) fn stop(&self) -> Result<()> {
        let _guard = self.transactions.lock();
        self.send(0x18, &[0x11, 0, 0])
    }

    pub(crate) fn disable(&self) -> Result<()> {
        let _guard = self.transactions.lock();
        self.send(0x18, &[0x10, 0, 0])
    }

    pub(crate) fn set_zero_position(&self) -> Result<()> {
        let _guard = self.transactions.lock();
        // Origin changes require released output, as in the manufacturer's web tool.
        self.send(0x18, &[0x10, 0, 0])?;
        std::thread::sleep(Duration::from_millis(20));
        self.send(0x17, &[0])?;
        self.previous
            .lock()
            .map_err(|_| MotorError::Io("UART position lock poisoned".to_string()))?
            .take();
        self.state_valid.store(false, Ordering::Release);
        Ok(())
    }

    pub(crate) fn request_motor_feedback(&self, timeout: Duration) -> Result<()> {
        let _guard = self.transactions.lock();
        let previous = self
            .previous
            .lock()
            .map_err(|_| MotorError::Io("UART position lock poisoned".to_string()))?
            .take();
        let (data, received_at) = self.request(0x16, None, 17, timeout)?;
        let position =
            i32::from_le_bytes(data[11..15].try_into().expect("validated monitor length"));
        let current = u16::from_le_bytes([data[4], data[5]]);
        let temperature = u16::from_le_bytes([data[8], data[9]]);
        let velocity = previous
            .and_then(|(last_position, last_at)| {
                let elapsed = received_at.duration_since(last_at).as_secs_f64();
                (elapsed > 0.0).then(|| {
                    ((f64::from(position) - f64::from(last_position)) * std::f64::consts::PI
                        / 1800.0
                        / elapsed) as f32
                })
            })
            .unwrap_or(f32::NAN);
        self.state.update(MotorFeedbackState {
            can_id: self.motor_id as u8,
            arbitration_id: u32::from(CAN_ID),
            status_code: data[10],
            pos: position as f32 * (std::f32::consts::PI / 1800.0),
            vel: velocity,
            current_a: f32::from(current) / 1000.0,
            temperature_c: protocol::temperature_c(temperature),
        })?;
        self.state_valid.store(true, Ordering::Release);
        *self
            .previous
            .lock()
            .map_err(|_| MotorError::Io("UART position lock poisoned".to_string()))? =
            Some((position, received_at));
        Ok(())
    }

    pub(crate) fn latest_state_snapshot(&self) -> Option<FeedbackSnapshot<MotorFeedbackState>> {
        if !self.state_valid.load(Ordering::Acquire) {
            return None;
        }
        self.state.latest().ok().flatten()
    }

    pub(crate) fn latest_state(&self) -> Option<MotorFeedbackState> {
        self.latest_state_snapshot().map(|s| s.state)
    }
}

impl MotorDevice<UartFrame> for UartMotor {
    fn vendor(&self) -> &'static str {
        "fashionstar"
    }

    fn model(&self) -> &str {
        "uart"
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        // UART addresses replies by ServoID; retain the common interface placeholder.
        CAN_ID
    }

    fn enable(&self) -> Result<()> {
        UartMotor::enable(self)
    }

    fn disable(&self) -> Result<()> {
        UartMotor::disable(self)
    }

    fn accepts_frame(&self, frame: &UartFrame) -> bool {
        frame.is_rx && frame.data.get(1) == Some(&(self.motor_id as u8))
    }

    fn process_feedback_frame(&self, frame: UartFrame) -> Result<()> {
        if !self.accepts_frame(&frame) {
            return Err(MotorError::Protocol(
                "invalid FashionStar UART response".to_string(),
            ));
        }
        let mut pending = self
            .pending
            .lock()
            .map_err(|_| MotorError::Io("UART reply lock poisoned".to_string()))?;
        if let Some(pending) = pending.as_mut() {
            if pending.reply.is_none()
                && frame.data.len() == pending.length
                && frame.data[0] == pending.command
                && pending
                    .address
                    .is_none_or(|address| frame.data.get(2) == Some(&address))
            {
                pending.reply = Some((frame.data, Instant::now()));
                self.ready.notify_one();
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FashionstarController;
    use std::sync::mpsc::{self, Receiver, Sender};

    struct TestBus {
        sent: Sender<UartFrame>,
        received: Mutex<Receiver<UartFrame>>,
    }

    impl CanBus<UartFrame> for TestBus {
        fn send(&self, frame: UartFrame) -> Result<()> {
            self.sent.send(frame).unwrap();
            Ok(())
        }
        fn recv(&self, _: Duration) -> Result<Option<UartFrame>> {
            Ok(self.received.lock().unwrap().try_recv().ok())
        }
        fn shutdown(&self) -> Result<()> {
            Ok(())
        }
    }

    fn setup() -> (
        FashionstarController,
        Receiver<UartFrame>,
        Sender<UartFrame>,
    ) {
        let (sent, commands) = mpsc::channel();
        let (replies, received) = mpsc::channel();
        (
            FashionstarController::new_uart_bus(Arc::new(TestBus {
                sent,
                received: Mutex::new(received),
            })),
            commands,
            replies,
        )
    }

    fn reply(replies: &Sender<UartFrame>, data: &[u8]) {
        replies
            .send(UartFrame {
                data: data.to_vec(),
                is_rx: true,
            })
            .unwrap();
    }

    #[test]
    fn uart_commands_and_invalid_arguments() {
        let (controller, commands, _) = setup();
        assert!(controller.add_motor(255, 0, "uart").is_err());
        assert!(controller.add_motor(0, 0, "fashionstar").is_err());
        let motor = controller.add_motor(0, CAN_ID, "uart").unwrap();
        motor
            .send_cmd_pos_vel(-std::f32::consts::PI, std::f32::consts::PI)
            .unwrap();
        assert_eq!(
            commands.recv().unwrap().data,
            [0x0f, 0, 0xf8, 0xf8, 0xff, 0xff, 8, 7, 20, 0, 20, 0, 0, 0]
        );
        for (operation, expected) in [
            (motor.enable(), vec![0x18, 0, 0x11, 0, 0]),
            (motor.disable(), vec![0x18, 0, 0x10, 0, 0]),
        ] {
            operation.unwrap();
            assert_eq!(commands.recv().unwrap().data, expected);
        }
        let zero_started = Instant::now();
        motor.set_zero_position().unwrap();
        assert!(zero_started.elapsed() >= Duration::from_millis(20));
        assert_eq!(commands.recv().unwrap().data, [0x18, 0, 0x10, 0, 0]);
        assert_eq!(commands.recv().unwrap().data, [0x17, 0, 0]);
        motor
            .write_register_u32(0x91, (-900i16) as u16 as u32)
            .unwrap();
        assert_eq!(commands.recv().unwrap().data, [4, 0, 51, 0x7c, 0xfc]);
        motor.write_register_u32(0x81, 2).unwrap();
        assert_eq!(commands.recv().unwrap().data, [4, 0, 34, 2]);
        motor.write_register_u32(0x82, 5).unwrap();
        assert_eq!(commands.recv().unwrap().data, [4, 0, 36, 5]);
        for (rid, value) in [
            (0x03, 0),
            (0x81, 255),
            (0x82, 0),
            (0x82, 9),
            (0x87, 4096),
            (0x91, 65536),
            (0x8a, 1),
        ] {
            assert!(motor.write_register_u32(rid, value).is_err());
        }
        assert!(motor.send_cmd_pos_vel(f32::NAN, 1.0).is_err());
        assert!(motor.send_cmd_pos_vel(0.0, 0.0).is_err());
        assert!(motor.ensure_control_mode(3, Duration::ZERO).is_err());
        assert!(commands.try_recv().is_err());
        controller.close_bus().unwrap();
    }

    #[test]
    fn uart_queries_match_address_and_serialize_the_bus() {
        let (controller, commands, replies) = setup();
        let first = controller.add_motor(1, CAN_ID, "uart").unwrap();
        let second = controller.add_motor(2, CAN_ID, "uart").unwrap();
        let query = std::thread::spawn(move || {
            assert_eq!(
                first
                    .get_register_u32(0x01, Duration::from_secs(1))
                    .unwrap(),
                0x1234
            );
            first.get_register_u32(0x01, Duration::from_secs(1))
        });
        assert_eq!(
            commands.recv_timeout(Duration::from_secs(1)).unwrap().data,
            [3, 1, 6]
        );
        let write = std::thread::spawn(move || second.disable());
        reply(&replies, &[3, 1, 7, 0, 0]); // wrong register
        reply(&replies, &[3, 2, 6, 0, 0]); // wrong motor
        reply(&replies, &[4, 1, 6, 1]); // optional write ACK
        assert!(commands.recv_timeout(Duration::from_millis(20)).is_err());
        reply(&replies, &[3, 1, 6, 0x34, 0x12]);
        assert_eq!(
            commands.recv_timeout(Duration::from_secs(1)).unwrap().data,
            [0x18, 2, 0x10, 0, 0]
        );
        write.join().unwrap().unwrap();
        assert_eq!(
            commands.recv_timeout(Duration::from_secs(1)).unwrap().data,
            [3, 1, 6]
        );
        reply(&replies, &[3, 1, 6, 0x34, 0x12]);
        assert_eq!(query.join().unwrap().unwrap(), 0x1234);
        controller.close_bus().unwrap();
    }

    #[test]
    fn uart_reads_use_common_parameter_ids() {
        let (controller, commands, replies) = setup();
        let motor = controller.add_motor(0, CAN_ID, "uart").unwrap();
        assert_eq!(motor.motor_id(), 0);
        assert_eq!(motor.feedback_id(), CAN_ID);
        for (rid, request, response, expected) in [
            (
                0x03,
                vec![3, 0, 8],
                vec![3, 0, 8, 0x78, 0x56, 0x34, 0x12],
                0x12345678,
            ),
            (
                0x04,
                vec![3, 0, 8],
                vec![3, 0, 8, 0x78, 0x56, 0x34, 0x12],
                0x1234,
            ),
            (0x10, vec![3, 0, 1], vec![3, 0, 1, 0xe0, 0x2e], 12000),
            (0x14, vec![3, 0, 5], vec![3, 0, 5, 0x80], 0x80),
            (
                0x15,
                vec![0x10, 0],
                vec![0x10, 0, 0xf8, 0xf8, 0xff, 0xff, 0xff, 0xff],
                (-1800i32) as u32,
            ),
            (
                0x16,
                vec![0x10, 0],
                vec![0x10, 0, 0xf8, 0xf8, 0xff, 0xff, 0xff, 0xff],
                0xffff,
            ),
            (
                0x17,
                vec![0x10, 0],
                vec![0x10, 0, 0xf8, 0xf8, 0xff, 0xff, 0xff, 0xff],
                0xffff,
            ),
            (0x18, vec![0x0a, 0], vec![0x0a, 0, 0xf8, 0xf8], 0xf8f8),
        ] {
            let cloned = Arc::clone(&motor);
            let query =
                std::thread::spawn(move || cloned.get_register_u32(rid, Duration::from_secs(1)));
            assert_eq!(
                commands.recv_timeout(Duration::from_secs(1)).unwrap().data,
                request
            );
            reply(&replies, &response);
            assert_eq!(query.join().unwrap().unwrap(), expected);
        }
        assert!(motor
            .get_register_u32(0x53, Duration::from_secs(1))
            .is_err());
        assert!(commands.try_recv().is_err());
        controller.close_bus().unwrap();
    }

    #[test]
    fn uart_feedback_maps_signed_position_current_temperature_and_resets_velocity() {
        let (controller, commands, replies) = setup();
        let motor = controller.add_motor(0, CAN_ID, "uart").unwrap();
        for (index, position) in [-1800i32, -900, 0].into_iter().enumerate() {
            let cloned = Arc::clone(&motor);
            let query =
                std::thread::spawn(move || cloned.request_motor_feedback(Duration::from_secs(1)));
            assert_eq!(
                commands.recv_timeout(Duration::from_secs(1)).unwrap().data,
                [0x16, 0]
            );
            let mut data = vec![0x16, 0, 0xe0, 0x2e, 0xe8, 3, 0, 0, 0, 8, 0x80];
            data.extend_from_slice(&position.to_le_bytes());
            data.extend_from_slice(&(-1i16).to_le_bytes());
            reply(&replies, &data);
            query.join().unwrap().unwrap();
            let state = motor.latest_state().unwrap();
            assert!((state.pos - position as f32 * std::f32::consts::PI / 1800.0).abs() < 1e-5);
            assert_eq!(state.current_a, 1.0);
            assert_eq!(state.temperature_c, 25.0);
            assert_eq!(state.status_code, 0x80);
            assert_eq!(state.arbitration_id, u32::from(CAN_ID));
            if index == 1 {
                assert!(state.vel.is_finite() && state.vel > 0.0);
            } else {
                assert!(state.vel.is_nan());
            }
            if index == 1 {
                assert!(motor
                    .request_motor_feedback(Duration::from_millis(10))
                    .is_err());
                assert_eq!(commands.recv().unwrap().data, [0x16, 0]);
            }
        }
        motor.set_zero_position().unwrap();
        assert!(motor.latest_state().is_none());
        controller.close_bus().unwrap();
    }
}

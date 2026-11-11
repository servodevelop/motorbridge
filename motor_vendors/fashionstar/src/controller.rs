use crate::motor::FashionstarMotor;
use crate::uart_serial::FashionstarUartBus;
use motor_core::bus::{open_can_bus, CanBus, UartFrame};
use motor_core::controller::CoreController;
use motor_core::error::Result;
use motor_core::vendor_controller::VendorController;
use parking_lot::FairMutex;
use std::sync::Arc;

pub struct FashionstarController {
    controller: ControllerInner,
}

enum ControllerInner {
    Can(VendorController<FashionstarMotor>),
    Uart {
        controller: VendorController<FashionstarMotor, UartFrame>,
        transactions: Arc<FairMutex<()>>,
    },
}

impl FashionstarController {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            controller: ControllerInner::Can(VendorController::new(bus)),
        }
    }

    pub fn new_shared(core: Arc<CoreController>) -> Self {
        Self {
            controller: ControllerInner::Can(VendorController::new_shared(core)),
        }
    }

    pub fn new_socketcan(channel: &str) -> Result<Self> {
        Ok(Self::new(open_can_bus(channel)?))
    }

    pub fn new_uart(port: &str, baud: u32) -> Result<Self> {
        Ok(Self::new_uart_bus(Arc::new(FashionstarUartBus::open(
            port, baud,
        )?)))
    }

    pub fn new_uart_bus(bus: Arc<dyn CanBus<UartFrame>>) -> Self {
        Self {
            controller: ControllerInner::Uart {
                controller: VendorController::new(bus),
                transactions: Arc::new(FairMutex::new(())),
            },
        }
    }

    pub fn add_motor(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<Arc<FashionstarMotor>> {
        match &self.controller {
            ControllerInner::Can(controller) => controller.add_motor_with(motor_id, |bus| {
                FashionstarMotor::new(motor_id, feedback_id, model, bus)
            }),
            ControllerInner::Uart {
                controller,
                transactions,
            } => controller.add_motor_with(motor_id, |bus| {
                FashionstarMotor::new_uart(
                    motor_id,
                    feedback_id,
                    model,
                    bus,
                    Arc::clone(transactions),
                )
            }),
        }
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<FashionstarMotor>> {
        match &self.controller {
            ControllerInner::Can(controller) => controller.get_motor(motor_id),
            ControllerInner::Uart { controller, .. } => controller.get_motor(motor_id),
        }
    }

    pub fn poll_feedback_once(&self) -> Result<()> {
        match &self.controller {
            ControllerInner::Can(controller) => controller.poll_feedback_once(),
            ControllerInner::Uart { controller, .. } => controller.poll_feedback_once(),
        }
    }

    pub fn enable_all(&self) -> Result<()> {
        match &self.controller {
            ControllerInner::Can(controller) => controller.enable_all(),
            ControllerInner::Uart { controller, .. } => controller.enable_all(),
        }
    }

    pub fn disable_all(&self) -> Result<()> {
        match &self.controller {
            ControllerInner::Can(controller) => controller.disable_all(),
            ControllerInner::Uart { controller, .. } => controller.disable_all(),
        }
    }

    pub fn shutdown(&self) -> Result<()> {
        match &self.controller {
            ControllerInner::Can(controller) => controller.shutdown(),
            ControllerInner::Uart { controller, .. } => controller.shutdown(),
        }
    }

    pub fn close_bus(&self) -> Result<()> {
        match &self.controller {
            ControllerInner::Can(controller) => controller.close_bus(),
            ControllerInner::Uart { controller, .. } => controller.close_bus(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CAN_ID;
    use motor_core::CanFrame;
    use std::collections::VecDeque;
    use std::sync::Mutex;
    use std::time::Duration;

    #[derive(Default)]
    struct CanBusStub {
        sent: Mutex<Vec<CanFrame>>,
        received: Mutex<VecDeque<CanFrame>>,
    }

    impl CanBus for CanBusStub {
        fn send(&self, frame: CanFrame) -> Result<()> {
            self.sent.lock().unwrap().push(frame);
            if matches!(frame.data[0], b'r' | b'R') {
                let value: u32 = match frame.data[2] {
                    0x15 => -900i32 as u32,
                    0x14 => 0x80,
                    0x13 => 2048,
                    0x11 => 500,
                    _ => 0,
                };
                let bytes = value.to_le_bytes();
                let pair = frame.data[0] == b'R';
                self.received.lock().unwrap().push_back(CanFrame {
                    arbitration_id: u32::from(CAN_ID),
                    data: [
                        if pair { b'V' } else { b'v' },
                        frame.data[1],
                        frame.data[2],
                        bytes[0],
                        bytes[1],
                        frame.data[2] + 1,
                        bytes[2],
                        bytes[3],
                    ],
                    dlc: if pair { 8 } else { 5 },
                    is_extended: false,
                    is_rx: true,
                });
            }
            Ok(())
        }
        fn recv(&self, _: Duration) -> Result<Option<CanFrame>> {
            Ok(self.received.lock().unwrap().pop_front())
        }
        fn shutdown(&self) -> Result<()> {
            Ok(())
        }
    }

    #[test]
    fn can_model_preserves_commands_and_feedback() {
        let bus = Arc::new(CanBusStub::default());
        let controller = FashionstarController::new(bus.clone());
        assert!(controller.add_motor(0, CAN_ID, "uart").is_err());
        let motor = controller.add_motor(0, CAN_ID, "fashionstar").unwrap();
        assert_eq!(motor.vendor(), "fashionstar");
        assert_eq!(motor.model(), "fashionstar");
        assert_eq!(motor.motor_id(), 0);
        assert_eq!(motor.feedback_id(), CAN_ID);
        motor
            .send_cmd_pos_vel(-std::f32::consts::PI, std::f32::consts::PI)
            .unwrap();
        let sent = bus.sent.lock().unwrap();
        assert_eq!(sent.len(), 4);
        assert_eq!(&sent[0].data[..5], &[b'w', 0, 0x52, 2, 0]);
        assert_eq!(sent[1].data, [b'W', 0, 0x53, 0xf8, 0xf8, 0x54, 0xff, 0xff]);
        assert_eq!(&sent[2].data[..5], &[b'w', 0, 0x58, 8, 7]);
        assert_eq!(&sent[3].data[..5], &[b'w', 0, 0x59, 1, 0]);
        drop(sent);
        motor
            .request_motor_feedback(Duration::from_secs(1))
            .unwrap();
        let state = motor.latest_state().unwrap();
        assert!((state.pos + std::f32::consts::FRAC_PI_2).abs() < 1e-5);
        assert!(state.vel.is_nan());
        assert_eq!(state.current_a, 0.5);
        assert_eq!(state.temperature_c, 25.0);
        assert_eq!(state.arbitration_id, u32::from(CAN_ID));
        assert_eq!(state.status_code, 0x80);
        controller.close_bus().unwrap();
    }
}

pub mod controller;
pub mod motor;
pub mod protocol;
pub mod registers;
mod uart;
pub mod uart_serial;

pub use controller::FashionstarController;
pub use motor::{FashionstarMotor, MotorFeedbackState};
pub use protocol::CAN_ID;

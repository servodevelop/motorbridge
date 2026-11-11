use motor_core::bus::{CanBus, UartFrame};
use motor_core::error::{MotorError, Result};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{Read, Write};
use std::sync::Mutex;
use std::time::{Duration, Instant};

struct Inner {
    port: Option<Box<dyn SerialPort>>,
    buffer: Vec<u8>,
    last_tx: Option<Instant>,
}

/// FashionStar native UART/RS-485 framing. The adapter handles half-duplex direction.
pub struct FashionstarUartBus {
    inner: Mutex<Inner>,
}

impl FashionstarUartBus {
    pub fn open(port: &str, baud: u32) -> Result<Self> {
        if baud == 0 {
            return Err(MotorError::InvalidArgument(
                "UART baud must be positive".to_string(),
            ));
        }
        let port = serialport::new(port, baud)
            .timeout(Duration::from_millis(10))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()
            .map_err(|e| MotorError::Io(format!("open FashionStar UART port failed: {e}")))?;
        Ok(Self {
            inner: Mutex::new(Inner {
                port: Some(port),
                buffer: Vec::new(),
                last_tx: None,
            }),
        })
    }

    fn encode(frame: &UartFrame) -> Result<Vec<u8>> {
        let Some((&command, params)) = frame.data.split_first() else {
            return Err(MotorError::InvalidArgument(
                "empty FashionStar UART frame".to_string(),
            ));
        };
        let length = u8::try_from(params.len()).map_err(|_| {
            MotorError::InvalidArgument("FashionStar UART payload exceeds 255 bytes".to_string())
        })?;
        if frame.is_rx {
            return Err(MotorError::InvalidArgument(
                "cannot send a UART response frame".to_string(),
            ));
        }
        let mut packet = vec![0x12, 0x4c, command, length];
        packet.extend_from_slice(params);
        packet.push(packet.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)));
        Ok(packet)
    }

    fn decode(buffer: &mut Vec<u8>) -> Option<UartFrame> {
        loop {
            let Some(start) = buffer.windows(2).position(|w| w == [0x05, 0x1c]) else {
                let keep = usize::from(buffer.last() == Some(&0x05));
                buffer.drain(..buffer.len() - keep);
                return None;
            };
            buffer.drain(..start);
            if buffer.len() < 4 {
                return None;
            }
            let valid_length = match buffer[2] {
                0x01 => buffer[3] == 1,
                0x03 => matches!(buffer[3], 3 | 4 | 6),
                0x04 => buffer[3] == 3,
                0x0a => buffer[3] == 3,
                0x0f | 0x17 | 0x18 => buffer[3] == 2,
                0x10 => buffer[3] == 7,
                0x16 => buffer[3] == 16,
                _ => false,
            };
            if !valid_length {
                buffer.remove(0);
                continue;
            }
            let length = usize::from(buffer[3]) + 5;
            if buffer.len() < length {
                return None;
            }
            let checksum = buffer[..length - 1]
                .iter()
                .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
            if checksum != buffer[length - 1] {
                buffer.remove(0);
                continue;
            }
            let mut data = vec![buffer[2]];
            data.extend_from_slice(&buffer[4..length - 1]);
            buffer.drain(..length);
            return Some(UartFrame { data, is_rx: true });
        }
    }
}

impl CanBus<UartFrame> for FashionstarUartBus {
    fn send(&self, frame: UartFrame) -> Result<()> {
        let packet = Self::encode(&frame)?;
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("UART lock poisoned".to_string()))?;
        // Protocol recommends 5–10 ms between commands, including optional ACKs.
        if let Some(last) = inner.last_tx {
            std::thread::sleep(Duration::from_millis(10).saturating_sub(last.elapsed()));
        }
        let port = inner
            .port
            .as_mut()
            .ok_or_else(|| MotorError::Io("UART bus is closed".to_string()))?;
        port.write_all(&packet)
            .map_err(|e| MotorError::Io(format!("UART write failed: {e}")))?;
        port.flush()
            .map_err(|e| MotorError::Io(format!("UART flush failed: {e}")))?;
        inner.last_tx = Some(Instant::now());
        Ok(())
    }

    fn recv(&self, timeout: Duration) -> Result<Option<UartFrame>> {
        let start = Instant::now();
        loop {
            {
                let mut inner = self
                    .inner
                    .lock()
                    .map_err(|_| MotorError::Io("UART lock poisoned".to_string()))?;
                if inner.port.is_none() {
                    return Err(MotorError::Io("UART bus is closed".to_string()));
                }
                if let Some(frame) = Self::decode(&mut inner.buffer) {
                    return Ok(Some(frame));
                }
                let port = inner.port.as_mut().expect("checked port");
                let available = port
                    .bytes_to_read()
                    .map_err(|e| MotorError::Io(format!("UART read status failed: {e}")))?
                    as usize;
                if available > 0 {
                    let mut bytes = [0u8; 256];
                    let count = available.min(bytes.len());
                    match port.read(&mut bytes[..count]) {
                        Ok(n) => inner.buffer.extend_from_slice(&bytes[..n]),
                        Err(e)
                            if matches!(
                                e.kind(),
                                std::io::ErrorKind::TimedOut
                                    | std::io::ErrorKind::WouldBlock
                                    | std::io::ErrorKind::Interrupted
                            ) => {}
                        Err(e) => return Err(MotorError::Io(format!("UART read failed: {e}"))),
                    }
                    if let Some(frame) = Self::decode(&mut inner.buffer) {
                        return Ok(Some(frame));
                    }
                }
            }
            if start.elapsed() >= timeout {
                return Ok(None);
            }
            std::thread::sleep(Duration::from_micros(200));
        }
    }

    fn shutdown(&self) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("UART lock poisoned".to_string()))?;
        inner.port.take();
        inner.buffer.clear();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_encoding_and_stream_resynchronization() {
        let frame = UartFrame {
            data: vec![0x16, 0],
            is_rx: false,
        };
        assert_eq!(
            FashionstarUartBus::encode(&frame).unwrap(),
            [0x12, 0x4c, 0x16, 1, 0, 0x75]
        );
        let reply = [0x05, 0x1c, 0x01, 0x01, 0x02, 0x25];
        let mut buffer = vec![0xff, 0x05];
        assert!(FashionstarUartBus::decode(&mut buffer).is_none());
        buffer.extend_from_slice(&reply[1..3]);
        assert!(FashionstarUartBus::decode(&mut buffer).is_none());
        buffer.extend_from_slice(&reply[3..]);
        buffer.extend_from_slice(&[0x05, 0x1c, 0x16, 0xff]);
        buffer.extend_from_slice(&[0x05, 0x1c, 0xff, 0xff]);
        buffer.extend_from_slice(&[0x05, 0x1c, 0x01, 0xff]);
        buffer.extend_from_slice(&[0x05, 0x1c, 1, 1, 2, 0]);
        buffer.extend_from_slice(&reply);
        for _ in 0..2 {
            assert_eq!(
                FashionstarUartBus::decode(&mut buffer).unwrap(),
                UartFrame {
                    data: vec![1, 2],
                    is_rx: true
                }
            );
        }
        assert!(buffer.is_empty());
    }
}

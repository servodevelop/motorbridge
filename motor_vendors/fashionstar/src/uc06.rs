use crate::protocol::CAN_ID;
use motor_core::bus::{CanBus, CanFrame};
use motor_core::error::{MotorError, Result};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{Read, Write};
use std::sync::Mutex;
use std::time::{Duration, Instant};

const CAN_BITRATES: [u32; 10] = [
    50_000, 100_000, 125_000, 200_000, 250_000, 400_000, 500_000, 750_000, 800_000, 1_000_000,
];

struct Inner {
    port: Option<Box<dyn SerialPort>>,
    buffer: Vec<u8>,
    status: Option<[u8; 12]>,
}

enum Packet {
    Frame(CanFrame),
    Status([u8; 12]),
}

/// UC-06 serial bridge for FashionStar CAN frames. USB serial runs at 1 Mbps.
pub struct Uc06Bus {
    inner: Mutex<Inner>,
    next_tx: Mutex<Instant>,
    tx_gap: Duration,
}

impl Uc06Bus {
    pub fn open(port: &str, can_bitrate: u32) -> Result<Self> {
        let index = CAN_BITRATES
            .iter()
            .position(|&rate| rate == can_bitrate)
            .ok_or_else(|| {
                MotorError::InvalidArgument(format!("unsupported UC-06 CAN bitrate: {can_bitrate}"))
            })? as u8;
        let mut port = serialport::new(port, 1_000_000)
            .timeout(Duration::from_millis(10))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()
            .map_err(|e| MotorError::Io(format!("open UC-06 port failed: {e}")))?;
        port.clear(serialport::ClearBuffer::Input)
            .map_err(|e| MotorError::Io(format!("clear UC-06 input failed: {e}")))?;
        // This configures the bridge's CAN controller, not the servo's baud register.
        port.write_all(&[0xfe, 0x01, index, 0xaa])?;
        port.flush()?;
        let bus = Self {
            inner: Mutex::new(Inner {
                port: Some(port),
                buffer: Vec::new(),
                status: None,
            }),
            next_tx: Mutex::new(Instant::now()),
            // Reserve a request/reply pair's worst-case CAN airtime. UC-06 also
            // needs a 1 ms packet gap to avoid dropping back-to-back commands.
            tx_gap: Duration::from_secs_f64(270.0 / f64::from(can_bitrate))
                .max(Duration::from_millis(1)),
        };
        let started = Instant::now();
        while started.elapsed() < Duration::from_millis(500) {
            bus.recv(Duration::from_millis(10))?;
            let status = bus
                .inner
                .lock()
                .map_err(|_| MotorError::Io("UC-06 lock poisoned".to_string()))?
                .status;
            if status.is_some_and(|s| s[1] == 0x81 && s[2] == 0 && s[3] == index) {
                return Ok(bus);
            }
        }
        Err(MotorError::Timeout(
            "UC-06 CAN bitrate configuration timeout".to_string(),
        ))
    }

    fn encode(frame: &CanFrame) -> Result<Vec<u8>> {
        if frame.is_rx
            || frame.is_extended
            || frame.arbitration_id != u32::from(CAN_ID)
            || !matches!(
                (frame.data[0], frame.dlc),
                (b'r', 3) | (b'R', 4) | (b'w', 5) | (b'W', 8)
            )
        {
            return Err(MotorError::InvalidArgument(
                "UC-06 requires FashionStar standard CAN read/write frames at 0x321".to_string(),
            ));
        }
        let mut packet = frame.data[..usize::from(frame.dlc)].to_vec();
        packet.push(0xaa);
        Ok(packet)
    }

    fn decode(buffer: &mut Vec<u8>) -> Option<Packet> {
        loop {
            let length = match buffer.first()? {
                b'v' => 5,
                b'V' => 8,
                0xfe => 12,
                _ => {
                    buffer.remove(0);
                    continue;
                }
            };
            if buffer.len() < length {
                return None;
            }
            if buffer[0] == 0xfe {
                if buffer[11] != 0xaa {
                    buffer.remove(0);
                    continue;
                }
                let mut status = [0; 12];
                status.copy_from_slice(&buffer[..12]);
                buffer.drain(..12);
                return Some(Packet::Status(status));
            }
            let mut data = [0; 8];
            data[..length].copy_from_slice(&buffer[..length]);
            buffer.drain(..length);
            return Some(Packet::Frame(CanFrame {
                arbitration_id: u32::from(CAN_ID),
                data,
                dlc: length as u8,
                is_extended: false,
                is_rx: true,
            }));
        }
    }

    fn next_frame(inner: &mut Inner) -> Result<Option<CanFrame>> {
        while let Some(packet) = Self::decode(&mut inner.buffer) {
            match packet {
                Packet::Frame(frame) => return Ok(Some(frame)),
                Packet::Status(status) => {
                    if status[1] == 0x81 {
                        inner.status = Some(status);
                    }
                    if status[2] != 0 || status[4] != 0 {
                        return Err(MotorError::BusStatus(format!(
                            "UC-06 CAN error: result={}, error={}, failed_transmissions={}",
                            status[2],
                            status[4],
                            u16::from_le_bytes([status[9], status[10]])
                        )));
                    }
                }
            }
        }
        Ok(None)
    }
}

impl CanBus for Uc06Bus {
    fn send(&self, frame: CanFrame) -> Result<()> {
        let packet = Self::encode(&frame)?;
        let mut next_tx = self
            .next_tx
            .lock()
            .map_err(|_| MotorError::Io("UC-06 TX lock poisoned".to_string()))?;
        // Leave the receive lock available while pacing outgoing packets.
        if let Some(delay) = next_tx.checked_duration_since(Instant::now()) {
            std::thread::sleep(delay);
        }
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("UC-06 lock poisoned".to_string()))?;
        let port = inner
            .port
            .as_mut()
            .ok_or_else(|| MotorError::Io("UC-06 bus is closed".to_string()))?;
        port.write_all(&packet)
            .map_err(|e| MotorError::Io(format!("UC-06 write failed: {e}")))?;
        port.flush()
            .map_err(|e| MotorError::Io(format!("UC-06 flush failed: {e}")))?;
        *next_tx = Instant::now() + self.tx_gap;
        Ok(())
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>> {
        let started = Instant::now();
        loop {
            {
                let mut inner = self
                    .inner
                    .lock()
                    .map_err(|_| MotorError::Io("UC-06 lock poisoned".to_string()))?;
                if let Some(frame) = Self::next_frame(&mut inner)? {
                    return Ok(Some(frame));
                }
                let port = inner
                    .port
                    .as_mut()
                    .ok_or_else(|| MotorError::Io("UC-06 bus is closed".to_string()))?;
                let available = port
                    .bytes_to_read()
                    .map_err(|e| MotorError::Io(format!("UC-06 read status failed: {e}")))?
                    as usize;
                if available > 0 {
                    let mut bytes = [0; 256];
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
                        Err(e) => return Err(MotorError::Io(format!("UC-06 read failed: {e}"))),
                    }
                }
                if let Some(frame) = Self::next_frame(&mut inner)? {
                    return Ok(Some(frame));
                }
            }
            if started.elapsed() >= timeout {
                return Ok(None);
            }
            std::thread::sleep(Duration::from_micros(200));
        }
    }

    fn shutdown(&self) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("UC-06 lock poisoned".to_string()))?;
        inner.port.take();
        inner.buffer.clear();
        inner.status = None;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol;

    #[test]
    fn can_payloads_keep_length_and_embedded_terminators() {
        let commands = [
            (protocol::read(0, 1, false).unwrap(), vec![0x72, 0, 1, 0xaa]),
            (
                protocol::read(0, 3, true).unwrap(),
                vec![0x52, 0, 3, 4, 0xaa],
            ),
            (
                protocol::write(0, 0x7b, 0x10, false).unwrap(),
                vec![0x77, 0, 0x7b, 0x10, 0, 0xaa],
            ),
            (
                protocol::write(0, 0x53, 0xaaffaa00, true).unwrap(),
                vec![0x57, 0, 0x53, 0, 0xaa, 0x54, 0xff, 0xaa, 0xaa],
            ),
        ];
        for (frame, expected) in commands {
            assert_eq!(Uc06Bus::encode(&frame).unwrap(), expected);
        }
        let frame = protocol::read(0, 1, false).unwrap();
        for invalid in [
            CanFrame {
                arbitration_id: 1,
                ..frame
            },
            CanFrame {
                is_extended: true,
                ..frame
            },
            CanFrame {
                is_rx: true,
                ..frame
            },
            CanFrame { dlc: 255, ..frame },
        ] {
            assert!(Uc06Bus::encode(&invalid).is_err());
        }
    }

    #[test]
    fn replies_and_adapter_status_are_separate_stream_packets() {
        let mut buffer = vec![0, 0xff, b'v', 7, 1, 0xaa];
        assert!(Uc06Bus::decode(&mut buffer).is_none());
        buffer.extend_from_slice(&[5, 0xfe, 0x81, 0, 9, 0, 0, 0, 0, 0, 11, 0, 0xaa]);
        buffer.extend_from_slice(&[b'V', 7, 3, 0xaa, 0xfe, 4, 0x56, 0x76]);
        let Some(Packet::Frame(single)) = Uc06Bus::decode(&mut buffer) else {
            panic!("missing single reply")
        };
        assert_eq!(single.dlc, 5);
        assert_eq!(&single.data[..5], &[b'v', 7, 1, 0xaa, 5]);
        assert!(protocol::decode(&single, 7).is_ok());
        let Some(Packet::Status(status)) = Uc06Bus::decode(&mut buffer) else {
            panic!("missing adapter status")
        };
        assert_eq!(status[3], 9);
        let Some(Packet::Frame(dual)) = Uc06Bus::decode(&mut buffer) else {
            panic!("missing dual reply")
        };
        assert_eq!(
            protocol::decode(&dual, 7).unwrap().second,
            Some((4, 0x7656))
        );
        assert!(buffer.is_empty());
        let mut inner = Inner {
            port: None,
            buffer: status.to_vec(),
            status: None,
        };
        let mut update = status;
        update[1] = 0x82;
        inner.buffer.extend_from_slice(&update);
        inner.buffer.extend_from_slice(&single.data[..5]);
        assert_eq!(Uc06Bus::next_frame(&mut inner).unwrap().unwrap().dlc, 5);
        assert_eq!(inner.status, Some(status));
        assert!(inner.buffer.is_empty());
        update[2] = 1;
        inner.buffer.extend_from_slice(&update);
        assert!(matches!(
            Uc06Bus::next_frame(&mut inner),
            Err(MotorError::BusStatus(_))
        ));
        assert!(Uc06Bus::open("unused", 123456).is_err());
    }
}

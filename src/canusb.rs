use serial::unix::TTYPort;
use serial::SerialPort;
use std::io::{Read, Write};
use std::time::Duration;

pub struct CANUSB {
    serial_port: TTYPort,
    inc_buffer: Vec<u8>,
}

impl CANUSB {
    pub fn new(path: &str) -> Self {
        let mut port = serial::open(path).expect("Could not open serial port");
        port.reconfigure(&|settings| {
            settings
                .set_baud_rate(serial::BaudOther(2_000_000))
                .expect("Could not set baudrate");
            settings.set_char_size(serial::Bits8);
            settings.set_parity(serial::ParityNone);
            settings.set_stop_bits(serial::Stop1);
            settings.set_flow_control(serial::FlowNone);
            Ok(())
        })
        .expect("Could not configure serial port");
        port.set_timeout(Duration::from_millis(100))
            .expect("Could not set serial port timeout");

        let init_payload = init_canusb_payload();
        port.write(&init_payload)
            .expect("Could not initialize CAN dongle");

        println!("CAN dongle initialized");

        Self {
            serial_port: port,
            inc_buffer: vec![],
        }
    }

    pub fn write_extended_frame(&mut self, id: &[u8; 4], payload: &[u8; 8]) {
        // println!("{:X?} {:X?}", id, payload);
        let mut frame = vec![0xaa];

        let mut can_bus_data_frame_info: u8 = 0xC0;
        can_bus_data_frame_info |= 0x20; // EXT frame
        can_bus_data_frame_info &= 0xEF; // Data
        can_bus_data_frame_info |= payload.len() as u8;
        frame.push(can_bus_data_frame_info);

        frame.extend_from_slice(id);

        frame.extend_from_slice(payload);

        frame.push(0x55);

        self.serial_port
            .write_all(&frame)
            .expect("Could not write to serial port");
    }

    pub fn get_incoming(&mut self) -> Option<([u8; 4], [u8; 8])> {
        let mut buf: [u8; 1] = [0; 1];
        loop {
            match self.serial_port.read(&mut buf) {
                Ok(bytes) => {
                    if bytes >= 1 {
                        self.inc_buffer.push(buf[0]);
                        if self.inc_buffer.len() == 15 {
                            break;
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    eprintln!("Recv timeout");
                    break;
                }
                Err(e) => eprintln!("{:?}", e),
            }
        }

        if self.inc_buffer.first().is_some_and(|x| *x == 0xAA) {
            // We got a message!
            // Byte 2 is:
            // bits7~6: 0b11
            // bit5: 0==std frame, 1==ext frame
            // bit4: 0==data frame, 1==remote frame
            // bits3~0: frame data length
            let byte2 = self.inc_buffer.get(1).unwrap_or(&0);
            let frame_length = 7 + (byte2 & 0b1111) as usize;
            if frame_length != 15 {
                println!(
                    "Wrong frame length: {} ({:X?})",
                    frame_length, self.inc_buffer
                );
            }
            if ((byte2 & 0x20) > 0) && (self.inc_buffer.len() >= 15) {
                // Full Extended frame received
                let can_id: [u8; 4] = self.inc_buffer[2..6].try_into().unwrap();
                let can_data: [u8; 8] = self.inc_buffer[6..14].try_into().unwrap();
                // println!("Received CAN_ID {:X?}, data: {:X?}", can_id, can_data);
                self.inc_buffer.clear();
                return Some((can_id.clone(), can_data.clone()));
            }
        } else {
            if self.inc_buffer.len() > 0 {
                self.inc_buffer.remove(0);
            }
        }

        None
    }
}

fn init_canusb_payload() -> Vec<u8> {
    let mut frame = vec![0xaa, 0x55, 0x12];
    frame.push(0x01); // Speed 1M
    frame.push(0x02); // Frame type extended
    frame.extend_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]); // Filter ID
    frame.extend_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF]); // Mask ID
    frame.push(0x00); // Mode normal
    frame.extend_from_slice(&[0x01, 0x00, 0x00, 0x00, 0x00]); // Auto restransmit off

    // Checksum is just an overflowing sum
    let cksum = frame.iter().map(|x| (*x) as u32).fold(0, |acc, i| acc + i);
    frame.push((cksum & 0xff) as u8);

    frame
}

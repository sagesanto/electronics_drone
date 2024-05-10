pub const START_BYTE: u8 = 0x01;
pub const STOP_BYTE: u8 = 0x7F;

use crc::{Crc, CRC_32_ISO_HDLC};

const ISO_HDLC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

use serial2::{SerialPort};


pub fn min_listen(port: &mut SerialPort, timeout_ms: f32) -> Result<(u8,String),std::io::Error> {
    // listen until we recieve one MIN packet, then return the message (id, content) or an error
    let mut buffer = Vec::new();
    let mut byte = [0; 1];
    let mut count = 0;
    let start_time = std::time::Instant::now();
    let timeout = std::time::Duration::from_millis(timeout_ms as u64);
    let do_timeout = timeout_ms > 0.0;
    if do_timeout {
        port.set_read_timeout(timeout)?;
    }
    while do_timeout && start_time.elapsed() < timeout || !do_timeout{
        match port.read(&mut byte) {
            Ok(_) => {
                // right here, we count up to see if we have 3 0xAA in a row (indicating the start of a msg)
                if byte[0] == START_BYTE {
                    count += 1;
                } else {
                    count = 0;
                }
                if count == 3 {
                    // we have 3 start bytes in a row, so we start reading the message
                    buffer.extend_from_slice(&[START_BYTE, START_BYTE, START_BYTE]);
                    while byte[0] != STOP_BYTE && (do_timeout && start_time.elapsed() < timeout || !do_timeout) {
                        port.read(&mut byte)?;
                        buffer.push(byte[0]);
                    }
                    let msg = decode_min(&buffer);
                    return Ok(msg)
                }
            },
            Err(e) => return Err(e)
        }
    }
    Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "Timed out"))
}

// read the checksum that was written into the message, and decode it
fn decode_cksum(msg_subbuffer: &[u8]) -> u32 {
    (msg_subbuffer[0] as u32) << 24 | (msg_subbuffer[1] as u32) << 16 | (msg_subbuffer[2] as u32) << 8 | (msg_subbuffer[3] as u32) << 0
}

// take in an array of bytes read from serial and decode MIN packet
pub fn decode_min(msg: &Vec<u8>) -> (u8,String) {
    for i in 0..3 {
        if msg[i] != START_BYTE {return (97,"BAD READ: WRONG START BYTE".to_string())}
    } 
    let id = msg[3];
    let msg_len = msg[4] as usize;

    if msg.len() != msg_len+10 { return (98,format!("BAD READ: LENGTH. Expected: {:?}, Actual: {:?}",msg_len+10,msg.len()).to_string())} 
    
    let content = &msg[5 .. msg_len+5];
    let reported_checksum = decode_cksum(&msg[msg_len+5 .. msg_len+9]);
    
    let mut digest = ISO_HDLC.digest();
    digest.update(&msg[ .. msg_len+5]);
    let calc_cksum = digest.finalize();
    let agreement = reported_checksum == calc_cksum;
    
    if !agreement { return (99,format!("BAD CHECKSUM: (Expected: {:X}, Calculated: {:X})",reported_checksum,calc_cksum).to_string())}

    (id, String::from_utf8_lossy(content).into_owned())
}
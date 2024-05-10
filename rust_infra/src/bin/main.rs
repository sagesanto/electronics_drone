extern crate nalgebra as na;
use na::{Vector3, Quaternion};
use serial2::{SerialPort, Settings, CharSize, StopBits, Parity, FlowControl};
use reqwest::Client;
use tokio;

use matrix_math::min_listen;
use matrix_math::Orientation;

use std::env;

pub const ORIENTATION_SERVER_ADDR: &str = "http://127.0.0.1:8080/orientation";

// v' = v + 2 * r x (s * v + r x v) / m where x represents the cross product, s and r are the scalar and vector parts of the quaternion, 
// respectively, and m is the sum of the squares of the components of the quaternion.
fn rotate(q:&Quaternion<f64>, v: &Vector3<f64>) -> Vector3<f64> {
    let s = q.scalar();
    let r = q.vector();
    let m = q.magnitude_squared();

    v + 2.0 * &r.cross( &(s * v + r.cross(v))) / m
}

fn orientation(q:Quaternion<f64>) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
    let x  = Vector3::new(1.0, 0.0, 0.0);
    let y  = Vector3::new(0.0, 1.0, 0.0);
    let z  = Vector3::new(0.0, 0.0, 1.0);

    (rotate(&q,&x), rotate(&q, &y), rotate(&q, &z))
}

fn open_port(port_name:&String) -> Result<SerialPort,std::io::Error> {
    SerialPort::open(port_name, |mut settings: Settings| {
        settings.set_raw();
        settings.set_baud_rate(57600)?;
        settings.set_char_size(CharSize::Bits8);
        settings.set_stop_bits(StopBits::One);
        settings.set_parity(Parity::None);
        settings.set_flow_control(FlowControl::None);
        Ok(settings)
     })
}

fn parse_gyro(content: String) -> Result<Quaternion<f64>, std::io::Error> {
    let mut iter = content.split(" ");
    let x = iter.next().unwrap().parse::<f64>().unwrap();
    let y = iter.next().unwrap().parse::<f64>().unwrap();
    let z = iter.next().unwrap().parse::<f64>().unwrap();
    let w = iter.next().unwrap().parse::<f64>().unwrap();
    Ok(Quaternion::new(w,x,y,z))
}

async fn send_orientation(client: Client, o: &Orientation) -> Result<(), reqwest::Error> {
    let _res = client.post(ORIENTATION_SERVER_ADDR)
        .json(&o)
        .send()
        .await?;
    Ok(())
}

async fn gyro_handler(_id:u8, content:String, client: Client) {
    // make a quaternion from the packet we recieved
    let q = parse_gyro(content).unwrap();
    // make an x-y-z coordinate system from the quaternion
    let (x_prime,y_prime,z_prime) = orientation(q); 
    // make an Orientation struct from the x-y-z coordinate system
    let o = Orientation {
        x: [x_prime.x,x_prime.y,x_prime.z],
        y: [y_prime.x,y_prime.y,y_prime.z],
        z: [z_prime.x,z_prime.y,z_prime.z],
    };
    // send the orientation to the server
    let _ = send_orientation(client, &o).await;

}

// cross run --target x86_64-pc-windows-gnu
// cross build --target x86_64-pc-windows-gnu --release

async fn dispatch(id: u8, content: String, client: Client) {
    if id == 0 {
        // gyroscope packet recieved
        gyro_handler(id, content, client).await;
    } else if id == 97 || id == 98 || id == 99 {
        // MIN error reading
        println!("MIN ERROR: {:?}",content);
    } else if id == 1 {
        // force sensor packet received
        println!("Recieved force measurement of {:?} N",content);
    } 
    // else if id == 2{
    //     // PID packet (target x, target y, x, y, motor vals) recieved
    //     PID_handler(id,content,client).await;
    // }
    else {
        // unknown id
        println!("MISSING HANDLER. ID: {:?}, Content: {:?}",id,content);
    }
}
#[tokio::main]
async fn main() {
    let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    let (x_prime,y_prime,z_prime) = orientation(q); 
    println!("x': {:?}",x_prime);
    println!("y': {:?}",y_prime);
    println!("z': {:?}",z_prime);

    let client = Client::new();

    let args: Vec<String> = env::args().collect();
    let port_num = &args[1];
    let mut port = open_port(port_num).expect("Failed to open the port");

    loop {
        let r = min_listen(&mut port, 1000.0);
        match r {
            Ok((id,content)) => {
                let _ = tokio::spawn(dispatch(id,content, client.clone()));
            },
            Err(e) => {
                println!("{:?}",e);
            }
        }
    }
}
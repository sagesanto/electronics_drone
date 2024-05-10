use serde::{Serialize, Deserialize};

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct Orientation {
    pub x: [f64; 3], // x axis, rotated by the orientation of the drone
    pub y: [f64; 3], // y axis, rotated by the orientation of the drone
    pub z: [f64; 3], // z axis, rotated by the orientation of the drone
}

#[derive(Clone, Serialize, Deserialize, Debug)]
pub struct PID {
    pub x0: [f64; 3], // target x value
    pub y0: [f64; 3], // target y value

    pub x: [f64; 3], // x coord (orientation?) of the drone 
    pub y: [f64; 3], // y coord (orientation?) of the drone - driving the FCW motor will induce positive rotation about y, driving BCW motor decreases it
    
    pub fcw: [i32; 3], // commanded value of front clockwise motor
    pub bcw: [i32; 3], // commanded value of back clockwise motor
    pub fccw: [i32; 3], // commanded value of front counterclockwise motor
    pub bccw: [i32; 3], // commanded value of back coutnerclockwise motor
}
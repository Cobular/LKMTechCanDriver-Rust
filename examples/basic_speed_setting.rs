/// Basic program to take speed goals from stdin and send them to the motor.
/// 

use std::io::{self, BufRead};
use lkmtech_motor_driver::MgMotor;

#[tokio::main]
async fn main() -> io::Result<()> {
    let mut motor = MgMotor::new("vcan0", 0x1, 10)?;

    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let speed: i32 = line?.trim().parse().expect("Failed to parse speed");
        motor.send_speed_closed_loop_control(speed).await.unwrap();
    }

    Ok(())
}

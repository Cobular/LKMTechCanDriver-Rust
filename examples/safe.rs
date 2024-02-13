use lkmtech_motor_driver::MgMotor;
/// Basic program to take speed goals from stdin and send them to the motor.
///
use std::io::{self, BufRead};

#[tokio::main]
async fn main() -> io::Result<()> {
    let motor = MgMotor::new("can0", 0x2, 10)?;

    motor.send_torque_closed_loop_control(0.0).unwrap();

    Ok(())
}

use lkmtech_motor_driver::MgMotor;
/// Basic program to take speed goals from stdin and send them to the motor.
///
use std::io::{self, BufRead};

#[tokio::main]
async fn main() -> io::Result<()> {
    let motor = MgMotor::new("can0", 0x2, 10)?;

    motor.register_print_callback().await.unwrap();

    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let speed: i32 = line?.trim().parse().expect("Failed to parse speed");
        motor
            .send_speed_closed_loop_control(speed as f32)
            .await
            .unwrap();
    }

    Ok(())
}

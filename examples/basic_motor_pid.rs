use std::sync::Arc;

use lkmtech_motor_driver::{MgMotor, MotorData};
use pid::Pid;
use tokio::sync::RwLock;

const POS_P_GAIN: f32 = 0.0;
const POS_I_GAIN: f32 = 0.0;
const POS_D_GAIN: f32 = 0.0;

const VEL_P_GAIN: f32 = 0.0;
const VEL_I_GAIN: f32 = 0.0;
const VEL_D_GAIN: f32 = 0.0;

const LOOP_HZ: f32 = 100.0;

#[tokio::main]
async fn main() {
    let mut position_controller: Pid<f32> = Pid::new(0.0, 100.0);
    position_controller.p(POS_P_GAIN, 100.0);
    position_controller.i(POS_I_GAIN, 100.0);
    position_controller.d(POS_D_GAIN, 100.0);

    let mut velocity_controller: Pid<f32> = Pid::new(0.0, 100.0);
    velocity_controller.p(VEL_P_GAIN, 100.0);
    velocity_controller.i(VEL_I_GAIN, 100.0);
    velocity_controller.d(VEL_D_GAIN, 100.0);

    let mut motor = MgMotor::new("vcan0", 0x1, 10).unwrap();

    let motor_data = Arc::new(RwLock::new(MotorData::default()));

    let mut reciever = motor.subscribe().unwrap();
    let motor_data2 = motor_data.clone();
    tokio::spawn(async move {
        let motor_data = motor_data2;
        loop {
            let data = reciever.recv().await.unwrap();
            let mut motor_data = motor_data.write().await;
            motor_data.update(data);
        }
    });

    motor.send_read_motor_state2().await.unwrap();

    let mut target_position = 0.0;

    let reciever = motor.subscribe().unwrap();

    let mut interval = tokio::time::interval(tokio::time::Duration::from_secs_f32(1.0 / LOOP_HZ));

    loop {
        interval.tick().await;

        let motor_data = motor_data.read().await.to_owned();

        let position = motor_data.cur_encoder_pos.unwrap() as f32;
        let velocity = motor_data.cur_speed.unwrap() as f32;

        let position_error = target_position - position;
        let position_output = position_controller.next_control_output(position_error);

        let velocity_error = position_output.output - velocity;
        let velocity_output = velocity_controller.next_control_output(velocity_error);

        println!("{}", velocity_output.output);

        motor.send_torque_closed_loop_control(velocity_output.output as i16).await.unwrap();


        motor.send_read_motor_state2().await.unwrap();
    }
}

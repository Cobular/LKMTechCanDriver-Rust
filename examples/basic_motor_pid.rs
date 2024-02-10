use std::sync::Arc;

use tokio::fs::File;

use tokio::io::{self, AsyncWriteExt, BufWriter, AsyncBufReadExt};
use tokio::sync::RwLock;
use tokio::sync::mpsc::{self, Receiver, Sender};
use tokio_stream::wrappers::LinesStream;

use futures::stream::StreamExt;

use lkmtech_motor_driver::{MgMotor, MotorData};
use pid::Pid;

const POS_P_GAIN: f32 = 0.55;
const POS_I_GAIN: f32 = 0.005;
const POS_D_GAIN: f32 = 0.75;

const VEL_P_GAIN: f32 = 0.08;
const VEL_I_GAIN: f32 = 0.005;
const VEL_D_GAIN: f32 = 0.15;

const LOOP_HZ: f32 = 100.0;

const POST_SCALER: f32 = 2.0;

#[tokio::main]
async fn main() {
    let mut position_controller: Pid<f32> = Pid::new(0.0, 100.0);
    position_controller.p(POS_P_GAIN / POST_SCALER, 100.0);
    position_controller.i(POS_I_GAIN / POST_SCALER, 3.0);
    position_controller.d(POS_D_GAIN / POST_SCALER, 100.0);

    let mut velocity_controller: Pid<f32> = Pid::new(0.0, 100.0);
    velocity_controller.p(VEL_P_GAIN / POST_SCALER, 100.0);
    velocity_controller.i(VEL_I_GAIN / POST_SCALER, 3.0);
    velocity_controller.d(VEL_D_GAIN / POST_SCALER, 100.0);

    let motor = MgMotor::new("can0", 0x2, 10).unwrap();

    let motor_data = Arc::new(RwLock::new(MotorData::new(10)));

    let mut reciever = motor.subscribe().unwrap();
    let motor_data2 = motor_data.clone();
    tokio::spawn(async move {
        let motor_data = motor_data2;
        loop {
            let data = reciever.recv().await.unwrap();
            let mut motor_data = motor_data.write().await;
            motor_data.update(data);
            // println!("{:?}, {:?}", motor_data.angle_deg(), motor_data.speed_dps());
        }
    });

    motor.send_read_motor_state2().await.unwrap();
    motor.refresh().await.unwrap();


    // Open a file in write mode, this will create the file if it does not exist, or truncate it if it does.
    let file = File::create("motor_data.csv").await.unwrap();
    let mut writer = BufWriter::new(file);
    // Write the CSV headers
    writer.write_all(b"Time,Position,Velocity,Position Error,Velocity Setpoint,Velocity Error,Target Torque,Target Pos\n").await.unwrap();


    // Set up mpsc channel for stdin input
    let (tx, mut rx): (Sender<String>, Receiver<String>) = mpsc::channel(100);

    // Spawn a task to read from stdin and send to the channel
    tokio::spawn(async move {
        let lines = io::BufReader::new(io::stdin()).lines();
        let mut lines_stream = LinesStream::new(lines);
        while let Some(line) = lines_stream.next().await {
            if let Ok(line) = line {
                if let Err(e) = tx.send(line).await {
                    eprintln!("Failed to send line: {}", e);
                    break;
                }
            }
        }
    });

    // Sleep for 10 seconds to let the motor data update
    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;

    let mut target_position = 270.0;

    let _reciever = motor.subscribe().unwrap();

    let mut interval = tokio::time::interval(tokio::time::Duration::from_secs_f32(1.0 / LOOP_HZ));

    let start_time = tokio::time::Instant::now();

    let mut iters = 0;

    loop {
        interval.tick().await;

        // Non-blocking check for newline input from stdin
        match rx.try_recv() {
            Ok(line) => {
                if line.trim() == "n" && iters < 20 {
                    iters += 1;
                    // Set target position to a random value within a range of 90 deg
                    target_position += (2.0 * (0.5 - rand::random::<f32>()) * 60.0) + 45.0;
                    println!("Setting target position to: {}", target_position);
                } else {
                    println!("Safing the motor");
                    motor.send_torque_closed_loop_control(0.0).await.unwrap();
                    break; // Exit the loop if "exit" command is received
                }                
            },
            Err(e) => {
                if e != mpsc::error::TryRecvError::Empty {
                    eprintln!("Channel receive error: {}", e);
                    break;
                }
                // No input available, continue with the loop
            }
        }

        let elapsed = start_time.elapsed().as_secs_f32();

        let motor_data = motor_data.read().await.to_owned();

        let position = motor_data.angle_deg().unwrap() as f32;
        let velocity = motor_data.speed_dps().unwrap() as f32;

        // println!("Position: {}, Velocity: {}", position, velocity);

        let position_error = target_position - position;

        // println!("Position error: {}", position_error);

        let velocity_setpoint = position_controller.next_control_output(position_error);

        // println!("Velocity Setpoint: {}", velocity_setpoint.output);

        let velocity_error = (-1.0 * velocity_setpoint.output) - velocity;
        let torque_setpoint = velocity_controller.next_control_output(velocity_error);

        // println!("Target torque: {}", torque_setpoint.output);

        // Write the data to the CSV file
        let data = format!("{},{},{},{},{},{},{},{}\n", elapsed, position, velocity, position_error, velocity_setpoint.output, velocity_error, torque_setpoint.output, target_position);
        writer.write_all(data.as_bytes()).await.unwrap();

        // It was going the wrong way :C
        motor.send_torque_closed_loop_control(-1.0 * torque_setpoint.output).await.unwrap();

        motor.refresh().await.unwrap();
    }
}

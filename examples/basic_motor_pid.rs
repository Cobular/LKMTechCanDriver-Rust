use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use std::fs::File;
use std::io::{self, BufRead, BufWriter, Write};
use std::sync::mpsc::TryRecvError;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::RwLock;
use thread_priority::*;

use lkmtech_motor_driver::{MgMotor, MotorData};
use pid::Pid;

const POS_P_GAIN: f32 = 0.85;
const POS_I_GAIN: f32 = 0.0;
const POS_D_GAIN: f32 = 0.75;

const VEL_P_GAIN: f32 = 1.1;
const VEL_I_GAIN: f32 = 0.008;
const VEL_D_GAIN: f32 = 0.15;

const LOOP_HZ: f32 = 500.0;

const POST_SCALER: f32 = 2.0;

fn main() {
    assert!(set_current_thread_priority(ThreadPriority::Max).is_ok());

    println!("Basic Motor PID Starting with params: Ppos: {}, Ipos: {}, Dpos: {}, Pvel: {}, Ivel: {}, Dvel: {}", POS_P_GAIN, POS_I_GAIN, POS_D_GAIN, VEL_P_GAIN, VEL_I_GAIN, VEL_D_GAIN);

    let mut position_controller: Pid<f32> = Pid::new(0.0, 180.0);
    position_controller.p(POS_P_GAIN / POST_SCALER, 180.0);
    position_controller.i(POS_I_GAIN / POST_SCALER, 3.0);
    position_controller.d(POS_D_GAIN / POST_SCALER, 100.0);

    let mut velocity_controller: Pid<f32> = Pid::new(0.0, 32.0);
    velocity_controller.p(VEL_P_GAIN / POST_SCALER, 100.0);
    velocity_controller.i(VEL_I_GAIN / POST_SCALER, 5.0);
    velocity_controller.d(VEL_D_GAIN / POST_SCALER, 100.0);

    let motor = MgMotor::new("can0", 0x2, 10).unwrap();

    let motor_data = Arc::new(RwLock::new(MotorData::new(10)));

    let reciever = motor.subscribe();
    let motor_data2 = motor_data.clone();
    std::thread::spawn(move || {
        let motor_data = motor_data2;
        loop {
            let data = reciever.recv().unwrap();
            let motor_data = motor_data.write();
            motor_data
                .expect("Motor data write to be successful")
                .update(data);
            // println!("{:?}, {:?}", motor_data.angle_deg(), motor_data.speed_dps());
        }
    });

    motor.send_read_motor_state2().unwrap();
    motor.refresh().unwrap();

    // Open a file in write mode, this will create the file if it does not exist, or truncate it if it does.
    let file = File::create("/mnt/ramdisk/motor_data.csv").unwrap();
    let mut writer = BufWriter::new(file);
    // Write the CSV headers
    writer.write_all(b"Time,Position,Velocity,Position Error,Velocity Setpoint,Velocity Error,Target Torque,Target Pos\n").unwrap();

    // Set up mpsc channel for stdin input
    let (tx, rx): (Sender<String>, Receiver<String>) = mpsc::channel();

    // Spawn a task to read from stdin and send to the channel
    std::thread::spawn(move || {
        let mut lines = io::BufReader::new(io::stdin()).lines();
        while let Some(line) = lines.next() {
            if let Ok(line) = line {
                if let Err(e) = tx.send(line) {
                    eprintln!("Failed to send line: {}", e);
                    break;
                }
            }
        }
    });

    // Sleep for 5 seconds to let the motor data update
    thread::sleep(Duration::from_secs(1));

    let mut target_position = 90.0;

    let _reciever = motor.subscribe();

    let start_time = Instant::now();
    let mut iters = 0;
    let mut last_tick = start_time;

    // Track rolling average velocity
    let mut avg_velocity = 0.0;

    loop {
        let next_tick = last_tick + Duration::from_secs_f32(1.0 / LOOP_HZ);
        let now = Instant::now();
        if now < next_tick {
            // Sleep until the next tick
            thread::sleep(next_tick - now);
        }
        last_tick = next_tick;

        // Non-blocking check for newline input from stdin
        match rx.try_recv() {
            Ok(line) => {
                if line.trim() == "n" && iters < 20 {
                    iters += 1;
                    // Set target position to a random value within a range of 90 deg
                    target_position += 2.0 * (0.5 - rand::random::<f32>()) * 90.0;
                    println!("Setting target position to: {}", target_position);
                } else {
                    println!("Safing the motor");
                    motor.send_torque_closed_loop_control(0.0).unwrap();
                    break; // Exit the loop if "exit" command is received
                }
            }
            Err(e) => {
                if e != TryRecvError::Empty {
                    eprintln!("Channel receive error: {}", e);
                    break;
                }
                // No input available, continue with the loop
            }
        }

        let elapsed = start_time.elapsed().as_secs_f32();

        let motor_data = motor_data.read().unwrap().to_owned();

        let position = motor_data.angle_deg().unwrap() as f32;
        let velocity = motor_data.speed_dps().unwrap() as f32;

        // avg_velocity = avg_velocity * 0.9 + velocity * 0.1;

        // Safe the robot if the velocity is too high
        if velocity > 100.0 {
            println!("Safing the motor");
            motor.send_torque_closed_loop_control(0.0).unwrap();
            break;
        }

        // println!("Position: {}, Velocity: {}", position, velocity);

        let position_error = target_position - position;

        // println!("Position error: {}", position_error);

        let velocity_setpoint = position_controller.next_control_output(position_error);

        // println!("Velocity Setpoint: {}", velocity_setpoint.output);

        let velocity_error = (-1.0 * velocity_setpoint.output) - velocity;
        let torque_setpoint = velocity_controller.next_control_output(velocity_error);

        // println!("{}", elapsed);

        // Write the data to the CSV file
        let data = format!(
            "{:.32},{:.32},{:.32},{:.32},{:.32},{:.32},{:.32},{:.32}\n",
            elapsed,
            position,
            velocity,
            position_error,
            velocity_setpoint.output,
            velocity_error,
            torque_setpoint.output,
            target_position
        );
        writer.write_all(data.as_bytes()).unwrap();

        // It was going the wrong way :C
        motor
            .send_torque_closed_loop_control(-1.0 * torque_setpoint.output)
            .unwrap();

        motor.refresh().unwrap();
    }
}

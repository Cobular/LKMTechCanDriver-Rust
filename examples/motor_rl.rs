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

use ndarray::{Array2, Array1};
use ort::{GraphOptimizationLevel, Session};

const TORQUE_CURRENT_FACTOR: f32 = 22.0;

const CTRL_LOOP_HZ: f32 = 25.0;
const LOOP_HZ: f32 = 500.0;

const LOOP_SKIP: u32 = (LOOP_HZ / CTRL_LOOP_HZ) as u32;

fn wait_for_next_loop(last_tick: Instant, loop_hz: f32) {
    let next_tick = last_tick + Duration::from_secs_f32(1.0 / loop_hz);
    let now = Instant::now();
    if now < next_tick {
        // Sleep until the next tick
        thread::sleep(next_tick - now);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    assert!(set_current_thread_priority(ThreadPriority::Max).is_ok());

    let motor = MgMotor::new("can0", 0x2, 10).unwrap();

    // Model is a 3, 64, 64, 1 neural network
    let model = Session::builder()?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(4)?
        .with_model_from_file("model.onnx")?;

    let state_ins = Array1::<f32>::from_shape_vec(1, vec![0.0; 1])?;

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

    // Sleep for 1 seconds to let the motor data update
    thread::sleep(Duration::from_secs(1));

    let mut target_position = 180.0;

    let start_time = Instant::now();
    let mut iters = 0;
    let mut last_tick = start_time;

    let mut loop_counter = 0;

    loop {
        // Non-blocking, check if there's any input from the channel
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

        // Now, wait the remainder of the loop time
        wait_for_next_loop(last_tick, LOOP_HZ);
        last_tick = Instant::now();

        // Need to decide if a loop is skipped or not
        loop_counter += 1;
        if loop_counter % LOOP_SKIP != 0 {
            continue;
        }

        let elapsed = start_time.elapsed().as_secs_f32();

        let motor_data = motor_data.read().unwrap().to_owned();

        let position_rad = motor_data.angle_deg().unwrap() * std::f32::consts::PI / 180.0;
        let velocity_rad_p_s = motor_data.speed_dps().unwrap() * std::f32::consts::PI / 180.0;
        let torque = motor_data.cur_torque_current.unwrap() / 33.0;

        let input = Array2::<f32>::from_shape_vec((1, 3), vec![velocity_rad_p_s, position_rad, torque])?;

        let outputs = model.run(ort::inputs!["obs" => input.clone(), "state_ins" => state_ins.clone()]?)?;

        // Get the action from the tensor
        let action = outputs["output"].extract_tensor::<f32>()?.view().t().to_owned();
        let action_slice = action.as_slice().unwrap();

        println!("Action: {:?}", action_slice[0]);

        // Write the data to the CSV file
        let data = format!(
            "{:.32},{:.32},{:.32},{:.32},{:?}\n",
            elapsed,
            position_rad,
            velocity_rad_p_s,
            input,
            action_slice,
        );
        writer.write_all(data.as_bytes()).unwrap();

        // It was going the wrong way :C
        // motor
        //     .send_torque_closed_loop_control(action_slice[0] * TORQUE_CURRENT_FACTOR)
        //     .unwrap();

        motor.refresh().unwrap();
    }

    Ok(())
}

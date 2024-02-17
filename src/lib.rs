use std::io;

use socketcan::{
    CanFilter, CanFrame, CanSocket, EmbeddedFrame, Id, Socket, SocketOptions, StandardId,
};

use crate::error::{Error, Result};

use crate::commands::{Commands, DataArray, Direction};

use crossbeam_channel::{self, Receiver, Sender};

pub mod commands;
pub mod error;
pub mod messages;

#[derive(Debug, Clone, Copy)]
pub struct MotorData {
    gear_ratio: u8, // Gear ratio of the gearbox. For a 1:10 gearbox, this would be 10.

    pub cur_temp: Option<u8>,
    pub cur_torque_current: Option<f32>, // Amps, motor actual torque current range is-33A~33A.
    pub cur_encoder_pos: Option<u64>,
    pub voltage: Option<i16>,
    pub a_phase_current: Option<i16>,
    pub b_phase_current: Option<i16>,
    pub c_phase_current: Option<i16>,

    /// Degrees per second, pre gearbox
    cur_speed: Option<f32>,
    /// 0.01°/LSB, pre gearbox. Only updated from the `Read`
    cur_angle: Option<i64>,
}

impl MotorData {
    pub fn new(gear_ratio: u8) -> Self {
        Self {
            gear_ratio,
            cur_angle: None,
            cur_temp: None,
            cur_torque_current: None,
            cur_speed: None,
            cur_encoder_pos: None,
            voltage: None,
            a_phase_current: None,
            b_phase_current: None,
            c_phase_current: None,
        }
    }

    pub fn new_initalized(gear_ratio: u8) -> Self {
        Self {
            gear_ratio,
            cur_angle: Some(0),
            cur_temp: Some(0),
            cur_torque_current: Some(0.0),
            cur_speed: Some(0.0),
            cur_encoder_pos: Some(0),
            voltage: Some(0),
            a_phase_current: Some(0),
            b_phase_current: Some(0),
            c_phase_current: Some(0),
        }
    }

    /// Merge in new data
    pub fn update(&mut self, data: MotorData) {
        self.cur_angle = data.cur_angle.or(self.cur_angle);
        self.cur_temp = data.cur_temp.or(self.cur_temp);
        self.cur_torque_current = data.cur_torque_current.or(self.cur_torque_current);
        self.cur_speed = data.cur_speed.or(self.cur_speed);
        self.cur_encoder_pos = data.cur_encoder_pos.or(self.cur_encoder_pos);
        self.voltage = data.voltage.or(self.voltage);
        self.a_phase_current = data.a_phase_current.or(self.a_phase_current);
        self.b_phase_current = data.b_phase_current.or(self.b_phase_current);
        self.c_phase_current = data.c_phase_current.or(self.c_phase_current);
    }

    /// Get speed in degrees per second.
    pub fn speed_dps(&self) -> Option<f32> {
        self.cur_speed.map(|speed| speed / self.gear_ratio as f32)
    }

    /// Get position in degrees. Handles wrapping gracefully.
    /// ONLY updated from the `ReadMultiAngleLoop` command.
    pub fn angle_deg(&self) -> Option<f32> {
        self.cur_angle
            .map(|angle| angle as f32 / 100.0 / self.gear_ratio as f32)
    }
}

#[derive(Debug)]
pub struct MgMotor {
    socket: CanSocket,
    id: Id,

    _reciever_thread_handle: std::thread::JoinHandle<()>,

    broadcast_sender_handle: Sender<MotorData>,
    broadcast_receiver_handle: Receiver<MotorData>,

    gear_ratio: u8,
}

impl MgMotor {
    pub fn new(socket_name: &str, id: u8, gear_ratio: u8) -> io::Result<Self> {
        let socket_name = socket_name.to_string();

        let socket = CanSocket::open(&socket_name)?;

        socket.set_filters(&[CanFilter::new(0x140 + id as u32, u32::MAX)])?;

        let id = Id::Standard(
            StandardId::new(0x140 + id as u16)
                .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "Invalid ID"))?,
        );

        let socket_rx = CanSocket::open(&socket_name)?;

        let (tx, rx) = crossbeam_channel::bounded::<MotorData>(64);

        let tx2 = tx.clone();
        let reciever_thread_handle = std::thread::spawn(move || {
            let socket_rx = socket_rx;
            let tx = tx2;

            while let frame = socket_rx.read_frame() {
                if let Ok(frame) = frame {
                    if frame.id() == id {
                        let data: &[u8] = frame.data();

                        let data: &[u8; 8] = data.try_into().unwrap();

                        match messages::Response::try_from(data) {
                            Ok(message) => {
                                let mut motor_data = MotorData::new(gear_ratio);

                                if let Some(temp) = message.temp() {
                                    if temp == 0 {
                                        // Print the message type
                                        // println!("Zero message type: {:?}", message);
                                        continue;
                                    }
                                    motor_data.cur_temp = Some(temp);
                                }

                                match message {
                                    messages::Response::TorqueClosedLoop(closedloop_message) => {
                                        // If temp is zero, don't do anything
                                        if closedloop_message.motor_temp == 0 {
                                            continue;
                                        }

                                        // Motor torque current value iq,int16_t, range is -2048~2048, motor actual torque current range is-33A~33A.
                                        motor_data.cur_torque_current = Some(
                                            closedloop_message.torque_current_iq as f32 / 2048.0
                                                * 33.0,
                                        );

                                        // println!(
                                        //     "Raw torque: {:?}, Torque current: {:?}",
                                        //     closedloop_message.torque_current_iq,
                                        //     motor_data.cur_torque_current
                                        // );


                                        // Motor speed value, f32, degrees per second pre gearbox.
                                        motor_data.cur_speed = Some(
                                            closedloop_message.speed as f32 / gear_ratio as f32,
                                        );

                                        // Motor encoder position, u64, range is 0~65535(keeping high 16bit, Omit the lower 2 bit).
                                        motor_data.cur_encoder_pos =
                                            Some(closedloop_message.encoder_position as u64);
                                    }
                                    messages::Response::ReadEncoder(read_encoder_message) => {
                                        motor_data.cur_encoder_pos =
                                            Some(read_encoder_message.encoder as u64);
                                    }
                                    messages::Response::ReadState1(
                                        read_motor_state1_and_error_state_message,
                                    ) => {
                                        motor_data.voltage = Some(
                                            read_motor_state1_and_error_state_message.voltage
                                                as i16,
                                        );
                                    }
                                    messages::Response::ReadState3(
                                        read_motor_state1_and_error_state_message,
                                    ) => {
                                        motor_data.a_phase_current = Some(
                                            read_motor_state1_and_error_state_message
                                                .a_phase_current,
                                        );
                                        motor_data.b_phase_current = Some(
                                            read_motor_state1_and_error_state_message
                                                .b_phase_current,
                                        );
                                        motor_data.c_phase_current = Some(
                                            read_motor_state1_and_error_state_message
                                                .c_phase_current,
                                        );
                                    }
                                    messages::Response::MultiAngle(multi_angle_message) => {
                                        motor_data.cur_angle = Some(multi_angle_message.angle);
                                    }
                                    _ => (),
                                }

                                match tx.send(motor_data) {
                                    Ok(_) => (),
                                    Err(e) => println!("Error sending message: {:?}", e),
                                }
                            }
                            Err(Error::MessageBodyAllZero) => {}
                            Err(_) => {
                                println!("Invalid message: {:?}", data);
                            }
                        }
                    }
                } else {
                    println!("Error reading frame");
                }
            }
        });

        socket.set_write_timeout(std::time::Duration::from_millis(100))?;

        Ok(Self {
            socket,
            id,
            gear_ratio,

            _reciever_thread_handle: reciever_thread_handle,
            broadcast_sender_handle: tx,
            broadcast_receiver_handle: rx,
        })
    }

    /// Subscribe to the broadcast channel. Will receive all updated MotorData packets
    pub fn subscribe(&self) -> Receiver<MotorData> {
        self.broadcast_receiver_handle.clone()
    }

    /// Register a basic subscriber to track motor data to stdout
    pub async fn register_print_callback(&self) -> Result<()> {
        let rx = self.subscribe();

        std::thread::spawn(move || {
            while let Ok(motor_data) = rx.recv() {
                println!("{:?}", motor_data);
            }
        });

        Ok(())
    }

    /// Send any command to the motor. Takes a `Commands` enum, which can represent any possible command.
    pub fn send_arbitrary_command(&self, message: Commands) -> Result<()> {
        let data: DataArray = message.into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Poll for motor data.
    ///
    /// Sends a `ReadState1AndErrorState` command, a `ReadState2` command, and a `ReadMultiAngleLoop` command.
    pub fn refresh(&self) -> Result<()> {
        // Temp, voltage
        self.send_read_motor_state1_and_error_state()?;
        // Temp, torque current, speed, encoder
        self.send_read_motor_state2()?;
        // Total angle
        self.send_read_multi_angle_loop()?;
        Ok(())
    }

    /// Send a raw message by passing a `DataArray` struct.
    ///
    /// This is a low level function, and should not be used unless you know what you are doing.
    /// Exists to simplify ID and CanFrame creation.
    fn send_message(&self, message: &DataArray) -> Result<()> {
        // println!("Sending message: {:?}", message);
        let frame = CanFrame::new(self.id, message)
            .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "Invalid frame"))
            .map_err(Error::StdioError)?;
        self.socket.write_frame(&frame).map_err(Error::StdioError)?;
        Ok(())
    }

    /// Shutdown the motor. Will respond to commands but will not move until turned back on.
    pub fn send_motor_off(&self) -> Result<()> {
        let data: DataArray = crate::commands::MotorOffCommand::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Turn the motor on. Will respond to commands and move.
    pub fn send_motor_on(&self) -> Result<()> {
        let data: DataArray = crate::commands::MotorOnCommand::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Stop motion, but will keep moving
    pub fn send_motor_stop(&self) -> Result<()> {
        let data: DataArray = crate::commands::MotorStopCommand::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Send an open loop command to move with some given power.
    pub fn send_open_loop_control(&self, power: i16) -> Result<()> {
        let data: DataArray = crate::commands::OpenLoopControl::new(power)?.into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Send a closed loop torque command. Motor will produce a consistent torque.
    ///
    /// WARNING: DANGEROUS! This can easily cause runaway speeds if not controlled carefully.
    /// Be ready with an estop and maybe set a software estop based on motor speed!
    ///
    /// Input is amps, where 1.0 corresponds to 33A.
    ///
    pub fn send_torque_closed_loop_control(&self, iq: f32) -> Result<()> {
        // Motor torque current value iq,int16_t, range is -2048~2048, motor actual torque current range is-33A~33A.
        let iq = (iq / 33.0 * 2048.0) as i16;

        let data: DataArray = crate::commands::TorqueClosedLoopControlCommand::new(iq)?.into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Send a closed loop speed command. Motor will attempt to maintain a constant speed.
    ///
    /// Speedcontrol is f32, where 1.0 corresponds to 1dps.
    ///
    /// WARNING: Can easily cause injury! It will output arbitrary torque to attempt to maintain the target speed.
    pub fn send_speed_closed_loop_control(&self, speed: f32) -> Result<()> {
        // SpeedControl value is int32_t, corresponding actual speed is 0.01dps/LSB.
        // So 1.0 corresponds to 100dps. Also need to account for the gearbox.
        let speed = (speed * 100.0 * self.gear_ratio as f32) as i32;
        let data: DataArray = crate::commands::SpeedClosedLoopControlCommand::new(speed)?.into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Send a closed loop command to move to a specific angle.
    ///
    /// "Multi" means you can request angles beyond 360 degrees, and the motor will correctly wrap around.
    ///
    /// Units are in terms floating point degrees.
    pub fn send_multi_angle_control(&self, angle: f32) -> Result<()> {
        // The actual command wants units of 0.01 degree/LSB, so 360 degrees is 36000.
        // We also need to account for the gearbox - the motor will move 1/10th of the angle we tell it to.
        let angle = (angle * 100.0 * self.gear_ratio as f32) as i32;

        let data: DataArray = crate::commands::MultiAngleControl::new(angle).into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Send a closed loop command to move to a specific angle, with a speed limit.
    ///
    /// "Multi" means you can request angles beyond 360 degrees, and the motor will correctly wrap around.
    ///
    /// Units are in terms floating point degrees.
    /// MaxSpeed is uint16_t, corresponding actual speed is 1dps/LSB, i.e 360 corresponding to 360dps.
    pub fn send_mutli_angle_control_speed_limit(&self, angle: f32, max_speed: u16) -> Result<()> {
        // The actual command wants units of 0.01 degree/LSB, so 360 degrees is 36000.
        // We also need to account for the gearbox - the motor will move 1/10th of the angle we tell it to.
        let angle = (angle * 100.0 * self.gear_ratio as f32) as i32;

        // The speed limit is passed in as-is
        let data: DataArray =
            crate::commands::MultiAngleControlSpeedLimit::new(angle, max_speed).into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Send a closed loop command to move to a specific angle.
    ///
    /// "Single" means no wrapping around - it's all relative to a single rotation.
    ///
    /// Units are in terms floating point degrees.
    pub fn send_single_angle_control(&self, direction: Direction, angle: f32) -> Result<()> {
        // The actual command wants units of 0.01 degree/LSB, so 360 degrees is 36000.
        // We also need to account for the gearbox - the motor will move 1/10th of the angle we tell it to.
        let angle = (angle * 100.0 * self.gear_ratio as f32) as i32;

        let data: DataArray = crate::commands::SingleAngleControl::new(direction, angle).into();
        self.send_message(&data)?;
        Ok(())
    }

    /// Send a closed loop command to move to a specific angle with a speed limit.
    ///
    /// "Single" means no wrapping around - it's all relative to a single rotation.
    ///
    /// Units are in terms floating point degrees.
    /// MaxSpeed is uint16_t, corresponding actual speed is 1dps/LSB, i.e 360 corresponding to 360dps.
    pub fn send_single_angle_control_with_speed_limit(
        &self,
        direction: Direction,
        angle: f32,
        max_speed: u16,
    ) -> Result<()> {
        // The actual command wants units of 0.01 degree/LSB, so 360 degrees is 36000.
        // We also need to account for the gearbox - the motor will move 1/10th of the angle we tell it to.
        let angle = (angle * 100.0 * self.gear_ratio as f32) as i32;

        let data: DataArray =
            crate::commands::SingleAngleControlSpeedLimit::new(direction, angle, max_speed).into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_increment_angle_control(&self, angle_increment: i32) -> Result<()> {
        let data: DataArray = crate::commands::IncrementAngleControl1::new(angle_increment).into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_increment_angle_control_speedlimit(
        &self,
        angle_increment: i32,
        max_speed: u16,
    ) -> Result<()> {
        let data: DataArray =
            crate::commands::IncrementAngleControl2::new(angle_increment, max_speed).into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_pid_parameter(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadPIDParameter::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_write_pid_params_to_ram(
        &self,
        params: crate::commands::WritePIDParamsToRAM,
    ) -> Result<()> {
        let data: DataArray = params.into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_write_pid_params_to_rom(
        &self,
        params: crate::commands::WritePIDParamsToROM,
    ) -> Result<()> {
        let data: DataArray = params.into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_acceleration(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadAcceleration::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_write_acceleration_to_ram(&self, accel: i32) -> Result<()> {
        let data: DataArray = crate::commands::WriteAccelerationToRAM::new(accel).into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_encoder(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadEncoder::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_write_encoder_val_to_rom(&self, offset: u16) -> Result<()> {
        let data: DataArray = crate::commands::WriteEncoderValToROM::new(offset).into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_write_current_position_to_rom(&self) -> Result<()> {
        let data: DataArray = crate::commands::WriteCurrentPositionToROM::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_multi_angle_loop(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMultiAngleLoop::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_single_angle_loop(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadSingleAngleLoop::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_clear_motor_angle_loop(&self) -> Result<()> {
        let data: DataArray = crate::commands::ClearMotorAngleLoop::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_motor_state1_and_error_state(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMotorState1AndErrorState::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_clear_motor_error_state(&self) -> Result<()> {
        let data: DataArray = crate::commands::ClearMotorErrorState::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_motor_state2(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMotorState2::default().into();
        self.send_message(&data)?;
        Ok(())
    }

    pub fn send_read_motor_state3(&self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMotorState3::default().into();
        self.send_message(&data)?;
        Ok(())
    }
}

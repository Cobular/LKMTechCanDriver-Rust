use std::io;

use futures_util::StreamExt;
use socketcan::{
    tokio::CanSocket, CanFilter, CanFrame, EmbeddedFrame, Id, SocketOptions, StandardId,
};

use crate::error::{Error, Result};

use crate::commands::{Commands, DataArray};

pub mod commands;
pub mod error;
pub mod messages;

#[derive(Default, Debug, Clone, Copy)]
pub struct MotorData {
    pub cur_angle: Option<i64>,
    pub cur_temp: Option<u8>,
    pub cur_torque_current: Option<f32>,
    pub cur_speed: Option<f32>,
    pub cur_encoder_pos: Option<u64>,
    pub voltage: Option<i16>,
    pub a_phase_current: Option<i16>,
    pub b_phase_current: Option<i16>,
    pub c_phase_current: Option<i16>,
}

impl MotorData {
    /// Merge in new data
    pub fn update(&mut self, data: MotorData) {
        if let Some(cur_angle) = data.cur_angle {
            self.cur_angle = Some(cur_angle);
        }
        if let Some(cur_temp) = data.cur_temp {
            self.cur_temp = Some(cur_temp);
        }
        if let Some(cur_torque_current) = data.cur_torque_current {
            self.cur_torque_current = Some(cur_torque_current);
        }
        if let Some(cur_speed) = data.cur_speed {
            self.cur_speed = Some(cur_speed);
        }
        if let Some(cur_encoder_pos) = data.cur_encoder_pos {
            self.cur_encoder_pos = Some(cur_encoder_pos);
        }
        if let Some(voltage) = data.voltage {
            self.voltage = Some(voltage);
        }
        if let Some(a_phase_current) = data.a_phase_current {
            self.a_phase_current = Some(a_phase_current);
        }
        if let Some(b_phase_current) = data.b_phase_current {
            self.b_phase_current = Some(b_phase_current);
        }
        if let Some(c_phase_current) = data.c_phase_current {
            self.c_phase_current = Some(c_phase_current);
        }
    }
}

pub struct MgMotor {
    socket: CanSocket,
    id: Id,

    _reciever_thread_handle: tokio::task::JoinHandle<()>,

    broadcast_sender_handle: tokio::sync::broadcast::Sender<MotorData>,

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

        let (tx, _) = tokio::sync::broadcast::channel::<MotorData>(64);

        let tx2 = tx.clone();
        let reciever_thread_handle = tokio::spawn(async move {
            let mut socket_rx = socket_rx;
            let tx = tx2;

            while let Some(frame) = socket_rx.next().await {
                if let Ok(frame) = frame {
                    if frame.id() == id {
                        let data: &[u8] = frame.data();

                        let data: &[u8; 8] = data.try_into().unwrap();

                        if let Ok(message) = messages::Response::try_from(data) {
                            let mut motor_data = MotorData::default();

                            if let Some(temp) = message.temp() {
                                motor_data.cur_temp = Some(temp);
                            }

                            match message {
                                messages::Response::TorqueClosedLoop(closedloop_message) => {
                                    // Motor torque current value iq,int16_t, range is -2048~2048, motor actual torque current range is-33A~33A.
                                    motor_data.cur_torque_current = Some(
                                        closedloop_message.torque_current_iq as f32 / 2048.0 * 33.0,
                                    );

                                    // Motor speed value, f32, degrees per second pre gearbox.
                                    motor_data.cur_speed =
                                        Some(closedloop_message.speed as f32 / gear_ratio as f32);

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
                                        read_motor_state1_and_error_state_message.voltage as i16,
                                    );
                                }
                                messages::Response::ReadState3(
                                    read_motor_state1_and_error_state_message,
                                ) => {
                                    motor_data.a_phase_current = Some(
                                        read_motor_state1_and_error_state_message.a_phase_current,
                                    );
                                    motor_data.b_phase_current = Some(
                                        read_motor_state1_and_error_state_message.b_phase_current,
                                    );
                                    motor_data.c_phase_current = Some(
                                        read_motor_state1_and_error_state_message.c_phase_current,
                                    );
                                }
                                _ => (),
                            }

                            if tx.send(motor_data).is_err() {
                                println!("Error sending message")
                            }
                        } else {
                            println!("Invalid message: {:?}", data);
                        }
                    }
                } else {
                    println!("Error reading frame");
                }
            }
        });

        Ok(Self {
            socket,
            id,
            gear_ratio,

            _reciever_thread_handle: reciever_thread_handle,
            broadcast_sender_handle: tx,
        })
    }

    pub fn subscribe(&self) -> Result<tokio::sync::broadcast::Receiver<MotorData>> {
        Ok(self.broadcast_sender_handle.subscribe())
    }

    pub async fn register_print_callback(&self) -> Result<()> {
        let mut rx = self.subscribe()?;

        tokio::spawn(async move {
            while let Ok(motor_data) = rx.recv().await {
                println!("{:?}", motor_data);
            }
        });

        Ok(())
    }

    pub async fn send_arbitrary_command(&mut self, message: Commands) -> Result<()> {
        let data: DataArray = message.into();
        self.send_message(&data).await?;
        Ok(())
    }

    async fn send_message(&mut self, message: &[u8]) -> Result<()> {
        let frame = CanFrame::new(self.id, message)
            .ok_or_else(|| io::Error::new(io::ErrorKind::Other, "Invalid frame"))
            .map_err(Error::StdioError)?;
        self.socket
            .write_frame(frame)
            .map_err(Error::SocketcanError)?
            .await
            .map_err(Error::StdioError)?;
        Ok(())
    }

    pub async fn send_motor_off(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::MotorOffCommand::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_motor_on(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::MotorOnCommand::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_motor_stop(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::MotorStopCommand::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_open_loop_control(&mut self, power: i16) -> Result<()> {
        let data: DataArray = crate::commands::OpenLoopControl::new(power)?.into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_torque_closed_loop_control(&mut self, iq: i16) -> Result<()> {
        let data: DataArray = crate::commands::TorqueClosedLoopControlCommand::new(iq)?.into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_speed_closed_loop_control(&mut self, speed: i32) -> Result<()> {
        let data: DataArray = crate::commands::SpeedClosedLoopControlCommand::new(speed)?.into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_absolute_angle_control(&mut self, angle: i32) -> Result<()> {
        let data: DataArray = crate::commands::AbsoluteAngleControl::new(angle).into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_absolute_angle_control_with_speed_limit(
        &mut self,
        angle: i32,
        max_speed: u16,
    ) -> Result<()> {
        let data: DataArray =
            crate::commands::AbsoluteAngleControlSpeedLimit::new(angle, max_speed).into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_relative_angle_control(&mut self, direction: u8, angle: i32) -> Result<()> {
        let data: DataArray = crate::commands::RelativeAngleControl::new(direction, angle).into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_relative_angle_control_with_speed_limit(
        &mut self,
        direction: u8,
        angle: i32,
        max_speed: u16,
    ) -> Result<()> {
        let data: DataArray =
            crate::commands::RelativeAngleControlSpeedLimit::new(direction, angle, max_speed)
                .into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_increment_angle_control(&mut self, angle_increment: i32) -> Result<()> {
        let data: DataArray = crate::commands::IncrementAngleControl1::new(angle_increment).into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_increment_angle_control_speedlimit(
        &mut self,
        angle_increment: i32,
        max_speed: u16,
    ) -> Result<()> {
        let data: DataArray =
            crate::commands::IncrementAngleControl2::new(angle_increment, max_speed).into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_pid_parameter(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadPIDParameter::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_write_pid_params_to_ram(
        &mut self,
        params: crate::commands::WritePIDParamsToRAM,
    ) -> Result<()> {
        let data: DataArray = params.into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_write_pid_params_to_rom(
        &mut self,
        params: crate::commands::WritePIDParamsToROM,
    ) -> Result<()> {
        let data: DataArray = params.into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_acceleration(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadAcceleration::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_write_acceleration_to_ram(&mut self, accel: i32) -> Result<()> {
        let data: DataArray = crate::commands::WriteAccelerationToRAM::new(accel).into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_encoder(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadEncoder::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_write_encoder_val_to_rom(&mut self, offset: u16) -> Result<()> {
        let data: DataArray = crate::commands::WriteEncoderValToROM::new(offset).into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_write_current_position_to_rom(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::WriteCurrentPositionToROM::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_multi_angle_loop(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMultiAngleLoop::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_single_angle_loop(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadSingleAngleLoop::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_clear_motor_angle_loop(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ClearMotorAngleLoop::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_motor_state1_and_error_state(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMotorState1AndErrorState::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_clear_motor_error_state(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ClearMotorErrorState::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_motor_state2(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMotorState2::default().into();
        self.send_message(&data).await?;
        Ok(())
    }

    pub async fn send_read_motor_state3(&mut self) -> Result<()> {
        let data: DataArray = crate::commands::ReadMotorState3::default().into();
        self.send_message(&data).await?;
        Ok(())
    }
}

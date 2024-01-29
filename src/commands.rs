use crate::error::{Error, Result};
use serde::{Deserialize, Serialize};

pub type DataArray = [u8; 8];

#[derive(Serialize, Deserialize, Debug, PartialEq, Eq, Clone, Copy)]
pub enum Commands {
    MotorOff(MotorOffCommand),
    MotorOn(MotorOnCommand),
    MotorStop(MotorStopCommand),
    OpenLoopControl(OpenLoopControl),
    TorqueClosedLoopControl(TorqueClosedLoopControlCommand),
    SpeedClosedLoopControl(SpeedClosedLoopControlCommand),
    MultiAngleControl(MultiAngleControl),
    MultiAngleControlSpeedLimit(MultiAngleControlSpeedLimit),
    SingleAngleControl(SingleAngleControl),
    SingleAngleControlSpeedLimit(SingleAngleControlSpeedLimit),

    // Get data out
    ReadSingleAngleLoop(ReadSingleAngleLoop),
    ClearMotorAngleLoop(ClearMotorAngleLoop),
    ReadMotorState1AndErrorState(ReadMotorState1AndErrorState),
    ClearMotorErrorState(ClearMotorErrorState),
    ReadMotorState2(ReadMotorState2),
    ReadMotorState3(ReadMotorState3),
}

impl From<Commands> for DataArray {
    fn from(val: Commands) -> Self {
        match val {
            Commands::MotorOff(val) => val.into(),
            Commands::MotorOn(val) => val.into(),
            Commands::MotorStop(val) => val.into(),
            Commands::OpenLoopControl(val) => val.into(),
            Commands::TorqueClosedLoopControl(val) => val.into(),
            Commands::SpeedClosedLoopControl(val) => val.into(),
            Commands::MultiAngleControl(val) => val.into(),
            Commands::MultiAngleControlSpeedLimit(val) => val.into(),
            Commands::SingleAngleControl(val) => val.into(),
            Commands::SingleAngleControlSpeedLimit(val) => val.into(),

            Commands::ReadSingleAngleLoop(val) => val.into(),
            Commands::ClearMotorAngleLoop(val) => val.into(),
            Commands::ReadMotorState1AndErrorState(val) => val.into(),
            Commands::ClearMotorErrorState(val) => val.into(),
            Commands::ReadMotorState2(val) => val.into(),
            Commands::ReadMotorState3(val) => val.into(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct MotorOffCommand {}

impl From<MotorOffCommand> for DataArray {
    fn from(_val: MotorOffCommand) -> Self {
        [0x80, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct MotorOnCommand {}

impl From<MotorOnCommand> for DataArray {
    fn from(_val: MotorOnCommand) -> Self {
        [0x88, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct MotorStopCommand {}

impl From<MotorStopCommand> for DataArray {
    fn from(_val: MotorStopCommand) -> Self {
        [0x81, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct OpenLoopControl {
    power_control: i16,
}

impl OpenLoopControl {
    pub fn new(power_control: i16) -> Result<Self> {
        // Power control should be in range -850 to 850
        if power_control.abs() > 850 {
            return Err(Error::InvalidDataArguments);
        }

        Ok(Self { power_control })
    }
}

impl From<OpenLoopControl> for DataArray {
    fn from(val: OpenLoopControl) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA0; // Command byte for Open Loop Control
        arr[4] = val.power_control as u8; // Low byte
        arr[5] = (val.power_control >> 8) as u8; // High byte
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct TorqueClosedLoopControlCommand {
    iq_control: i16,
}

impl TorqueClosedLoopControlCommand {
    pub fn new(iq_control: i16) -> Result<Self> {
        // iq_control should be in range -2048 to 2048
        if !(-2048..=2048).contains(&iq_control) {
            return Err(Error::InvalidDataArguments);
        }

        Ok(Self { iq_control })
    }
}

impl From<TorqueClosedLoopControlCommand> for DataArray {
    fn from(val: TorqueClosedLoopControlCommand) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA1; // Command byte for Torque Closed Loop Control
        arr[4] = val.iq_control as u8; // Low byte
        arr[5] = (val.iq_control >> 8) as u8; // High byte
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct SpeedClosedLoopControlCommand {
    speed_control: i32,
}

impl SpeedClosedLoopControlCommand {
    pub fn new(speed_control: i32) -> Result<Self> {
        Ok(Self { speed_control })
    }
}

impl From<SpeedClosedLoopControlCommand> for DataArray {
    fn from(val: SpeedClosedLoopControlCommand) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA2; // Command byte for Speed Closed Loop Control
        arr[4] = val.speed_control as u8; // Byte 0
        arr[5] = (val.speed_control >> 8) as u8; // Byte 1
        arr[6] = (val.speed_control >> 16) as u8; // Byte 2
        arr[7] = (val.speed_control >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct MultiAngleControl {
    angle_control: i32,
}

impl MultiAngleControl {
    pub fn new(angle_control: i32) -> Self {
        Self { angle_control }
    }
}

impl From<MultiAngleControl> for DataArray {
    fn from(val: MultiAngleControl) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA3; // Command byte for Absolute Angle Control
        arr[4] = val.angle_control as u8; // Byte 0
        arr[5] = (val.angle_control >> 8) as u8; // Byte 1
        arr[6] = (val.angle_control >> 16) as u8; // Byte 2
        arr[7] = (val.angle_control >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct MultiAngleControlSpeedLimit {
    angle_control: i32,
    max_speed: u16,
}

impl MultiAngleControlSpeedLimit {
    pub fn new(angle_control: i32, max_speed: u16) -> Self {
        Self {
            angle_control,
            max_speed,
        }
    }
}

impl From<MultiAngleControlSpeedLimit> for DataArray {
    fn from(val: MultiAngleControlSpeedLimit) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA4; // Command byte for Absolute Angle Control Speed Limit
        arr[2] = val.max_speed as u8; // Max speed low byte
        arr[3] = (val.max_speed >> 8) as u8; // Max speed high byte
        arr[4] = val.angle_control as u8; // Byte 0
        arr[5] = (val.angle_control >> 8) as u8; // Byte 1
        arr[6] = (val.angle_control >> 16) as u8; // Byte 2
        arr[7] = (val.angle_control >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Eq, Clone, Copy)]
#[repr(u8)]
pub enum Direction {
    Clockwise = 0x00,
    CounterClockwise = 0x01,
}

impl TryFrom<u8> for Direction {
    type Error = Error;

    fn try_from(val: u8) -> Result<Self> {
        match val {
            0x00 => Ok(Direction::Clockwise),
            0x01 => Ok(Direction::CounterClockwise),
            _ => Err(Error::InvalidDataArguments),
        }
    }
}

impl Default for Direction {
    fn default() -> Self {
        Direction::Clockwise
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct SingleAngleControl {
    spin_direction: Direction,
    angle_control: i32,
}

impl SingleAngleControl {
    pub fn new(spin_direction: Direction, angle_control: i32) -> Self {
        Self {
            spin_direction,
            angle_control,
        }
    }
}

impl From<SingleAngleControl> for DataArray {
    fn from(val: SingleAngleControl) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA5; // Command byte for Relative Angle Control
        arr[1] = val.spin_direction as u8; // Spin direction byte
        arr[2] = val.angle_control as u8; // Byte 0
        arr[3] = (val.angle_control >> 8) as u8; // Byte 1
        arr[4] = (val.angle_control >> 16) as u8; // Byte 2
        arr[5] = (val.angle_control >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct SingleAngleControlSpeedLimit {
    spin_direction: Direction,
    angle_control: i32,
    max_speed: u16,
}

impl SingleAngleControlSpeedLimit {
    pub fn new(spin_direction: Direction, angle_control: i32, max_speed: u16) -> Self {
        Self {
            spin_direction,
            angle_control,
            max_speed,
        }
    }
}

impl From<SingleAngleControlSpeedLimit> for DataArray {
    fn from(val: SingleAngleControlSpeedLimit) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA6; // Command byte for Relative Angle Control Speed Limit
        arr[1] = val.spin_direction as u8; // Spin direction byte
        arr[2] = val.max_speed as u8; // Max speed low byte
        arr[3] = (val.max_speed >> 8) as u8; // Max speed high byte
        arr[4] = val.angle_control as u8; // Byte 0
        arr[5] = (val.angle_control >> 8) as u8; // Byte 1
        arr[6] = (val.angle_control >> 16) as u8; // Byte 2
        arr[7] = (val.angle_control >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct IncrementAngleControl1 {
    angle_increment: i32,
}

impl IncrementAngleControl1 {
    pub fn new(angle_increment: i32) -> Self {
        Self { angle_increment }
    }
}

impl From<IncrementAngleControl1> for DataArray {
    fn from(val: IncrementAngleControl1) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA7; // Command byte for Increment Angle Control 1
        arr[4] = val.angle_increment as u8; // Byte 0
        arr[5] = (val.angle_increment >> 8) as u8; // Byte 1
        arr[6] = (val.angle_increment >> 16) as u8; // Byte 2
        arr[7] = (val.angle_increment >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct IncrementAngleControl2 {
    angle_increment: i32,
    max_speed: u16,
}

impl IncrementAngleControl2 {
    pub fn new(angle_increment: i32, max_speed: u16) -> Self {
        Self {
            angle_increment,
            max_speed,
        }
    }
}

impl From<IncrementAngleControl2> for DataArray {
    fn from(val: IncrementAngleControl2) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0xA8; // Command byte for Increment Angle Control 2
        arr[2] = val.max_speed as u8; // Max speed low byte
        arr[3] = (val.max_speed >> 8) as u8; // Max speed high byte
        arr[4] = val.angle_increment as u8; // Byte 0
        arr[5] = (val.angle_increment >> 8) as u8; // Byte 1
        arr[6] = (val.angle_increment >> 16) as u8; // Byte 2
        arr[7] = (val.angle_increment >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadPIDParameter {}

impl ReadPIDParameter {
    pub fn new() -> Self {
        Self {}
    }
}

impl From<ReadPIDParameter> for DataArray {
    fn from(_val: ReadPIDParameter) -> Self {
        [0x30, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct WritePIDParamsToRAM {
    angle_pid_kp: u8,
    angle_pid_ki: u8,
    speed_pid_kp: u8,
    speed_pid_ki: u8,
    iq_pid_kp: u8,
    iq_pid_ki: u8,
}

impl WritePIDParamsToRAM {
    pub fn new(
        angle_pid_kp: u8,
        angle_pid_ki: u8,
        speed_pid_kp: u8,
        speed_pid_ki: u8,
        iq_pid_kp: u8,
        iq_pid_ki: u8,
    ) -> Self {
        Self {
            angle_pid_kp,
            angle_pid_ki,
            speed_pid_kp,
            speed_pid_ki,
            iq_pid_kp,
            iq_pid_ki,
        }
    }
}

impl From<WritePIDParamsToRAM> for DataArray {
    fn from(val: WritePIDParamsToRAM) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0x31; // Command byte for Write PID Params to RAM
        arr[2] = val.angle_pid_kp;
        arr[3] = val.angle_pid_ki;
        arr[4] = val.speed_pid_kp;
        arr[5] = val.speed_pid_ki;
        arr[6] = val.iq_pid_kp;
        arr[7] = val.iq_pid_ki;
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct WritePIDParamsToROM {
    angle_pid_kp: u8,
    angle_pid_ki: u8,
    speed_pid_kp: u8,
    speed_pid_ki: u8,
    iq_pid_kp: u8,
    iq_pid_ki: u8,
}

impl WritePIDParamsToROM {
    pub fn new(
        angle_pid_kp: u8,
        angle_pid_ki: u8,
        speed_pid_kp: u8,
        speed_pid_ki: u8,
        iq_pid_kp: u8,
        iq_pid_ki: u8,
    ) -> Self {
        Self {
            angle_pid_kp,
            angle_pid_ki,
            speed_pid_kp,
            speed_pid_ki,
            iq_pid_kp,
            iq_pid_ki,
        }
    }
}

impl From<WritePIDParamsToROM> for DataArray {
    fn from(val: WritePIDParamsToROM) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0x32; // Command byte for Write PID Params to ROM
        arr[2] = val.angle_pid_kp;
        arr[3] = val.angle_pid_ki;
        arr[4] = val.speed_pid_kp;
        arr[5] = val.speed_pid_ki;
        arr[6] = val.iq_pid_kp;
        arr[7] = val.iq_pid_ki;
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadAcceleration {}

impl From<ReadAcceleration> for DataArray {
    fn from(_val: ReadAcceleration) -> Self {
        [0x33, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct WriteAccelerationToRAM {
    accel: i32,
}

impl WriteAccelerationToRAM {
    pub fn new(accel: i32) -> Self {
        Self { accel }
    }
}

impl From<WriteAccelerationToRAM> for DataArray {
    fn from(val: WriteAccelerationToRAM) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0x34; // Command byte for Write Acceleration to RAM
        arr[4] = val.accel as u8; // Byte 0
        arr[5] = (val.accel >> 8) as u8; // Byte 1
        arr[6] = (val.accel >> 16) as u8; // Byte 2
        arr[7] = (val.accel >> 24) as u8; // Byte 3
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadEncoder {}

impl From<ReadEncoder> for DataArray {
    fn from(_val: ReadEncoder) -> Self {
        [0x90, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct WriteEncoderValToROM {
    encoder_offset: u16,
}

impl WriteEncoderValToROM {
    pub fn new(encoder_offset: u16) -> Self {
        Self { encoder_offset }
    }
}

impl From<WriteEncoderValToROM> for DataArray {
    fn from(val: WriteEncoderValToROM) -> Self {
        let mut arr: DataArray = [0; 8];
        arr[0] = 0x91; // Command byte for Write Encoder Value to ROM
        arr[6] = val.encoder_offset as u8; // Encoder offset low byte
        arr[7] = (val.encoder_offset >> 8) as u8; // Encoder offset high byte
        arr
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct WriteCurrentPositionToROM {}

impl From<WriteCurrentPositionToROM> for DataArray {
    fn from(_val: WriteCurrentPositionToROM) -> Self {
        [0x19, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadMultiAngleLoop {}

impl From<ReadMultiAngleLoop> for DataArray {
    fn from(_val: ReadMultiAngleLoop) -> Self {
        [0x92, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadSingleAngleLoop {}

impl From<ReadSingleAngleLoop> for DataArray {
    fn from(_val: ReadSingleAngleLoop) -> Self {
        [0x94, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ClearMotorAngleLoop {}

impl From<ClearMotorAngleLoop> for DataArray {
    fn from(_val: ClearMotorAngleLoop) -> Self {
        [0x95, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadMotorState1AndErrorState {}

impl From<ReadMotorState1AndErrorState> for DataArray {
    fn from(_val: ReadMotorState1AndErrorState) -> Self {
        [0x9A, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ClearMotorErrorState {}

impl From<ClearMotorErrorState> for DataArray {
    fn from(_val: ClearMotorErrorState) -> Self {
        [0x9B, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadMotorState2 {}

impl From<ReadMotorState2> for DataArray {
    fn from(_val: ReadMotorState2) -> Self {
        [0x9C, 0, 0, 0, 0, 0, 0, 0]
    }
}

#[derive(Serialize, Deserialize, Debug, Default, PartialEq, Eq, Clone, Copy)]
pub struct ReadMotorState3 {}

impl From<ReadMotorState3> for DataArray {
    fn from(_val: ReadMotorState3) -> Self {
        [0x9D, 0, 0, 0, 0, 0, 0, 0]
    }
}

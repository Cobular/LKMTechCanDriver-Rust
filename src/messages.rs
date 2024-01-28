type DataArray = [u8; 8];

#[derive(Debug, Default, PartialEq, Eq)]
struct MotorOffCommand {}

impl serde::Serialize for MotorOffCommand {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where S: serde::Serializer {
        let arr: DataArray = [0x80, 0, 0, 0, 0, 0, 0, 0];
        serializer.serialize_bytes(&arr)
    }
}

#[derive(Debug, Default, PartialEq, Eq)]
struct MotorOnCommand {}

def serde::Serialize for MotorOnCommand {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where S: serde::Serializer {
        let arr: DataArray = [0x88, 0, 0, 0, 0, 0, 0, 0];
        serializer.serialize_bytes(&arr)
    }
}



#[derive(Debug, Default, PartialEq, Eq)]
struct MotorStopCommand {}

impl serde::Serialize for MotorStopCommand {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where S: serde::Serializer {
        let arr: DataArray = [0x81, 0, 0, 0, 0, 0, 0, 0];
        serializer.serialize_bytes(&arr)
    }
}

#[derive(Debug, Default, PartialEq, Eq)]
struct OpenLoopControl {
  power_control: u16
}

impl OpenLoopControl {
  fn new(power_control: u16) -> Result<Self≥ {
    Self { power_control }
  }
}


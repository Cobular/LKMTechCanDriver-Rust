#include "controlCommands.h"

LKMTechMotorController::LKMTechMotorController() {
  mcp = nullptr;
}

LKMTechMotorController::LKMTechMotorController(Adafruit_MCP2515 *_mcp) {
  mcp = _mcp;
}

void LKMTechMotorController::initalize(Adafruit_MCP2515 *_mcp) {
  mcp = _mcp;
}

void noopResponseParser(uint8_t *arr) {
  return;
}

void parseMotorOffResponse(uint8_t* packet) {
    Serial.println("Motor Off Command Response:");
    Serial.print("Command Byte: 0x");
    Serial.println(packet[0], HEX);
}

ResponseParser LKMTechMotorController::sendMotorOff(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x80);
  for (int i = 0; i < 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::sendMotorOn(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x88);  // Command byte
  for (int i = 0; i < 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::sendMotorStop(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x81);  // Command byte
  for (int i = 0; i < 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

void parseOpenLoopResponse(uint8_t* packet) {
    // Assuming the response includes motor temperature, power, speed, and encoder position
    // Please adjust based on the actual response format
    // Note: Power here ranges from -850 to 850
    Serial.print("{");
    Serial.print("\"temp\": ");
    Serial.print(packet[1], DEC);
    Serial.print(", \"power\": ");
    Serial.print(*(int16_t*)&packet[2], DEC);
    Serial.print(", \"speed\": ");
    Serial.print(*(int16_t*)&packet[4], DEC);
    Serial.print(", \"pos\": ");
    Serial.print(*(uint16_t*)&packet[6], DEC);
    Serial.println("}");
}

ResponseParser LKMTechMotorController::sendOpenLoopControl(int id, int16_t powerControl) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA0);  // Command byte
  mcp->write(0x00);  // NULL bytes
  mcp->write(0x00);
  mcp->write(0x00);
  mcp->write(lowByte(powerControl));  // PowerControl low byte
  mcp->write(highByte(powerControl));  // PowerControl high byte
  mcp->write(0x00);  // NULL bytes
  mcp->write(0x00);
  mcp->endPacket();

  return &parseOpenLoopResponse;
}

void parseClosedLoopResponse(uint8_t* packet) {
    // The response includes motor temperature, torque current, speed, and encoder position

    // Note power here is "Motor torque current value iq", range is -2048 to 2048 and maps to -33A to 33A
    Serial.print("{\"temp\": ");
    Serial.print(packet[1], DEC);
    Serial.print(", \"current\": ");
    Serial.print(*(int16_t*)&packet[2], DEC);
    Serial.print(", \"speed\": ");
    Serial.print(*(int16_t*)&packet[4], DEC);
    Serial.print(", \"pos\": ");
    Serial.print(*(uint16_t*)&packet[6], DEC);
    Serial.println("}");
}

ResponseParser LKMTechMotorController::sendTorqueClosedLoopControl(int id, int16_t iqControl) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA1);  // Command byte
  mcp->write(0x00);  // NULL bytes
  mcp->write(0x00);
  mcp->write(0x00);
  mcp->write(lowByte(iqControl));  // Torque current control low byte
  mcp->write(highByte(iqControl));  // Torque current control high byte
  mcp->write(0x00);  // NULL bytes
  mcp->write(0x00);
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendSpeedClosedLoopControl(int id, int32_t speedControl) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA2);  // Command byte
  for (int i = 1; i <= 3; i++) {
    mcp->write((uint8_t)(speedControl >> (8 * (i - 1))));  // Speed control bytes
  }
  for (int i = 4; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendAbsoluteAngleControl(int id, int32_t angleControl) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA3);  // Command byte
  for (int i = 1; i <= 3; i++) {
    mcp->write((uint8_t)(angleControl >> (8 * (i - 1))));  // Angle control bytes
  }
  for (int i = 4; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendAbsoluteAngleControlSpeedLimit(int id, int32_t angleControl, uint16_t maxSpeed) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA4);  // Command byte
  mcp->write(0x00);  // NULL byte
  mcp->write(lowByte(maxSpeed));  // Max speed low byte
  mcp->write(highByte(maxSpeed));  // Max speed high byte
  for (int i = 3; i <= 6; i++) {
    mcp->write((uint8_t)(angleControl >> (8 * (i - 3))));  // Angle control bytes
  }
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendRelativeAngleControl(int id, uint8_t spinDirection, int32_t angleControl) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA5);  // Command byte
  mcp->write(spinDirection);  // Spin direction byte
  for (int i = 2; i <= 5; i++) {
    mcp->write((uint8_t)(angleControl >> (8 * (i - 2))));  // Angle control bytes
  }
  for (int i = 6; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendRelativeAngleControlSpeedLimit(int id, uint8_t spinDirection, int32_t angleControl, uint16_t maxSpeed) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA6);  // Command byte
  mcp->write(spinDirection);  // Spin direction byte
  mcp->write(lowByte(maxSpeed));  // Max speed low byte
  mcp->write(highByte(maxSpeed));  // Max speed high byte
  for (int i = 4; i <= 7; i++) {
    mcp->write((uint8_t)(angleControl >> (8 * (i - 4))));  // Angle control bytes
  }
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendIncrementAngleControl1(int id, int32_t angleIncrement) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA7);  // Command byte
  for (int i = 1; i <= 4; i++) {
    mcp->write((uint8_t)(angleIncrement >> (8 * (i - 1))));  // Angle increment bytes
  }
  for (int i = 5; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendIncrementAngleControl2(int id, int32_t angleIncrement, uint16_t maxSpeed) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0xA8);  // Command byte
  mcp->write(0x00);  // NULL byte
  mcp->write(lowByte(maxSpeed));  // Max speed low byte
  mcp->write(highByte(maxSpeed));  // Max speed high byte
  for (int i = 3; i <= 6; i++) {
    mcp->write((uint8_t)(angleIncrement >> (8 * (i - 3))));  // Angle increment bytes
  }
  mcp->endPacket();

  return &parseClosedLoopResponse;
}

ResponseParser LKMTechMotorController::sendReadPIDParameter(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x30);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::writePIDParamsToRAM(int id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x31);  // Command byte
  mcp->write(0x00);  // NULL byte
  mcp->write(anglePidKp);
  mcp->write(anglePidKi);
  mcp->write(speedPidKp);
  mcp->write(speedPidKi);
  mcp->write(iqPidKp);
  mcp->write(iqPidKi);
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::writePIDParamsToROM(int id, uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x32);  // Command byte
  mcp->write(0x00);  // NULL byte
  mcp->write(anglePidKp);
  mcp->write(anglePidKi);
  mcp->write(speedPidKp);
  mcp->write(speedPidKi);
  mcp->write(iqPidKp);
  mcp->write(iqPidKi);
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::readAcceleration(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x33);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::writeAccelerationToRAM(int id, int32_t Accel) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x34);  // Command byte
  mcp->write(0x00);  // NULL bytes
  for (int i = 0; i < 4; i++) {
    mcp->write((uint8_t)(Accel >> (8 * i)));  // Acceleration bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::readEncoder(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x90);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::writeEncoderValToROM(int id, uint16_t encoderOffset) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x91);  // Command byte
  mcp->write(0x00);  // NULL bytes
  mcp->write(0x00);
  mcp->write(0x00);
  mcp->write(0x00);
  mcp->write(0x00);
  mcp->write((uint8_t)(encoderOffset & 0xFF));  // Encoder offset low byte
  mcp->write((uint8_t)(encoderOffset >> 8));   // Encoder offset high byte
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::writeCurrentPositionToROM(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x19);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::readMultiAngleLoop(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x92);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::readSingleAngleLoop(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x94);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::clearMotorAngleLoop(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x95);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::readMotorState1AndErrorState(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x9A);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::clearMotorErrorState(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x9B);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::readMotorState2(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x9C);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

ResponseParser LKMTechMotorController::readMotorState3(int id) {
  mcp->beginPacket(0x140 + id);
  mcp->write(0x9D);  // Command byte
  for (int i = 1; i <= 7; i++) {
    mcp->write(0x00);  // NULL bytes
  }
  mcp->endPacket();

  return &noopResponseParser;
}

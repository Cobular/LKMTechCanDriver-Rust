# LKM Tech Motor Driver

This project is a basic program that takes speed goals from stdin and sends them to the motor. It's written in Rust and uses the `lkmtech_motor_driver` library to control the motor.

## Prerequisites

- Rust programming language
- `lkmtech_motor_driver` library

## Usage

To use this program, you need to run the `basic_speed_setting.rs` file. This file reads speed goals from the standard input (stdin) and sends them to the motor.

```bash
cargo run --example basic_speed_setting
```

Then, you can input your speed goals line by line. The program will send these speed goals to the motor using closed-loop control.

## Code Overview

The `basic_speed_setting.rs` file contains the main function of the program. It creates a new `MgMotor` instance, which represents the motor. The motor is connected to the "vcan0" virtual CAN interface, and it uses the CAN ID 0x1 and a timeout of 10 milliseconds.

The program then reads lines from stdin in a loop. Each line is parsed as an integer, which represents the speed goal. The speed goal is then sent to the motor using the `send_speed_closed_loop_control` method of the `MgMotor` instance.

## Contributing

Contributions are welcome! Please feel free to submit a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
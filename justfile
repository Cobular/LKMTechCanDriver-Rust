reset:
  sudo ip link set can0 down
  sudo ip link set can0 up

build:
  cargo build --example basic_motor_pid

run:
  cargo run --example basic_motor_pid

safe:
  cargo run --example safe

plot:
  python ./plot.py 
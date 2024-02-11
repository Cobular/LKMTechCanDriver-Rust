reset:
  sudo ip link set can0 down
  sudo ip link set can0 up

build:
  cargo build --release --example basic_motor_pid

run:
  sudo mkdir -p /mnt/ramdisk && mount | grep -q '/mnt/ramdisk ' || sudo mount -t tmpfs -o size=512M tmpfs /mnt/ramdisk
  sudo ./target/release/examples/basic_motor_pid


safe:
  cargo run --example safe

plot:
  python ./plot.py 

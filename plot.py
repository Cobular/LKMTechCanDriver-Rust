import matplotlib.pyplot as plt
import numpy as np

# Load the data
data = np.genfromtxt('motor_data.csv', delimiter=',', skip_header=1)

# Extract columns
time = data[:, 0]
position = data[:, 1]
velocity = data[:, 2]
position_error = data[:, 3]
velocity_setpoint = data[:, 4]
velocity_error = data[:, 5]
target_torque = data[:, 6]

# Create plots
fig, axs = plt.subplots(3, 2, figsize=(15, 10))
fig.suptitle('Motor Data Analysis')

axs[0, 0].plot(time, position, label='Position')
axs[0, 0].set_title('Position over Time')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].set_ylabel('Position (degrees)')

axs[0, 1].plot(time, velocity, label='Velocity', color='orange')
axs[0, 1].set_title('Velocity over Time')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].set_ylabel('Velocity (degrees/s)')

axs[1, 0].plot(time, position_error, label='Position Error', color='green')
axs[1, 0].set_title('Position Error over Time')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Position Error (degrees)')

axs[1, 1].plot(time, velocity_setpoint, label='Velocity Setpoint', color='red')
axs[1, 1].set_title('Velocity Setpoint over Time')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Velocity Setpoint (degrees/s)')

axs[2, 0].plot(time, velocity_error, label='Velocity Error', color='purple')
axs[2, 0].set_title('Velocity Error over Time')
axs[2, 0].set_xlabel('Time (s)')
axs[2, 0].set_ylabel('Velocity Error (degrees/s)')

axs[2, 1].plot(time, target_torque, label='Target Torque', color='brown')
axs[2, 1].set_title('Target Torque over Time')
axs[2, 1].set_xlabel('Time (s)')
axs[2, 1].set_ylabel('Target Torque')

for ax in axs.flat:
    # ax.label_outer()
    ax.legend()

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.savefig('motor_data_analysis.png')

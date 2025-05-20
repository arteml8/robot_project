import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from imu import IMU

import time

imu = IMU(force_calibration=True)
positions = []

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

while True:
    imu.update()
    x, y, z = imu.get_position()
    print(f"Position: x={x:.4f}, y={y:.4f}, z={z:.4f}")
    positions.append((x, y, z))

    if len(positions) > 100:  # sliding window
        positions.pop(0)

    xs, ys, zs = zip(*positions)
    ax.clear()
    ax.plot(xs, ys, zs, color='blue')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    plt.draw()
    plt.pause(0.01)
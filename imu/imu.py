import time
import math
import numpy as np

from adxl345_sensor import ADXL345Sensor
from hmc5883l_sensor import MagnetometerHMC5883L
from itg3200_sensor import GyroscopeITG3200
from calibration import Calibration


class IMU:
    def __init__(self, force_calibration=False):
        # Initialize sensors
        self.accel = ADXL345Sensor()
        self.gyro = GyroscopeITG3200()
        self.mag  = MagnetometerHMC5883L()

        if force_calibration:
        	for sensor in (self.accel, self.gyro, self.mag):
        		sensor.calibrate()

        # Orientation state (radians)
        self.pitch = 0.0
        self.roll  = 0.0
        self.yaw   = 0.0

        # Position state (meters)
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

        # Timing
        self.last_time = time.time()

    def read_sensors(self):
        ax, ay, az = self.accel.read_offset_and_scaled()
        gx, gy, gz = self.gyro.read_offset_and_scaled()      # rad/s
        mx, my, mz = self.mag.read_offset_and_scaled()
        # print(f'Accel: {ax}, {ay}, {az}, Gyro: {gx}, {gy}, {gz}, Mag: {mx}, {my}, {mz}')
        return (ax, ay, az), (gx, gy, gz), (mx, my, mz)

    def get_accel_angles(self, ax, ay, az):
        # Avoid division by zero
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        roll  = math.atan2(ay, az)
        return pitch, roll

    def complementary_filter(self, acc_pitch, acc_roll, gx, gy, dt, alpha=0.98):
        self.pitch = alpha * (self.pitch + gx * dt) + (1 - alpha) * acc_pitch
        self.roll  = alpha * (self.roll  + gy * dt) + (1 - alpha) * acc_roll

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        (ax, ay, az), (gx, gy, gz), (mx, my, mz) = self.read_sensors()

        acc_pitch, acc_roll = self.get_accel_angles(ax, ay, az)
        self.complementary_filter(acc_pitch, acc_roll, gx, gy, dt)

        # Optional: compute yaw using magnetometer (will add later)
        # Optional: transform acceleration into world frame and integrate
        acc = np.array([ax, ay, az])
        acc_world = self.rotate_vector(acc, self.pitch, self.roll)  # yaw omitted for now

        # Subtract gravity
        acc_world[2] -= 9.81

        # Threshold small noise
        acc_world[np.abs(acc_world) < 0.05] = 0

        # Integrate to velocity and position
        self.velocity += acc_world * dt
        self.position += self.velocity * dt

        # Zero-velocity update if stationary
        if np.linalg.norm(acc_world) < 0.1 and np.linalg.norm([gx, gy, gz]) < 0.01:
            self.velocity = np.zeros(3)

    def rotate_vector(self, v, pitch, roll):
        # Rotate using pitch and roll (yaw to be added later)
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(pitch), -math.sin(pitch)],
            [0, math.sin(pitch), math.cos(pitch)]
        ])
        Ry = np.array([
            [math.cos(roll), 0, math.sin(roll)],
            [0, 1, 0],
            [-math.sin(roll), 0, math.cos(roll)]
        ])
        return Ry @ Rx @ v

    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.pitch, self.roll, self.yaw  # yaw placeholder

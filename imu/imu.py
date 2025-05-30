import time
import math
import numpy as np

from imu.adxl345_sensor import ADXL345Sensor
from imu.hmc5883l_sensor import MagnetometerHMC5883L
from imu.itg3200_sensor import GyroscopeITG3200


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

    def get_yaw(self, mx, my, mz):
        # Use current pitch and roll
        pitch = self.pitch
        roll = self.roll

        # Normalize magnetometer readings
        norm = math.sqrt(mx**2 + my**2 + mz**2)
        if norm == 0:
            norm = 1e-6
        mx, my, mz = mx / norm, my / norm, mz / norm

        # Tilt-compensated magnetic field
        mag_x = mx * math.cos(pitch) + mz * math.sin(pitch)
        mag_y = (
            mx * math.sin(roll) * math.sin(pitch)
            + my * math.cos(roll)
            - mz * math.sin(roll) * math.cos(pitch)
        )

        yaw = math.atan2(-mag_y, mag_x)  # NED convention: yaw CW from north

        # Apply magnetic declination
        DECLINATION = math.radians(-13.9667)  # Boston, MA
        yaw += DECLINATION

        # Wrap yaw to [-pi, pi]
        if yaw > math.pi:
            yaw -= 2 * math.pi
        elif yaw < -math.pi:
            yaw += 2 * math.pi

        return yaw

    # def complementary_filter(self, acc_pitch, acc_roll, gx, gy, dt, alpha=0.95):
    #     self.pitch = alpha * (self.pitch + gx * dt) + (1 - alpha) * acc_pitch
    #     self.roll  = alpha * (self.roll  + gy * dt) + (1 - alpha) * acc_roll
    def complementary_filter(self, acc_pitch, acc_roll, gx, gy, gz, mag_yaw, dt, alpha=0.95):
        self.pitch = alpha * (self.pitch + gx * dt) + (1 - alpha) * acc_pitch
        self.roll  = alpha * (self.roll  + gy * dt) + (1 - alpha) * acc_roll
        self.yaw = alpha * (self.yaw + gz * dt) + (1 - alpha) * mag_yaw

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        (ax, ay, az), (gx, gy, gz), (mx, my, mz) = self.read_sensors()

        acc_pitch, acc_roll = self.get_accel_angles(ax, ay, az)
        mag_yaw = self.get_yaw(mx, my, mz)

        self.complementary_filter(acc_pitch, acc_roll, gx, gy, gz, mag_yaw, dt)

        acc = np.array([ax, ay, az])

        # Rotate accel into world frame
        acc_world = self.rotate_vector_rpy(acc, self.roll, self.pitch, self.yaw)

        print(f"Total accel magnitude: {np.linalg.norm(acc)}")
        print(f"acc_world before gravity removal: {acc_world}")

        # Remove gravity (assumed to be [0, 0, 1.0] in body frame before rotation)
        gravity_body = np.array([0.0, 0.0, 1.0])
        gravity_world = self.rotate_vector_rpy(gravity_body, self.roll, self.pitch, self.yaw)
        acc_world -= gravity_world

        print(f"acc_world after gravity removal: {acc_world}")
        # Threshold small noise
        acc_world[np.abs(acc_world) < 0.05] = 0

        # Integrate to velocity and position
        self.velocity += acc_world * dt
        self.position += self.velocity * dt

        # Zero-velocity update if stationary
        if np.linalg.norm(acc_world) < 0.1 and np.linalg.norm([gx, gy, gz]) < 0.01:
            self.velocity = np.zeros(3)

    def rotate_vector(self, v, pitch, roll):
        # Roll (X), then pitch (Y)
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        return Ry @ Rx @ v

    def rotate_vector_rpy(self, vec, roll, pitch, yaw):
        # Rotation matrix from body to world frame
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        R = np.array([
            [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
            [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
            [-sp,     sr * cp,                cr * cp]
        ])

        return R @ vec

    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.pitch, self.roll, self.yaw  # yaw placeholder

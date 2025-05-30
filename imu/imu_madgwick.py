from datetime import datetime

import numpy as np
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

from imu.adxl345_sensor import ADXL345Sensor
from imu.hmc5883l_sensor import MagnetometerHMC5883L
from imu.itg3200_sensor import GyroscopeITG3200
from imu.velocity_ekf import VelocityEKF

class IMU:
    def __init__(self, force_calibration=False):
        self.accel = ADXL345Sensor()
        self.gyro = GyroscopeITG3200()
        self.mag = MagnetometerHMC5883L()
        self.velocity_ekf = VelocityEKF()
        self.accel_filtered = np.zeros(3)
        self.accel_bias = np.zeros(3)

        if force_calibration:
            for sensor in (self.accel, self.gyro, self.mag):
                sensor.calibrate()

        # Orientation filter
        self.ahrs = Madgwick()

        # Initial orientation quaternion
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z

        # Velocity and position estimation
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

        self.last_time = datetime.utcnow().isoformat()

    def update(self):
        now = datetime.utcnow().isoformat()
        dt = min(max(now - self.last_time, 0.001), 0.1)  # clamp between 1ms and 100ms
        self.last_time = now

        # Read sensors
        accel = np.array(self.accel.read_offset_and_scaled())
        gyro = np.array(self.gyro.read_offset_and_scaled())  # in rad/s
        mag = np.array(self.mag.read_offset_and_scaled())
        accel_mps2 = accel * 9.80665

        # Update Madgwick filter
        self.quaternion = self.ahrs.updateMARG(
            self.quaternion, gyr=gyro, acc=accel_mps2, mag=mag
        )

        # Convert quaternion to Euler angles
        r = R.from_quat([
            self.quaternion[1], self.quaternion[2],
            self.quaternion[3], self.quaternion[0]  # x, y, z, w
        ])
        pitch, roll, yaw = r.as_euler('xyz', degrees=False)
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

        # Rotate acceleration to world frame
        acc_world = r.apply(accel_mps2)

        # Subtract gravity
        g_world = r.apply([0, 0, 9.80665])
        # gravity = np.array([0.0, 0.0, 9.80665])
        acc_world -= g_world

        # Add a low pass filter for acceleration accumulation
        alpha = 0.15
        self.accel_filtered = alpha * acc_world + (1 - alpha) * self.accel_filtered

        # Threshold noise
        self.accel_filtered[np.abs(self.accel_filtered) < 0.05] = 0

        # EKF prediction
        self.velocity_ekf.predict(self.accel_filtered, dt)

        # ZUPT condition
        if np.linalg.norm(self.accel_filtered) < 0.1 and np.linalg.norm(gyro) < 0.01:
            self.velocity_ekf.update_zupt()

        # Update state
        self.velocity = self.velocity_ekf.get_velocity()
        self.position = self.velocity_ekf.get_position()

        with open("imu_log.csv", "a") as f:
            writer = csv.writer(f)
            writer.writerow([
                self.last_time,
                *self.position,
                *self.velocity,
                *self.accel_bias,
                self.pitch, self.roll, self.yaw
            ])

    def get_orientation(self):
        return self.pitch, self.roll, self.yaw

    def get_position(self):
        return self.position
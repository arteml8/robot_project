import time
import numpy as np
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

from adxl345_sensor import ADXL345Sensor
from hmc5883l_sensor import MagnetometerHMC5883L
from itg3200_sensor import GyroscopeITG3200


class IMU:
    def __init__(self, force_calibration=False):
        self.accel = ADXL345Sensor()
        self.gyro = GyroscopeITG3200()
        self.mag = MagnetometerHMC5883L()

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

        self.last_time = time.time()

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Read sensors
        accel = np.array(self.accel.read_offset_and_scaled())
        gyro = np.array(self.gyro.read_offset_and_scaled())  # in rad/s
        mag = np.array(self.mag.read_offset_and_scaled())

        # Update Madgwick filter
        self.quaternion = self.ahrs.updateIMU(self.quaternion, gyr=gyro, acc=accel, mag=mag)

        # Convert quaternion to Euler angles
        r = R.from_quat([self.quaternion[1], self.quaternion[2], self.quaternion[3], self.quaternion[0]])  # x, y, z, w
        pitch, roll, yaw = r.as_euler('xyz', degrees=False)

        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

        # Rotate acceleration to world frame
        acc_world = r.apply(accel)

        # Subtract gravity
        gravity = np.array([0.0, 0.0, 9.81])
        acc_world -= gravity

        # Threshold noise
        acc_world[np.abs(acc_world) < 0.05] = 0

        # Integrate to velocity and position
        self.velocity += acc_world * dt
        self.position += self.velocity * dt

        # Zero-velocity update
        if np.linalg.norm(acc_world) < 0.1 and np.linalg.norm(gyro) < 0.01:
            self.velocity = np.zeros(3)

    def get_orientation(self):
        return self.pitch, self.roll, self.yaw

    def get_position(self):
        return self.position
from datetime import datetime

import numpy as np

from imu.adxl345_sensor import ADXL345Sensor
from imu.hmc5883l_sensor import MagnetometerHMC5883L
from imu.itg3200_sensor import GyroscopeITG3200

class ImuRaw:
    def __init__(self, force_calibration=False):
        self.accel = ADXL345Sensor()
        self.gyro = GyroscopeITG3200()
        self.mag = MagnetometerHMC5883L()
        self.last_time = time.time()

        if force_calibration:
            for sensor in (self.accel, self.gyro, self.mag):
                sensor.calibrate()

def update(self):
        now = time.time()
        dt = min(max(now - self.last_time, 0.001), 0.1)
        self.last_time = now

        accel = np.array(self.accel.read_offset_and_scaled()) * 9.80665  # m/s²
        gyro = np.array(self.gyro.read_offset_and_scaled())              # rad/s
        mag = np.array(self.mag.read_offset_and_scaled())                # µT or similar

        return {
            "accel": accel,
            "gyro": gyro,
            "mag": mag,
            "dt": dt,
            "timestamp": now
        }

import numpy as np

class PositionEstimator:
    def __init__(self):
        self.state = {
            "position": np.zeros(3),     # x, y, z or lat/lon/alt
            "velocity": np.zeros(3),     # vx, vy, vz
            "orientation": np.zeros(4),  # quaternion
        }

    def update_imu(self, accel, gyro, mag, dt):
        """
        Called on every new IMU reading.
        accel, gyro, mag: np.array([x, y, z])
        dt: delta time since last update
        """
        # Stub — just store data or integrate for testing
        self.last_imu = {
            "accel": accel,
            "gyro": gyro,
            "mag": mag,
            "dt": dt,
        }

    def update_gnss(self, lat, lon, alt, speed=None, heading=None, timestamp=None):
        """
        Called when new GNSS data arrives.
        """
        self.last_gnss = {
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "speed": speed,
            "heading": heading,
            "timestamp": timestamp,
        }

    def estimate(self):
        """
        Return the best current estimate of position, velocity, orientation.
        """
        return self.state
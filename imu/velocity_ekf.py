import numpy as np


class VelocityEKF:
    def __init__(self):
        self.x = np.zeros(6)  # [vx, vy, vz, px, py, pz]
        self.P = np.eye(6) * 0.01  # Covariance matrix
        self.Q = np.eye(6) * 0.05  # Process noise
        self.R = np.eye(3) * 0.01  # ZUPT measurement noise (velocity)
        self.F = np.eye(6)         # State transition matrix
        self.H = np.zeros((3, 6))  # Observation matrix
        self.H[:3, :3] = np.eye(3)  # Observe velocity only

    def predict(self, acc_world, dt):
        # Update state transition matrix for motion
        self.F[:3, :3] = np.eye(3)       # velocity update
        self.F[3:, :3] = np.eye(3) * dt  # position update from velocity

        # Control input
        B = np.zeros((6, 3))
        B[:3, :] = np.eye(3) * dt  # v += a * dt

        # Predict state and covariance
        self.x = self.F @ self.x + B @ acc_world
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_zupt(self):
        z = np.zeros(3)  # zero velocity measurement
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_velocity(self):
        return self.x[:3]

    def get_position(self):
        return self.x[3:]
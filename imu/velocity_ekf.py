import numpy as np


class VelocityEKF:
    def __init__(self):
        # State vector: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, bias_ax, bias_ay, bias_az]
        self.x = np.zeros((9, 1))
        self.P = np.eye(9) * 0.1

        self.F = np.eye(9)  # state transition matrix
        self.H = np.zeros((3, 9))  # measurement matrix
        self.H[:, 3:6] = np.eye(3)  # we observe velocity

        # Process noise: small for position, moderate for velocity, tiny for bias drift
        self.Q = np.diag([1e-4]*3 + [1e-2]*3 + [1e-6]*3)

        # Measurement noise (ZUPT): we trust the zero velocity condition strongly
        self.R = np.eye(3) * 1e-3

    def predict(self, acc_world, dt):
        acc_world = acc_world.reshape((3, 1))

        # Update transition matrix F
        self.F[:3, 3:6] = np.eye(3) * dt
        self.F[3:6, 6:9] = -np.eye(3) * dt  # velocity affected by bias

        # Predict state
        self.x = self.F @ self.x
        self.x[3:6] += acc_world * dt  # apply actual acceleration input

        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_zupt(self):
        z = np.zeros((3, 1))  # measured velocity is 0
        y = z - self.x[3:6]   # residual
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(9) - K @ self.H) @ self.P

    def get_position(self):
        return self.x[0:3].flatten()

    def get_velocity(self):
        return self.x[3:6].flatten()

    def get_accel_bias(self):
        return self.x[6:9].flatten()

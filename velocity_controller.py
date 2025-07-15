import math
import time

class JetsonVelocityController:
    def __init__(self, kinematics, tracker, ble_client, control_interval=0.1, Kp=0.8):
        self.kinematics = kinematics
        self.tracker = tracker
        self.ble = ble_client
        self.control_interval = control_interval
        self.last_pose = (0.0, 0.0, 0.0)
        self.last_time = time.time()

        self.target_velocity = (0.0, 0.0, 0.0)  # (vx, vy, omega)
        self.Kp = Kp  # Simple proportional gain for feedback

    def set_target_velocity(self, vx, vy, omega):
        self.target_velocity = (vx, vy, omega)

    def compute_actual_velocity(self):
        current_pose = self.tracker.get_pose()
        now = time.time()
        dt = now - self.last_time

        dx = current_pose[0] - self.last_pose[0]
        dy = current_pose[1] - self.last_pose[1]
        dtheta = current_pose[2] - self.last_pose[2]

        vx = dx / dt
        vy = dy / dt
        omega = dtheta / dt

        self.last_pose = current_pose
        self.last_time = now
        return (vx, vy, omega)

    async def update(self):
        actual = self.compute_actual_velocity()
        desired = self.target_velocity
        error = tuple(d - a for d, a in zip(desired, actual))

        # --- Proportional control (simplified placeholder for future PID) ---
        control_output = tuple(self.Kp * e for e in error)

        vx_cmd = desired[0] + control_output[0]
        vy_cmd = desired[1] + control_output[1]
        omega_cmd = desired[2] + control_output[2]

        # Clamp for safety
        vx_cmd = max(min(vx_cmd, 0.5), -0.5)
        vy_cmd = max(min(vy_cmd, 0.5), -0.5)
        omega_cmd = max(min(omega_cmd, 1.0), -1.0)

        # TODO: Replace with full PID later if necessary
        cmd = f"CMD:DRIVE:{vx_cmd:.3f},{vy_cmd:.3f},{omega_cmd:.3f}\n"
        await self.ble.send(cmd)
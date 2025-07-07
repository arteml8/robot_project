import math
import time

class JetsonVelocityController:
    def __init__(self, kinematics, tracker, ble_client, control_interval=0.1):
        self.kinematics = kinematics
        self.tracker = tracker
        self.ble = ble_client
        self.control_interval = control_interval
        self.last_pose = (0.0, 0.0, 0.0)
        self.last_time = time.time()

        self.target_velocity = (0.0, 0.0, 0.0)  # (vx, vy, omega)

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
        # Call this periodically (e.g., every 100ms)
        # Compute control and send to robot
        actual = self.compute_actual_velocity()
        desired = self.target_velocity

        error = tuple(d - a for d, a in zip(desired, actual))

        # For now, we’ll just pass through the desired values
        # (could scale or dampen based on error if needed)
        cmd = f"CMD:DRIVE:{desired[0]:.3f},{desired[1]:.3f},{desired[2]:.3f}\n"
        await self.ble.send(cmd)

        # Query encoder right after command is sent
        await self.ble.send("CMD:GET_ENCODERS\n")
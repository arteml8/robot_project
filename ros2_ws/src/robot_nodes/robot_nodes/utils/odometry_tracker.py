import math
from mecanum_kinematics import MecanumKinematics

class OdometryTracker:
    def __init__(self, kinematics: MecanumKinematics):
        self.kinematics = kinematics
        self.last_ticks = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # in radians

    def update_ticks(self, current_ticks):
        if self.last_ticks == current_ticks:
            return  # Avoid redundant updates

        if self.last_ticks is None:
            self.last_ticks = current_ticks
            return

        delta_ticks = [curr - prev for curr, prev in zip(current_ticks, self.last_ticks)]
        dx, dy, dtheta = self.kinematics.forward_kinematics(delta_ticks)

        # Rotate dx, dy from robot frame to global frame
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        global_dx = dx * cos_theta - dy * sin_theta
        global_dy = dx * sin_theta + dy * cos_theta

        self.x += global_dx
        self.y += global_dy
        self.theta += dtheta

        self.last_ticks = current_ticks

    def get_pose(self):
        return self.x, self.y, self.theta
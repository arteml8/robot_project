import math

class MecanumKinematics:
    def __init__(self, wheel_radius, wheel_base_length, wheel_base_width, ticks_per_rev, gear_ratio=1.0):
        self.r = wheel_radius  # in meters
        self.L = wheel_base_length  # front-back
        self.W = wheel_base_width   # left-right
        self.ticks_per_rev = ticks_per_rev
        self.gear_ratio = gear_ratio

        # Distance per tick
        self.ticks_to_meters = (2 * math.pi * self.r) / (self.ticks_per_rev * self.gear_ratio)

    def forward_kinematics(self, delta_ticks):
        """
        Compute robot-centric motion (dx, dy, dtheta) given tick deltas from each wheel.
        Wheel order: [FL, FR, BR, BL]
        """
        if len(delta_ticks) != 4:
            raise ValueError("Expected 4 tick values")

        # Convert ticks to linear distance for each wheel
        distances = [ticks * self.ticks_to_meters for ticks in delta_ticks]
        v_fl, v_fr, v_br, v_bl = distances

        # Inverse kinematics for mecanum wheels (assuming center of robot)
        dx = (v_fl + v_fr + v_br + v_bl) / 4.0
        dy = (-v_fl + v_fr + v_br - v_bl) / 4.0
        dtheta = (-v_fl + v_fr - v_br + v_bl) / (4 * (self.L + self.W))

        return dx, dy, dtheta
import math

# Hardware constants for the robot chassis
WHEEL_RADIUS = 1.5         # 3in diameter
WHEEL_BASE_LENGTH = 4.875  # 4 and 7/8 in center-to-center
WHEEL_BASE_WIDTH = 4.875   # 4 and 7/8 in center-to-center
TICKS_PER_REV = 46.0       # 23 apertures x 2 (since the encoder changes twice)
GEAR_RATIO = 1.0           # Encoders on the output shaft, but allowing for a diff config
UNIT = 'IN'                # Setting the units to inches, to make sure the init converts them to m


class MecanumKinematics:
    def __init__(self, 
            wheel_radius=WHEEL_RADIUS,
            wheel_base_length=WHEEL_BASE_LENGTH,
            wheel_base_width=WHEEL_BASE_WIDTH,
            ticks_per_rev=TICKS_PER_REV,
            gear_ratio=GEAR_RATIO,
            unit=UNIT):

        self.conversion = None
        # Convert inches to meters if needed
        if unit.lower() == 'in':
            self.conversion = 0.0254  # 1 inch = 0.0254 meters

        elif unit.lower() == 'm':
            self.conversion = 1

        else:
            raise ValueError("Wrong unit declared")

        wheel_radius, wheel_base_length, wheel_base_width = [
            x * conversion for x in (wheel_radius, wheel_base_length, wheel_base_width)
        ]
        self.r = wheel_radius
        self.L = wheel_base_length
        self.W = wheel_base_width
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
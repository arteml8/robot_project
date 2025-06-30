import numpy as np

class MecanumModel:
    def __init__(self, wheel_radius, robot_width, robot_length, ticks_per_rev):
        self.r = wheel_radius
        self.l = robot_length
        self.w = robot_width
        self.ticks_per_rev = ticks_per_rev
        self.distance_per_tick = 2 * np.pi * self.r / self.ticks_per_rev

    def encoder_ticks_to_velocity(self, ticks):
        # ticks: [FL, FR, BL, BR]
        meters = [t * self.distance_per_tick for t in ticks]
        vx = (meters[0] + meters[1] + meters[2] + meters[3]) / 4.0
        vy = (-meters[0] + meters[1] + meters[2] - meters[3]) / 4.0
        omega = (-meters[0] + meters[1] - meters[2] + meters[3]) / (4.0 * (self.l + self.w))
        return vx, vy, omega

    def velocity_to_encoder_ticks(self, vx, vy, omega, dt):
        # compute per wheel distances
        v_fl = vx - vy - (self.l + self.w) * omega
        v_fr = vx + vy + (self.l + self.w) * omega
        v_bl = vx + vy - (self.l + self.w) * omega
        v_br = vx - vy + (self.l + self.w) * omega
        distances = [v * dt for v in [v_fl, v_fr, v_bl, v_br]]
        ticks = [int(d / self.distance_per_tick) for d in distances]
        return ticks
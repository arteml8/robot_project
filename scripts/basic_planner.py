import math

class BasicPlanner:
    def __init__(self, tolerance=0.05):
        self.tolerance = tolerance  # meters

    def compute_command(self, current_pose, target_pose):
        x, y, theta = current_pose
        x_target, y_target = target_pose

        dx = x_target - x
        dy = y_target - y

        distance = math.hypot(dx, dy)

        if distance < self.tolerance:
            return 0.0, 0.0, 0.0  # Stop

        # Transform target direction to robot frame
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - theta

        # Normalize angle
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # If not roughly facing target, rotate in place
        if abs(angle_error) > 0.2:
            return 0.0, 0.0, 0.3 * (1 if angle_error > 0 else -1)

        # If roughly facing target, move forward
        forward_speed = min(0.3, distance * 0.5)

        return forward_speed, 0.0, 0.0  # Drive forward
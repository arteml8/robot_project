import math
import asyncio

class BasicWaypointPlanner:
    def __init__(self, controller, tracker, waypoint_tolerance=0.05):
        self.controller = controller
        self.tracker = tracker
        self.waypoints = []
        self.waypoint_tolerance = waypoint_tolerance
        self.current_target = None

        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.8  # rad/s

    def load_waypoints(self, waypoints):
        """waypoints: list of (x, y, theta) in meters/radians"""
        self.waypoints = waypoints

    def distance_and_angle_to_target(self):
        x, y, theta = self.tracker.get_pose()
        tx, ty, ttheta = self.current_target

        dx = tx - x
        dy = ty - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx) - theta

        return distance, angle_to_target

    async def run(self):
        while self.waypoints:
            self.current_target = self.waypoints.pop(0)
            print(f"🎯 New waypoint: {self.current_target}")

            while True:
                distance, angle_error = self.distance_and_angle_to_target()

                if distance < self.waypoint_tolerance:
                    print("✅ Reached waypoint")
                    self.controller.set_target_velocity(0.0, 0.0, 0.0)
                    await asyncio.sleep(0.5)
                    break  # Go to next waypoint

                # Basic controller: Rotate first, then drive forward
                if abs(angle_error) > 0.2:
                    vx = 0.0
                    vy = 0.0
                    omega = max(min(angle_error, 0.5), -0.5)
                else:
                    vx = self.linear_speed
                    vy = 0.0
                    omega = max(min(angle_error, 0.5), -0.5) * 0.5  # minor angular correction while driving

                self.controller.set_target_velocity(vx, vy, omega)
                await self.controller.update()
                await asyncio.sleep(self.controller.control_interval)

        # Path complete
        print("🏁 Path complete")
        self.controller.set_target_velocity(0.0, 0.0, 0.0)
        await self.controller.update()
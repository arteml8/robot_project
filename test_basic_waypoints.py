import asyncio
import math

from robot_ble_client import RobotBLEClient
from mecanum_kinematics import MecanumKinematics
from odometry_tracker import OdometryTracker
from velocity_controller import JetsonVelocityController
from basic_motion_planner import BasicWaypointPlanner

# Example waypoints
waypoints = [
    (0.5, 0.0, 0.0),  # Move forward 0.5 meters
    (0.5, 0.5, math.pi/2),  # Move to the side and rotate
    (0.0, 0.5, math.pi),  # Move back to left and rotate
]

velocity_controller = JetsonVelocityController()


async def main():
    kinematics = MecanumKinematics()
    tracker = OdometryTracker(kinematics)
    client = RobotBLEClient()
    controller = JetsonVelocityController(kinematics, tracker, client)
	planner = BasicWaypointPlanner(velocity_controller, tracker)
	planner.load_waypoints(waypoints)

	await planner.run()


if __name__ == "__main__":
    asyncio.run(main())
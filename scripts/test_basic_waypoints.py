import asyncio
import math

from basic_motion_planner import BasicWaypointPlanner
from robot_ble_client import RobotBLEClient
from mecanum_kinematics import MecanumKinematics
from odometry_tracker import OdometryTracker
from velocity_controller import JetsonVelocityController

async def main():
    # Initialize hardware interface
    kinematics = MecanumKinematics()
    tracker = OdometryTracker(kinematics)
    client = RobotBLEClient()
    controller = JetsonVelocityController(kinematics, tracker, client)

    # Example waypoints (adjust as needed)
    waypoints = [
        (0.5, 0.0, 0.0),
        (0.5, 0.5, math.pi / 2),
        (0.0, 0.5, math.pi),
    ]
    planner = BasicWaypointPlanner(controller, tracker)
    planner.load_waypoints(waypoints)

    # Handle encoder ticks
    async def handle_ticks(ticks):
        tracker.update_ticks(ticks)

    client.on_encoder_update = handle_ticks

    # Connect to robot and start encoder polling
    if await client.connect():
        print("✅ Connected to robot")
        asyncio.create_task(client.poll_encoders(200))  # Poll every 200ms

        # Run motion planner
        await planner.run()

        # Stop robot at end (extra safety)
        controller.set_target_velocity(0.0, 0.0, 0.0)
        await controller.update()

        await asyncio.sleep(1.0)
        await client.disconnect()
        print("🔌 Disconnected from robot")

    else:
        print("❌ Failed to connect to robot.")

if __name__ == "__main__":
    asyncio.run(main())
import asyncio
from robot_ble_client import RobotBLEClient
from mecanum_kinematics import MecanumKinematics
from odometry_tracker import OdometryTracker

# Hardware constants
WHEEL_RADIUS = 0.03  # 3cm
WHEEL_BASE_LENGTH = 0.18  # 18cm
WHEEL_BASE_WIDTH = 0.16   # 16cm
TICKS_PER_REV = 360
GEAR_RATIO = 1.0

async def main():
    kinematics = MecanumKinematics(WHEEL_RADIUS, WHEEL_BASE_LENGTH, WHEEL_BASE_WIDTH, TICKS_PER_REV)
    tracker = OdometryTracker(kinematics)
    client = RobotBLEClient()

    async def handle_ticks(ticks):
        tracker.update_ticks(ticks)
        x, y, theta = tracker.get_pose()
        print(f"📍 Pose: x={x:.3f} m, y={y:.3f} m, θ={theta:.2f} rad")

    client.on_encoder_update = handle_ticks

    if await client.connect():
        print("✅ Connected to robot")

        # Begin polling for encoders in background
        asyncio.create_task(client.poll_encoders(200))  # Poll every 200ms

        # Move forward briefly
        await client.send("CMD:DRIVE:0.2,0.0,0.0\n")
        await asyncio.sleep(2.0)
        await client.send("CMD:STOP\n")

        await asyncio.sleep(1.0)  # Let a few encoder readings come in
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
import asyncio
from robot_ble_client import RobotBLEClient

async def main():
    kinematics = MecanumKinematics(WHEEL_RADIUS, WHEEL_BASE_LENGTH, WHEEL_BASE_WIDTH, TICKS_PER_REV)
    tracker = OdometryTracker(kinematics)
    client = RobotBLEClient()
    controller = JetsonVelocityController(kinematics, tracker, client)

    async def handle_ticks(ticks):
        tracker.update_ticks(ticks)

    client.on_encoder_update = handle_ticks

    if await client.connect():
        print("✅ Connected to robot")
        asyncio.create_task(client.poll_encoders(200))

        # Set a forward motion target
        controller.set_target_velocity(0.2, 0.0, 0.0)

        for _ in range(20):
            await controller.update()
            await asyncio.sleep(controller.control_interval)

        # Stop after motion
        controller.set_target_velocity(0.0, 0.0, 0.0)
        await controller.update()
        await asyncio.sleep(1.0)

        await client.disconnect()

asyncio.run(main())
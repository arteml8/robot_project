import asyncio
from robot_ble_client import RobotBLEClient

async def main():
    client = RobotBLEClient()

    connected = await client.connect()
    if not connected:
        print("❌ Could not connect.")
        return

    try:
        # Move forward for a bit
        await client.send("CMD:DRIVE:0.2,0,0\n")
        await asyncio.sleep(2)

        # Stop the robot
        await client.send("CMD:STOP\n")
        await asyncio.sleep(0.5)

        # Get encoder readings
        await client.send("CMD:GET_ENCODERS\n")
        await asyncio.sleep(1)

    finally:
        await client.disconnect()

asyncio.run(main())
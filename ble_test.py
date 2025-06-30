import asyncio
from robot_ble_client import RobotBLEClient

async def main():
    robot = RobotBLEClient()
    if await robot.connect():
        await robot.send("CMD:DRIVE:0.2,0.0,0.0\n")
        await asyncio.sleep(3)
        await robot.send("CMD:STOP\n")
        await robot.disconnect()

asyncio.run(main())
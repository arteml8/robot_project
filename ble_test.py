import asyncio
from robot_ble_client import RobotBLEClient

async def main():
    robot = RobotBLEClient()
    if await robot.connect():
        print("CONNECTED!")
        await robot.send("CMD:DRIVE:0.2,0.0,0.0\n")
        print("Sending go command")
        await asyncio.sleep(3)
        await robot.send("CMD:STOP\n")
        print("sending stop command")
        await robot.disconnect()

asyncio.run(main())
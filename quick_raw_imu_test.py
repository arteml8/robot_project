import time
from fusion.imu_raw import RawIMU

IMU = RawIMU()
while True:
	print(IMU.update())
	time.sleep(0.1)
import time
from imu_raw import ImuRaw

IMU = ImuRaw()
while True:
	print(IMU.update())
	time.sleep(0.1)
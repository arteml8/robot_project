# fusion/test_position_estimator.py

from fusion.position_estimator import PositionEstimator
from fusion.imu_raw import RawIMU
from gnss.gnss_receiver import GNSSReceiver
import time

imu = RawIMU()
gnss = GNSSReceiver()
estimator = PositionEstimator()

last_time = time.time()

while True:
    # --- IMU Update ---
    imu_dict = imu.update()
    accel, gyro, mag, dt = imu_dict['accel'], imu_dict['gyro'], imu_dict['mag'], imu_dict['dt']
    estimator.update_imu(accel, gyro, mag, dt)

    # --- GNSS Update ---
    gps_data = gnss.read()
    if gps_data is not None:
        estimator.update_gnss(
            lat=gps_data["latitude"],
            lon=gps_data["longitude"],
            speed=gps_data.get("speed"),
            timestamp=gps_data.get("timestamp")
        )

    # --- Get Estimate ---
    state = estimator.estimate()
    print("Estimated position:", state["position"])

    time.sleep(0.05)  # Simulate 20Hz update loop
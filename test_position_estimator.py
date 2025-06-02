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
    accel, gyro, mag = imu.read_all()  # numpy arrays
    now = time.time()
    dt = now - last_time
    last_time = now
    estimator.update_imu(accel, gyro, mag, dt)

    # --- GNSS Update ---
    gps_data = gnss.read()
    if gps_data is not None:
        estimator.update_gnss(
            lat=gps_data["lat"],
            lon=gps_data["lon"],
            alt=gps_data["alt"],
            speed=gps_data.get("speed"),
            heading=gps_data.get("heading"),
            timestamp=gps_data.get("timestamp")
        )

    # --- Get Estimate ---
    state = estimator.estimate()
    print("Estimated position:", state["position"])

    time.sleep(0.05)  # Simulate 20Hz update loop
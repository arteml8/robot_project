from imu.adxl345_sensor import ADXL345Sensor
from imu.hmc5883l_sensor import MagnetometerHMC5883L
from imu.itg3200_sensor import GyroscopeITG3200

def main():
    accel = ADXL345Sensor()
    gyro = GyroscopeITG3200()
    mag = MagnetometerHMC5883L()

    accel.calibrate()
    gyro.calibrate()
    mag.calibrate()

if __name__ == "__main__":
    main()
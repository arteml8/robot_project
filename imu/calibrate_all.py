from adxl345_sensor import ADXL345Sensor
from hmc5883l_sensor import MagnetometerHMC5883L
from itg3200_sensor import GyroscopeITG3200

def main():
    accel = ADXL345Sensor()
    gyro = GyroscopeITG3200()
    mag = MagnetometerHMC5883L()

    accel.perform_calibration()
    gyro.perform_calibration()
    mag.perform_calibration()

if __name__ == "__main__":
    main()
from adxl345 import ADXL345Sensor
from itg3200 import GyroscopeITG3200
from hmc5883l import MagnetometerHMC5883L

def main():
    accel = ADXL345Sensor()
    gyro = GyroscopeITG3200()
    mag = MagnetometerHMC5883L()

    accel.perform_calibration()
    gyro.perform_calibration()
    mag.perform_calibration()

if __name__ == "__main__":
    main()
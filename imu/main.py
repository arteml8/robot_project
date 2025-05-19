from adxl345_sensor import ADXL345Sensor
from hmc5883l_sensor import MagnetometerHMC5883L
from itg3200_sensor import GyroscopeITG3200
from calibration import Calibration
from plotter import LivePlot2D
import time

def data_gen(sensor): #, calib):
	x, y, z = sensor.read_raw()
	# x, y, z = calib.apply(x, y, z)
	print(f'{sensor.name}: X: {x}, Y: {y}, Z: {z} \n')
	time.sleep(0.1)

if __name__ == '__main__':
	adxl = ADXL345Sensor()
	hmc = MagnetometerHMC5883L()
	itg = GyroscopeITG3200()

	try: 
		while True: 
			for sensor in (adxl, hmc, itg):
				data_gen(sensor)
	except KeyboardInterrupt:
		print("Exited because keyboard")

  # calib = Calibration(offset=(0,0,0), scale=(1,1,1))
  # plot = LivePlot2D()
  # plot.start(lambda: data_gen(adxl, calib))

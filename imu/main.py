from adxl345_sensor import ADXL345Sensor
from calibration import Calibration
from plotter import LivePlot2D
import time

def data_gen(sensor, calib):
  while True:
    x, y, z = sensor.read_raw()
    x, y, z = calib.apply(x, y, z)
    yield x, y
    time.sleep(0.1)
   
if __name__ == '__main__':
  adxl = ADXL345Sensor()
  calib = Calibration(offset=(0,0,0), scale=(1,1,1))
  plot = LivePlot2D()
  plot.start(lambda: data_gen(adxl, calib))

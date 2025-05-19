from i2c_sensor import I2CSensor


class ADXL345Sensor(I2CSensor):
  def __init__(self, bus_id=1, address=0x53):
    # Call the I2CSensor base constructor
    super().__init__(bus_id, address, name = "ADXL345")
    self._initialize()

  def _initialize(self):
    # Power on the sensor and set measurement mode
    self.bus.write_byte_data(self.address, 0x2D, 0x08) # Power Register
  
  def read_raw(self):
    # Read raw data for each axis (2 bytes each)
    return self.read_raw_data(self.address, 0x32) 

    # if self.calibration:
    #   x-= self.calibration.get(x_offset, 0)
    #   y-= self.calibration.get(y_offset, 0)
    #   z-= self.calibration.get(z_offset, 0)

    # return x, y, z

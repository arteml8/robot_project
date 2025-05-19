from i2c_sensor import I2CSensor


class ADXL345Sensor(I2CSensor):
  def __init__(self, bus_id=1, address=0x53):
    # Call the I2CSensor base constructor
    scale_factor = 256 # default 10 bit signed scale +/-2G 1G @ 256
    super().__init__(bus_id, address, name="ADXL345", scale_factor)
    self._initialize()

  def _initialize(self):
    # Power on the sensor and set measurement mode
    self.bus.write_byte_data(self.address, 0x2D, 0x08) # Power Register
  
  def read_raw(self):
    # Read raw data for each axis (2 bytes each)
    return self.read_raw_data(0x32)

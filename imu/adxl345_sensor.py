from i2c_sensor import I2CSensor


class ADXL345Sensor(I2CSensor):
  def __init__(self, address=0x53, bus_id=1, calibration=None):
    # Call the I2CSensor base constructor
    super().__init__(bus_id, address, calibration)
    
    # Power on the sensor and set measurement mode
    self.bus.write_byte_data(self.address, 0x2D, 0x08) # Power Register
  
  def read(self):
    # Read raw data for each axis (2 bytes each)
    raw_data = self.read_bytes(0x32, 6)
    
    x = self._combine_bytes(raw_data[0], raw_data[1])
    y = self._combine_bytes(raw_data[2], raw_data[3])
    z = self._combine_bytes(raw_data[4], raw_data[5])
    
    # Apply calibration offsets if available
    if self.calibration:
      x-= self.calibration.get(x_offset, 0)
      y-= self.calibration.get(y_offset, 0)
      z-= self.calibration.get(z_offset, 0)

    return x, y, z

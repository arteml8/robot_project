from imu.i2c_sensor import I2CSensor


class MagnetometerHMC5843(I2CSensor):
  def __init__(self, bus_id=1, address=0x1e):
    super().__init__(bus_id, address, name = "HMC5843")
    self._initialize()

  def _initialize(self):
    self.bus.write_byte_data(self.address, 0x00, 0x70) # Config Register A
    self.bus.write_byte_data(self.address, 0x01, 0xA0) # Config Register B
    self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous Measurement Mode
  
  def read_raw(self):
    return self.read_raw_data(self.address, 0x03)

	# return self.apply_offsets(x, y, z)

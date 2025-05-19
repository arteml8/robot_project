from i2c_sensor import I2CSensor


class HMC5843(I2CSensor):
  def __init__(self, address=0x1e, bus_id=1):
    super().__init__(bus_id, address, name = "HMC5843")
    self.bus.write_byte_data(self.address, 0x00, 0x70) # Config Register A
    self.bus.write_byte_data(self.address, 0x01, 0xA0) # Config Register B
    self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous Measurement Mode
  
  def read_raw(self):
    x = self.read_word(0x03)
    y = self.read_word(0x05)
    z = self.read_word(0x07)
	return self.apply_offsets(x, y, z)

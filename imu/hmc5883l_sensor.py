from imu.i2c_sensor import I2CSensor


class MagnetometerHMC5883L(I2CSensor):
  def __init__(self, bus_id=1, address=0x1e):
    scale_factor = 1090
    name="HMC5883L"
    msb_first = True
    super().__init__(bus_id, address, name, msb_first, scale_factor)
    self._initialize()
    self._load_offsets()

  def _initialize(self):
    self.bus.write_byte_data(self.address, 0x00, 0x70) # 8-average, 15 Hz
    self.bus.write_byte_data(self.address, 0x01, 0xA0) # Gain = 1.3 Ga
    self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous Measurement Mode

  def calibrate(self, samples=0):
    print(f"[{self.name}] Placeholder calibration (not yet implemented).")
    self.offsets = {'x': 0, 'y': 0, 'z': 0}
    self.cal_manager.save_offsets(self.offsets)  

  def read_raw(self):
    return self.read_raw_data(0x03)

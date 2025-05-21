from i2c_sensor import I2CSensor


class ADXL345Sensor(I2CSensor):
  def __init__(self, bus_id=1, address=0x53):
    # Call the I2CSensor base constructor
    scale_factor = 256 # default 10 bit signed scale +/-2G
    name="ADXL345"
    msb_first = False
    super().__init__(bus_id, address, name, msb_first, scale_factor)
    self._initialize()
    self._load_offsets()

  def _initialize(self):
    # Power on the sensor and set measurement mode
    self.bus.write_byte_data(self.address, 0x2D, 0x08) # Power Register
    self.bus.write_byte_data(self.address, 0x31, 0x08) # Sell full res 2G mode
  
  def calibrate(self, samples=100):
    print(f"[{self.name}] Calibrating Accelerometer... Hold Still, flat surface.")
    x_vals, y_vals, z_vals = [], [], []
    for _ in range(samples):
        x, y, z = self.read_raw()
        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        time.sleep(0.01)

    avg_x = sum(x_vals) / samples
    avg_y = sum(y_vals) / samples
    avg_z = sum(z_vals) / samples

    # Assume Z points up during calibration; remove 1g worth of counts
    g_raw = self.lsb_scale  # 256 raw units ≈ 1g
    self.offsets = {
        'x': avg_x,
        'y': avg_y,
        'z': avg_z - g_raw
    }
    print(f"[{self.name}] Calibration complete with gravity correction: {self.offsets}")
    self.cal_manager.save_offsets(self.offsets)

  def read_raw(self):
    # Read raw data for each axis (2 bytes each)
    return self.read_raw_data(0x32)

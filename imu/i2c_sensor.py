# A basic class for initializing i2c comms to read from 2-byte registers

import smbus2
import time

class I2CSensor:
  def __init__(self, bus=1, address=None, name='generic'):
    self.address = address
    self.bus = smbus2.SMBus(bus_id)
    self.name = name
    self.offsets = {'x': 0, 'y': 0, 'z': 0}

  def _combine_bytes(self, low, high):
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value

  def read_raw_data(self, address, reg):
    data = self.bus.read_i2c_block_data(self.address, reg, 6)
    x = self._combine_bytes(data[0], data[1])
    y = self._combine_bytes(data[2], data[3])
    z = self._combine_bytes(data[4], data[5])
    return x, y, z

  def calibrate(self, sameples=100):
    print(f'Calibrating {self.name}... Hold Still.')
    x_vals, y_vals, z_vals = [], [], []
    for _ in range(samples):
      x, y, z = self.read_raw_data()
      x_vals.append(x)
      y_vals.append(y)
      z_vals.append(z)
      time.sleep(0.01)
    self.offsets = {
        'x': sum(x_vals)/samples,
        'y': sum(y_vals)/samples,
        'z': sum(z_vals)/samples,
    }
    print(f'Calibration complete for {self.name}: {self.offsets}')

  def apply_offsets(self, x, y, z):
    return x-self.offsets['x'], y-self.offsets['y'], z-self.offsets['z']

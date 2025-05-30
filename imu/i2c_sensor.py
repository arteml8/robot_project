# A basic class for initializing i2c comms to read from 2-byte registers

import smbus2
import time

from imu.calibration_manager import CalibrationManager


class I2CSensor:
  def __init__(self, bus=1, address=None, name='generic', msb_first=False, scale_factor=1):
    self.address = address
    self.bus = smbus2.SMBus(bus)
    self.name = name
    self.msb_first = msb_first
    self.lsb_scale = scale_factor
    self.offsets = {'x': 0, 'y': 0, 'z': 0}
    self.cal_manager = CalibrationManager(self.name)

  def _combine_bytes(self, byte1, byte2):
    high, low = (byte1, byte2) if self.msb_first else (byte2, byte1)
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value

  def _load_offsets(self):
    self.load_or_calibrate()

    if any(self.offsets[axis] == 0 for axis in ('x', 'y', 'z')):
      print(f'WARNING: Sensor {self.name} might not have proper calibration in place.')

  def read_raw_data(self, reg):
    data = self.bus.read_i2c_block_data(self.address, reg, 6)
    x = self._combine_bytes(data[0], data[1])
    y = self._combine_bytes(data[2], data[3])
    z = self._combine_bytes(data[4], data[5])
    return x, y, z

  def calibrate(self, samples=100):
    print(f'Calibrating {self.name}... Hold Still.')
    x_vals, y_vals, z_vals = [], [], []
    for _ in range(samples):
      x, y, z = self.read_raw()
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
    self.cal_manager.save_offsets(self.offsets)

  def load_or_calibrate(self, force=False, tolerance=20):
    existing = self.cal_manager.get_offsets()
    axes = ('x', 'y') if self.name == 'ADXL345' else ('x', 'y', 'z')
    
    if not force:
        # Take a quick sample of 20 readings to compare against stored values
        x_vals, y_vals, z_vals = [], [], []
        samples = 20
        for _ in range(samples):
          x, y, z = self.read_raw()
          x_vals.append(x)
          y_vals.append(y)
          z_vals.append(z)
          time.sleep(0.01)
        x, y, z = sum(x_vals)/samples, sum(y_vals)/samples, sum(z_vals)/samples
        deviation = {
            'x': abs(x - existing['x']),
            'y': abs(y - existing['y']),
            'z': abs(z - existing['z']),
        }
        if all(deviation[axis] < tolerance for axis in axes):
            print(f"Loaded existing calibration for {self.name}: {existing}")
            self.offsets = existing
            return

    # Perform new calibration and save
    self.calibrate()
    self.cal_manager.save_offsets(self.offsets)

  def read_scaled(self):
    x, y, z = self.read_raw()
    return x / self.lsb_scale, y / self.lsb_scale, z / self.lsb_scale

  def read_offset_and_scaled(self):
    x, y, z = self.read_raw()
    x, y, z = x-self.offsets['x'], y-self.offsets['y'], z-self.offsets['z']
    return x / self.lsb_scale, y / self.lsb_scale, z / self.lsb_scale

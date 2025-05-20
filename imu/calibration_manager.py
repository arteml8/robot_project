import json
import os

class CalibrationManager:
  def __init__(self, sensor_name, filename='calibration.json'):
    self.filename = filename
    self.sensor_name = sensor_name
    self.calibration_data = {}

  def save_offsets(self, new_cal_data):
    calibration = {self.sensor_name : new_cal_data}
    with open(self.filename, 'w') as f:
      json.dump(calibration, f, indent=4)

  def get_offsets(self):
    if os.path.exists(self.filename):
      with open(self.filename, 'r') as f:
        self.calibration_data = json.load(f)
    return self.calibration_data.get(self.sensor_name, {'x': 0, 'y': 0, 'z': 0})

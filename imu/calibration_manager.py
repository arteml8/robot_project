import json
import os

class CalibrationManager:
  def __init__(self, filename='calibration.json'):
    self.filename = filename
    self.calibration_data = {}
    
  def load(self):
    if os.path.exists(self.filename):
      with open(self.filename, 'r') as f:
        self.calibration_data = json.load(f)
    return self.calibration_data
  
  def save(self, data):
    self.calibration_data = data
    with open(self.filename, 'w') as f:
      json.dump(self.calibration_data, f, indent=4)
  
  def get_offsets(self, sensor_name):
    return self.calibration_data.get(sensor_name, {'x': 0, 'y': 0, 'z': 0})

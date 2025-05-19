class Calibration:
  def __init__(self, offset=(0,0,0), scale=(1,1,1)):
    self.offset = offset
    self.scale = scale
  
  def apply(self, x, y, z):
    x_corr = (x - self.offset[0]) * self.scale[0]
    y_corr = (y - self.offset[1]) * self.scale[1]
    z_corr = (z - self.offset[2]) * self.scale[2]
    
    return x_corr, y_corr, z_corr

import matplotlib.pyplot as plt
import matplotlib.animation as animation

class LivePlot2D:
  def __init__(self):
    self.fig, self.ax = plt.subplots()
    self.x_data, self.y_data = [], []
    self.line, = self.ax.plot([], [], 'ro')
    self.ax.set_xlim(-200,200)
    self.ax.set_ylim(-200,200)
  
  def update(self, frame):
    x,y = frame
    self.x_data.append(x)
    self.y_data.append(y)
    self.line.set_data(self.x_data,self.y_data)
    return self.line,
    
  def start(self, data_source):
    ani=animation.FuncAnimation(
        self.fig, self.update, data_source, blit=True, interval=100)
    plt.show()

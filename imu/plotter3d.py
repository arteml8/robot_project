import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

class Plotter3D:
  def __init__(self):
    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(111, projection='3d')
    self.scatter = self.ax.scatter([], [], [], c='r', marker='o')
    self.x_data, self.y_data, self.z_data = [], [], []
  
  def update_plot(self, new_data):
    x,y,z = new_data
    self.x_data.append(x)
    self.y_data.append(y)
    self.z_data.append(z)
    
    self.x_data = self.x_data[-100:]
    self.y_data = self.y_data[-100:]
    self.z_data = self.z_data[-100:]
    
    self.ax.clear()
    self.ax.set_xlim([-1.5, 1.5])
    self.ax.set_xlim([-1.5, 1.5])
    self.ax.set_xlim([-1.5, 1.5])
    self.ax.set_xlabel('X')
    self.ax.set_ylabel('Y')
    self.ax.set_zlabel('Z')
    self.ax.scatter(self.x_data, self.y_data, self.z_data, c='r', marker='o')
    
  def animate(self, data_callback):
    ani=animation.FuncAnimation(
        self.fig, lambda i: self.update_plot(data_callback()), interval=100)
    plt.show()

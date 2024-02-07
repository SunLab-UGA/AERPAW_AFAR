import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pickle

# load the pickle file
with open('data.pickle', 'rb') as f:
    data = pickle.load(f)
    f.close()

# access the numpy array in the pickle file called 'ugv_grid'
ugv_grid = data['ugv_grid']
ugv_grid_step = data['ugv_grid_step'] # the step size of the grid (implied square grid)

# ugv_grid is a 1D array (time) of 1D arrays(z) which need to be reshaped to 2D arrays
timesteps = ugv_grid.shape[0]
ugv_grid = np.reshape(ugv_grid, (timesteps, ugv_grid_step, ugv_grid_step))

# create the x and y axes (2D static)
x = np.arange(0, ugv_grid_step)
y = np.arange(0, ugv_grid_step)
xx, yy = np.meshgrid(x, y)

#setup the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d') # add a 3D subplot
plt.title('UGV Grid')

# initialize the surface plot
surf = ax.plot_surface(xx, yy, ugv_grid[0], cmap='plasma', linewidth=0, antialiased=False)

# set the initial view
ax.view_init(30, 30)

# set the initial colorbar
fig.colorbar(surf, shrink=0.5, aspect=5)

# animate the surface plot
def update(frame_number, zarray, plot):
    ax.clear()
    ax.plot_surface(xx, yy, zarray[frame_number], cmap='plasma', linewidth=0, antialiased=False)
    ax.set_title(f'UGV Grid at Time {frame_number}')
    return [ax]

ani = plt.FuncAnimation(fig, update, timesteps, fargs=(ugv_grid, surf), interval=100, blit=False)

plt.show()


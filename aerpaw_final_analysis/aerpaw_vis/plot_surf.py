import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pickle

# load the pickle file
with open('gp.pickle', 'rb') as f:
    data = pickle.load(f)
    f.close()

# list the keys in the pickle file
print(data.keys())

print("number of readings: ", len(data['readings']))
print(data['readings'])
print(data['reading_positions'])

print()

# access the numpy array in the pickle file called 'ugv_grid'
ugv_grid = data['linespace_ugv']
print(ugv_grid)
print(ugv_grid.shape)
ugv_grid_step = data['grid_resolution_ugv'] # the step size of the grid (implied square grid)
print('step size=',ugv_grid_step)

# ugv_grid is the corrdinates of the grid points in the x-y plane, we need to reshape to 2D arrays for plotting
# get all the x coordinates
xx = ugv_grid[:,0]
# get all the y coordinates
yy = ugv_grid[:,1]
# get all the z coordinates
z = data['Z_history'][-1] # these are the most current predictions

std = data['std']

# reshape the arrays to 2D
xx = np.reshape(xx, (ugv_grid_step, ugv_grid_step))
yy = np.reshape(yy, (ugv_grid_step, ugv_grid_step))
z = np.reshape(z, (ugv_grid_step, ugv_grid_step))

# create the figure and plot the surface as a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111)
plt.title('UGV Grid')
surf = ax.pcolormesh(xx, yy, z)
plt.show()


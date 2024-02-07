import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter

import math
import os
import pickle

def plot_reading_positions(ax, limit=500):
    # reduce limit to the number of readings
    if limit > len(positions):
        limit = len(positions)
    # add markers for the readings_positions
    for i in range(0, limit):
        ax.scatter(positions[i][1], positions[i][0], marker='x', color='red')
    # add a marker for the rover(ugv) position
    # ax.scatter(-78.6985370, 35.7297144, marker='o', color='yellow') # run1
    # ax.scatter(-78.6983391, 35.7281357, marker='o', color='yellow') # run2
    # ax.scatter(-78.698694, 35.729065, marker='o', color='yellow') # run8
    # ax.scatter(-78.6997489, 35.7278146, marker='o', color='yellow') # run9
    ax.scatter(rover_location[0], rover_location[1], marker='o', color='yellow') # readme.txt

def plot_optimizer_history(ax, index=0):
    for i in range(0, len(optimizer_history)):
        ax.scatter(optimizer_history[i][1], optimizer_history[i][0], marker='*', color='green')

# the folder where the pickle file is located
# automagically search for the highest numbered folder within 'pickles/' directory
parent_folder = 'pickles/'
for i in range(0, 100): # check up to 100 folders
    if os.path.isdir(parent_folder + 'run' + str(i)):
        folder = parent_folder + 'run' + str(i) + '/'

# override the automagic folder selection
# folder = 'pickles/run52/'
# folder = 'pickles/2023-12-04_14-57-49/' # final run 1 (testbed) doesn't seem valid
# folder = 'pickles/2023-12-13_12-45-02/' # final run 1 (testbed)
folder = 'pickles/2023-12-13_16-27-20/' # final run 3 (testbed)

print('folder: ', folder)
filename = 'gp.pickle'
path = folder + filename

# read the readme.txt file
with open(folder + 'readme.txt', 'r') as f:
    rover_location = f.read()
    f.close()
print(f'rover location: {rover_location}')
# convert the readme string to a list of floats
rover_location = rover_location.split(',')
rover_location = [float(i) for i in rover_location] # lon, lat

def distance(a,b): # a and b are gps coordinates (lat,lon) [0:1]
    d2r = math.pi / 180
    dlon = (b[1] - a[1]) * d2r
    dlat = (b[0] - a[0]) * d2r
    a = math.pow(math.sin(dlat / 2), 2) + math.cos(a[0] * d2r) * math.cos(
                    b[0] * d2r) * math.pow(math.sin(dlon / 2), 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = 6367 * c
    return math.hypot(d * 1000, 0) # other.alt - self.alt = 0


# load the pickle file
with open(path, 'rb') as f:
    data = pickle.load(f)
    f.close()

# list the keys in the pickle file
print(data.keys())

# print the final kernel parameters
print('final kernel parameters')
print(data['kernel'])
      
timesteps = data['num_readings']
positions = np.array(data['reading_positions'])
optimizer_history = np.array(data['optimizer_history']) # this is the history of the optimizer locations



if 'peak_prediction_coords_history' in data:
    prediction = np.array(data['peak_prediction_coords_history'])
    print('prediction',len(prediction))
    print('first',prediction[0])
    print('last',prediction[-1])

if 'refined_peak_prediction_coords_history' in data:
    refined_prediction = np.array(data['refined_peak_prediction_coords_history'])
    print('refined_prediction',len(refined_prediction))
    print('first',refined_prediction[0])
    print('last',refined_prediction[-1])

if 'small_linespace_history' in data:
    small_linespace_history = np.array(data['small_linespace_history'])
    print('small_linespace_history',len(small_linespace_history))

if 'log_marginal_likelihood' in data:
    log_marginal_likelihood = np.array(data['log_marginal_likelihood'])


start_rover_distance = distance(positions[0][:], rover_location[::-1])
print(f'start rover distance: {start_rover_distance} meters')
if 'peak_prediction_coords_history' in data:
    # find the distance between the rover and the last reading make sure to flip the order of the coordinates for the (rover location)!
    rover_distance = distance(positions[-1][:], rover_location[::-1])
    print(f'final rover distance: {rover_distance} meters')


# print how far the guess was from the actual rover location
if 'peak_prediction_coords_history' in data:
    guess_distance = distance(prediction[-1][:], rover_location[::-1])
    print(f'final guess was {guess_distance} meters away from the actual rover location')

if 'refined_peak_prediction_coords_history' in data:  
    refined_rover_distance = distance(refined_prediction[-1][:], rover_location[::-1])
    print(f'finaler rover distance: {refined_rover_distance} meters')

print()

# print('opt hist',len(optimizer_history))
print('readings ',positions.shape)
# print(positions)

# test distance function to decimal place (5 digit precision ~= 1 meter)
# test_a = [35.730350, -78.699_850]
# test_b = [35.730350, -78.699_860]
# print(f'test distance (lon): {distance(test_a, test_b)} meters')

# test_a = [35.730350, -78.699_850]
# test_b = [35.730360, -78.699_850]
# print(f'test distance (lat): {distance(test_a, test_b)} meters')


# print(positions.shape)
# print(positions)
# print("number of readings: ", len(data['readings']))
# print('readings')
# print(data['readings'])
# print('positions')

print()

uav_grid = data['linespace_uav']
uav_grid_step = data['grid_resolution_uav']

aa = uav_grid[:,0]
bb = uav_grid[:,1]
cc = data['Z_history_uav'][-1] # last prediction
#cc = np.zeros(len(aa))-20

aa = np.reshape(aa, (uav_grid_step, uav_grid_step))
bb = np.reshape(bb, (uav_grid_step, uav_grid_step))
cc = np.reshape(cc, (uav_grid_step, uav_grid_step))

# access the numpy array in the pickle file called 'ugv_grid'
ugv_grid = data['linespace_ugv']
# print(ugv_grid)
# print(ugv_grid.shape)
ugv_grid_step = data['grid_resolution_ugv'] # the step size of the grid (implied square grid)
# print('step size=',ugv_grid_step)

# ugv_grid is the corrdinates of the grid points in the x-y plane, we need to reshape to 2D arrays for plotting
# get all the linespace coordinates
xx = ugv_grid[:,0]
yy = ugv_grid[:,1]
# get all the z coordinates
z = data['Z_history'][-1] # last prediction
# z = np.zeros(len(xx))+50

std = data['std']

# reshape the arrays to 2D
xx = np.reshape(xx, (ugv_grid_step, ugv_grid_step)) # xx is lat
yy = np.reshape(yy, (ugv_grid_step, ugv_grid_step)) # yy is lon
z = np.reshape(z, (ugv_grid_step, ugv_grid_step))

# create the figure and plot the surface
fig = plt.figure()
# change the windw title
fig.canvas.set_window_title(f'{folder}')
ax = fig.add_subplot(111)
plt.title(f'UGV Grid {folder}')
# change the decimal precision of the axis labels
ax.xaxis.set_major_formatter('{x:.5f}')
ax.yaxis.set_major_formatter('{x:.5f}')

surf = ax.pcolormesh(yy, xx, z, alpha=1)
surf2 = ax.pcolormesh(bb, aa, cc, alpha=0.5)
plot_reading_positions(ax)
plot_optimizer_history(ax)


# set a global axis limit
ax.set_xlim(-78.699850, -78.696150)
ax.set_ylim(35.726825, 35.730350)

# make sure the figure is square
fig.set_size_inches(8, 8)


def update(frame_number, zarray):
    # print('frame number ',frame_number)
    ax.clear()
    z = zarray[frame_number]
    z = np.reshape(z, (ugv_grid_step, ugv_grid_step))

    cc = data['Z_history_uav'][frame_number]
    cc = np.reshape(cc, (uav_grid_step, uav_grid_step))

    surf = ax.pcolormesh(yy, xx, z, alpha=1)
    surf2 = ax.pcolormesh(bb, aa, cc, alpha=0.5)

    ax.set_title(f'UGV Grid at Time {frame_number} of ({timesteps})')
    if 'log_marginal_likelihood' in data:
        ax.set_title(f'UGV Grid at Time {frame_number} of ({timesteps}) lml: {log_marginal_likelihood[frame_number]:.2f}')
    # change the decimal precision of the axis labels
    ax.xaxis.set_major_formatter('{x:.5f}')
    ax.yaxis.set_major_formatter('{x:.5f}')

    ax.set_xlim(min(positions[:,0]), max(positions[:,0]))
    ax.set_ylim(min(positions[:,1]), max(positions[:,1]))

    plot_reading_positions(ax, frame_number)   

    # add markers for the readings_positions
    # ax.scatter(positions[frame_number][0], positions[frame_number][1], marker='x', color='red')
    num_init_waypoints = 4

    # dont show the optimizer history if there are less optimizer points than predicted points
    if len(optimizer_history) == len(prediction):
        if frame_number >= num_init_waypoints: # optimizer history is delayed by [num_init_waypoints] frames
            ax.scatter(optimizer_history[frame_number-num_init_waypoints][1], optimizer_history[frame_number-num_init_waypoints][0], marker='*', color='green')

    # # add a guess marker for the rover(ugv) position
    # if 'peak_prediction_coords_history' in data:
    #     ax.scatter(prediction[frame_number][1], prediction[frame_number][0], marker='p', color='blue')

    # add a refined guess marker for the rover(ugv) position
    if 'refined_peak_prediction_coords_history' in data:
        ax.scatter(refined_prediction[frame_number][1], refined_prediction[frame_number][0], marker='o', color='purple')

    # add a marker for each point in the small linespace
    if 'small_linespace_history' in data:
        for i in range(0, len(small_linespace_history[frame_number])):
            if i == 1:
                # ax.scatter(small_linespace_history[frame_number][i][1], small_linespace_history[frame_number][i][0], marker='.', color='black', s=1)
                ax.scatter(small_linespace_history[frame_number][i][1], small_linespace_history[frame_number][i][0], marker='.', color='orange', s=1)
            elif i == len(small_linespace_history[frame_number])-1:
                # ax.scatter(small_linespace_history[frame_number][i][1], small_linespace_history[frame_number][i][0], marker='.', color='red', s=1)
                ax.scatter(small_linespace_history[frame_number][i][1], small_linespace_history[frame_number][i][0], marker='.', color='orange', s=1)
            else:
                ax.scatter(small_linespace_history[frame_number][i][1], small_linespace_history[frame_number][i][0], marker='.', color='orange', s=1)


    # set a global axis limit
    ax.set_xlim(-78.699850, -78.696150)
    ax.set_ylim(35.726825, 35.730350)
    # set the pcolormesh scale
    # surf.set_clim(-50, 50)

    return [ax]
# hold the first frame for 5 seconds
ani = FuncAnimation(fig, update, timesteps, fargs=(data['Z_history'],), interval=800, blit=False, repeat=False)

# save the animation as a gif
# writer = PillowWriter(fps=0.5, bitrate=1800)
writer = PillowWriter(fps=1, bitrate=1800)
ani.save(f'plot_surf_animated_w_rover.gif', writer=writer, dpi=200)
print('gif saved')
# plt.show()

# plot the log marginal likelihood
# if 'log_marginal_likelihood' in data:
#     fig = plt.figure()
#     ax = fig.add_subplot(111)
#     plt.title('Log Marginal Likelihood')
#     ax.set_xlim(0, timesteps)
#     ax.set_ylim(min(log_marginal_likelihood), max(log_marginal_likelihood))
#     ax.plot(log_marginal_likelihood)
#     plt.show()


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

import plotly.express as px
import plotly.graph_objects as go

import pickle
import os

# the folder where the pickle file is located
# search for the highest numbered folder within 'pickles/' directory
parent_folder = 'pickles/'
for i in range(0, 100): # check up to 100 folders
    if os.path.isdir(parent_folder + 'run' + str(i)):
        folder = parent_folder + 'run' + str(i) + '/'

# override automagic folder selection
# folder = 'pickles/run26/'
folder = 'pickles/2023-12-04_14-57-49/' # final run 1 (testbed) #discarded
folder = 'pickles/2023-12-13_12-45-02/' # final run 1 (testbed)
folder = 'pickles/2023-12-13_16-27-20/' # final run 3 (testbed)

print('folder: ', folder)

filename = 'data.pickle'
path = folder + filename

print('path: ', path)

# load the pickle file
with open(path, 'rb') as f:
    data = pickle.load(f)
    f.close()


# print the length of the data
print("len(data): ", len(data))

# list the keys in the pickle file
# d datapoints in m measurements [m][d]
print(dict(data[0]).keys()) # data is not nested because it is continuous

# pull all the channels measurements into a list (also quality, intensity, etc.)
channel = []
quality = []
position = []
heading = []
moving = []
battery = []
velocity = []
gps_status = []

# keep track of the time index of the end of each measurement
radio_time = []
gps_time = []


# print the first message in the data
print(data[0])

# itterate through the data and pull out the values into lists for plotting
for i in range(0, len(data)): # for each measurement
    channel.append(data[i]['Channel'])
    quality.append(data[i]['Quality'])
    position.append(data[i]['Position']) if 'Position' in data[i] else position.append(0)
    heading.append(data[i]['Heading']) if 'Heading' in data[i] else heading.append(0)
    moving.append(int(data[i]['Moving'])) if 'Moving' in data[i] else moving.append(-1)
    battery.append(data[i]['Battery']) if 'Battery' in data[i] else battery.append(0)
    velocity.append(data[i]['Velocity']) if 'Velocity' in data[i] else velocity.append(0)
    gps_status.append(data[i]['GPS_status']) if 'GPS_status' in data[i] else gps_status.append(0)
    radio_time.append(data[i]['radio_timestamp'])
    gps_time.append(data[i]['gps_timestamp'])




# print len channel
print("num of datapoints: ",len(channel))
print("num of measurements: ",len(data))
#===============================================================================
# plot with plotly, add title and axis labels
fig = go.Figure()

fig.update_layout(title=f'rover measurements FULL {folder}', xaxis_title='datapoint', yaxis_title='A.U.')

# fig = px.line(x=range(0, len(channel)), y=channel, title='rover measurements')
# add a scatter plot for channel
fig.add_scatter(y=channel, mode='lines', name='channel')

# add another plot for quality onto the same figure
fig.add_scatter(y=quality, mode='lines', name='quality')

# fig.add_scatter(y=heading, mode='lines', name='heading')
# fig.add_scatter(y=moving, mode='lines', name='moving')

# fig.add_scatter(y=battery, mode='lines', name='battery')
# fig.add_scatter(y=velocity, mode='lines', name='velocity')
# fig.add_scatter(y=gps_status, mode='lines', name='gps_status')

fig.show()

# save the html file to the same folder as the pickle file
fig.write_html(folder + 'plotly_full.html')
print('saved plotly_full.html to ', folder)

#===============================================================================
# # calculate the CDF of the quality varience
# # sort the quality varience
# quality_varience.sort()
# # calculate the CDF
# q_var_cdf = np.cumsum(quality_varience)
# # normalize the CDF
# # q_var_cdf = q_var_cdf / q_var_cdf[-1]

# # plot the CDF
# fig = px.line(x=range(0, len(q_var_cdf)), y=q_var_cdf, title='quality varience CDF')

# # plot a virtical line at 10% of the total number of datapoints
# fig.add_vline(x=int(len(q_var_cdf)*0.1), line_width=1, line_dash="dash", line_color="green")
# fig.show()


#===============================================================================
# plot the time measurements
# fig2 = go.Figure()
# fig2.update_layout(title=f'rover measurements {folder}', xaxis_title='datapoint', yaxis_title='time')
# fig2.add_scatter(y=radio_time, mode='lines', name='radio time')
# fig2.add_scatter(y=gps_time, mode='lines', name='gps time')
# # plot a virtical line at the end of each measurement
# for i in range(0, len(vline_index)):
#     fig2.add_vline(x=vline_index[i], line_width=1, line_dash="dash", line_color="green")

# fig2.show()









# OLD CODE
# #===============================================================================
# #===============================================================================
# # plot intensity
# fig = px.line(x=range(0, len(intensity)), y=intensity, title='rover measurements (intensity)')
# # plot a virtical line at the end of each measurement
# for i in range(0, len(vline_index)):
#     fig.add_vline(x=vline_index[i], line_width=1, line_dash="dash", line_color="green")
# fig.show()
# #===============================================================================
# # plot phase
# fig = px.line(x=range(0, len(phase)), y=phase, title='rover measurements (phase)')
# # plot a virtical line at the end of each measurement
# for i in range(0, len(vline_index)):
#     fig.add_vline(x=vline_index[i], line_width=1, line_dash="dash", line_color="green")
# fig.show()
# #===============================================================================
# # plot phase offset
# fig = px.line(x=range(0, len(phase_offset)), y=phase_offset, title='rover measurements (phase offset)')
# # plot a virtical line at the end of each measurement
# for i in range(0, len(vline_index)):
#     fig.add_vline(x=vline_index[i], line_width=1, line_dash="dash", line_color="green")
# fig.show()


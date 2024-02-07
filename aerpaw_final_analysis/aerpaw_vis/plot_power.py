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

filename = 'm_data.pickle'
path = folder + filename

# load the pickle file
with open(path, 'rb') as f:
    data = pickle.load(f)
    f.close()

# list the keys in the pickle file
# d datapoints in m measurements [m][d]
print(dict(data[0][0]).keys())

# pull all the channels measurements into a list (also quality, intensity, etc.)
channel = []
quality = []
intensity = []
phase = []
phase_offset = []
heading = []
moving = []
matrix = []

# keep track of the time index of the end of each measurement
radio_time = []
gps_time = []


vline_index = []
vline_counter = 0
for i in range(0, len(data)): # for each measurement
    # get the x axis offset for this measurement
    vline_index.append(vline_counter)
    for j in range(0, len(data[i])): # for each datapoint
        channel.append(data[i][j]['Channel'])
        quality.append(data[i][j]['Quality'])
        intensity.append(data[i][j]['Intensity']) if 'Intensity' in data[i][j] else intensity.append(0)
        phase.append(data[i][j]['Phase']) if 'Phase' in data[i][j] else phase.append(0)
        phase_offset.append(data[i][j]['PhaseOffset']) if 'PhaseOffset' in data[i][j] else phase_offset.append(0)
        heading.append(data[i][j]['heading']) if 'heading' in data[i][j] else heading.append(0)
        moving.append(int(data[i][j]['moving'])) if 'moving' in data[i][j] else moving.append(-1)

        # time index
        radio_time.append(data[i][j]['radio_timestamp'])
        gps_time.append(data[i][j]['gps_timestamp'])

        vline_counter += 1

# go through the measurements using vline_index to reconstruct the varience of the measurements, padding the same value for each measurement
channel_varience = []
quality_varience = []
start = 0
for i in range(0, len(vline_index)): # for each measurement
    end = vline_index[i]
    for m in range(start, end):
        channel_varience.append(np.var(channel[start:end]))
        quality_varience.append(np.var(quality[start:end]))
    start = end




# print len channel
print("num of datapoints: ",len(channel))
print("num of measurements: ",len(data))
#===============================================================================
# plot with plotly, add title and axis labels
fig = go.Figure()
fig.update_layout(title=f'rover measurements {folder}', xaxis_title='datapoint', yaxis_title='A.U.')

# fig = px.line(x=range(0, len(channel)), y=channel, title='rover measurements')
# add a scatter plot for channel
fig.add_scatter(y=channel, mode='lines', name='channel')

# plot a virtical line at the end of each measurement
for i in range(0, len(vline_index)):
    fig.add_vline(x=vline_index[i], line_width=1, line_dash="dash", line_color="green")

# add another plot for quality onto the same figure
fig.add_scatter(y=quality, mode='lines', name='quality')

# add another plot for intensity etc onto the same figure
fig.add_scatter(y=intensity, mode='lines', name='intensity')
fig.add_scatter(y=phase, mode='lines', name='phase')
fig.add_scatter(y=phase_offset, mode='lines', name='phase offset')
fig.add_scatter(y=heading, mode='lines', name='heading')
fig.add_scatter(y=moving, mode='lines', name='moving')

# add another plot for varience
fig.add_scatter(y=channel_varience, mode='lines', name='channel varience')
fig.add_scatter(y=quality_varience, mode='lines', name='quality varience')
fig.show()

# save the html file to the same folder as the pickle file
fig.write_html(folder + 'plotly.html')
print('saved plotly.html to ', folder)

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


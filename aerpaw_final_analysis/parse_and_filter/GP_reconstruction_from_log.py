# used to parse the vehicle logs into relavent data which can be used to reconstruct the GP from the drone logs
# specifically for the 2nd flight test

from datetime import datetime
import os
import numpy as np

# load the data from the file
path = 'logs'

# hard code the file name
# file_name_prefix = '2023-11-09_19_27_30' # sim results ?
# file_name_prefix = '2023-11-09_21_03_36' # sim results ?

# file_name_prefix = '2023-12-04_14_45_44' # run 1 (testbed) discarded
# file_name_prefix = '2023-12-13_12_33_02' # run 1 (testbed) 
file_name_prefix = '2023-12-13_14_28_47' # run 2 (testbed)
# file_name_prefix = '2023-12-13_16_15_36' # run 3 (testbed)


file_suffix = '_vehicle_log.txt'
file_name = file_name_prefix + file_suffix

print('file_name: \t\t', file_name)

# Load Data, set the date format, and parse the data into 3 lists (time, D, and Power)
date_format = '%Y-%m-%d %H:%M:%S.%f'

# print the full path to the file and check if it exists
if file_name is not None:
    file_path = f'{path}//{file_name}'
    print("looking for:", file_path)
    if os.path.exists(file_path):
        print("file exists")
    else:
        # error out if the file does not exist
        raise FileNotFoundError(f'file: {file_path} does not exist')

data = [] # list of lines which we will store a dict of the parsed line

# open the file and read the lines
with open(file_path, 'r') as file:
    for line in file:
        # parse the data from the line by taking the first 28 characters as datetime
        date = datetime.strptime(line[1:27], date_format)
        # print(date)
        _data = line[29:] # one space skipped after the date
        # add the data to a dict
        data.append({'date': date, 'data': _data})

# go through the data and add to the dictionary the relavent data based on keywords
keywords = ["++Quality varience is too high, do not use the data++",
            "Filtered data (avg):"
            ]

def parse_line_to_key_value_pairs(line):
    # Remove curly brackets and quotations
    clean_line = line.replace("{", "").replace("}", "").replace("'", "").replace('"', '')
    
    # Split the line into key-value pairs
    parts = clean_line.split(",")
    
    # Initialize a dictionary to hold the parsed key-value pairs
    result = {}
    for part in parts:
        # Split each part into key and value
        key, value = part.split(":", 1)
        key = key.strip()
        try:
            # Attempt to convert numeric values
            result[key] = float(value.strip())
        except ValueError:
            # Keep as string if conversion fails
            print(f'failed to convert {value} to float')
            result[key] = value.strip() 
    return result

for i, line in enumerate(data):
    for keyword in keywords:
        if keyword in line['data']:
            print(f'keyword: {keyword} found in line {i}')
            if keyword == keywords[1]: # Filtered data (avg)
                # add the line to the dict
                data[i]['Filtered data'] = True
                # parse the data (remove the keyword)
                _line = line['data'][21:]
                # parse the line into key-value pairs
                parsed_dict = parse_line_to_key_value_pairs(_line)
                # add the parsed data to the dict
                data[i].update(parsed_dict)
            if keyword == keywords[0]: # Quality varience is too high
                # add the line to the dict
                data[i]['Quality varience'] = True
                # add a discard flag to the last filtered data point
                # get the last filtered data point
                for j in range(i, -1, -1):
                    if 'Filtered data' in data[j]:
                        data[j]['discard'] = True
                        print(f'discarding data point {j}')
                        break

# find the number of filtered data points
filtered_data = [line for line in data if 'Filtered data' in line]
print(f'number of filtered data points: {len(filtered_data)}')

# find the number of discarded data points
discarded_data = [line for line in data if 'discard' in line]
print(f'number of discarded data points: {len(discarded_data)}')


# now we can reconstruct a simple GP from the filtered data
# construct a custom GP class
import numpy as np
import random
from itertools import product
from typing import List

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
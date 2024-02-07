# go through all the results and compile the runs into a single file

import os
import math
import json

write_to_file = False # set to true to write the interrim results to a file
write_json = True # set to true to write the final results to a json file
        
# hard coded true values
true_locations = [
                    [35.72806709, -78.69730398],
                    [35.72911779, -78.69918128],
                    [35.72985129, -78.69711002]]


# make an array of all the files that end in "_vehicle_log.txt" in the directory "Results_FINAL"
# this is the directory that contains all the results from the runs
vehicle_log_paths = []
for file in os.listdir("Results_FINAL"):
    if file.endswith("_vehicle_log.txt"):
        vehicle_log_paths.append(os.path.join("Results_FINAL", file))

print("length of vehicle_log_paths:", len(vehicle_log_paths))
print(vehicle_log_paths)

# create a dict to track the results back to each run or file
main_dict = {}
# add the paths to the dict
for ii, vehicle_log_path in enumerate(vehicle_log_paths):
    main_dict[ii] = {"path":vehicle_log_path}


_10_min_logs = []
_3_min_logs = []

def parse_for_results(vehicle_log_path):
    # open the file
    vehicle_log_file = open(vehicle_log_path, "r")

    # read the file
    vehicle_log_file_contents = vehicle_log_file.read()

    # close the file
    vehicle_log_file.close()

    # split the file into lines
    vehicle_log_file_lines = vehicle_log_file_contents.split("\n")

    found_10_min_results = False
    found_3_min_results = False
    # find the line above indicating the start of the results
    for line in vehicle_log_file_lines:
        # if the line contains the string "=====HERE IS THE 10 MIN REPORTED POSITION====="
        if "=====HERE IS THE 10 MIN REPORTED POSITION=====" in line:
            _10_min_line_index = vehicle_log_file_lines.index(line) +1 # the next line is the results
            print("10 min line index:", _10_min_line_index)
            found_10_min_results = True
        
        # if the line contains the string "=====HERE IS THE 3 MIN REPORTED POSITION====="
        if "=====HERE IS THE 3 MIN REPORTED POSITION=====" in line:
            _3_min_line_index = vehicle_log_file_lines.index(line) +1 # the next line is the results
            print("3 min line index:", _3_min_line_index)
            found_3_min_results = True
    if not found_10_min_results:
        print("ERROR: 10 min results not found in:", vehicle_log_path)
        return "incomplete", "incomplete"
    if not found_3_min_results:
        print("ERROR: 3 min results not found in:", vehicle_log_path)
        return "incomplete", "incomplete"
    return vehicle_log_file_lines[_10_min_line_index], vehicle_log_file_lines[_3_min_line_index]

# go through all the files and parse for the results
for vehicle_log_path in vehicle_log_paths:
    print("parsing for results in:", vehicle_log_path)
    _10_min_log, _3_min_log = parse_for_results(vehicle_log_path)
    _10_min_logs.append(_10_min_log)
    _3_min_logs.append(_3_min_log)

    # add the results to the dict
    main_dict[vehicle_log_paths.index(vehicle_log_path)]["10_min_log"] = _10_min_log
    main_dict[vehicle_log_paths.index(vehicle_log_path)]["3_min_log"] = _3_min_log

# print(main_dict)

if write_to_file:
    # write the results to a file
    _10_min_results_file = open("Results_FINAL/10_min_results.txt", "w")
    _10_min_results_file.write("\n".join(_10_min_logs))
    _10_min_results_file.close()

    _3_min_results_file = open("Results_FINAL/3_min_results.txt", "w")
    _3_min_results_file.write("\n".join(_3_min_logs))
    _3_min_results_file.close()
    
# def parse the lines for the results
def parse_to_lat_lon(line):
    # if the line contains "incomplete" then return ["incomplete","incomplete"] for lat and lon
    if "incomplete" in line:
        return ["incomplete","incomplete"]
    # extract the lat and lon from the line example below:
    # [2023-11-09 19:30:34.519973] Refined reported position: {"lat": 35.72806107957245, "lon": -78.69665002283071, "alt": 0}
    # grab everything within the curly braces
    dict = line.split("{")[1].split("}")[0]
    # change the string to a dictionary
    dict = eval("{" + dict + "}")
    # return the lat and lon
    return [dict["lat"], dict["lon"]]

# print("10 min results:")
# parse the 10 min results
_10_min_results = []
for line in _10_min_logs:
    _10_min_results.append(parse_to_lat_lon(line))
# print(_10_min_results)

# print("3 min results:")
# parse the 3 min results
_3_min_results = []
for line in _3_min_logs:
    _3_min_results.append(parse_to_lat_lon(line))
# print(_3_min_results)

# add the results to the dict
for ii, result in enumerate(_10_min_results):
    main_dict[ii]["10_min_result"] = result
for ii, result in enumerate(_3_min_results):
    main_dict[ii]["3_min_result"] = result

# calculate the error for the 10 min results and the 3 min results accross all the runs
def distance(a,b): # a and b are gps coordinates (lat,lon) [0:1]
    # check if either a or b is "incomplete"
    if "incomplete" in a or "incomplete" in b:
        return "incomplete"
    # calculate the distance between a and b
    d2r = math.pi / 180
    dlon = (b[1] - a[1]) * d2r
    dlat = (b[0] - a[0]) * d2r
    a = math.pow(math.sin(dlat / 2), 2) + math.cos(a[0] * d2r) * math.cos(
                    b[0] * d2r) * math.pow(math.sin(dlon / 2), 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = 6367 * c
    return math.hypot(d * 1000, 0) # other.alt - self.alt = 0
    


# calculate the distance for each result to a true location
for ii, result in enumerate(_10_min_results):
    for jj, true_location in enumerate(true_locations):
        main_dict[ii]["10_min_distance_to_true_location_" + str(jj)] = distance(result, true_location)

for ii, result in enumerate(_3_min_results):
    for jj, true_location in enumerate(true_locations):
        main_dict[ii]["3_min_distance_to_true_location_" + str(jj)] = distance(result, true_location)

# find the min distance for each result
for ii, result in enumerate(_10_min_results):
    main_dict[ii]["10_min_min_distance_to_true_location"] = min(main_dict[ii]["10_min_distance_to_true_location_0"], main_dict[ii]["10_min_distance_to_true_location_1"], main_dict[ii]["10_min_distance_to_true_location_2"])

for ii, result in enumerate(_3_min_results):
    main_dict[ii]["3_min_min_distance_to_true_location"] = min(main_dict[ii]["3_min_distance_to_true_location_0"], main_dict[ii]["3_min_distance_to_true_location_1"], main_dict[ii]["3_min_distance_to_true_location_2"])

print(main_dict)

# write the results to a file as a human readable json in the Results_FINAL directory as run_results.txt
if write_json:
    with open("Results_FINAL/run_results.txt", 'w') as outfile:
        json.dump(main_dict, outfile, indent=4)

exit()

# # remove the incomplete results
# _10_min_results = [result for result in _10_min_results if "incomplete" not in result]
# _3_min_results = [result for result in _3_min_results if "incomplete" not in result]
# print(_10_min_results)
# print(_3_min_results)

# # take each result and calculate the distance from each true location
# _10_min_distances = []
# for result in _10_min_results:
#     from_true_location = []
#     for true_location in true_locations:
#         from_true_location.append(distance(true_location, result))
#     _10_min_distances.append(from_true_location)
# print("10 min distances:")
# print(_10_min_distances)
# # take the min distance for each result
# _10_min_min_distances = []
# for distance in _10_min_distances:
#     _10_min_min_distances.append(min(distance))
# print("10 min min distances:")
# print(_10_min_min_distances)

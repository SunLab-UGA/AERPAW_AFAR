# Paul Kudyba, OCT-2023

print("Starting rover search script")

import asyncio
import math
import datetime
import csv
import os

from typing import List
from struct import unpack
from argparse import ArgumentParser

from aerpawlib.runner import StateMachine
from aerpawlib.vehicle import Vehicle, Drone
from aerpawlib.runner import state, timed_state, background, at_init
from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.safetyChecker import SafetyCheckerClient

import numpy as np

print("Loading other libraries")

from init import TrialInit
print("Imported init.py")
from data_manager import DataManager, DataPoint
print("Imported data_manager.py")
from GP import ProcessFunction
print("Imported GP.py")

# Instatiate the trial init (holds constants, waypoints, and starting values)
INIT = TrialInit()

# print(f"NOTE: This adds a {INIT.db_offset} db offset to the RSSI values before predicting, the subtraction is left to the user")

#CONSTANTS (defined in init.py)
FOCUS_HEADING = INIT.NORTHWEST
SEARCH_ALTITUDE = INIT.SEARCH_ALTITUDE

MEASUREMENT_SPAN = INIT.MEASUREMENT_SPAN # max distance between measurement points
MIN_MEASUREMENT_SPAN = INIT.MIN_MEASUREMENT_SPAN # min distance between measurement points (used to prevent stagnation)
MEASUREMENT_SPAN_AFTER_3_MIN = INIT.MEASUREMENT_SPAN_AFTER_3_MIN # max distance between measurement points after 3 minutes


# HARD CODED THRESHOLDS


# DEBUGGING
DEBUG_DATA_MANAGER = False

# starting kernel for GP
kernel = INIT.kernel
# Initialize the GP 
# this is the process function that takes in data and returns a rover position estimate, and waypoint
GP = ProcessFunction(kernel=kernel)
# print("Default Linespace")
# GP.print_linespace(linespace_id="UGV") # print the default linespace

class RoverSearch(StateMachine):
    """State machine for rover search script"""

    def initialize_args(self, extra_args: List[str]):
        """Parse arguments passed to vehicle script"""
        # Default output CSV file for search data
        defaultFile = "ROVER_SEARCH_DATA_%s.csv" % datetime.datetime.now().strftime(
            "%Y-%m-%d_%H:%M:%S"
        )

        parser = ArgumentParser()
        parser.add_argument("--safety_checker_ip", help="ip of the safety checker server")
        parser.add_argument("--safety_checker_port", help="port of the safety checker server")
        parser.add_argument(
            "--log",
            help="File to record kml of search path and RSSI values",
            required=False,
            default=defaultFile,
        )
        parser.add_argument(
            "--save_csv",
            help="Whether to save a kml file with the search path and RSSI values",
            default=False, #True
        )
        parser.add_argument(
            "--search_time",
            help="How long in minutes to search for the rover",
            required=False,
            default=10,
            type=int,
        )

        args = parser.parse_args(args=extra_args)

        self.safety_checker = SafetyCheckerClient(args.safety_checker_ip, args.safety_checker_port)
        # self.log_file = open(args.log, "w+")
        # self.save_csv = args.save_csv
        self.max_search_time = datetime.timedelta(minutes=args.search_time)

        # Create a CSV writer object if we are saving data
        # if self.save_csv:
        #     self.csv_writer = csv.writer(self.log_file)
        #     self.csv_writer.writerow(["longitude", "latitude", "altitude", "RSSI"])
    
    def check_end(self):
        """Check if the search is over"""
        if datetime.datetime.now() - self.start_time > self.max_search_time:
            return True
        else:
            return False
        
    def create_aux_path(self, center: Coordinate):
        '''create an aux path hopefully around the rover'''
        waypoints = GP.aux_path(center=center, radius=self.AUXILIARY_PATH_SPAN, num_points=18)
        for w in waypoints:
            # ensure they have the correct altitude
            w.alt = SEARCH_ALTITUDE
            INIT.waypoints.append(w)
        # ensure aux is only used once
        self.aux_computed = True
    
    ########################################################################################### AT_INIT
    @at_init
    async def at_init(self, vehicle: Vehicle):
        print("Initializing rover search script (at_init)")
        print("rover_search PID IS: ", os.getpid())
        # Initialize the data manager 
        # this spawns a process which keeps all data without blocking the main thread
        # however, should be polled occasionally within @background so that DM maintains the latest data
        # DM then with the latest data can be used to update the GP (it sanitizes the data)
        self.DM = DataManager()
        # Start the data manager
        self.DM.start()
        self.valid_measurement = False # flag for if the measurement is to be filtered and used in the GP

        # global variables for reporting
        self.min_3_reported = False
        self.min_10_reported = False
        self.mission_started = False

        self.highest_power = -1000 # used to track the highest power measurement
        self.highest_power_location = None # used to track the location of the highest power measurement

        self.last_waypoint = INIT.waypoints[-1] # the last waypoint in the list

        self.quality_varience_cuttoff = INIT.QUALITY_VARIENCE_CUTTOFF

        self.AUXILIARY_PATH_TRIGGER = INIT.AUXILIARY_PATH_TRIGGER # distance from rover to trigger aux path
        self.AUXILIARY_PATH_SPAN = INIT.AUXILIARY_PATH_SPAN # radius of aux path
        self.aux_computed = False # flag for if the aux path has been computed

        self.DROPPED_MEASUREMENT_LIMIT_AUX = INIT.DROPPED_MEASUREMENT_LIMIT_AUX # number of dropped measurements before switching to aux path

        # GP waypoint optimizer
        self.epsilon = INIT.epsilon
        self.epsilon_decay = INIT.epsilon_decay
        self.op_method = INIT.op_method

        # stats tracking
        self.dropped_measurements = 0 # number of measurements dropped due to quality varience
        self.consecutive_dropped_measurements = 0
        self.consecutive_nudges = 0
        self.total_nudges = 0

        self.Q_var_history = [] # history of the quality varience
        self.num_valid_measurements = 0 # number of valid measurements

        self.settle_count_limit = INIT.settle_count_limit # the number of times that a guess is within the settle range, triggers aux path
        self.settle_range = INIT.settle_range #(m) the range that a guess must be within to count as a settled guess
        self.settle_counter = 0

        # print the linespace distances of the GP (print_linespace_grid_resolution)
        print("GP LINESPACE RESOLUTION")
        GP.print_linespace_grid_resolution(linespace_id='UGV')
        GP.print_linespace_grid_resolution(linespace_id='UAV')

        # print the time
        print(f"Current time is {datetime.datetime.now().strftime('%H:%M:%S')}")
        print("Done initializing rover search script (at_init)")

    ########################################################################################### BACKGROUND
    @background
    async def data_manager(self, vehicle: Vehicle):
        '''new data should be processed if within a valid measurement time
        new data is used to update the GP
        the GP is used to append a new waypoint to the waypoint list
        the waypoint list is used to update the vehicle's mission'''

        # use the data manager to get the latest data
        # collect data and mark valid if at a measurement waypoint
        num_samples_collected = self.DM.run(mark_measurement=self.valid_measurement, vehicle=vehicle) 

        if DEBUG_DATA_MANAGER:
            print(f"background: Collected {num_samples_collected} samples")
            # print the interval between the last two data points
            print(f"background: Interval between last two data points is {self.DM.interval} seconds")
            # print the last data point
            print(f"background: Last data point: {self.DM.get_human_readable(-1)}")

        # dynamically change the sleep time based upon the current queue size
        # this is to ensure that the data manager is not overloaded
        q = self.DM.get_queue_size()
        if q >= 2: # if the queue is large, sleep for a shorter time
            await asyncio.sleep(0.01)
        else: # if the queue is small, sleep for a longer time
            await asyncio.sleep(0.1)
    @background
    async def mission_timer(self, vehicle: Vehicle):
        '''this function is used to report the position of the rover at 3 and 10 minutes into the mission'''
        if self.mission_started:
            # check if a search time has eleapsed
            if datetime.datetime.now() > self.min_3_time:
                if not self.min_3_reported:
                    print('time now is: ', datetime.datetime.now())
                    print("=====HERE IS THE 3 MIN REPORTED POSITION=====")
                    # print(f"Reported position: {GP.peak_prediction_coords_history[-1]}")
                    print(f'Refined reported position: {GP.refined_peak_prediction_coords_history[-1]}')
                    self.min_3_reported = True

                    # reduce the MEASUREMENT_SPAN now that 3 minutes have elapsed (hopefully we are close to the rover)
                    MEASUREMENT_SPAN = MEASUREMENT_SPAN_AFTER_3_MIN

            if datetime.datetime.now() > self.min_10_time:
                if not self.min_10_reported:
                    print('time now is: ', datetime.datetime.now())
                    print("=====HERE IS THE 10 MIN REPORTED POSITION=====")
                    # print(f"Reported position: {GP.peak_prediction_coords_history[-1]}")
                    print(f'Refined reported position: {GP.refined_peak_prediction_coords_history[-1]}')
                    # print(f"Reported position: {GP.get_peak_prediction()}")
                    self.min_10_reported = True

            #---------------------------------------------------------------------------AUX PATH TRIGGER
            if datetime.datetime.now() > self.aux_time:
                if not self.aux_computed:
                    print("++Aux path triggered by timer++")
                    self.create_aux_path(center=vehicle.position)
  
        
        await asyncio.sleep(1) # check at least once per second

        
    ########################################################################################### START
    @state(name="start", first=True)
    async def start(self, vehicle: Drone):
        '''this is the first (takeoff) state of the rover search script'''
        
        # record the start time of the search, and the times for the 3 and 10 minute reports
        self.start_time = datetime.datetime.now()
        self.min_3_time = self.start_time + datetime.timedelta(minutes=3)
        self.min_10_time = self.start_time + datetime.timedelta(minutes=10)
        self.aux_time = self.start_time + datetime.timedelta(minutes=9) # estimate 1 minute for aux path to complete
        self.mission_started = True # LETS GOoooo!!! (enable the mission timer background task)
        print(f"start time marked @ {self.start_time}")

        # log the start location BEFORE takeoff (takeoff sets the _home_location)
        self.home_location = vehicle.position
        print(f"Home location is {vehicle.position.toJson()}")
        
        # Takeoff
        print(f"Taking off... (after arm command) to {SEARCH_ALTITUDE} meters")
        await vehicle.takeoff(SEARCH_ALTITUDE) # need some way to track the time since armed
        await vehicle.await_ready_to_move()
        print(f"\"mission\" start @ {self.start_time.strftime('%H:%M:%S%f')}")
        await vehicle.await_ready_to_move()
        print(f"marked takeoff complete: @ {datetime.datetime.now().strftime('%H:%M:%S%f')}")
        print(f"Takeoff took {datetime.datetime.now() - self.start_time} seconds")

        # Turn to face the search direction
        turning = asyncio.ensure_future(vehicle.set_heading(FOCUS_HEADING,blocking=True))
        # wait for vehicle to finish turning
        while not turning.done():
            await asyncio.sleep(0.1)

        # go to measurement state
        return "take_measurement"
    
    ########################################################################################### FORWARD
    @state(name="go_forward")
    async def go_forward(self, vehicle: Vehicle):

        # get the next waypoint
        next_waypoint = INIT.get_next_waypoint()
        if next_waypoint is None: # this should never happen
            print("No more waypoints in planned route, retrieve next waypoint from ProcessFunction!")
            next_waypoint = vehicle.home_coords 
            print(f"Next waypoint is {next_waypoint.toJson()}")
        else:    
            print(f"Moving to next waypoint: {next_waypoint.toJson()}")
            print(f"There are now {len(INIT.waypoints)} waypoints left in any precomputed route")    

        # move forward to the next location
        moving = asyncio.ensure_future(vehicle.goto_coordinates(next_waypoint))

        print("Command to waypoint issued")
        # wait until the vehicle is done moving
        while not moving.done():
            #print(f"my position is {vehicle.position.toJson()}")
            await asyncio.sleep(0.1)
        await moving

        # # turn to face the search direction
        turning = asyncio.ensure_future(vehicle.set_heading(FOCUS_HEADING))
        print("Command to turn issued")
        # wait for vehicle to finish turning
        while not turning.done():
            await asyncio.sleep(0.1)
        await turning
        
        return "take_measurement" if not self.check_end() else "end"
    
    ########################################################################################### MEASURE
    @timed_state(name="take_measurement", duration=6, loop=True)
    async def take_measurement(self, vehicle: Drone):
        self.valid_measurement = True
        # take a measurement 
        print(f"Taking measurements @ {datetime.datetime.now().strftime('%H:%M:%S:%f')}")

        # If the search time has ended, end the script
        if datetime.datetime.now() - self.start_time > self.max_search_time:
            next = "end"
        await asyncio.sleep(0.5)

        return "end_measurement"
    @state(name="end_measurement")############################################################# END MEASURE
    async def end_measurement(self, vehicle: Drone):
        self.valid_measurement = False # reset the valid measurement flag

        # filter the data and update the GP
        avg_power, avg_quality, avg_location, avg_power_varience, avg_quality_varience, avg_heading = self.DM.filter_data_mean()
        print(f"Filtered data (avg): Pwr:{avg_power},P_Var:{avg_power_varience}, Qual:{avg_quality}, Q_Var:{avg_quality_varience}, {avg_location.toJson()}, heading:{avg_heading}")
        # track the highest power measurement and location
        if avg_power > self.highest_power:
            self.highest_power = avg_power
            self.highest_power_location = avg_location
            print(f"New highest power measurement: {self.highest_power} @ {self.highest_power_location.toJson()}")

        if avg_quality_varience < self.quality_varience_cuttoff: # data is valid
            self.num_valid_measurements += 1

            GP.add_reading(position=avg_location, reading=avg_power)
            GP.predict(linespace_id='ugv') # update the GP and predict the ugvs position
            GP.predict(linespace_id='uav') # used to predict the uav's next waypoint

            # update the GP optimizer epsilon
            self.epsilon = self.epsilon * self.epsilon_decay
            print(f"GP optimizer epsilon: {self.epsilon}")

        else: # if the quality varience is too high, don't use the data
            print("++Quality varience is too high, do not use the data++")
            self.dropped_measurements += 1
            print(f"++Dropped measurements: {self.dropped_measurements}++")
            # try to get out of a null state
            if (self.dropped_measurements >= self.DROPPED_MEASUREMENT_LIMIT_AUX and
                self.aux_computed == False):  # if we have dropped too many measurements, switch to aux path! something is going wrong!
                print("++Too many dropped measurements, switch to aux!!++")
                self.create_aux_path(center=vehicle.position)
                # run though all the q_var values and look for a new (higher) cuttoff
                # first sort the q_var values in ascending order
                # q_var_sorted = np.sort(self.Q_var_history)
                # print(f"Sorted quality varience: {q_var_sorted}")
                # # walk through and find the differences between each value
                # q_var_diff = np.diff(q_var_sorted)
                # # find the index of the largest difference
                # max_diff_index = np.argmax(q_var_diff)
                # # use the index to find the new cuttoff (the value before the largest difference)
                # self.quality_varience_cuttoff = q_var_sorted[max_diff_index-1]
                # print(f"New quality varience cuttoff: {self.quality_varience_cuttoff}")


            # check if the rover has at least one measurement
            if len(GP.peak_prediction_coords_history) == 0: # if there are no measurements we need a measruement to start the GP
                print("FAILED TO GET A CRITICAL START MEASUREMENT, is the TX on?")
                # increase the quality varience cuttoff... (this is undesired behavior, but it is better than stagnation)
                self.quality_varience_cuttoff += 2**self.dropped_measurements
                print(f"Increased quality varience cuttoff to {self.quality_varience_cuttoff}")
                return "take_measurement"
            

        # print some stats about the GP
        # log marginal likelihood & gradient
        print(f"GP marginal likelihood: {GP.log_marginal_likelihood}")
        # print(f"GP marginal likelihood gradient: {GP.log_marginal_likelihood_gradient}")

        pred=GP.peak_prediction_coords_history[-1] # get the peak prediction
        mse = GP.peak_prediction_history_mse[-1] # get the mean squared error
        refined_pred = GP.refined_peak_prediction_coords_history[-1] # get the refined peak prediction
        # print(f"===GP prediction: {pred.toJson()}===")
        print(f"===GP refined prediction: {refined_pred}===")
        # print(f"===GP prediction mse: {mse}===")
        # print the distance from vehicle to prediction
        print(f"Distance from vehicle to prediction: {vehicle.position.distance(refined_pred)}")

        #-------------------------------------------------------------------------- SETTLE COUNTER
        # check if the prediction is within the settle range (only after 3 minutes)
        if self.min_3_reported == True and self.aux_computed == False:
            pred_sub2 = GP.peak_prediction_coords_history[-2] # get the second to last prediction
            if pred_sub2.distance(pred) < self.settle_range: # if the distance between the last two predictions is within the settle range
                self.settle_counter += 1
                print(f"Settle counter: {self.settle_counter}")
            else:
                self.settle_counter = 0
                print(f"Settle counter: {self.settle_counter}")
            # check if the settle counter has reached the limit
            if self.settle_counter >= self.settle_count_limit:
                print("++Settle counter reached, switch to aux path++")
                self.create_aux_path(center=vehicle.position) # create an aux path around the last prediction

        #-------------------------------------------------------------------------- CLOSE TO ROVER
        # if the rover is determined to be within the UAV boundry and we are close to the rover, switch to aux path 
        # (a circle path around the rover)
        if (GP.check_rover_in_uav_boundry(vehicle.position, tolerence=self.AUXILIARY_PATH_TRIGGER) and 
            len(INIT.waypoints) == 0 and # don't switch to aux if there are still waypoints in the list
            self.aux_computed == False): # don't switch to aux if aux has already been computed
            
            print("++Rover is estimated within in UAV boundry and we are close, switch to aux path++")
            self.create_aux_path(center=vehicle.position)
        
        #-------------------------------------------------------------------------- GP OPTIMIZER for next waypoint
        # if waypoint list is empty, get the next waypoint from the GP
        if len(INIT.waypoints) == 0:
            # get the next waypoint from the GP
            next_waypoint = GP.process_optimizer(optimizer=self.op_method, epsilon=self.epsilon)
            # convert the next waypoint to a Coordinate
            next_waypoint = Coordinate(next_waypoint[0], next_waypoint[1], SEARCH_ALTITUDE)
            print(f"Desired waypoint from GP: {next_waypoint.toJson()}")

            # ensure that the waypoint is not too close (or too far) from the current position (to prevent "stagnation" or "long jumps")
            next_waypoint_limited_nudged, nudged = GP.nudge(next_waypoint, vehicle.position,
                                                    min_dist=MIN_MEASUREMENT_SPAN, max_dist=MEASUREMENT_SPAN,
                                                    direction="random")
            # reset the altitude to the search altitude (beacuse our linespace_coord is 2D)
            next_waypoint_limited_nudged.alt = SEARCH_ALTITUDE
            # count the consecutive nudges
            if nudged:
                self.total_nudges += 1
                self.consecutive_nudges += 1
            else:
                self.consecutive_nudges = 0
            print(f"Next waypoint from GP (limited and nudged): {next_waypoint_limited_nudged.toJson()}")
            # add the next waypoint to the list of waypoints
            INIT.waypoints.append(next_waypoint_limited_nudged)
            # update the last waypoint
            self.last_waypoint = next_waypoint_limited_nudged

        #-------------------------------------------------------------------------- NUDGE to aux path
        # if we're nudging too much, switch to aux path
        if self.consecutive_nudges >= 3 and self.aux_computed == False:
            print("++Too many nudges, switch to aux path++")
            self.create_aux_path(center=vehicle.position)

        # write the current GP data to a pickle file for later analysis
        GP.save_pickle("gp.pickle")
        print("Saved GP data to gp.pickle")
        print(f"Number of (Q_var) valid measurements: {self.num_valid_measurements}")
        self.DM.write_measured_to_pickle()
        print("Saved DM data to m_data.pickle")

        # check_end and return the appropriate state
        return "go_forward" if not self.check_end() else "end"
    
    
    
    ########################################################################################### END
    @state(name="end")
    async def end(self, vehicle: Drone):
        # save the data_manager data to a pickle file
        # self.DM.write_measured_to_pickle()
        self.DM.write_all_data_to_pickle()
        

        home_coords = Coordinate(vehicle.home_coords.lat, vehicle.home_coords.lon, vehicle.position.alt)
        print("Ending rover search script")
        print(f"Returning to launch @ {home_coords.toJson()}")

        print()
        # print stats about the run
        print("-----------------STATS-----------------")
        print(f"Total number of dropped measurements: {self.dropped_measurements}")
        print(f"Total number of valid measurements: {self.num_valid_measurements}")
        print(f"Total number of nudges: {self.total_nudges}")
        print("Aux path triggered:", self.aux_computed)
        # gp stats
        print(f"GP marginal likelihood: {GP.log_marginal_likelihood}")
        print(f"GP marginal likelihood gradient: {GP.log_marginal_likelihood_gradient}")
        print(f"GP optimizer epsilon: {self.epsilon}")
        print(f"GP optimizer method: {self.op_method}")
        print(f"GP optimizer alpha: {GP.alpha}")
        # kernel stats
        print(f"GP kernel init: {GP.kernel}")
        print(f"GP kernel final: {GP.GP.kernel_}")
        # highest power measurement
        print(f"Highest power measurement: {self.highest_power} @ {self.highest_power_location.toJson()}")

        print("-----------------STATS-----------------")
        print()

        await vehicle.goto_coordinates(home_coords)
        print("Landing...")
        await vehicle.land()
        print("Done!")












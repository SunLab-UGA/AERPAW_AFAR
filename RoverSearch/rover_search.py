# Paul Kudyba, OCT-2023

# TODO: V1.0

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

print("Loading other libraries")

from init import TrialInit
print("Imported init.py")
from data_manager import DataManager, DataPoint
print("Imported data_manager.py")
from GP import ProcessFunction
print("Imported GP.py")

# Instatiate the trial init (holds constants, waypoints, and starting values)
INIT = TrialInit()

#CONSTANTS (defined in init.py)
STEP_SIZE = INIT.STEP_SIZE  # when going forward - how far, in meters
FOCUS_HEADING = INIT.NORTHWEST
SEARCH_ALTITUDE = INIT.SEARCH_ALTITUDE


# Initialize the GP 
# this is the process function that takes in data and returns a rover position estimate, and waypoint
GP = ProcessFunction()
print("Default Linespace")
GP.print_linespace(linespace_id="UGV") # print the default linespace

class RoverSearch(StateMachine):

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
    
    ########################################################################################### AT_INIT
    @at_init
    async def at_init(self, vehicle: Vehicle):
        print("Initializing rover search script (at_init)")
        print("rover_search PID IS: ", os.getpid())
        # Initialize the data manager 
        # this spawns a process which keeps all data without blocking the main thread
        # however, should be polled occasionally within @background so that DM maintains the latest data
        # DM then with the latest data can be used to update the GP (it sanitizes the data)
        self.DM = DataManager(vehicle)
        # Start the data manager
        self.DM.start()
        self.valid_measurement = False # flag for if the measurement is to be filtered and used in the GP

        # global variables for reporting
        self.min_3_reported = False
        self.min_10_reported = False
        self.mission_started = False

        # print the time
        print(f"Current time is {datetime.datetime.now().strftime('%H:%M:%S')}")
        print("Done initializing rover search script (at_init)")

    ########################################################################################### BACKGROUND
    @background
    async def background(self, vehicle: Vehicle):
        # use the data manager to get the latest data
        num_samples_collected = self.DM.run(mark_measurement=self.valid_measurement)

        print(f"background: Collected {num_samples_collected} samples")
        # print the interval between the last two data points
        print(f"background: Interval between last two data points is {self.DM.interval} seconds")

        if self.mission_started:
        # new data should be processed if within a valid measurement time
        # new data is used to update the GP
        # the GP is used to append a new waypoint to the waypoint list
        # the waypoint list is used to update the vehicle's mission

            # check if a search time has eleapsed
            if datetime.datetime.now() > self.min_3_time:
                if not self.min_3_reported:
                    print("3 minutes have elapsed")
                    print("=====HERE IS THE REPORTED POSITION=====")
                    self.min_3_reported = True
            if datetime.datetime.now() > self.min_10_time:
                if not self.min_10_reported:
                    print("10 minutes have elapsed")
                    print("=====HERE IS THE REPORTED POSITION=====")
                    self.min_10_reported = True

        await asyncio.sleep(0.1) # was 1

    ########################################################################################### START
    @state(name="start", first=True)
    async def start(self, vehicle: Drone):

        # record the start time of the search, and the times for the 3 and 10 minute reports
        self.start_time = datetime.datetime.now()
        self.min_3_time = self.start_time + datetime.timedelta(minutes=3)
        self.min_10_time = self.start_time + datetime.timedelta(minutes=10)
        self.mission_started = True # LETS GOoooo!!!

        # log the start location BEFORE takeoff (takeoff sets the _home_location)
        self.home_location = vehicle.position
        print(f"Home location is {vehicle.position.toJson()}")
        
        # Takeoff
        await vehicle.takeoff(SEARCH_ALTITUDE)
        print("Taking off...")
        await vehicle.await_ready_to_move()
        print(f"marked start time as {self.start_time.strftime('%H:%M:%S%f')}")
        await vehicle.await_ready_to_move()
        print(f"Done taking off @ {datetime.datetime.now().strftime('%H:%M:%S%f')}")
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
        # go forward and continually log the vehicle's position

        # get the next waypoint
        next_waypoint = INIT.get_next_waypoint()
        if next_waypoint is None:
            print("No more waypoints in planned route, retrieve next waypoint from ProcessFunction")
            # TEMP set next waypoint to be the home location
            next_waypoint = vehicle.home_coords # temporary until we get the next waypoint from the GP
            print(f"Next waypoint is {next_waypoint.toJson()}")
        else:    
            print(f"Moving to next waypoint: {next_waypoint.toJson()}")
            print(f"There are {len(INIT.waypoints)} waypoints left in the list")    

        # # use the current vehicle heading to move forward
        # heading = vehicle.heading
        # heading_rad = heading * 2 * math.pi / 360
        # move_vector = VectorNED(
        #     STEP_SIZE * math.cos(heading_rad), STEP_SIZE * math.sin(heading_rad), 0
        # )

        # # ensure the next location is inside the geofence
        # cur_pos = vehicle.position
        # next_pos = vehicle.position + move_vector
        # (valid_waypoint, msg) = self.safety_checker.validateWaypointCommand(
        #     cur_pos, next_pos
        # )
        
        # # if the next location violates the geofence turn 90 degrees
        # if not valid_waypoint:
        #     print("Can't go there:")
        #     print(msg)
        #     return "turn_right_90"

        # # otherwise move forward to the next location
        # moving = asyncio.ensure_future(
        #     vehicle.goto_coordinates(vehicle.position + move_vector)
        # )

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
    @timed_state(name="take_measurement", duration=5, loop=True)
    async def take_measurement(self, vehicle: Drone):
        self.valid_measurement = True
        # take a measurement 
        print(f"Taking measurement @ {datetime.datetime.now().strftime('%H:%M:%S:%f')}")

        # If the search time has ended, end the script
        if datetime.datetime.now() - self.start_time > self.max_search_time:
            next = "end"
        await asyncio.sleep(0.5)

        return "end_measurement"
    @state(name="end_measurement")############################################################# END MEASURE
    async def end_measurement(self, vehicle: Drone):
        self.valid_measurement = False # reset the valid measurement flag

        # filter the data and update the GP
        avg_power, avg_quality, avg_location = self.DM.filter_data_avg()
        print(f"Filtered data (avg): Pwr:{avg_power}, Qual:{avg_quality}, {avg_location.toJson()}")

        GP.add_reading(position=avg_location, reading=avg_power)
        GP.predict() # update the GP
        pred=GP.get_peak_prediction() # get the peak prediction
        print(f"GP prediction: {pred}")
        # if waypoint list is empty, get the next waypoint from the GP
        if len(INIT.waypoints) == 0:
            # get the next waypoint from the GP
            next_waypoint = GP.process_optimizer()
            # convert the next waypoint to a Coordinate
            next_waypoint = Coordinate(next_waypoint[0], next_waypoint[1], SEARCH_ALTITUDE)
            # add the next waypoint to the list of waypoints
            INIT.waypoints.append(next_waypoint)
            print(f"Next waypoint from GP: {next_waypoint.toJson()} added to waypoint list")
        # check_end and return the appropriate state
        return "go_forward" if not self.check_end() else "end"
    
    
    ########################################################################################### END
    @state(name="end")
    async def end(self, vehicle: Drone):
        home_coords = Coordinate(vehicle.home_coords.lat, vehicle.home_coords.lon, vehicle.position.alt)
        print("Ending rover search script")
        print(f"Returning to launch @ {home_coords.toJson()}")

        await vehicle.goto_coordinates(home_coords)
        await vehicle.land()
        print("Done!")

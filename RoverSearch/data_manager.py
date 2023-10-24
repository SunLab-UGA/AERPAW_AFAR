# Paul Kudyba, OCT-2023

# Designed to keep track of all polled data and serve requests for that data
# as an interpolated coherent data stream

# this class is responsible for aggregating and filtering all data into single DataPoints 
# which can be used by the GP to determine the next search point

# it spawns a process which polls the GPS and radio data at a set rate
# it then is periodically run to gather the data from the queue and add it to the data structure
# when the data is valid, it is added surmised into a single location and channel measurement

# it also is responsible for keeping a log of all data and keeping the NamedTuple DataPoint structure

import numpy as np
from multiprocessing import Process, Queue, Manager
from typing import NamedTuple
import datetime
from aerpawlib.util import Coordinate, VectorNED
import dronekit
from aerpawlib.vehicle import Vehicle
import time
import zmq
# from struct import unpack

#NOTES and TODOs
# potentially use a deque for the data structures to limit memory usage
# add a form of vehicle identification to the data structures for multiple vehicle logging support
# form a stationary filter and feedback to the vehicle to if the data collected has settled to a stationary point

# tie worker directly into dronekit to get the vehicle data?

# CONSTANTS
POLL_RATE = 30 # Hz (~29 Hz output from the radio), keep worker from taking too much CPU
DEBUG_WORKER = False
DEBUG_PARENT = False


class DataPoint(NamedTuple):
    '''Data structure for a single data point'''
    # time data ----------------
    radio_timestamp: datetime.datetime
    gps_timestamp: datetime.datetime
    # gps data -----------------
    position: Coordinate
    heading: float
    # dronekit.GPSInfo (can't pickle) so we use a list, packed as:
    # ( fix_type, satellites_visible, eph (error horz.), epv (error vert.) )
    gps_status: list 

    velocity: VectorNED

    # dronekit.Attitude (can't pickle) so we use a list, packed as:
    # (pitch, roll, yaw)
    attitude: float 
    # dronekit.Battery (can't pickle) so we use a list, packed as:
    # (level, voltage, current)
    battery: float 
    moving: bool # track if the drone has an active movement command
    # radio data ----------------
    Channel: float
    Quality: float
    #Valid: bool # track if the radio has a valid signal lock
    #phase_offset: int # track the phase offset of the signal
    #raw_amplitude: float # track the raw amplitude of the signal (before any processing/filtering)

class RadioData(NamedTuple): # this needs to be split out because I cannot pass the vehicle object to the worker...
    '''Data structure for the radio part of a single data point'''
    # time data ----------------
    timestamp: datetime.datetime
    # radio data ----------------
    Channel: float
    Quality: float
    #Valid: bool # track if the radio has a valid signal lock
    #phase_offset: int # track the phase offset of the signal
    #raw_amplitude: float # track the raw amplitude of the signal (before any processing/filtering)

class DataManager:
    '''Class for managing the data from the GPS and radio'''
    ############################################################################# DATA WORKER
    def _data_worker(self, data_queue:Queue) -> None:
        '''Worker function for polling the data from the vehicle and radio'''
        # initialize the zmq context
        context = zmq.Context()
        # initialize the zmq socket
        socket = context.socket(zmq.SUB)
        # connect to the radio data zmq publisher
        socket.connect("tcp://127.0.0.1:64000") # loopback, port 64000
        # subscribe to all topics
        socket.setsockopt(zmq.SUBSCRIBE, b"") # subscribe to all topics

        # report that the worker has started
        print("Data Manager worker started")

        while True:
            read1 = datetime.datetime.now()
            # blocking recv waits for a message
            msg = socket.recv() 
            # unpack the data into a numpy array
            data = np.frombuffer(msg, dtype=np.float32, count=-1) # power is a float32, count=-1 means read all
            # only take the first element of the array, in case there is more than one
            power = data[0]
            # static temp value for quality
            quality = 0.0
            timestamp = datetime.datetime.now()

            # create the radio data structure
            radio_data = RadioData(timestamp, power, quality)

            # add the data to the queue
            data_queue.put(radio_data)

            read4 = datetime.datetime.now()
            # print some debug info
            if DEBUG_WORKER:
                # print human readable data
                for field in radio_data._fields:
                    print(f"{field}: {getattr(radio_data, field)}")
                print(f"loop begin: {read1}")
                print(f"loop end: {read4}")
                # print the difference between the reads
                print(f"loop time: {read4-read1}")

            # sleep for the poll rate
            time.sleep(1/POLL_RATE)

    ############################################################################# INIT
    def __init__(self) -> None:
        '''Initialize the data manager'''

        self.data = []
        self.data_filtered = [] # data that is filtered into a single data point for the GP
        self.interval = 0 # the time interval between the last two data points
        
        # initialize the queues and processes
        self.queue = Queue()
        self.data_process = Process(target=self._data_worker, args=(self.queue,))

    def start(self):
        '''Start the data manager'''
        self.data_process.start()
        print("Data Manager started")
        # print the pid of the process
        print(f"Data Manager spawned a process with PID: {self.data_process.pid}")

    def terminate_worker(self):
        '''Terminate the data worker process'''
        # self.data_process.join()
        self.data_process.terminate()
        print("Data Manager subprocess terminated")

    ############################################################################# RUN
    def run(self, mark_measurement=False, vehicle:Vehicle=None):
        '''Run the data manager gather loop
        returns the number of data points in the data structure
        if mark_measurement is positive, then it will add the measurement to a seperate data structure 
        (indexed by mark_measurement) to be filtered into one datapoint for the GP
        The RadioData is combined with the GPS data to form a single DataPoint 
        '''
        try:
            if not self.queue.empty():
                if DEBUG_PARENT:
                    print(f"Data Manager: queue size {self.queue.qsize()}")

                for _ in range(self.queue.qsize()):
                    # get the radio data
                    radio_data = RadioData(*self.queue.get(block=False)) # unpack, * is the unpack operator
                    # create the data point
                    data_point = DataPoint(
                        radio_timestamp=radio_data.timestamp,
                        gps_timestamp=datetime.datetime.now(),
                        position=vehicle.position,
                        heading=vehicle.heading,
                        gps_status=[vehicle.gps.fix_type, vehicle.gps.satellites_visible, vehicle.gps.eph, vehicle.gps.epv],
                        velocity=vehicle.velocity,
                        attitude=[vehicle.attitude.pitch, vehicle.attitude.roll, vehicle.attitude.yaw],
                        battery=[vehicle.battery.level, vehicle.battery.voltage, vehicle.battery.current],
                        moving=vehicle.done_moving,
                        Channel=radio_data.Channel,
                        Quality=radio_data.Quality
                    )
                    self.data.append(data_point)
                    if mark_measurement == True:
                        self.data_filtered.append(self.data[-1]) # copy the (just added) data point into the filtered data array
                    # calculate the interval
                    if len(self.data) > 1:
                        self.interval = self.data[-1].radio_timestamp - self.data[-2].radio_timestamp
                    return len(self.data)

        except KeyboardInterrupt:
            print("Keyboard interrupt detected, exiting")
            self.terminate_worker()
        except Exception as e:
            print(f"Exception in data manager run loop: {e}")
            self.terminate_worker()
        # finally: # make sure to close the process
        #     self.data_process.terminate()
        #     print("Data Manager subprocess terminated potentially prematurely!")

    def get_queue_size(self) -> int:
        '''Get the size of the queue'''
        return self.queue.qsize()
    
    def filter_data_avg(self) -> DataPoint:
        '''Filter the data into a single data point
        returns a single averaged DataPoint
        '''
        avg_power = np.mean([d.Channel for d in self.data_filtered])
        avg_quality = np.mean([d.Quality for d in self.data_filtered])
        avg_lat = np.mean([d.position.lat for d in self.data_filtered])
        avg_lon = np.mean([d.position.lon for d in self.data_filtered])
        avg_alt = np.mean([d.position.alt for d in self.data_filtered])
        avg_location = Coordinate(lat=avg_lat, lon=avg_lon, alt=avg_alt)
        avg_heading = np.mean([d.heading for d in self.data_filtered])
        avg_varience = np.var([d.Channel for d in self.data_filtered])

        # clear the data_filtered structure
        self.data_filtered = []

        return avg_power, avg_quality, avg_location, avg_varience, avg_heading
    
    def get_human_readable(self, index:int) -> str:
        '''Get the data[index] in a human readable format'''
        # check if the call is valid
        if len(self.data) == 0:
            return "No data collected yet"
        
        data = self.data[index] 

        for field in data._fields:
            if field == "position":
                print(f"{field}: {getattr(data, field).toJson()}") # print human readable position
            else: 
                print(f"{field}: {getattr(data, field)}")
    # method to create a file with all the data
    def write_data_to_file(self,filename:str):
        '''Write the data to a file'''
        with open(filename, "w") as f:
            for d in self.data:
                f.write(self.get_human_readable(d))
        print(f"Data written to {filename}")
        print(f"Data length: {len(self.data)}")


            


    


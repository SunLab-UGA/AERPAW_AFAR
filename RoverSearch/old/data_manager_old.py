# Paul Kudyba, OCT-2023

# Designed to keep track of all polled data and serve requests for that data 
# as an interpolated coherent data stream

# it also is responsible for keeping a log of all data and keeping the NamedTuple data structures

import numpy as np
from multiprocessing import Process, Queue
from typing import NamedTuple
import datetime
from aerpawlib.util import Coordinate, VectorNED
import dronekit
from aerpawlib.vehicle import Vehicle, Drone
import time
import zmq

#NOTES and TODOs
# potentially use a deque for the data structures to limit memory usage
# add a form of vehicle identification to the data structures for multiple vehicle logging support

# CONSTANTS
GPS_POLL_RATE = 30 # Hz
RADIO_POLL_RATE = 30 # Hz

TIME_SYNC_THRESHOLD = 0.05 # seconds, the maximum time difference considered syncronized (otherwise it is interpolated)
TIME_MAX_INTERPOLATE = 0.1 # seconds, the maximum time difference to interpolate (otherwise it is dropped)

################################################################################ GPS DATA "STRUCT"
class GPSData(NamedTuple):
    '''Data structure for GPS data'''
    # gathered data
    timestamp: datetime.datetime
    position: Coordinate
    heading: float
    gps_status: dronekit.GPSInfo
    velocity: VectorNED
    attitude: dronekit.Attitude
    battery: dronekit.Battery
    moving: bool # track if the drone has an active movement command

################################################################################ RADIO DATA "STRUCT"
class RadioData(NamedTuple):
    '''Data structure for radio data'''
    # gathered data
    timestamp: datetime.datetime
    Channel: float
    Quality: float
    #Valid: bool # track if the radio has a valid signal lock
    phase_offset: int # track the phase offset of the signal
    #raw_amplitude: float # track the raw amplitude of the signal (before any processing/filtering)

class DataPoint(NamedTuple):
    '''Data structure for a single Interpolated data point'''
    # gathered data
    timestamp: datetime.datetime
    position: Coordinate
    heading: float
    gps_status: dronekit.GPSInfo
    velocity: VectorNED
    attitude: dronekit.Attitude
    battery: dronekit.Battery
    moving: bool # track if the drone has an active movement command
    Channel: float
    Quality: float
    #Valid: bool # track if the radio has a valid signal lock
    phase_offset: int # track the phase offset of the signal
    #raw_amplitude: float # track the raw amplitude of the signal (before any processing/filtering)

class DataManager:
    '''Class for managing the data from the GPS and radio'''
    ############################################################################# GPS WORKER
    def _gps_worker(self, gps_queue: Queue, V:Vehicle) -> None:
        '''Worker function for polling the GPS/Vehicle State'''
        while True:
            timestamp = datetime.datetime.now()
            # poll the GPS
            gpsInfo = V.gps
            position = V.position
            heading = V.heading
            velocity = V.velocity
            attitude = V.attitude
            battery = V.battery
            moving = V.done_moving

            # create the data structure
            gps_data = GPSData(timestamp, position, heading, gpsInfo, velocity, attitude, battery, moving)

            # add the data to the queue
            gps_queue.put(gps_data)

            # sleep for the poll rate
            time.sleep(1/GPS_POLL_RATE)
    
    ############################################################################# RADIO WORKER
    def _radio_worker(self, radio_queue: Queue) -> None:
        '''Worker function for polling the radio using ZMQ'''
        def poll_socket(socket, type=np.float32, vector_limit=1):
            '''Helper function to poll a ZMQ socket
            socket: the ZMQ socket to poll
            type: the type of data to poll
            vector_limit: the number of vector elements to poll, drop any extra to prevent downstream issues
            '''
            if socket.poll(10) != 0:
                msg = socket.recv() # receive the message
                data = np.frombuffer(msg, dtype=type)
                if len(data) > vector_limit:
                    data = data[:vector_limit]
                return data, True # return the data and a valid flag
            else:
                return None, False # return None and an invalid flag

        # initialize the zmq context and sockets
        zmq_ports = [64000,64001,64002] # ports for power, quality, phase_offset
        zmq_contexts = []
        zmq_sockets = []
        for port in zmq_ports:
            zmq_contexts.append(zmq.Context())
            zmq_sockets.append(zmq_contexts[-1].socket(zmq.SUB))
            zmq_sockets[-1].connect(f"tcp://localhost:{port}") # connect, not bind, the PUB will bind, only 1 can bind
            zmq_sockets[-1].setsockopt_string(zmq.SUBSCRIBE, b'') # subscribe to topic of all (needed or else it won't work)
            print(f"Connected to ZMQ port {port}")
        
        # poll the radio sockets
        while True:
            valid_tag = False
            data = []
            for ii,socket in enumerate(zmq_sockets):
                data[ii], valid = poll_socket(socket)
                valid_tag = valid_tag and valid # if any socket is invalid, the data is invalid
            if valid_tag:
                timestamp = datetime.datetime.now()
                # create the data structure
                radio_data = RadioData(timestamp, data[0], data[1], data[2])
                # add the data to the queue
                radio_queue.put(radio_data)

            # sleep for the poll rate
            time.sleep(1/RADIO_POLL_RATE)

    ############################################################################# INIT
    def __init__(self, V:Vehicle) -> None:
        '''Initialize the data manager'''
        # initialize the data structures and vehicle
        self.V = V
        self.gps_data = []
        self.radio_data = []
        
        # initialize the queues and processes
        self.gps_queue = Queue()
        self.radio_queue = Queue()
        self.gps_process = Process(target=self._gps_worker, args=(self.gps_queue,))
        self.radio_process = Process(target=self._radio_worker, args=(self.radio_queue,))
        self.gps_process.start()
        self.radio_process.start()

    ############################################################################# RUN
    def run(self):
        '''Run the data manager gather loop'''
        try:
            while True:
                if not self.gps_queue.empty():
                    for _ in range(self.gps_queue.qsize()):
                        self.gps_data.append(self.gps_queue.get())
                if not self.radio_queue.empty():
                    for _ in range(self.radio_queue.qsize()):
                        self.radio_data.append(self.radio_queue.get())
        except KeyboardInterrupt:
            print("Keyboard interrupt detected, exiting")
        finally: # make sure to close the processes
            self.gps_process.terminate()
            self.radio_process.terminate()
    
    ############################################################################# INTERPOLATE
    def interpolate(self, timestamp: datetime.datetime) -> (GPSData, RadioData):
        '''Interpolate the GPSData to the given RadioData timestamp'''
        # find the closest GPSData timestamp
        pass


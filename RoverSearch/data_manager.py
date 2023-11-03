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
import pickle

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
    Channel: float # this is the default sounder script output [power]
    Quality: float # this is the default sounder script output [quality]
    Intensity: int # this is the Xcorrelation intensity of the signal code (0-4095)
    Phase: float # this is the phase of the signal code (0-2pi)
    PhaseOffset: int # this is the phase index of the signal code (0-4095)

class RadioData(NamedTuple): # this needs to be split out because I cannot dynamically/directly access the vehicle object from the worker without "SIGNIFICANT" work
    '''Data structure for the radio part of a single data point'''
    # time data ----------------
    timestamp: datetime.datetime
    # radio data ----------------
    Channel: float
    Quality: float
    Intensity: int
    Phase: float
    PhaseOffset: int

class DataManager:
    '''Class for managing the data from the GPS and radio'''
    ############################################################################# DATA WORKER
    def _data_worker(self, data_queue:Queue) -> None:
        '''Worker function for polling the data from the vehicle and radio'''

        ports = {
            "Power": 64000, 
            "Quality": 64001, 
            "Intensity": 64002, 
            "Phase": 64003,
            "PhaseOffset": 64004, 
        }

        # for each port, create a zmq context and socket
        contexts = [] # list of zmq contexts
        sockets = [] # list of zmq sockets
        for port in ports:
            contexts.append(zmq.Context())
            sockets.append(contexts[-1].socket(zmq.SUB))
            # connect the last added socket to the port
            sockets[-1].connect(f"tcp://127.0.0.1:{ports[port]}") # loopback
            # subscribe to all topics
            sockets[-1].setsockopt(zmq.SUBSCRIBE, b"") # b"" = subscribe to all topics

        # report that the worker has started
        print("Data Manager worker started")

        while True:
            read1 = datetime.datetime.now() # for loop timing

            # loop though each socket and get a message
            recieved_data = {}
            for socket,port in zip(sockets, ports): # zip the sockets and ports together (we want the port names)
                msg = socket.recv() # blocking recv waits for a message
                # unpack the data into a numpy array
                data_msg = np.frombuffer(msg, dtype=np.float32, count=-1) # power is a float32, count=-1 means read all
                # only take the first element of the array, in case there was more than one 
                # (this shouldn't happen, but unfortuantely it can 
                # because we cleared the buffer with reading all? or more likely because of GNU radio)
                data_msg = data_msg[0]
                # add the data to the dictionary
                recieved_data[port] = data_msg

            # create the radio data structure
            radio_data = RadioData(
                timestamp=datetime.datetime.now(),
                Channel=recieved_data["Power"],
                Quality=recieved_data["Quality"],
                Intensity=int(recieved_data["Intensity"]), # convert to int because it is an index, so np.float32 -> int
                Phase=recieved_data["Phase"],
                PhaseOffset=int(recieved_data["PhaseOffset"]) # convert to int because it is an index
            )

            # add the data to the queue
            data_queue.put(radio_data)

            read4 = datetime.datetime.now() # for loop timing
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
        self.corr_matrix = [] # correlation matrix for the WSS filter
        self.data = []
        self.data_filtered = [] # data that is filtered into a single data point for the GP
        self.data_filtered_history = [] # history of the filtered data, each element the data points before filtering
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
        if mark_measurement is true, then it will add the measurement to a seperate data structure 
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
                    # create the data point with the radio data and best upto date vehicle data (this could be better with a direct connection to the vehicle)
                    data_point = DataPoint(
                        radio_timestamp=radio_data.timestamp,
                        gps_timestamp=datetime.datetime.now(),
                        position=vehicle.position,
                        heading=vehicle.heading,
                        gps_status=[vehicle.gps.fix_type, vehicle.gps.satellites_visible, vehicle.gps.eph, vehicle.gps.epv],
                        velocity=vehicle.velocity,
                        attitude=[vehicle.attitude.pitch, vehicle.attitude.roll, vehicle.attitude.yaw],
                        battery=[vehicle.battery.level, vehicle.battery.voltage, vehicle.battery.current],
                        moving=vehicle.done_moving(), # check if the vehicle is "moving" or "busy"
                        Channel=radio_data.Channel,
                        Quality=radio_data.Quality,
                        Intensity=radio_data.Intensity,
                        Phase=radio_data.Phase,
                        PhaseOffset=radio_data.PhaseOffset
                    )
                    # check for any invalid data or NANs
                    if np.isnan(data_point.Channel) or np.isnan(data_point.Quality):
                        print("****Invalid RADIO data detected, skipping****")
                        continue
                    if (np.isnan(data_point.position.lat) 
                        or np.isnan(data_point.position.lon) 
                        or np.isnan(data_point.position.alt)):
                        print("****Invalid GPS data detected, skipping****")
                        continue

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
    
    ############################################################################# FILTER MEAN
    def filter_data_mean(self) -> DataPoint:
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
        avg_power_varience = np.var([d.Channel for d in self.data_filtered])
        avg_quality_varience = np.var([d.Quality for d in self.data_filtered])

        # TEMP check and present wss of the quality data
        # wss = self.filter_data_wss()

        # add all the filtered data to the history before clearing it
        self.data_filtered_history.append(self.data_filtered)

        # clear the data_filtered structure
        self.data_filtered = []

        return avg_power, avg_quality, avg_location, avg_power_varience, avg_quality_varience, avg_heading
    
    ############################################################################# FILTER WSS
    def filter_data_wss(self): #-> (DataPoint, list, bool):
        '''Filter the data into a single data point
        returns a single averaged DataPoint and a bool if the data is stationary
        only add the data to the history if it is stationary
        '''
        def autocorrelate(x, y):
            """Compute the autocorrelation between two lists x and y"""
            mean_x, mean_y = np.mean(x), np.mean(y)
            numerator = sum([(x[i] - mean_x) * (y[i] - mean_y) for i in range(len(x))])
            denominator_x = sum([(xi - mean_x) ** 2 for xi in x]) ** 0.5
            denominator_y = sum([(yi - mean_y) ** 2 for yi in y]) ** 0.5
            if denominator_x * denominator_y == 0:
                return 0
            return numerator / (denominator_x * denominator_y)
        def generate_correlation_matrix(samples, bin_size=30):
            """Generate a correlation matrix for sample data."""
            # Split samples into chunks of 30
            chunks = [samples[i:i+bin_size] for i in range(0, len(samples), bin_size)] # take the samples and split them into chunks of bin_size
            # remove chunks that are not the bin_size
            chunks = [chunk for chunk in chunks if len(chunk) == bin_size]
            # Generate a correlation matrix for the chunks
            matrix = []
            for i in range(len(chunks)):
                row = []
                for j in range(len(chunks)):
                    correlation = autocorrelate(chunks[i], chunks[j])
                    row.append(correlation)
                matrix.append(row)
            print("WSS matrix:")
            print(matrix)

            # remove the diagonal (because it is always 1)
            for i in range(len(matrix)):
                matrix[i][i] = 0
            
            # get the highest correlation
            max_correlation = np.max(matrix)
            print(f"max correlation: {max_correlation}")
            self.corr_matrix = matrix
            return matrix
        # def is_stationary(matrix):
        #     """Determine if a correlation matrix is stationary."""
        #     # Check if the matrix is symmetric
        #     for i in range(len(matrix)):
        #         for j in range(len(matrix)):
        #             if matrix[i][j] != matrix[j][i]:
        #                 return False
        #     # Check if the matrix is diagonal
        #     for i in range(len(matrix)):
        #         for j in range(len(matrix)):
        #             if i != j and matrix[i][j] != 0:
        #                 return False
        #     return True
        # def get_stationary_samples(samples, bin_size=30):
        #     """Get the stationary samples from a list of samples."""
        #     # Generate a correlation matrix for the samples
        #     matrix = generate_correlation_matrix(samples, bin_size)
        #     # Check if the matrix is stationary
        #     if is_stationary(matrix):
        #         return samples
        #     # If the matrix is not stationary, split the samples in half and try again
        #     half = len(samples) // 2
        #     return get_stationary_samples(samples[:half], bin_size) + get_stationary_samples(samples[half:], bin_size)
        # def get_stationary_samples_from_data(data):
        #     """Get the stationary samples from a list of DataPoints."""
        #     # Get the samples from the data
        #     samples = [d.Quality for d in data]
        #     # Get the stationary samples
        #     return get_stationary_samples(samples)

        # test matrix generation
        generate_correlation_matrix([d.Quality for d in self.data_filtered])

        # samp = get_stationary_samples_from_data(self.data_filtered)
        # return samp
        
    
    #====================================================================================================
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
    
    def write_data_to_file(self,filename:str):
        '''Write the data to a file'''
        with open(filename, "w") as f:
            for d in self.data:
                f.write(self.get_human_readable(d))
        print(f"Data written to {filename}")
        print(f"Data length: {len(self.data)}")

    def write_measured_to_pickle(self, filename="m_data.pickle"):
        '''Write the measured data to a pickle file'''
        # go through the data and convert named tuples to dicts (without coordnate or vectorNED)
        # (it's easier to pickle and unpack that way)
        # print(f'num data_filtered_history: {len(self.data_filtered_history)}')
        save_data = []
        for m in self.data_filtered_history: # m is a measurement
            # print(f'num datapoints in measurment: {len(m)}')
            messurand = []
            for d in m: # d is a DataPoint in the measurement
                # print the first data point in the measurement
                # print(f"first data point in measurement: {d}") if d == m[0] else None
                save_dict = { # some of these are commented out temporarly to reduce the size of the pickle file
                    "radio_timestamp": d.radio_timestamp,
                    "gps_timestamp": d.gps_timestamp,
                    "position": [d.position.lat, d.position.lon, d.position.alt],
                    "heading": d.heading,

                    "gps_status": d.gps_status,
                    "velocity": [d.velocity.north, d.velocity.east, d.velocity.down],
                    "attitude": d.attitude,
                    "battery": d.battery,

                    "moving": d.moving,
                    "Channel": d.Channel,
                    "Quality": d.Quality,
                    "Intensity": d.Intensity,
                    "Phase": d.Phase,
                    "PhaseOffset": d.PhaseOffset
                }
                messurand.append(save_dict)
            save_data.append(messurand)

        with open(filename, "wb") as f:
            pickle.dump(save_data, f)
        print(f"{len(self.data_filtered_history)} MEASURE Data points written to {filename}")

    def write_all_data_to_pickle(self, filename="data.pickle"):
        '''Write all the data to a pickle file'''
        # go through the data and convert named tuples to dicts (without coordnate or vectorNED)
        # (it's easier to pickle and unpack that way)
        save_data = []
        for d in self.data:
            save_dict = {
                "radio_timestamp": d.radio_timestamp,
                "gps_timestamp": d.gps_timestamp,
                "position": [d.position.lat, d.position.lon, d.position.alt],
                "heading": d.heading,
                "gps_status": d.gps_status,
                "velocity": [d.velocity.north, d.velocity.east, d.velocity.down],
                "attitude": d.attitude,
                "battery": d.battery,
                "moving": d.moving,
                "Channel": d.Channel,
                "Quality": d.Quality
            }
            save_data.append(save_dict)

        with open(filename, "wb") as f:
            pickle.dump(save_data, f)

        print(f"{len(self.data)} Data points written to {filename}")


            


    


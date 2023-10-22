import numpy as np
from multiprocessing import Process, Queue, current_process
from datetime import datetime

import time


class DataManager:
    def __init__(self):
        print("Initializing DataManager...")
        self.gps_queue = Queue()
        self.radio_queue = Queue()
        
        self.gps_worker = Process(target=self.gps_worker_fn)
        print("GPS worker created")

        
        self.radio_worker = Process(target=self.radio_worker_fn)
        print("Radio worker created")

        self.data = []

    def gps_worker_fn(self):
        print("HI! IM A GPS worker!")
        i = 0
        while True:
            # Simulate GPS data polling
            time.sleep(3)
            i += 1
            print("GPS worker generating data...")
            data = np.array([datetime.now(), np.random.randn(),i])  # [timestamp, GPS data]
            self.gps_queue.put(data)
            print("GPS worker put data in queue...")
            
    def radio_worker_fn(self):
        print("HI! IM A RADIO worker!")
        while True:
            # Simulate Radio data polling
            time.sleep(2)
            print("Radio worker generating data...")
            data = np.array([datetime.now(), np.random.randn() * 10])  # [timestamp, Radio data]
            self.radio_queue.put(data)
            print("Radio worker put data in queue...")

    def terminate_workers(self):
        self.gps_worker.terminate()
        self.radio_worker.terminate()
        self.gps_worker.join()
        self.radio_worker.join()

    def run(self):
        self.gps_worker.start()
        print("GPS worker started")
        print("GPS worker pid:", self.gps_worker.pid)

        self.radio_worker.start()
        print("Radio worker started")
        print("Radio worker pid:", self.radio_worker.pid)

        try:
            while True:
                print("Checking queues...1")
                if not self.gps_queue.empty():
                    print("GPS queue not empty")
                    data = self.gps_queue.get()
                    self.data.append(["GPS", data[0], data[1]])  # ["GPS", timestamp, GPS data]
                    print("GPS data:", data)
                print("Checking queues...1")
                if not self.radio_queue.empty():
                    data = self.radio_queue.get()
                    self.data.append(["Radio", data[0], data[1]])  # ["Radio", timestamp, Radio data]
                    print("Radio data:", data)

                if self.gps_queue.empty() and self.radio_queue.empty():
                    print("No data in both queues")
                
                print("sleeping...")
                time.sleep(1)

        except KeyboardInterrupt:
            print("Interrupted! Terminating workers...")
            print("Data:", self.data)
        finally:
            self.terminate_workers()

if __name__ == "__main__":
    # print the pid of the main process
    print("Main process pid:", current_process().pid)
    manager = DataManager()
    manager.run()

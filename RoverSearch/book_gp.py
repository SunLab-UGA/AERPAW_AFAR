# this is intended to be a library of kernels and mean functions for an ensemble GP
# time is not on our side

import numpy as np
from scipy.optimize import minimize
from scipy.stats import norm

from typing import List
from itertools import product

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
from sklearn.gaussian_process.kernels import Kernel

from scipy.stats import norm

from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.util import inside, readGeofence

import warnings
warnings.filterwarnings("ignore") # ignore the warnings from the GP

# we bypass INIT here sadly

class EnsembleGP():
    '''
    this is an ensemble of GPs, each GP is trained on the same set of waypoints
    but each GP has a different kernel and mean function
    the log likelihood of each GP is normalized and used to weight the predictions of each GP
    (for speed we use the log likelihood instead of the likelihood)
    (also we use the gp functions we already have)
    '''

    # vehicle_type == "ugv": # rover
    default_ne_coord_ugv = Coordinate(35.73030799378120, -78.69670002283071) # "north-east" => max lat, max lon
    default_sw_coord_ugv = Coordinate(35.72774492720433, -78.69980159100491) # "south-east" => min lat, min lon

    # only interested in the ugv estimates
    def create_linespace(self,
                            ne_coord=default_ne_coord_ugv, 
                            sw_coord=default_sw_coord_ugv,
                            resolution=50 # number of points in the linespace
        ): 
            """Creates a linespace of points within a given area
            This is used to estimate the GP within the given area"""

            # calculate the relative ne and sw coordinates
            max_x,max_y = ne_coord.lat, ne_coord.lon
            min_x,min_y = sw_coord.lat, sw_coord.lon

            # create a linespace of coordinates
            x = np.linspace(min_x, max_x, resolution)
            y = np.linspace(min_y, max_y, resolution)
            xx = np.array(list(product(x, y))) # 2500x2
            linespace = xx # this is the raw float values of the linespace
            xx, yy = np.meshgrid(x, y)
            # create a list of coordinates
            linespace_coord = [] # this is the Coordinate class version of the linespace
            for i in range(resolution):
                for j in range(resolution):
                    linespace_coord.append(Coordinate(xx[i,j], yy[i,j]))
            # calculate the relative grid spacing between the first 2 points (in meters)
            dist = linespace_coord[0].distance(linespace_coord[1])
            return linespace, linespace_coord, dist, resolution

    def __init__(self) -> None:
        self.create_linespace()
        self.linespace = linespace

        self.kernel_list = []
        self.gp_list = []
        self.weights = [] # these are normalized weights
        self.weights_history = []

        for kernel in self.kernel_list: # create the GPs for each kernel
            self.gp_list.append(GaussianProcessRegressor(kernel=kernel, optimizer='fmin_l_bfgs_b',
                                                n_restarts_optimizer=30, #20
                                                copy_X_train=True,
                                                random_state=42))
            
# Paul Kudyba, OCT-2023

# this holds initializations for the rover search profile
# it holds constants and waypoints for scripted search patterns

from aerpawlib.util import Coordinate, VectorNED

from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
from sklearn.gaussian_process.kernels import Kernel, Hyperparameter, _check_length_scale

import numpy as np

class TrialInit:
    def __init__(self) -> None:
        # CONSTANTS
        self.STEP_SIZE = 40  # when going forward - how far, in meters (to be deprecated?)
        self.NORTHWEST = 315 # azimuth in degrees
        self.SEARCH_ALTITUDE = 35 # in meters

        self.MEASUREMENT_SPAN = 20 # in meters

        # GP waypoint optimizer
        self.epsilon = 0.1
        self.op_method = 'ucb'

        # waypoints
        self.waypoints = []
        #waypoint1
        self.waypoints.append(Coordinate(lat=35.727753, lon=-78.696723, alt=self.SEARCH_ALTITUDE))
        # #waypoint2
        self.waypoints.append(Coordinate(lat=35.727764, lon=-78.697129, alt=self.SEARCH_ALTITUDE))
        # #waypoint3
        self.waypoints.append(Coordinate(lat=35.728132, lon=-78.696719, alt=self.SEARCH_ALTITUDE))

        # RBF kernel for GP (default from robotarium)
        # self.kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2)) + \
        #                 WhiteKernel(noise_level=1, noise_level_bounds=(1e-10, 1e+1))
        
        # RBF kernel for GP (sim data tailoring)
        self.kernel = (C(9.18**2) * RBF(1.43) + \
                     WhiteKernel(noise_level=1.12e-1))

        # kernel for path loss (built-in)        


    def get_next_waypoint(self) -> Coordinate or None:
        '''returns (pops) the next waypoint in the list,
        if the list is empty, returns None'''
        return self.waypoints.pop(0) if len(self.waypoints)>0 else None
    
# create a non-stationary kernel for path loss
class NonStationaryKernel_PathLoss(Kernel):
    ''' Non-stationary kernel for path loss
    based on a LOS log-distance model (based on friis free space)
    a+b*10*log10(d) + c*log10(f)
    '''

    def __init__(self, ref_d=147.55, b=20, b_bounds=(20,60), c=20, c_bounds=(20,60), f=3.23e9, f_bounds=(3.08e9,3.38e9)):
        ''' inital values for ~3.23GHz
        ref_d = attenuation constant at reference distance (1m)
        b = path loss exponent (10*n) (n=2 for free space)
        c = frequency attenuation exponent (10*n) (n=2 for free space)
        f = center frequency in Hz
        '''
        self.ref_d = ref_d
        self.ref_d_bounds = (ref_d-10, ref_d+10) # default no adjustment
        self.b = b
        self.b_bounds = b_bounds
        self.c = c
        self.c_bounds = c_bounds
        self.f = f
        self.f_bounds = f_bounds

    @property
    def hyperparameter_ref_d(self):
        return Hyperparameter("ref_d", "numeric", self.a_bounds)
    
    @property
    def hyperparameter_b(self):
        return Hyperparameter("b", "numeric", self.b_bounds)
    
    @property
    def hyperparameter_c(self):
        return Hyperparameter("c", "numeric", self.c_bounds)
    
    @property
    def hyperparameter_f(self):
        return Hyperparameter("f", "numeric", self.f_bounds)
    
    @property
    def hyperparameters(self):
        return [self.hyperparameter_ref_d, self.hyperparameter_b, self.hyperparameter_c, self.hyperparameter_f]
    
    def is_stationary(self):
        return False
    
    def __call__(self, X, Y=None, eval_gradient=False):
        ''' X and Y are vectors of vectors
        X is the reference point
        Y is the point to evaluate
        '''
        if Y is None: # if Y is not provided, assume
            Y = X

        X = np.atleast_2d(X) # make sure X is a 2D array
        # Y = np.atleast_2d(Y)




        






        

    


    
    
        
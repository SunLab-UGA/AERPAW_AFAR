# Paul Kudyba, OCT-2023

# this holds initializations for the rover search profile
# it holds constants and waypoints for scripted search patterns

from aerpawlib.util import Coordinate, VectorNED

from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
from sklearn.gaussian_process.kernels import Kernel, Hyperparameter, _check_length_scale

import numpy as np

# import jax.numpy as jnp
# from jax import grad

class TrialInit:
    def __init__(self) -> None:
        # CONSTANTS        
        self.NORTHWEST = 315 # azimuth in degrees
        self.SEARCH_ALTITUDE = 40 # in meters # 35

        # offset for the dbm values (this should prevent negative values, by injecting an artificial mean)
        # we add this value before predicting, and subtract it after
        self.db_offset = 50 

        # movement parameters
        self.MEASUREMENT_SPAN = 30 # in meters # 30,45
        self.MEASUREMENT_SPAN_AFTER_3_MIN = 20
        self.MIN_MEASUREMENT_SPAN = 10 # minimum step size, in meters
        

        self.AUXILIARY_PATH_TRIGGER = 20 # dist in meters  #25
        self.AUXILIARY_PATH_SPAN = 50 # in meters ?? (radius to search)

        self.DROPPED_MEASUREMENT_LIMIT_AUX = 10 # number of dropped measurements before switching to aux path

        # GP waypoint optimizer
        self.epsilon = 1 # 0.01
        self.epsilon_decay = 0.50 # decay rate for epsilon # 0.85
        self.op_method = 'ucb'
        self.alpha = 2 # 1.5 # used to sharpen the peak finding of the gp, y=mean-std*alpha

        # if consecutive guesses are within a certain range, trigger the aux path routine
        self.settle_count_limit = 5 # the number of times that a guess is within the settle range, triggers aux path
        self.settle_range = 20 #(m) the range that a guess must be within to count as a settled guess


        # HARD CODED THRESHOLDS
        self.QUALITY_VARIENCE_CUTTOFF = 5 #10

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
        
        # RBF kernel for GP 
        # self.kernel = (C(9.18**2) * RBF(1.43, (1e-5,2.0)) + RBF(50,(10,90)) + \
        #              WhiteKernel(noise_level=1.12e-1))

        # RBF kernel for GP (found values)
        # self.kernel = (C(22.7**2) * RBF(1.93e-3, (1e-10,1)) + RBF(3.81e-5,(1e-5,1)) + # sum of 2 RBF kernels
        #             WhiteKernel(noise_level=8.97e-3)# noise level
        #             )
        
        # RBF kernel for GP (found values #2)
        # lets do variation over latitude and longitude at 2 different scales
        # self.kernel = (# sum of 2 RBF kernels (simulate a mean of signal propagation, and some fading variation)
        #                 C(30) * RBF(1.93e-3, (1e-10,1e1)) +  # long distance variation
        #                 RBF(3.81e-5,(1e-5,1)) + # short distance variation
        #                 WhiteKernel(noise_level=8.97e-3)# noise level
        #             )

        # 24.3**2 * RBF(length_scale=0.00215) + RBF(length_scale=0.00042) + WhiteKernel(noise_level=0.33)
        # self.kernel = (# sum of 2 RBF kernels (simulate a mean of signal propagation, and some fading variation)
        #                 C(24.3**2, constant_value_bounds="fixed") * RBF(0.00215, length_scale_bounds="fixed" ) +  # long distance variation
        #                 RBF(0.00042,(1e-5,1)) + # short distance variation
        #                 WhiteKernel(noise_level=0.33)# noise level
                    # )
        # self.kernel = (# sum of 2 RBF kernels (simulate a mean of signal propagation, and some fading variation)
        #                 24.3**2 * RBF(0.00215) +  # long distance variation (fixed scaling)
        #                 RBF(0.00042) + # short distance variation
        #                 WhiteKernel(noise_level=0.33)# noise level
        #             )

        # ~26db rise per 0.0037653511124474 abs distance lat/lon
        # after failed training: 8.31**2 * RBF(length_scale=0.00338) + 28.9**2 * RBF(length_scale=1.24e+04) + WhiteKernel(noise_level=0.389)
        # 56.9**2 * RBF(length_scale=0.00339) + WhiteKernel(noise_level=1.24)
        # 48.3**2 * RBF(length_scale=0.00382) + WhiteKernel(noise_level=2.59)
        # 54.6**2 * RBF(length_scale=0.00268) + WhiteKernel(noise_level=0.486)
        # 52.7**2 * RBF(length_scale=0.00281) + WhiteKernel(noise_level=0.646)
        # 57.9**2 * RBF(length_scale=0.005) + 0.505**2 * RBF(length_scale=0.000374) + WhiteKernel(noise_level=0.179)
        # 47**2 * RBF(length_scale=0.003) + 1.45**2 * RBF(length_scale=0.0002) + WhiteKernel(noise_level=0.761)
        # 40.5**2 * RBF(length_scale=0.003) + 1.26**2 * RBF(length_scale=0.0008) + WhiteKernel(noise_level=0.05)
        # 60.3**2 * RBF(length_scale=0.005) + 0.386**2 * RBF(length_scale=0.000528) + WhiteKernel(noise_level=0.098)
        # 53.6**2 * RBF(length_scale=0.00451) + 2.06**2 * RBF(length_scale=0.000306) + WhiteKernel(noise_level=0.193)
        # self.kernel = (# sum of 2 RBF kernels (simulate a mean of signal propagation, and some fading variation)
        #         C(1) * RBF(0.00276, (0.001,0.005)) +  # long distance variation # was maxed at 0.003
        #         C(1) * RBF(0.00042, (0.00020,0.00080)) + # short distance variation
        #         WhiteKernel(noise_level=0.33)# noise level
        #     )
        
        self.kernel = (# sum of 2 RBF kernels (simulate a mean of signal propagation, and some fading variation)
                C(1) * RBF(0.00276, (0.001,0.004)) +  # long distance variation # was maxed at 0.003
                # C(1) * RBF(0.00042, (0.00020,0.00080)) + # short distance variation
                WhiteKernel(noise_level=0.33)# noise level
            )


        # kernel for path loss (built-in)        
        # TODO!!!!

    def get_next_waypoint(self) -> Coordinate or None:
        '''returns (pops) the next waypoint in the list,
        if the list is empty, returns None'''
        return self.waypoints.pop(0) if len(self.waypoints)>0 else None
    
# class PathLossMeanFunction:
#     '''
#     this is the mean function for the GP based on a free space path loss model
#     a+b*10*log10((d1,d2)-(x1,y2))
#     and f is the frequency of the signal
#     a is the path loss at 1 meter
#     b is the path loss exponent
#     d1,d2 are the coordinates of the transmitter
#     x1,y2 are the coordinates of the receiver

#     length_scale is the length scale of the RBF kernel
#     '''
#     # TODO make bounds on a,b,f and d1,d2
#     def __init__(self, a:float, b:float, d1:float, d2:float, length_scale:float) -> None:
#         self.a = a
#         self.b = b
#         self.d1 = d1
#         self.d2 = d2
#         self.length_scale = length_scale

#     @staticmethod
#     def _log_distance(d1, d2, x1, x2):
#         return 10.0 * np.log10(np.sqrt((d1 - x1)**2 + (d2 - x2)**2))

#     def __call__(self, X:np.ndarray, Y=None, eval_gradient=False) -> np.ndarray:
#         '''returns the mean function for the GP'''
#         X1, X2 = np.split(X, 2, axis=1)
#         if Y is None:
#             Y = X
#         Y1, Y2 = np.split(Y, 2, axis=1)

#         mean_val = self.a + self.b * self._log_distance(self.d1, self.d2, X1, X2)

#         K  = np.exp(-0.5 * np.sum((X - Y)**2, axis=1) / self.length_scale**2)
#         K += mean_val

#         if eval_gradient:
#             return K, grad()
#             return K, np.empty((X.shape[0], X.shape[0], 0))





    



        






        

    


    
    
        
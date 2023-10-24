# Paul Kudyba, OCT-2023

# Designed to run a gaussian process on the data from the data manager

import numpy as np
from typing import List
from itertools import product

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
from sklearn.gaussian_process.kernels import Kernel

from scipy.stats import norm

from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.util import inside, readGeofence

import pickle

#TODO: add the creation of interactive figures for tracking the GP progress over time
#TODO: add the thresholding of the GP to assess the probability of the rover being with a certain area
#TODO: add Bayesian optimization to the GP to guide the search over time
#TODO: the bayesian optimization needs to select from only UAV linespace points!
#TODO: add a pickling function to save progress and data to a file

class ProcessFunction:

    # vehicle_type == "ugv": # rover
    default_ne_coord_ugv = Coordinate(35.73030799378120, -78.69670002283071) # "north-east" => max lat, max lon
    default_sw_coord_ugv = Coordinate(35.72774492720433, -78.69980159100491) # "south-east" => min lat, min lon

    # vehicle_type == "uav": # drone
    default_ne_coord_uav = Coordinate(35.72931030026633, -78.69621514941473) # "north-east" => max lat, max lon
    default_sw_coord_uav = Coordinate(35.72688213193035, -78.69953825817279) # "south-east" => min lat, min lon
    # this abstraction is only good for this quadrent of the globe

    # default path to the geofence file (the boundry limits of the uav search area)
    default_geofence_path_uav = "Geofences/AFAR_Drone.kml"

    # default kernel
    default_kernel = (C(9.18**2) * RBF(1.43) + \
                     WhiteKernel(noise_level=1.12e-1))

    def create_linespace(self,
                        ne_coord=default_ne_coord_uav, 
                        sw_coord=default_sw_coord_uav,
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
    
    def print_linespace(self, linespace_id="UGV", resolution=50):
        """print the linespace in a grid format"""
        if linespace_id == "UGV":
            linespace = self.linespace_ugv
        for i in range(len(linespace)):
            print(linespace[i], end=" ")
            if i % resolution == 0:
                print()

    def __init__(self,
                 geofence:str=default_geofence_path_uav,
                 kernel:Kernel=default_kernel
                 ) -> None:
        
        # UAV linespace and boundry to never violate       
        self.linespace_uav, self.linespace_uav_coords, self.grid_distance_uav, self.grid_resolution_uav \
            = self.create_linespace() # create the default linespace
        
        self.boundry_uav = readGeofence(geofence) # read the boundry from the file

        # UGV linespace (this is the space where the rover is allowed to be)
        self.linespace_ugv, self.linespace_ugv_coords, self.grid_distance_ugv, self.grid_resolution_ugv \
                                                    = self.create_linespace(
                                                    ne_coord=self.default_ne_coord_ugv,
                                                    sw_coord=self.default_sw_coord_ugv,
                                                    resolution=50
        )

        # GP data
        self.readings = [] # channel readings
        self.reading_positions = [] # positions of theose channel readings (lat,lon)
        self.num_readings = 0
        self.gp_ready = False # flag to indicate if the GP is fit to the most recent version of the data

        # track the last mean and std of the GP over the ugv linespace (used for plotting/searching)
        self.predictions = []
        self.std = []

        # track the means and mses
        self.Z_history = []
        self.Z_mse_history = []

        # track the last mean and std of the GP over the uav linespace (used for path optimization)
        self.predictions_uav = []
        self.std_uav = []
        self.Z_history_uav = []
        self.Z_mse_history_uav = []

        # track the optimizer history (locations of the next best point to sample)
        self.optimizer_history = []

        # kernel (default)
        # self.kernel = (C(0.01**2, (1e-4, 1e7)) * RBF(2.2, (1e-2, 1e2)) +
        #             WhiteKernel(noise_level=1e-10, noise_level_bounds=(1e-10, 1e+1)))
        
        # kernel (optimized)
        # self.kernel = (C(9.18**2) * RBF(1.43) +
        #             WhiteKernel(noise_level=1.12e-1))

        self.kernel = kernel # this is the initial kernel, the updated kernel is stored in the GP
        self.GP = GaussianProcessRegressor(kernel=self.kernel, optimizer='fmin_l_bfgs_b',
                                           n_restarts_optimizer=10, copy_X_train=True,
                                           random_state=42)

    def get_num_readings(self):
        return self.num_readings
    
    def update_auv_linespace(self, ne_coord, sw_coord, resolution=50):
        """update the linespace with a new boundry"""
        self.linespace_uav, self.grid_distance = self.create_linespace(ne_coord, sw_coord, resolution=resolution)
    
    def add_reading(self, position:Coordinate, reading, bias=1): # position is a list of [y,x] coordinates
        pos = [position.lat, position.lon] 
        z = reading * bias
        self.reading_positions.append(pos.copy())
        self.readings.append(z)

        self.num_readings += 1
        self.gp_ready = False # the GP is no longer ready (it needs to be updated)
        return self.reading_positions, self.readings
    
    def update_GP(self):
        """update the GP fit with newly added data"""
        if self.gp_ready: # if the GP is already ready, don't update it
            print("update_GP was called but the GP was already ready")
            return self.log_marginal_likelihood, self.log_marginal_likelihood_gradient
        else: 
            self.GP.fit(self.reading_positions, self.readings)
            self.gp_ready = True

            lml, lml_gradient = self.GP.log_marginal_likelihood(self.GP.kernel_.theta, eval_gradient=True)
            self.log_marginal_likelihood = lml
            self.log_marginal_likelihood_gradient = lml_gradient
            return self.log_marginal_likelihood, self.log_marginal_likelihood_gradient
        
    def predict(self, linespace_id="ugv"):
        """Predict the GP at the linespace points"""
        if not self.gp_ready:
            self.update_GP()
        
        # select the linespace to predict over
        if linespace_id == "ugv":
            linespace = self.linespace_ugv
            # predict the GP at the linespace points
            predictions, std = self.GP.predict(linespace, return_std=True)
            self.Z_history.append(predictions)
            self.Z_mse_history.append(std)
            self.predictions = predictions
            self.std = std

        elif linespace_id == "uav": # this is used ONLY for planning the next UAV waypoint
            linespace = self.linespace_uav
            # predict the GP at the linespace points
            predictions, std = self.GP.predict(linespace, return_std=True)
            self.Z_history_uav.append(predictions)
            self.Z_mse_history_uav.append(std)
            self.predictions_uav = predictions
            self.std_uav = std
        # elif linespace_id == "ugv_small":
        #     linespace = self.linespace_ugv_small



        return predictions, std
    
    def get_peak_prediction(self): # get the peak prediction of the GP for the ugv(rover)
        """get the peak prediction of the GP"""
        if not self.gp_ready:
            self.update_GP()
        
        self.peak_prediction = np.max(self.GP.predict(self.linespace_ugv))
        self.peak_prediction_index = np.argmax(self.GP.predict(self.linespace_ugv))
        return self.linespace_ugv[self.peak_prediction_index], self.peak_prediction
    
    ###################################################################################### VALIDATION
    def get_log_marginal_likelihood(self):
        """get the log marginal likelihood of the GP"""
        if not self.gp_ready:
            self.update_GP()
        return self.log_marginal_likelihood

    def get_probability(self):
        """get the probability of the GP to relate to a threshold"""
        if not self.gp_ready:
            self.update_GP()
        
        self.probability = norm.cdf(self.GP.predict(self.linespace_uav))
        return self.probability
    
    def get_probability_gradient(self):
        """get the probability gradient of the GP to relate to a threshold"""
        if not self.gp_ready:
            self.update_GP()
        
        self.probability_gradient = norm.pdf(self.GP.predict(self.linespace_uav))
        return self.probability_gradient
    
    ###################################################################################### File Saveing
    def save_pickle(self, filename="gp.pickle"):
        """save GP to a pickle file"""
        save_dict = {
                    # UAV
                    'linespace_uav':self.linespace_uav,
                    #  'linespace_uav_coords':self.linespace_uav_coords,
                    'grid_distance_uav':self.grid_distance_uav,
                    'grid_resolution_uav':self.grid_resolution_uav,

                    'predictions_uav':self.predictions_uav,
                    'std_uav':self.std_uav,
                    'Z_history_uav':self.Z_history_uav,
                    'Z_mse_history_uav':self.Z_mse_history_uav,
                    
                    # UGV
                    'linespace_ugv':self.linespace_ugv,
                    #  'linespace_ugv_coords':self.linespace_ugv_coords,
                    'grid_distance_ugv':self.grid_distance_ugv,
                    'grid_resolution_ugv':self.grid_resolution_ugv,

                    'predictions':self.predictions,
                    'std':self.std,
                    'Z_history':self.Z_history,
                    'Z_mse_history':self.Z_mse_history,
                     
                    'readings':self.readings, # channel readings
                    'reading_positions':self.reading_positions,
                    'num_readings':self.num_readings,
                    'kernel':self.GP.kernel_,

                    # optimizer history
                    'optimizer_history':self.optimizer_history
                     }
        with open(filename, 'wb') as f:
            pickle.dump(save_dict, f)
                    
    ###################################################################################### OPTIMIZATION
    # run bayesian optimization based on the current model
    # returns the next best point to sample based on the chosen acquisition function
    def process_optimizer(self, optimizer="ucb", epsilon=0.1):
        # simple acquisition function u(X) + epsilon*sigma(X)
        if optimizer == "ucb": # upper confidence bound
            # retrieve the most current gp predicted means and mses
            means = self.Z_history_uav[-1]
            mses = self.Z_mse_history_uav[-1]
            # calculate the ratios
            ratios = []
            for i in range(len(means)):
                ratios.append(means[i] + epsilon*mses[i])
            # find the max ratio
            # max_ratio = np.max(ratios)
            # find the index of the max ratio
            max_ratio_index = np.argmax(ratios)
            # find the x,y position of the max ratio
            max_ratio_pos = self.linespace_uav[max_ratio_index]
            # reshape the position to be a 2d array for plotting
            #Xplot = np.reshape(ratios, (self.stepx, self.stepy))

            # append the optimizer history
            self.optimizer_history.append(max_ratio_pos)
            return max_ratio_pos#, Xplot

        # POI = (u(X) - u(X+) - epsilon) / sigma(X), where X+ is the current argmax(u(X))
        # "if you know the best reward possible, it converges very quickly"
        elif optimizer == "poi": # probability of improvement
            # retrieve max mean from sample history
            mean_plus = np.max(self.readings)
            # retrieve the most current gp predicted means and mses
            means = self.Z_history_uav[-1]  # 2500x1
            mses = self.Z_mse_history_uav[-1]

            # calculate the POI
            poi = []
            for i in range(len(means)):
                poi.append((means[i] - mean_plus - epsilon) / mses[i])
            # find the max POI
            # max_poi = np.max(poi)
            # find the index of the max POI
            max_poi_index = np.argmax(poi)
            # find the gridspace x,y pos of the max POI
            xx = self.linespace_uav[max_poi_index]

            # reshape the X to be 2D for plotting
            #Xplot = np.reshape(poi, (self.stepx, self.stepy))

            # append the optimizer history
            self.optimizer_history.append(xx)
            return xx#, Xplot
        
        # EI = (u(X) - u(X+) - epsilon) * Phi(Z) + sigma(X) * phi(Z)
        elif optimizer == "ei": # expected improvement
            # retrieve max mean from sample history
            mean_plus = np.max(self.readings)
            # retrieve the most current gp predicted means and mses
            means = self.Z_history_uav[-1]  # 2500x1
            mses = self.Z_mse_history_uav[-1]

            # calculate the EI
            ei = []
            for i in range(len(means)):
                if mses[i] == 0:
                    ei.append(0)
                else:
                    Z = (means[i] - mean_plus - epsilon) / mses[i]
                    ei.append((means[i] - mean_plus - epsilon) * norm.cdf(Z) + mses[i] * norm.pdf(Z))
            # find the max EI
            # max_ei = np.max(ei)
            # find the index of the max EI
            max_ei_index = np.argmax(ei)
            # find the gridspace x,y pos of the max EI
            xx = self.linespace_uav[max_ei_index]
            
            # reshape the X to be 2D for plotting
            #Xplot = np.reshape(ei, (self.stepx, self.stepy))

            # append the optimizer history
            self.optimizer_history.append(xx)
            return xx#, Xplot





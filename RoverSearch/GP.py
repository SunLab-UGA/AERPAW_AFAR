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

import warnings
warnings.filterwarnings("ignore") # ignore the warnings from the GP

from init import TrialInit
# create a trial init object
INIT = TrialInit()



#TODO: add the thresholding of the GP to assess the probability of the rover being with a certain area

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

    def print_linespace_grid_resolution(self, linespace_id="UGV"):
        """print the linespace grid resolution"""
        if linespace_id == "UGV":
            grid_resolution = self.grid_resolution_ugv
        elif linespace_id == "UAV":
            grid_resolution = self.grid_resolution_uav
        print(f"{linespace_id} grid resolution:", grid_resolution)

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

        self.peak_prediction_history = [] # track the peak prediction magnitude over time
        self.peak_prediction_history_mse = [] # track the peak prediction mse over time
        self.peak_prediction_index_history = [] # track the peak prediction index over time
        self.peak_prediction_coords_history = [] # track the peak prediction coords over time (UGV LOCATION!)
        self.peak_prediction_linespace_history = [] # track the peak prediction linespace over time (UGV LOCATION!)

        self.refined_peak_prediction_coords_history = [] # track the refined peak prediction coords over time (UGV LOCATION!)
        self.refined_peak_prediction_linespace_history = [] # track the refined peak prediction linespace over time (UGV LOCATION!)
        self.small_linespace_history = [] # track the small linespace used to refine the peak prediction

        # track the means and mses
        self.Z_history = []
        self.Z_mse_history = []

        # track lml
        self.log_marginal_likelihood_history = []
        self.log_marginal_likelihood_gradient_history = []

        # track the last mean and std of the GP over the uav linespace (used for path optimization)
        self.predictions_uav = []
        self.std_uav = []
        self.Z_history_uav = []
        self.Z_mse_history_uav = []

        # functionalize the peak prediction with a constant alpha
        self.alpha = INIT.alpha
        self.db_offset = INIT.db_offset

        # track the optimizer history (locations of the next best point to sample)
        self.optimizer_history = []

        self.kernel = kernel # this is the initial kernel, the updated kernel is stored in the GP (_kernel)
        self.GP = GaussianProcessRegressor(kernel=self.kernel, optimizer='fmin_l_bfgs_b',
                                           n_restarts_optimizer=30, #20
                                           copy_X_train=True,
                                           random_state=42)

    def get_num_readings(self):
        return self.num_readings
    
    def update_auv_linespace(self, ne_coord, sw_coord, resolution=50):
        """update the linespace with a new boundry"""
        self.linespace_uav, self.grid_distance = self.create_linespace(ne_coord, sw_coord, resolution=resolution)
    
    def add_reading(self, position:Coordinate, reading): # position is a list of [y,x] coordinates
        pos = [position.lat, position.lon] 
        z = reading + self.db_offset # offset the reading to prevent negative values
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
            self.log_marginal_likelihood_history.append(lml)
            self.log_marginal_likelihood_gradient_history.append(lml_gradient)

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
            
            # functionalize the predictions by subtracting the mse from the mean for each point
            # this hopefully will make the GP more conservative in knowing its peak prediction
            for i in range(len(predictions)):
                predictions[i] = predictions[i] - self.alpha * std[i]

            # find the peak prediction
            self.peak_prediction = np.max(predictions)
            self.peak_prediction_index = np.argmax(predictions)
            # store the mse of the peak prediction
            peak_prediction_mse = std[self.peak_prediction_index]
            

            self.peak_prediction_history.append(self.peak_prediction)
            self.peak_prediction_index_history.append(self.peak_prediction_index)
            self.peak_prediction_coords_history.append(self.linespace_ugv_coords[self.peak_prediction_index])
            self.peak_prediction_linespace_history.append(self.linespace_ugv[self.peak_prediction_index])

            # and mse history
            self.peak_prediction_history_mse.append(peak_prediction_mse)

            # refine the peak prediction
            self.refine_peak_prediction()

        elif linespace_id == "uav": # this is used ONLY for planning the next UAV waypoint
            linespace = self.linespace_uav
            # predict the GP at the linespace points
            predictions, std = self.GP.predict(linespace, return_std=True)
            self.Z_history_uav.append(predictions)
            self.Z_mse_history_uav.append(std)
            self.predictions_uav = predictions
            self.std_uav = std

        # TODO!!!!
        # elif linespace_id == "ugv_small":
        #     linespace = self.linespace_ugv_small
        return predictions, std
    
    def refine_peak_prediction(self, resolution=10, length=0.000_01): # length is roughly degrees/meter, resolution is the number of meters
        '''refine the peak prediction to be more accurate
        by creating a smaller linespace around the peak prediction
        only call AFTER the GP has been updated, and the peak prediction has been found!'''
        scale = length * int(resolution/2) # roughly 10x10 meters
        # get the peak prediction coords
        peak_prediction = self.peak_prediction_linespace_history[-1]
        # split out the lat and lon
        lat = peak_prediction[0]
        lon = peak_prediction[1]
        # generate the north-east and south-west coordinates, shouldn't matter much which is which
        ne_coord = Coordinate(lat-scale, lon-scale)
        sw_coord = Coordinate(lat+scale, lon+scale)
        # use these to create a new linespace
        linespace, linespace_coord, dist, resolution = self.create_linespace(ne_coord, sw_coord, resolution=resolution)
        self.small_linespace_history.append(linespace) # track the small linespace history
        # print("small linespace:", linespace)

        #TODO remove any points that are outside of the boundry
        # remove_index = []
        # for i in range(len(linespace_coord)):
        #     if not inside(linespace_coord[i].lon, linespace_coord[i].lat, self.boundry_uav):
        #         remove_index.append(i)
        # # remove the points from the linespace
        # linespace = np.delete(linespace, remove_index, axis=0) 
        # linespace_coord = np.delete(linespace_coord, remove_index, axis=0)

        # predict the GP at the linespace points
        predictions = self.GP.predict(linespace)
        # find the peak prediction
        peak_prediction_index = np.argmax(predictions)
        # convert the index to a Coordinate
        refined_peak_prediction_linespace = linespace[peak_prediction_index]
        refined_peak_prediction_coords = linespace_coord[peak_prediction_index]
        # store the refined peak prediction
        self.refined_peak_prediction_linespace_history.append(refined_peak_prediction_linespace)
        self.refined_peak_prediction_coords_history.append(refined_peak_prediction_coords)
        
        return refined_peak_prediction_coords

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
    
    ###################################################################################### File Saving
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

                    'peak_prediction_history':self.peak_prediction_history,
                    'peak_prediction_coords_history':self.peak_prediction_linespace_history,
                    'refined_peak_prediction_coords_history':self.refined_peak_prediction_linespace_history,
                    'small_linespace_history':self.small_linespace_history, # going to find that bug!

                    # log marginal likelihood
                    'log_marginal_likelihood':self.log_marginal_likelihood_history,
                    'log_marginal_likelihood_gradient':self.log_marginal_likelihood_gradient_history,

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

    def nudge(self, waypoint:Coordinate, position:Coordinate, min_dist = 10, max_dist=35 ,direction="random"):
        """check min distance and nudge the waypoint to not repeat the same measurement"""
        limited_waypoint = position.limit_distance(waypoint, max_dist) # this is the default if no nudge is needed
        # check the distance between the waypoint and the current position
        dist = waypoint.distance(position)
        # print("distance between next waypoint and current position:", dist)
        nudged = False # flag to indicate if the waypoint was nudged
        if dist < min_dist:
            nudged = True
            # if the distance is too close, nudge the waypoint
            if direction == "random":
                print("~~nudging waypoint to prevent repeat measurement~~")
                # pick a random linespace point to go towards
                index = np.random.randint(0, len(self.linespace_uav))
                random_waypoint = self.linespace_uav_coords[index] # this is a Coordinate class
                # print("random waypoint selected:", random_waypoint.toJson())
                # relimit the distance to the min_dist (to prevent large steps)
                limited_nudged_waypoint = position.limit_distance(random_waypoint, min_dist)
                return limited_nudged_waypoint, nudged
            else: # safe default
                return limited_waypoint, nudged
        else: # safe default
            return limited_waypoint, nudged
        
    def check_rover_in_uav_boundry(self, mypos:Coordinate, tolerence=10) -> bool:
        """check if the mypos is within tolerence of rover and is within the UAV boundry"""
        # check distance to "rover" peak prediction
        rvr = self.peak_prediction_coords_history[-1]
        dist = mypos.distance(Coordinate(rvr.lat, rvr.lon, mypos.alt)) # take the altitude from the mypos
        if dist < tolerence:
            # check if the rover is within the UAV boundry
            if inside(rvr.lon, rvr.lat, self.boundry_uav):
                return True
            else:
                return False
        else:
            return False
   
    def aux_path(self, center=None, radius=50, num_points=10):
        """create a path around the rover peak prediction"""
        if center is None:
            rvr = self.peak_prediction_coords_history[-1] # use the most recent peak prediction
            # rvr = vehicle_location # just use the vehicle location instead (because we may call this with prediction outside of the boundry)
        else:
            rvr = center

        rvr = Coordinate(rvr.lat, rvr.lon, 0) # set the altitude to 0
        # create a circle of points around the rover peak prediction
        path = []
        # create a vector to rotate around
        pt = VectorNED(radius, 0, 0) # north, east, down
        for i in range(num_points):
            # calculate the angle
            angle = i * (360/num_points)
            # calculate the point
            point = pt.rotate_by_angle(angle)
            # add the rover peak prediction to the point
            point =  rvr + point
            # append the point to the path
            path.append(point)
        
        print(f"{len(path)} aux path created:")
        for i in range(len(path)):
            print(path[i].toJson())

        # check if the rover is within the UAV boundry
        path_in_boundry = []
        for i in range(len(path)):
            if inside(path[i].lon, path[i].lat, self.boundry_uav):
                path_in_boundry.append(path[i])
            else:
                # find the closest point in the linespace that is within the UAV boundry
                dist = np.ones(len(self.linespace_uav_coords)) * 1000000 # np array dist with a very large number becasue we want to find the min
                for j in range(len(self.linespace_uav_coords)):
                    dist[j] = path[i].distance(self.linespace_uav_coords[j])
                min_index = np.argmin(dist)
                path_in_boundry.append(self.linespace_uav_coords[min_index])
                    
        print(f"{len(path_in_boundry)} aux path after boundry check:")
        for i in range(len(path_in_boundry)):
            print(path_in_boundry[i].toJson())

        return path_in_boundry # this is a list of Coordinate objects, remember to check alt!








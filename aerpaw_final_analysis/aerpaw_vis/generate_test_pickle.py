import numpy as np
from scipy.stats import multivariate_normal
import pickle

# first create a single time step of the grid for testing

basis_size = 300
steps = 50

# create a 3D numpy array with a gaussian distribution centered in the middle
z = np.zeros((basis_size, basis_size))
# create a gaussian distribution
def f(X): # X is a 2D numpy array
    '''Gaussian distribution centered in the middle'''
    z = 0
    std_dev = 50
    mean = [basis_size/2, basis_size/2]
    z += multivariate_normal.pdf(X, mean=mean, cov=std_dev)
    return z

# create the x and y axes (2D static)
x = np.arange(0, basis_size, int(basis_size/steps))
y = np.arange(0, basis_size, int(basis_size/steps))
xx, yy = np.meshgrid(x, y)




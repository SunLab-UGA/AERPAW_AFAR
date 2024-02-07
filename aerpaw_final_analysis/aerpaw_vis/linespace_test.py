

import numpy as np
from itertools import product

def linespace(resolution=10, length=1):
        scale = length * int(resolution/2) # roughly 10x10 meters
        lat = 0
        lon = 20
        lats = np.linspace(lat-scale, lat+scale, resolution)
     
        lons = np.linspace(lon-scale, lon+scale, resolution) # this is flipped due to linespace creation ...the way it was written...

        linespace = np.array(list(product(lats, lons)))
        # linespace = np.array(list(product(lons, lats)))
        # linespace = linespace[:,[1,0]] # flip the order of the coordinates to match the order of the coordinates in the og linespace
        return linespace


# main entry point
if __name__ == '__main__':
        # test the function
        linespace = linespace(resolution=10, length=1)
        print(linespace)
        
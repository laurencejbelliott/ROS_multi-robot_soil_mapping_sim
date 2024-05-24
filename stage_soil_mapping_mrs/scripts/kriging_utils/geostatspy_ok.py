#!/usr/bin/env python3
import geostatspy.GSLIB as GSLIB                       # GSLIB utilies, visualization and wrapper
import geostatspy.geostats as geostats                 # GSLIB methods convert to Python
from matplotlib import pyplot as plt
import os
import math
import rospkg
import pandas as pd
import numpy as np

# Set the working directory
rospack = rospkg.RosPack()
package_path = rospack.get_path('stage_soil_mapping_mrs')
os.chdir(package_path + '/scripts/kriging_utils')


def predict_by_kriging(gridx, gridy, x_arr, y_arr, o_arr, variogram='spherical'):
    """
    Predict values at unsampled locations based on observations from other locations.
    Use Ordinary Kriging with a particular variogram implemented in PyKrige. 
    Note that at least three observations are recommended as input because otherwise the 
    OrdinaryKriging function generates some errors.
    Tutorial: https://geostat-framework.readthedocs.io/projects/pykrige/en/stable/examples/00_ordinary.html
    
    Inputs: 
    - gridx, gridy: 1D array or list of x,y coordinates to print in the output 
    - x_arr, y_arr: 1D array or list of x,y coordinates of observed locations 
    - o_arr: 1D array or list of observed values 
    - variogram: Type of variogram model to use, i.e., one of 'gaussian', 'exponential', 'spherical', or 'linear'
      - Ref: https://geostat-framework.readthedocs.io/projects/pykrige/en/stable/variogram_models.html
      
    Outputs:
    - m: 2D arrays of the means from Kriging predictions
    - v: 2D arrays of the variances from Kriging predictions
    """
    
    assert len(x_arr) == len(y_arr) and len(y_arr) == len(o_arr), 'Lengths must be equivalent in x_arr, y_arr, and o_arr'
    
    df = pd.DataFrame({'X': x_arr, 'Y': y_arr, 'Property': o_arr})
    print(df)

    # Data locations
    xmin = np.min(x_arr); xmax = np.max(x_arr)               # range of x values
    ymin = np.min(y_arr); ymax = np.max(y_arr)               # range of y values

    xsiz = 1; ysiz = 1                    # cell size
    nx = xmax; ny = ymax               # number of cells
    xmn = 0; ymn = 0                        # grid origin, location center of lower left cell

    cmap = plt.cm.viridis                    # color map

    # Kriging parameters
    skmean_por = 0.10                          # simple kriging mean (used if simple kriging is selected below)
    ktype = 1                                  # kriging type, 0 - simple, 1 - ordinary
    radius = 300                               # search radius for neighbouring data
    nxdis = 1; nydis = 1                       # number of grid discretizations for block kriging (not tested)
    ndmin = 0; ndmax = int(np.floor(np.max(o_arr))) # minimum and maximum data for an estimate
    por_min = 0.0; por_max = np.max(o_arr)     # minimum property value


    # Kriging interpolation and visualization
    por_vario = GSLIB.make_variogram(nug=0.0,nst=1,it1=1,cc1=1.0,azi1=45,hmaj1=300,hmin1=300) # Property variogram

    por_kmap, por_vmap = geostats.kb2d(df,'X','Y','Property',por_min,por_max,nx,xmn,xsiz,ny,ymn,ysiz,nxdis,nydis,
            ndmin,ndmax,radius,ktype,skmean_por,por_vario)

    # plt.subplot(121)
    GSLIB.locpix_st(por_kmap,xmin,xmax,ymin,ymax,xsiz,np.min(por_kmap),np.max(por_kmap),df,'X','Y','Property','Ordinary Kriging Estimates and Data','X(m)','Y(m)','Property',cmap)
    plt.show()
    plt.savefig("./geostatspy_outputs/Mean.png")
    plt.close()

    # plt.subplot(122)
    print("Variance map type: ", type(por_vmap))
    print("Variance map shape: ", por_vmap.shape)
    print("Variance map: ")
    print(por_vmap)

    GSLIB.pixelplt_st(por_vmap,xmin,xmax,ymin,ymax,xsiz,np.min(por_vmap),np.max(por_vmap),'Kriging Variance','X(m)','Y(m)',r'Variance',cmap)
    plt.show()
    plt.savefig("./geostatspy_outputs/Variance.png")
    plt.close()

    m = por_kmap
    v = por_vmap
    
    return m, v

if __name__ == "__main__":
    # Define parameters (Explanations are in kriging.py)
    xgrid = np.arange(1, 10, 1)
    ygrid = np.arange(1, 10, 1)

    x_arr = [1, 1,   2, 2,   3, 4,   5, 6, 7, 9  ]
    y_arr = [1, 9,   2, 5,   3, 6,   4, 5, 6, 4  ]
    o_arr = [3, 4.2, 4, 4.5, 5, 5.5, 5, 2, 1, 3.3]
    
    predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram='gaussian')
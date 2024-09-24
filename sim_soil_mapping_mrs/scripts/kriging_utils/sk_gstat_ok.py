#!/usr/bin/env python3
from skgstat import Variogram, OrdinaryKriging
import pandas as pd
import numpy as np


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
    
    # Format the data as a pandas DataFrame for the Variogram
    data = pd.DataFrame({'x': x_arr, 'y': y_arr, 'z': o_arr})
    print(data)
    
    V = Variogram(data[['x', 'y']].values,
              data.z.values,
              maxlag=90,
              n_lags=25,
              model=variogram,
              normalize=False)
    
    # Ordinary Kriging
    ok = OrdinaryKriging(V,
                         min_points=3,
                         mode='estimate',
                         sparse=True
                         )

    # Placeholder grid for predictions and variance
    xx, yy = np.meshgrid(gridx, gridy)

    # Create grid of predictions
    m = ok.transform(xx.flatten(), yy.flatten()).reshape(xx.shape)

    # Create grid of variances / kriging errors
    v = ok.sigma.reshape(xx.shape)
    return m, v

if __name__ == "__main__":
    # Define parameters (Explanations are in kriging.py)
    xgrid = np.arange(1, 10, 1)
    ygrid = np.arange(1, 10, 1)
    x_arr = [1, 1,   2, 2,   3, 4,   5, 6, 7, 9  ]
    y_arr = [1, 9,   2, 5,   3, 6,   4, 5, 6, 4  ]
    o_arr = [3, 4.2, 4, 4.5, 5, 5.5, 5, 2, 1, 3.3]
    
    predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram='gaussian')
#!/usr/bin/env python3
from sk_gstat_ok import predict_by_kriging
import matplotlib.pyplot as plt
import numpy as np

# Define parameters (Explanations are in kriging.py)
xgrid = np.arange(1, 10, 1)
ygrid = np.arange(1, 10, 1)
x_arr = [1, 1,   2, 2,   3, 4,   5, 6, 7, 9  ]
y_arr = [1, 9,   2, 5,   3, 6,   4, 5, 6, 4  ]
o_arr = [3, 4.2, 4, 4.5, 5, 5.5, 5, 2, 1, 3.3]

# Plot the data for comparison with the kriging prediction
figs_dir = "./skgstat_outputs/"

plt.figure('Data')
plt.scatter(x_arr, y_arr, c=o_arr)
plt.colorbar()
plt.title('Ground Truth Data')
plt.savefig(figs_dir+'Data.png')

variogram = 'gaussian'

# Run prediction 
m, v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram=variogram)

# Export as images for instruction purposes
plt.figure('Mean')
plt.imshow(m, origin='lower')
plt.colorbar()
plt.title('Kriging Prediction')
plt.savefig(figs_dir+'Mean.png')
plt.figure('Variance')
plt.imshow(v, origin='lower')
plt.colorbar()
plt.title('Kriging Variance')
plt.savefig(figs_dir+'Variance.png')
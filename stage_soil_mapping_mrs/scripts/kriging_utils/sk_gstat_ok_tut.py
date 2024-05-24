#!/usr/bin/env python3
from skgstat import Variogram, OrdinaryKriging
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import os


data = pd.read_csv('sample_lr.csv')

print(data)

V = Variogram(data[['x', 'y']].values,
              data.z.values,
              maxlag=90,
              n_lags=25,
              model='gaussian',
              normalize=False)

figs_dir = "./skgstat_outputs/"

# Create figs_dir if it does not exist
if not os.path.exists(figs_dir):
    os.makedirs(figs_dir)

# Plot the Variogram
V.plot(show=False)
plt.savefig(figs_dir+'Variogram.png')

# Ordinary Kriging
ok = OrdinaryKriging(V, min_points=3, mode='exact')

# Visualise the prediction
xx, yy = np.meshgrid(np.linspace(0, 100, 100), np.linspace(0, 100, 100))

# Create grid of predictions
field = ok.transform(xx.flatten(), yy.flatten()).reshape(xx.shape)
# Visualise the predictions
plt.figure()
plt.imshow(field, origin='lower')
plt.savefig(figs_dir+'Mean.png')

# Create grid of variances / kriging errors
s2 = ok.sigma.reshape(xx.shape)
# Visualise the variances
plt.figure()
plt.imshow(s2, origin='lower')
plt.savefig(figs_dir+'Variance.png')
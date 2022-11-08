__author__ = "Laurence Roberts-Elliott"

from matplotlib import pyplot as plt
from kriging_utils.kriging import predict_by_kriging
import numpy as np
import pickle
with open('../include/stage_soil_mapping_mrs/ground_truth/interpolated_jaime_compaction_0cm_kpas.pickle', 'rb') as f:
    ground_truth = pickle.load(f)

plt.imshow(ground_truth, origin='lower')
plt.title("Ground Truth")
# plt.show(block=False)
plt.show()
# plt.pause(2)
plt.close()

sampled_points = np.ones((ground_truth.shape[0], ground_truth.shape[1])) * np.nan
sampled_points[::10, ::10] = ground_truth[::10, ::10]
plt.imshow(sampled_points, origin='lower')
plt.title("Sampled Points")
plt.show(block=False)
# plt.show()
plt.pause(0.1)

# Perform kriging interpolation from sampled values in dataset
# Define parameters (Explanations are in kriging.py)

# Use Gaussian variogram model
variogram = 'gaussian'

# Set kriging grid dimensions to those the environment
x_grid = np.arange(0, ground_truth.shape[0], 1)
y_grid = np.arange(0, ground_truth.shape[1], 1)

# Define location and data of sampled points
x_arr = []
y_arr = []
for i in range(ground_truth.shape[0]):
    for j in range(ground_truth.shape[1]):
        if not np.isnan(sampled_points[i, j]):
            x_arr.append([i])
            y_arr.append([j])

o_arr = np.array([sampled_points[i, j] for i in range(ground_truth.shape[0]) for j in range(ground_truth.shape[1]) if
                  not np.isnan(sampled_points[i, j])])

print("Interpolating, please wait...")

# print(len(x_arr))
# print(len(y_arr))
# print(len(o_arr))

# print(x_arr.shape)
# print(y_arr.shape)
# print(o_arr.shape)
#
# print(x_arr[0])
# print(y_arr[0])
# print(o_arr[0])

# Run interpolation of sampled points
m, v = predict_by_kriging(y_grid, x_grid, y_arr, x_arr, o_arr, variogram=variogram)

plt.close()
plt.imshow(m, origin="lower")
plt.title("Interpolated Points")
# plt.show(block=False)
# plt.pause(10)
plt.close()

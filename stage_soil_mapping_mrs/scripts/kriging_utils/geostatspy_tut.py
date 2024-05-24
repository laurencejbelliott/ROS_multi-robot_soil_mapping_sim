#!/usr/bin/env python3
import geostatspy.GSLIB as GSLIB                       # GSLIB utilies, visualization and wrapper
import geostatspy.geostats as geostats                 # GSLIB methods convert to Python

import os                                               # to set current working directory 
import rospkg                                           # to get an absolute file path to package data
import numpy as np                                      # arrays and matrix math
import pandas as pd                                     # DataFrames
import matplotlib.pyplot as plt                         # plotting

# Set the working directory
rospack = rospkg.RosPack()
package_path = rospack.get_path('stage_soil_mapping_mrs')
os.chdir(package_path + '/scripts/kriging_utils')

fraction_data = 0.2                                     # extract a fraction of data for demonstration / faster runs, set to 1.0 for homework

#df = pd.read_csv("sample_data_MV_biased.csv")                     # read a .csv file in as a DataFrame
df = pd.read_csv("https://raw.githubusercontent.com/GeostatsGuy/GeoDataSets/master/sample_data_MV_biased.csv") # load the data from Dr. Pyrcz's GitHub repository

if fraction_data < 1.0:
    df = df.sample(frac = fraction_data,replace = False,random_state = 73073)
df = df.reset_index()
df = df.iloc[:,2:]

df['LogPerm'] = np.log(df['Perm'].values)

print(df.head())                                               # we could also use this command for a table preview 

print(df.describe().transpose())                          # summary table of all facies combined DataFrame statistics

# Data locations
xmin = 0.0; xmax = 1000.0               # range of x values
ymin = 0.0; ymax = 1000.0               # range of y values

xsiz = 10; ysiz = 10                    # cell size
nx = 100; ny = 100                      # number of cells
xmn = 5; ymn = 5                        # grid origin, location center of lower left cell

cmap = plt.cm.plasma                    # color map

# plt.subplot(121)
# GSLIB.locmap_st(df,'X','Y','Porosity',0,1000,0,1000,0,0.25,'Porosity - All Facies','X (m)','Y (m)','Nscore Porosity',cmap)

# plt.subplot(122)
# GSLIB.locmap_st(df,'X','Y','Perm',0,1000,0,1000,0,1000,'Permeability - All Facies','X (m)','Y (m)','Nscore Permeability',cmap)

# plt.subplots_adjust(left=0.0, bottom=0.0, right=2.0, top=1.0, wspace=0.3, hspace=0.3)
# plt.show()



# Kriging parameters
skmean_por = 0.10; skmean_perm = 65.0      # simple kriging mean (used if simple kriging is selected below)
ktype = 1                                  # kriging type, 0 - simple, 1 - ordinary
radius = 300                               # search radius for neighbouring data
nxdis = 1; nydis = 1                       # number of grid discretizations for block kriging (not tested)
ndmin = 0; ndmax = 40                      # minimum and maximum data for an estimate
por_min = 0.0; por_max = 0.3               # minimum property value
perm_min = 0.0; perm_max = 1000.0          # minimum property value


# Kriging interpolation and visualization
por_vario = GSLIB.make_variogram(nug=0.0,nst=1,it1=1,cc1=1.0,azi1=45,hmaj1=300,hmin1=300) # porosity variogram

por_kmap, por_vmap = geostats.kb2d(df,'X','Y','Porosity',por_min,por_max,nx,xmn,xsiz,ny,ymn,ysiz,nxdis,nydis,
         ndmin,ndmax,radius,ktype,skmean_por,por_vario)

# plt.subplot(121)
GSLIB.locpix_st(por_kmap,xmin,xmax,ymin,ymax,xsiz,0.0,0.25,df,'X','Y','Porosity','Ordinary Kriging Estimates and Data','X(m)','Y(m)','Porosity (%)',cmap)
plt.show()
plt.savefig("./geostatspy_outputs/Mean.png")
plt.close()

# plt.subplot(122)
GSLIB.pixelplt_st(por_vmap,xmin,xmax,ymin,ymax,xsiz,0.0,1.0,'Kriging Variance','X(m)','Y(m)',r'Porosity ($\%^2$)',cmap)
plt.show()
plt.savefig("./geostatspy_outputs/Variance.png")
plt.close()

# plt.subplots_adjust(left=0.0, bottom=0.0, right=2.0, top=1.0, wspace=0.3, hspace=0.3); plt.show()

# # Print working directory
# print(os.getcwd())
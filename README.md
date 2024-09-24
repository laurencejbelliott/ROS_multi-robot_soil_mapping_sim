# ROS Multi-Robot Soil Mapping Sim
A ROS coordinator and simulation for multi-robot soil properties mapping.

This centralised coordinator directs a team of robots in mapping geospatial data using Ordinary Kriging (OK) interpolation. The kriging variance, a metric of the uncertainty of the interpolation's predictions, is used to decide where robots should sample, and informs allocation of these sampling tasks to the robots.

A simulation for demonstration and testing of this soil mapping multi-robot system, using the [move_base_abstract](https://github.com/laurencejbelliott/move_base_abstract) simulator, is included alongside the coordinator in this repository.

## Setup
With your working directory as the `src` directory of a [catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace), clone the repo using:
```
git clone https://github.com/laurencejbelliott/ROS_multi-robot_soil_mapping_sim.git --recursive
```
Then navigate to the root directory of the workspace and install the package's dependencies with:
```
rosdep install --from-paths src --ignore-src -r -y
```
Finally, build the package with `catkin build`.

## Run Demo Soil Compaction Sampling Mission
To run an example sampling mission use the command:
```
rosrun sim_soil_mapping_mrs run_single_trial.py
```

## Run ICRA '25 Soil Compaction Sampling Trials
To replicate the soil compaction mapping trials run for the ICRA '25 submission use the command:
```
rosrun sim_soil_mapping_mrs run_bid_func_trials.py
```
Please note this will take a long time to complete, and the data produced from these trials will require a significant amount of free storage space.
# 3D Smooth path planning for UAVs 

## RRT + Pruning 
To get the RRT path and the pruned path, run the following code:
```
$ python Code/main.py
```
The starting and the ending position for this path planning are given to be
(-4,-4,0) and (4,4,0) respectively.

The output path of the code is written as an `npy` file, `final_rrt_prune_path_coords.npy`.

## Smoothing
The pruned path is then smoothed using the `smooth_path.py` code. Run the following command to get
the smooth path:
```
$ python Code/smooth_path.py
```
The above code reads the pruned path from the `final_rrt_prune_path_coords.npy` file and gives the 
smooth path as another `npy` file, `final_rrt_prune_smooth_path_coords.npy`.

This smooth path is then used by the simulator to simulate the final path.

## Simulation
-Install the px4 gazebo simulator from the fllowing website.
`https://dev.px4.io/v1.9.0/en/simulation/gazebo.html`

- Follow the instructions given in this website to launch the px4 quad.
`https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html`

- In these instructions while exectuting the `roslaunch px4 posix_sitl.launch
` use the following arguments.
```
roslaunch px4 posix_sitl.launch world:=/home/sandeep/px4_ws/src/px4_controller/world/asd.world
```

- Run the script `../src/px4_controller/rrt-uav/Code/main.py` to generate the path.

- To run the simulation run the following command in a new ros terminal.
```
rosrun px4_controller position_control.py 
```

This will start the simulation.

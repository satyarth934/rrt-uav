# Readme 

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

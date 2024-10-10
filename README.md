# Usage

Explain how to use the package, including launching the individual components and any necessary configuration steps.

```bash
ros2 launch map_server map_server.launch.py use_sim_time:=true
```

This command launches the map server node, which publishes the generated map of the warehouse. The map file for the simulation environment is specified.
Launch the map server with a real environment map:

```bash
ros2 launch map_server map_server.launch.py use_sim_time:=true
```

Similar to the previous command, but this time it launches the map server with the map of the real warehouse environment.
Launch the localization server with a simulation map:

```bash
ros2 launch localization_server localization.launch.py use_sim_time:=true
```

This command starts the localization server using the simulation map. The server is responsible for the robot's self-localization within the map.
Launch the localization server with a real environment map:

```bash
ros2 launch localization_server localization.launch.py use_sim_time:=true
```
This command is used to launch the localization server with the map of the real environment.
Launch the path planner server:

```bash
ros2 launch path_planner_server navigation.launch.py
```

This command initiates the path planning process, enabling the robot to navigate autonomously in the warehouse.

```bash
ros2 launch tracer_base sensor.xml
```

This command activate the sensor to gain the data with TF Tree.

```bash
ros2 launch tracer_base ros2_aruco.xml
```
This command activate the aruco node to transfer the 3D Pose into the map coordinate.

```bash
ros2 run navigatethroughpose navigatethroughpose
```
This command moves the Tracer from one point pose to another point pose with three times, then moving to the aruco pose position based on map coordinate.

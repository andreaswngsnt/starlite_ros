# starlite_ros
*This package is meant to be run with ROS2 Foxy Fitzroy on Ubuntu 20.04*

## Running this ROS package
*Assuming that `ros2` command is enabled already.*
Run these commands at first run, when new files are created, or if something does not run:
1. Build the packages: `colcon build --symlink-install`
2. Setup package: `source install/setup.bash`

Some things that can be done:
- Publish robot description: `ros2 launch starlite_ros rsp.launch.py use_sim_time:=true`
- Publish joint states & run its GUI: `ros2 run joint_state_publisher_gui joint_state_publisher_gui`
- Launch RVIZ: `rviz2`
- Launch Gazebo: `ros2 launch gazebo_ros gazebo.launch.py`
- Spawn the robot in Gazebo: `ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot`
- Publish robot description, launch Gazebo, and spawn entity at once: `ros2 launch starlite_ros launch_sim.launch.py world:=./src/starlite_ros/worlds/my_world.world`
- Control robot in Gazebo: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## Adding a Python Node
There are a few things that needs to be done:
- Create the .py file in the /scripts folder.
- Make sure to add `#!/usr/bin/env python3` as the first line of the file.
- Add any node dependencies to `package.xml` and add `<exec_depend>node_dependency</exec_depend>`
- If you want the Python script to be executable, run `chmod +x your_node.py`, edit `CMakeLists.txt` and add `scripts/your_node.py` after the `DIRECTORY` keyword.
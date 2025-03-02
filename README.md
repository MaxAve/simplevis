# simplevis
Simple utility built with ROS2 for visualizing messages and controlling the robot.\
Note that this tool was built specifically for Turtlebots and likely won't work any any other device for certain use cases.
<img src="Screenshot_20250104_193247.png" alt="Example use of simplevis" width="500"/>\
### Installation
```bash
# cd into your ROS2 workspace's src directory
cd ~/ros2_ws/src

# Clone repo
git clone https://github.com/MaxAve/simplevis.git

# Return to workspace
cd ..

# Install dependencies
rosdep install -i --from-path src --rosdistro humble -y

# Build package (note: run colcon build if the package can't be found)
colcon build --packages-select simplevis

# Source workspace
source install/setup.bash

# Run a node in simplevis to test the installation
ros2 run simplevis <node>
```
### Running
Run with
```
ros2 run simplevis turtlebot
```

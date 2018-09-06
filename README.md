# egh450_path_planning
 An example ROS package to perform high-level path-planning utilizing breadcrumb

## Download & Compile
Before you compile this node, ensure you have [downloaded and compiled breadcrumb](https://github.com/qutas/breadcrumb).

```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/egh450_path_planning
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Usage
To run a basic demonstration off of contrail navigation interface, first launch `uavusr_emulator` guidance example as per usual. Next, you will need to make a copy of the demo launch file:
```
mkdir -p ~/catkin_ws/launch
cd ~/catkin_ws/launch
roscp egh450_path_planning planner_demo.launch ./
```

Next, edit the newly created launch file, using the `remap` roslaunch commands to connect the planner interfaces with the breadcrumb and contrail interfaces (identify them using the `rostopic list`):
```
nano ~/catkin_ws/launch/planner_demo.launch
```

Once ready, save and run the launch file with the command:
```sh
roslaunch ~/catkin_ws/launch/planner_demo.launch
```

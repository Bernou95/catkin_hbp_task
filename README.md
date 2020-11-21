# catkin_hbp_task

Updated with the nodes that solves the two behaviors requested 

## Installation

1.-Clone the provided workspace and build it running catkin_make from the root folder (tested in Ubuntu 20.04 / ROS noetic / Gazebo 11)  
2.-Add the following lines to your .bashrc file and source it 

```bash
source /opt/ros/noetic/setup.bash
export CATKIN_WS=/replace/with/the/path/to/catkin_hbp_task
source $CATKIN_WS/devel/setup.bash
export GAZEBO_MODEL_Path=$CATKIN_WS/src/gazebo_environment/models
```

## Usage

```bash
roslaunch gazebo_environment hbp_husky.launch
```

in a second terminal for first behavior

```bash
rosrun tracking_husky tracking_husky
```
or for the second behavior
```bash
rosrun tracking_husky vision_husky
```

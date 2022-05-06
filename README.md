# OMPL in ROS for search based motion planning 

H. Lewis, Lu (luh.lewis@gmail.com)

The screenshot is the RRT-Connect implementation of the OMPL based on the OccupancyGrid model. You could replace the map model with Costmap as well.

![ompl_rrt_connect](result/Screenshot%20from%202022-05-06%2022-26-15.png)

To use this repository, please do as follows.

- Install the OMPL library from the source or from the ros.

- In your working directory:
```
mkdir -p ompl_ws/src
cd ompl_ws/src
git clone git@github.com:Lewis-Lu/ompl_planner.git
cd ..
catkin_make
source devel/setup.bash
roslaunch ompl_planner ompl_planner.launch
```

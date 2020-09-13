#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 3
xterm -e "roslaunch add_markers launch_rviz.launch" &
sleep 10
xterm -e "rosrun add_markers add_markers_node" &
xterm -e "rosrun pick_objects pick_objects_node" &
sleep 3

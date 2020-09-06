#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 2
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
xterm -e "rosrun add_markers add_markers_node" &
sleep 3

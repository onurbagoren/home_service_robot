#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/$USER/catkin_ws/src/world/new_world.world" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/$USER/catkin_ws/src/world/obagoren.yaml" &
sleep 3
xterm -e "roslaunch add_markers launch_rviz.launch" &
sleep 10
xterm -e "rosrun add_markers sample_node" &
sleep 3
#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/$USER/catkin_ws/src/world/new_world.world" &
sleep 3
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/$USER/catkin_ws/src/world/obagoren.yaml" &
sleep 3
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
xterm -e "rosrun pick_objects pick_objects_node" &
sleep 3
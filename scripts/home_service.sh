#!/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/$USER/catkin_ws/src/world/new_world.world" &
# xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/$USER/catkin_ws/src/world/obagoren.yaml" &
# xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 3
xterm -e "roslaunch add_markers launch_rviz.launch" &
sleep 10
xterm -e "rosrun pick_objects pick_objects_node" &
xterm -e "rosrun add_markers add_markers_node" &
sleep 3
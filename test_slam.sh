#!/bin/sh

# xterm -e " source /opt/ros/noetic/setup.bash; roscore" &
# sleep 2
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 3
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 3
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"

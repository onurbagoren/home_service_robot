#!/bin/sh

# xterm -e " source /opt/ros/noetic/setup.bash; roscore"
# sleep 1
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo slam_gmapping" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"

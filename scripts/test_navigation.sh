#1/bin/sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/$USER/catkin_ws/src/world/new_world.world" &
sleep 3
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch map_file:=/home/$USER/catkin_ws/src/world/obagoren.yaml" &
sleep 3
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"


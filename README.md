# Home Service Robot
## Onur Bagoren

## Description
Home service robot that will pick up an item on a map and then srop it off at a known location

## Execution
Create catkin workspace:
```
$ mkdir -p /home/$USER/catkin_ws/src
$ cd /home/$USER/catkin_ws/src/
$ catkin_init_workspace
$ cd ..
$ catkin_make
```
Clone appropriate libraries in order to execute:
```
sudo apt-get install xterm
cd /home/$USER/catkin_ws/src
git clone git@github.com:ros-perception/slam_gmapping.git
git clone git@github.com:turtlebot/turtlebot.git
git clone git@github.com:turtlebot/turtlebot_simulator.git
git clone git@github.com:turtlebot/turtlebot_interactions.git
```

Build:
```
cd /home/$USER/catkin_ws/
catkin_make
```

Execute test slam:
Execute home service:
```
cd /home/$USER/catkin_ws/
./src/scripts/test_slam.sh
```
Image from the map that was produced:


Execute home service:
```
cd /home/$USER/catkin_ws/
./src/scripts/home_service.sh
```

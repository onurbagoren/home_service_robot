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
```
cd /home/$USER/catkin_ws/
./src/scripts/test_slam.sh
```
Image from the map that was produced:
![1st Image from slam](https://github.com/onurbagoren/home_service_robot/blob/master/images/map_2.png)
![2nd Image from slam](https://github.com/onurbagoren/home_service_robot/blob/master/images/MAP.png)

Execute add markers:
Marker will appear at a location, wait 5 seconds, disappear, wait 5 seconds, appear at second location.
```
cd /home/$USER/catkin_ws/
./src/scripts/add_markers.sh
```

Execute pick objects:
This will move the robot to goal 1, wait for 5 seconds and then to goal 2
```
cd /home/$USER/catkin_ws/
./src/scripts/pick_objects.sh
```

Execute home service:
```
cd /home/$USER/catkin_ws/
./src/scripts/home_service.sh
```
Images from the home service:
![Image of robot at the marker pick up](https://github.com/onurbagoren/home_service_robot/blob/master/images/at_marker.png)

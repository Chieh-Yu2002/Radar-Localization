# Radar-Localization
Self-Driving Cars Project for NYCU Courses, Fall 2023

Radar localization with NDT * 2 + ICP

## File Structure
```
├── CMakeLists.txt
├── config
│   ├── localiztion.rviz
│   ├── map_modified.rviz
│   └── visualization.rviz
├── include
│   └── localization
├── launch
│   ├── localiztion.launch
│   ├── map_modified.launch
│   └── visualization.launch
└── src
    ├── localization.cpp (scan matching)
    ├── map_modified.cpp
    ├── map_pcd.cpp
    ├── radar.cpp (radar image to point cloud processing)
    └── visualization.cpp
```
## Installation
use container and ros to finish the radar localization

### ROS
[ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)
### Docker
```
$ xhost +local:
```
```
$ sudo docker run \
-it \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-p 2233:22 \
--rm \
--name ros \
--user root \
-e GRANT_SUDO=yes \
-v ~/midterm_ws:/root/catkin_ws \
softmac/sdc-course-docker:latest \
bash
```
Enter container
```
$ sudo docker exec -it ros bash
```

## Execute the code
### Localization
1. Set up parameters and arguments in localization.launch
   * Configure the map and save paths
   * Switch the bag to the correct rosbag
   * Set the -r (rate) argument to 0.01 or a smaller value.
3. Execute the following command in terminal
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch localization localiztion.launch

Hit space to start playing bag
```
### Visualization
1. Set up parameters and arguments in visualization.launch
   * Configure the map and save paths
   * Switch the bag to the correct rosbag
3. Execute the following command in terminal
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch localization visualization.launch

Hit space to start playing bag
```
### Map Modification
1. Set up parameters and arguments in map_modified.launch
   * Configure the map source and save paths
2. Execute the following command in terminal
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch localization map_modified.launch
```

### Results Demo (Two Tracks)
1. Track1 https://youtu.be/FWsR1p_jHv4
2. Track2 https://youtu.be/uieoKRxQlR4
3. Bonus Track https://youtu.be/ChkOh7PrzJE

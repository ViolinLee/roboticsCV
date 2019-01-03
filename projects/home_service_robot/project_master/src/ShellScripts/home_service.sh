#!/bin/sh
export ROBOT_INITIAL_POSE="-x -1 -y -4.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0"
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/leechan/catkin_ws/src/World/myWorld.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/leechan/catkin_ws/src/World/mymap.yaml
export TURTLEBOT_3D_SENSOR=kinect

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 6
xterm -e " rosrun rviz rviz -d ~/catkin_ws/src/RvizConfig/home_service.rviz " & 
sleep 6
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " & 
sleep 6
xterm -e " rosrun add_markers add_markers " &
sleep 6
xterm -e " rosrun pick_objects pick_objects "

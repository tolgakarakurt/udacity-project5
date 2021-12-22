#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find my_robot)/worlds/myoffice4.world
export ROBOT_INITIAL_POSE='-x -2 -y -0 -z 0 -R 0 -P 0 -Y 0'
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find my_robot)/maps/mymap.yaml

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 10
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 20
xterm  -e  " rosrun pick_objects pick_objects"

#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find my_robot)/worlds/myoffice4.world
export ROBOT_INITIAL_POSE='-x -2 -y -0 -z 0 -R 0 -P 0 -Y 0'

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 15
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 2
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"

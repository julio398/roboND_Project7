#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 8
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch"



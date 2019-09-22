#!/bin/bash

## run turtlebot launch file
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/RND_05_project/SOURCE/map/MyWorld.world" &
sleep 10

## run slam_gmapping
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 3

## run rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3

## mrun teleop
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch"

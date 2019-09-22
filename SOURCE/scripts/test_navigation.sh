#!/bin/bash

## STILL NEED TO CHECK IF WORKS!!!!!

## run turtlebot launch file
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/RND_05_project/SOURCE/map/MyWorld.world" &
sleep 10

## run turtlebot amcl
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/RND_05_project/SOURCE/map/map.yaml" &
sleep 3

## run rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
sleep 3
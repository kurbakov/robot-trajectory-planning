#!/bin/bash

## deploy a turtlebot in my environment
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/RND_05_project/SOURCE/map/MyWorld.world" &
sleep 10

## localize the turtlebot
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/RND_05_project/SOURCE/map/map.yaml" &
sleep 3

## observe the map in rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 3

## pick up virtual objects
xterm -e "rosrun add_markers add_markers_node"
sleep 3

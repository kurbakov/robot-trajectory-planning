#!/bin/bash

## run turtlebot
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/RND_05_project/SOURCE//map/MyWorld.world" &
sleep 10

## run acml
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/RND_05_project/SOURCE//map/map.yaml" &
sleep 2

## run in rviz
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 2

## run pick_objects
xterm -e "rosrun pick_objects pick_objects_node"
sleep 2
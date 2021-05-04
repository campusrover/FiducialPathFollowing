---
title: Home Page
layout: template
filename: index.md
--- 

# MazeSolver

## Description
This ROS package utilizes Dijkstra's algorithm and fiducials to pilot a robot through a maze.

## Flow
The project works like so:
* First, a robot moves through a maze via user-controlled teleop
* This takes a SLAM of the maze which is then saved as an image
* Dijkstra's algorithm is then used to analyze the SLAM image and output the shortest path through the maze
* Fiducials are then created in the maze based on the result of Dijkstra's which instruct what turns to make
* A second robot is placed in the maze and moves through the maze on it's own, based on the fiducials

## Contributors
* Jacqueline Zhou
* Harris Rippp

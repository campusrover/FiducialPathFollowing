# MazeSolver- Jacqueline Zhou & Harris Ripp

## Description
This ROS package utilizes Dijkstra's algorithm and fiducials to pilot a robot through a maze.

## Flow
The project works like so:
* First, a robot moves through a maze via user-controlled teleop
* This takes a SLAM of the maze which is then saved as an image
* Dijkstra's algorithm is then used to analyze the SLAM image and output the shortest path through the maze
* Fiducials are then created in the maze based on the result of Dijkstra's which instruct what turns to make
* A second robot is placed in the maze and moves through the maze on it's own, based on the fiducials

## How to run
* $ roslaunch FiducialPathFollowing maze_world_origin.launch
* $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
* $ teleop
* $ rosrun map_server map_saver -f maze_world_origin
* $ # stop all nodes
* $ python shortestpath.py # this writes into coordinates, doesn't work
* $ roslaunch FiducialPathFollowing maze
* $ python write_launch.py # this writes into dynamic_spawn.launch
* $ roslaunch FiducialPathFollowing dynamic_spawn.launch
* $ rosrun FiducialPathFollowing follow_marker.py

## Link to video
* first submission: https://youtu.be/9uh6TUvdxq0

## Link to project report
* https://campusrover.github.io/FiducialPathFollowing/

## Links to labnotebook entries
* Jacqueline: [Change Camera Pitch](https://github.com/campusrover/labnotebook/blob/master/faq/camera_pitch.md)
* Harris: [Detection of Edges](https://github.com/campusrover/labnotebook/blob/master/faq/edgeDetection.md)

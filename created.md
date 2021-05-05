---
title: What was created 
layout: template
filename: created.md
--- 

# What Was Created

## The First Robot Problem

One problem that we encountered in the project was how to meneuver the first robot. There were a number of ideas floating around but eventually we settled on using the teleop system. Using any number of different algorithms would have been too much of a hassle and would have taken far too long a time to actually complete. By using an algorithm that would make the first robot go through the maze using some form of artificial intelligence, the time may be shorter but there was not enough time to actually impliment a system like that. Perhaps in the future if this projext were to be redone, that would be the case and a more efficient way of moving the first robot could be implimented. 

## How to use Dijkstra's Algorithm

Another problem faced when completing this project was how how exactly Dijkstra's algorithm was to be implimented. An initial though of how this should be done was to use the robot's LIDAR. This method would have the robot analyze where in the maze it was using this method. Vertices for Dijkstra's algorithm would be created by the robot sensing whether or not it was at an intersection. If the robot could tell that there was empty space continuing in three or for directions or at a corner, a vertex would be created and used in the algorithm. After much deliberation, however, this idea was abandoned as it would have been too difficult to account for background noise, among other things.

Another idea that was a possibility was to use some image processing and create vertices that way. This algorithm would scan the SLAM image from the first robot and create a vertex the same way as the above method. This solution was also abandoned.

The solution that was eventually decided upon was to create vertices for Dijkstra's Algorithm by means of pixels. What this algorithm would do is iterate over the pixels in the maze after converting it to grayscale. This would ensure that there are solely black and white colors. What would happen, is the robot assigns higher values to the black colors so it makes sure it never goes there. This will be delved into further. This algorithm would result in the path only passing through white space and would work properly. Here, every pixel is counted as a vertex in Dijksta's. 

## Interesting Algorithms

One interesting algorithm used in this project was a part of Dijkstra's. Specificall, how walls were not counted as vertices. This is a simple algorithm but very useful. It is called the Euclidean squared distance formula and the way it works is by assigning a numerical value to each pixel. This value would count as the weight for Dijkstra's. Every pixel is made of RGB values. What the algorithm does is it subtracts each value from it's neighboring pixel and then squares that. These values are then added together with 0.1 added. This makes it so if the values are the same, the weight becomes 0 and will be a candidate for Dijkstra's. If the pixel is a wall color, the final value will become prohibitively high and the algorithm will not elect to use that vertex. The reason 0.1 is added is because Dijkstra's algorithm causes an error if a weight is 0 so a little must be added. 




#!/usr/bin/env python

# This processes all of the sensor info and defines movement pattern
# publishes the defined pattern NO. to 'state'
# @author Jacqueline Zhou @email jianingzhou@brandeis.edu

import rospy
import numpy as np
#from state_definitionsn import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from prrexamples.msg import Filtered_Laserscan
#------------------------------------------------------------------------------------------
#==============================## functions and callbacks ##==============================
#------------------------------------------------------------------------------------------
# Process all the data from the LIDAR
def laser_cb(msg):
    global state
    # calculate msg to publish to pid
    msg_pid = calc_laserscan(msg)
    # publish msg_pid
    pub_svh.publish(msg_pid)
    # define the state by processed ranges info
    state = state_define()
    # publish state
    pub_state.publish(state)

# update id sent from aruco_detect
def id_cb(msg):
    global id
    id = msg

# calculate and compile Filtered_Laserscan object
def calc_laserscan(msg):
    ranges = np.array(msg.ranges)
    for i in range(360):
        if ranges[i] == float('inf') or ranges[i] < msg.range_min:
            ranges[i] = 4
    calc_regions(ranges)
    msg_new = Filtered_Laserscan()
    msg_new.filtered_ranges = ranges
    return msg_new

# define regions
def calc_regions(ranges):
    global regions
    # calculate different region min
    North = np.append(ranges[0:30], ranges[330:360])
    regions = {
        "N":     min(North),                                # north
        "NW":    min(ranges[30:60]),                        # north-west
        "W":     min(ranges[60:120]),                       # west
        "SW":    min(ranges[120:150]),                      # south-west
        "S":     min(ranges[150:210]),                      # south
        "SE":    min(ranges[210:240]),                      # south-east
        "E":     min(ranges[240:300]),                      # east
        "NE":    min(ranges[300:330]),                      # north-east
    }

# define state by ranges
def state_define():
    # seeing fiducial id = 1 & left distance > 1 & front distance < 1.5
    if regions["W"] > 1 and regions["N"] < 1.5 and id == 1:
        return TURNING_LEFT

    # seeing fiducial id = 2 & right distance > 1 & front distance < 1.5
    elif regions["E"] > 1 and regions["N"] < 1.5 and id == 2:
        return TURNING_RIGHT

    # seeing fiducial & front distance > 1
    elif regions["N"] > 1 and (id == 1 or id == 2):
        return PREPARING_TURN

    # seeing fiducial id = 3
    elif id == 3:
        return PROCEED

    # no instruction received, follow left wall
    else:
        return FOLLOW
#------------------------------------------------------------------------------------------
#===========================## init node and define variables ##===========================
#------------------------------------------------------------------------------------------
# Init node
rospy.init_node('sensor_info_handler')

# Subscribers
laser_sub = rospy.Subscriber('scan', LaserScan, laser_cb)
id_sub = rospy.Subscriber('id', Int16, id_cb)

# Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
pub_svh = rospy.Publisher('cross_err', Filtered_Laserscan, queue_size = 1)

# Rate object
rate = rospy.Rate(10)

# fiducial id
id = 0

# in the process of turning left
TURNING_LEFT = 1
# in the process of turning right
TURNING_RIGHT = 2
# going straight, preparing to make a turn
PREPARING_TURN = 3
# going striaght
PROCEED = 4
# no instruction received, follow left wall
FOLLOW = 5

# starting state
state = 0

# regions
regions = {}
#------------------------------------------------------------------------------------------
#=====================================## while loop ##=====================================
#------------------------------------------------------------------------------------------
# buffer loop
while state == 0 or regions == {}: continue

# Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 
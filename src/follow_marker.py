#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# set absolute speed for robot
FORWARD_SPEED = 0.1
ABS_ROTATE = 0.3

class Marker_Follower:
    def __init__(self):
        # bridge object
        self.bridge = cv_bridge.CvBridge()
        # image subscriber
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_cb)
        # laserscan subscriber
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_cb)
        # id variable
        self.id = 0
        # tvec item
        self.tvec = []
        # cmd_vel publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # cmd_vel object
        self.twist = Twist()
        # rate objects
        self.r = 10
        # ranges
        self.ranges = []
        # pid objects
        self.dT = 1.0 / self.r
        self.curr_error = 0.0
        self.prev_error = 0.0
        self.sum_error = 0.0
        # threshold (desired distance from wall)
        self.EXP_DIST = 0.7
    
    def laser_cb(self, msg):
        # cleanup laser data
        self.ranges = self.calc_laserscan(msg)
    
    def image_cb(self, msg):
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        # transfer to grey image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # define the dictionary
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        # define parameters for aruco_detect
        parameters = aruco.DetectorParameters_create()
        # calibration matrices
        # default matrices used here since there is no distortion in simulation
        camera_matrix = np.array(
                         	[[image.shape[1], 0, image.shape[1]/2],
                         	[0, image.shape[1], image.shape[0]/2],
                         	[0, 0, 1]], dtype = "double"
                         	)
        dist = np.array( [0, 0, 0, 0, 0] )

        # get ID and corners
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # when there is marker in sight
        if ids is not None:

            # get image dimention
            h, w, d = image.shape
            # update id that is seen
            self.id = max(ids)
            # estimate pose of each marker
            ret = aruco.estimatePoseSingleMarkers(corners, 0.2, camera_matrix, dist)
            rvec, self.tvec = ret[0][0,0,:], ret[1][0,0,:]
            # clear numbers
            (rvec - self.tvec).any()
            # draw contours of markers
            aruco.drawDetectedMarkers(image, corners)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, "Id: " + str(self.id), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

            # find the center pixel of the marker with largest id
            x_sum = corners[0][0][0][0]+corners[0][0][1][0]+corners[0][0][2][0]+corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+corners[0][0][1][1]+corners[0][0][2][1]+corners[0][0][3][1]
            cx = int(x_sum*.25)
            cy = int(y_sum*.25)
            # put a circle there
            cv2.circle(image, (cx, cy), 15, (0,0,255), -1)

            # use "P" control to move
            err = cx - w/2
            self.twist.linear.x = 0.1
            self.twist.angular.z = -float(err) / 500 / 4
            self.cmd_vel_pub.publish(self.twist)
        
        # when there's no marker in sight
        else:
            if min(self.ranges[0:15]) < 1.2 or min(self.ranges[345:360]) < 1.2:
                if min(self.ranges[75:105]) < min(self.ranges[255:285]):
                    # turn right
                    self.twist.angular.z = ABS_ROTATE * (-1)
                    self.twist.linear.x = FORWARD_SPEED
                else:
                    # turn left
                    self.twist.angular.z = ABS_ROTATE
                    self.twist.linear.x = FORWARD_SPEED
                self.cmd_vel_pub.publish(self.twist)
            else:
                pid = self.calc_pid()
                self.twist.angular.z = ABS_ROTATE * pid
                self.twist.linear.x = FORWARD_SPEED
                self.cmd_vel_pub.publish(self.twist)
            
        # show the image window
        cv2.imshow('image', image)
        cv2.waitKey(3)
    
    # calculate and compile Filtered_Laserscan object
    def calc_laserscan(self, msg):
        ranges = np.array(msg.ranges)
        for i in range(360):
            if ranges[i] == float('inf') or ranges[i] < msg.range_min:
                ranges[i] = 4
        return ranges

    # calculate errors and pid components
    def calc_pid(self):
        self.error = min(self.ranges[240:300]) - self.EXP_DIST
        self.prev_error = self.curr_error
        self.curr_error = self.error
        self.sum_error += self.curr_error * self.dT
        pid = self.get_pid(self.curr_error, self.prev_error, self.sum_error)
        return pid

    # calculate pid value
    def get_pid(self, curr, prev, sum):
        # Multipliers used to tune the PID controller
        # Proportional constant
        P_CONSTANT = 0.7
        # Integral constant
        I_CONSTANT = 0.01
        # Derivative constant
        D_CONSTANT = 0
        # components
        p_component = curr
        d_component = (curr - prev)/self.dT
        i_component = sum
        return P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component

###############################################################################################
################################## RUN THE NODE ###############################################
###############################################################################################
rospy.init_node('detect_fiducial')
marker_follower = Marker_Follower()
rospy.spin()
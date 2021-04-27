#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from math import pi

class Marker_Follower:
    def __init__(self):
        # bridge object
        self.bridge = cv_bridge.CvBridge()
        # image subscriber
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        # cmd_vel publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # twist object
        self.twist = Twist()
        # id variable
        id = 0
        
    def image_callback(self, msg):
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        # transfer to grey image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # define the dictionary
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        # define parameters for aruco_detect
        parameters = aruco.DetectorParameters_create()
        # calibration matrices
        camera_matrix = np.array(
                         	[[image.shape[1], 0, image.shape[1]/2],
                         	[0, image.shape[1], image.shape[0]/2],
                         	[0, 0, 1]], dtype = "double"
                         	)
        dist = np.array( [0, 0, 0, 0, 0] )
        # get ID and corners
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # estimate pose of each marker
            ret = aruco.estimatePoseSingleMarkers(corners, 0.2, camera_matrix, dist)
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            # clear numbers
            (rvec - tvec).any()
            # draw contours of markers
            aruco.drawDetectedMarkers(image, corners)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, "Id: " + str(ids[0]), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)
        # show the image window
        cv2.imshow('image', image)
        cv2.waitKey(3)

###############################################################################################
################################## RUN THE NODE ###############################################
###############################################################################################
rospy.init_node('follow_fiducial')
marker_follower = Marker_Follower()
rospy.spin()
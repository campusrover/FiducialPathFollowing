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
        # define the dictionary
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        # define parameters for aruco_detect
        self.parameters = aruco.DetectorParameters_create()
        

    def image_callback(self, msg):
        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        # transfer to grey image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # calibration matrices
        camera_matrix = np.array(
                         	[[image.shape[1], 0, image.shape[1]/2],
                         	[0, image.shape[1], image.shape[0]/2],
                         	[0, 0, 1]], dtype = "double"
                         	)
        dist = np.array( [0, 0, 0, 0, 0] )
        # get ID and corners
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if np.all(ids) != None:
            for i in range(len(ids)):
                # estimate pose of each marker
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.1, self.camera_matrix, self.dist_coeffs)
                # clear numbers
                (rvec - tvec).any()
        
        # draw contours of markers
        aruco.drawDetectedMarkers(image, corners)
        # show the image window
        cv2.imshow('image', image)
        cv2.waitKey(3)

###############################################################################################
################################## RUN THE NODE ###############################################
###############################################################################################
rospy.init_node('follow_fiducial')
marker_follower = Marker_Follower()
rospy.spin()
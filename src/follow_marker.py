#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# set absolute speed for robot
FORWARD_SPEED = 0.2
ABS_ROTATE = 0.5

class Marker_Follower:
    def __init__(self):
        # bridge object
        self.bridge = cv_bridge.CvBridge()
        # image subscriber
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_cb)
        # id variable
        self.id = 0
        # tvec item
        self.tvec = []
        # cmd_vel publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # cmd_vel object
        self.twist = Twist()
    
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

        if ids is not None:
            # get image dimention
            h, w, d = image.shape
            # update id that is seen
            self.id = ids[0]
            # estimate pose of each marker
            ret = aruco.estimatePoseSingleMarkers(corners, 0.2, camera_matrix, dist)
            rvec, self.tvec = ret[0][0,0,:], ret[1][0,0,:]
            # clear numbers
            (rvec - self.tvec).any()
            # draw contours of markers
            aruco.drawDetectedMarkers(image, corners)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, "Id: " + str(ids[0]), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

            # find the center pixel of the marker with largest id
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
            cx = int(x_sum*.25)
            cy = int(y_sum*.25)
            # put a circle there
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # use "P" control to move
            err = cx - w/2
            self.twist.linear.x = 0.1
            self.twist.angular.z = -float(err) / 500 / 4
            self.cmd_vel_pub.publish(self.twist)
            
        # show the image window
        cv2.imshow('image', image)
        cv2.waitKey(3)

###############################################################################################
################################## RUN THE NODE ###############################################
###############################################################################################
rospy.init_node('detect_fiducial')
marker_follower = Marker_Follower()
rospy.spin()
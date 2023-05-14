#! /usr/bin/env python
# encoding: utf-8
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image 
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import threading
import matplotlib.pyplot as plt
import numpy as np


def Depthcallback(self,msg_depth): # TODO still too noisy!

    try:

        # The depth image is a single-channel float32 image

        # the values is the distance in mm in z axis

        cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")

        # Convert the depth image to a Numpy array since most cv2 functions

        # require Numpy arrays.

        cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))

        # Normalize the depth image to fall between 0 (black) and 1 (white)

        # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125

        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)

        # Resize to the desired size

        cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)

        self.depthimg = cv_image_resized

        cv2.imshow("Image from my node", self.depthimg)

        cv2.waitKey(1)

    except CvBridgeError as e:

        print(e)

#! /usr/bin/env python3.8
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

#定义线程函数
def thread_job():
    # print("ROSspin has started")
    rospy.spin() 

class depth:
		#订阅节点#初始化class时就创建
	def __init__(self):
            self.sub = rospy.Subscriber('/d435/depth/image_raw', Image, self.callback)
            self.desired_shape=(1280,720 )
            self.fx = 695.9951171875
            self.fy = 695.9951171875
            self.cx = 640.0
            self.cy = 360.0
            self.factor = 1000
        def callback(self, data):#你的回调函数	
                # Convert ROS Image message to numpy array
                bridge = CvBridge()
                depth_image = bridge.imgmsg_to_cv2(data)
                # Convert depth image to point cloud
                fx = self.fx
                fy = self.fy
                cx = self.cx
                cy = self.fy
                factor = self.factor
                points = []
                for v in range(depth_image.shape[0]):
                    for u in range(depth_image.shape[1]):
                        z = depth_image[v,u] / factor
                        x = (u - cx) * z / fx
                        y = (v - cy) * z / fy
                        points.append([x, y, z])
                # Create Open3D point cloud
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                # Visualize point cloud
                o3d.visualization.draw_geometries([pcd])
	
if __name__ == '__main__':
    rospy.init_node('self_locate')
    rate = rospy.Rate(10) # 10hz
    depth()   #订阅函数

    thread_rosspin = threading.Thread(target=thread_job)
    thread_rosspin.start()

#! /usr/bin/python

from __future__ import print_function
import copy
import rospy
import numpy as np
import pyrealsense2 as rs
import numpy as np
import cv2
from moveit_jaka import *
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import tf
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

class robot_scan:
    def __init__(self):
        self.topic = '/camera/depth/image_rect_raw'
        self.camera_info_topic = "/camera/depth/camera_info"

        self.camera_info_init_done = 0
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber(self.topic, msg_Image, self.imageDepthCallback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic,CameraInfo,self.init_camera_info)

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.factor = 1000.0
        self.depth_pc = rospy.Publisher('/depth_pointcloud', PointCloud2, queue_size=1)
        # # get tf
        # self.tf_listener = tf.TransformListener()
        # tmptimenow = rospy.Time.now()
        # self.tf_listener.waitForTransform("/d435_depth_frame", "/Link_0", tmptimenow, rospy.Duration(0.5))
        # try:
        #     self.camera_transform, self.camera_rotation = self.tf_listener.lookupTransform("/d435_depth_frame", "/Link_0", tmptimenow)
        #     self.tf_transform_ready = 1
        #     print(self.camera_transform)
        #     print(self.camera_rotation)
        # except:
        #     print("----------Failed to Get Camera Transform-----------")


    def init_camera_info(self,data):
        if not self.camera_info_init_done:
            self.fx = data.K[0]
            self.fy = data.K[4]
            self.cx = data.K[2]
            self.cy = data.K[5]
            self.camera_info_init_done=1

    def imageDepthCallback(self, data):
            # Convert ROS Image message to numpy array
            if not self.camera_info_init_done:
                return None
            bridge = CvBridge()
            depth_image = bridge.imgmsg_to_cv2(data)
            # Convert depth image to point cloud
            fx = self.fx
            fy = self.fy
            cx = self.cx
            cy = self.cy
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
            # o3d.visualization.draw_geometries([pcd])
            header = Header()
            header.frame_id =data.header.frame_id  #"d435_depth_frame" 
            header.stamp = data.header.stamp
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
            ]
            pc = point_cloud2.create_cloud(header, fields, points)
            
            self.depth_pc.publish(pc)

    # def imageDepthCallback(self, data):
    #     # Convert ROS Image message to numpy array
    #     if not self.camera_info_init_done:
    #         return None
    #     bridge = CvBridge()
    #     depth_image = bridge.imgmsg_to_cv2(data)
    #     # Convert depth image to point cloud
    #     fx = self.fx
    #     fy = self.fy
    #     cx = self.cx
    #     cy = self.cy
    #     # print([fx,fy,cx,cy])
    #     factor = self.factor
    #     points = []
    #     for v in range(depth_image.shape[0]):
    #         for u in range(depth_image.shape[1]):
    #             z = depth_image[v,u] / factor
    #             x = (u - cx) * z / fx
    #             y = (v - cy) * z / fy
    #             points.append([x, y, z])
    #             # print(depth_image[v,u])
    #             # print(factor)
    #             # print([x, y, z])
    #     pcd_msg = PointCloud2()
    #     pcd_msg.header.stamp = data.header.stamp
    #     pcd_msg.header.frame_id = data.header.frame_id
    #     pcd_msg.height = 1
    #     pcd_msg.width = len(points)
    #     pcd_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    #     pcd_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    #     pcd_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    #     pcd_msg.point_step = 12
    #     pcd_msg.row_step = 12 * len(points)
    #     pcd_msg.is_dense = True
    #     pcd_msg.data = np.array(points, dtype=np.float32)#.tostring()
    #     # Publish ROS PointCloud2 message
    #     self.depth_pc.publish(pcd_msg)

        ## pathplan according open3d point cloud


if __name__ == '__main__':
    rospy.init_node("robot_scan")
    robot_scan_node = robot_scan()
    rospy.spin()
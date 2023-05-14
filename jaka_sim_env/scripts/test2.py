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

# def callback(data):
#     # cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
#     cv_img = bridge.imgmsg_to_cv2(data, "16UC1")
#     # print(cv_img.shape) #(720, 1280)
#     cv2.imshow("frame" , cv_img)
#     cv2.waitKey(1)

# if __name__=="__main__":
#     topic = '/d435/depth/image_raw'
#     print(sys.version) # 查看python版本    
#     rospy.init_node('img_process_node', anonymous=True)
#     bridge = CvBridge()
#     # rospy.Subscriber('/d435/color/image_raw', Image, callback)
#     rospy.Subscriber('/d435/depth/image_raw', Image, callback)
#     rospy.spin()


#定义线程函数
def thread_job():
    # print("ROSspin has started")
    rospy.spin() 

class image:
	#初始化class时就创建		#订阅节点
	def __init__(self):
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber('/d435/color/image_raw', Image, self.callback)
	def callback(self, data):#你的回调函数
		self.cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		cv2.imwrite('./color.jpg',self.cv_img)
		# print(self.cv_img)
		cv2.imshow("color" , self.cv_img)
		cv2.waitKey(1)

class depth:
		#订阅节点#初始化class时就创建
	def __init__(self):
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber('/d435/depth/image_raw', Image, self.callback)
		self.desired_shape=(1280,720 )
	def callback(self, data):#你的回调函数
	    # try:
        #         cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        #         cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
        #         cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        #         cv_image_resized = cv2.resize(cv_image_norm, self.desired_shape, interpolation = cv2.INTER_CUBIC)
        #         self.depthimg = cv_image_resized
        #         cv2.imwrite('./depth.png',self.depthimg)
        #         # cv2.imshow("Image from my node", self.depthimg)
        #         # cv2.waitKey(1)
        #     except CvBridgeError as e:
        #         print(e)

		self.cv_img = self.bridge.imgmsg_to_cv2(data, "16UC1")
		cv2.imwrite('./depth.png',self.cv_img)
		# print(self.cv_img)
		cv2.imshow("depth" , self.cv_img)
		cv2.waitKey(1)
	
if __name__ == '__main__':
    rospy.init_node('self_locate')
    rate = rospy.Rate(10) # 10hz
    image() #第一个订阅函数
    depth()   #第二个订阅函数
    
    rospy.sleep(3)

    color_raw = o3d.io.read_image("./color.jpg")
    depth_raw = o3d.io.read_image("./depth.png")
    rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(color_raw, depth_raw)
    print(rgbd_image)

    plt.subplot(1, 2, 1)
    plt.title(' grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title(' depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()

    # http://www.open3d.org/docs/release/python_api/open3d.camera.PinholeCameraIntrinsic.html?highlight=pinholecameraintrinsic#open3d.camera.PinholeCameraIntrinsic
    width, height = 1280, 720    # Width of the image. Height of the image.
    cx = 640.0    # X-axis focal length  x轴焦距
    cy = 360.0    # Y-axis focal length.
    fx = 695.9951171875     # X-axis principle point.x轴原理点
    fy = 695.9951171875     # Y-axis principle point.
    
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)    # PinholeCameraIntrinsic类存储固有的相机矩阵，以及图像的高度和宽度。
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    o3d.visualization.draw_geometries([pcd])

    thread_rosspin = threading.Thread(target=thread_job)
    thread_rosspin.start()

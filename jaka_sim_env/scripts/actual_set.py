#! /usr/bin/env python3.8
# encoding: utf-8
import rospy
from std_msgs.msg import Int16MultiArray

def domsg(msg):
    # 修改参数
    rospy.set_param("Nx",msg.data[0])
    rospy.set_param("step_num",msg.data[1])
    rospy.set_param("scan_height_int",msg.data[2])
    rospy.set_param("rapid_height_int",msg.data[3])
    rospy.set_param("flag_curve",msg.data[4])
    print('set success!')

if __name__=="__main__":
    rospy.init_node("param_set")
    # 订阅话题
    sub=rospy.Subscriber("AA_path_param_set",Int16MultiArray,domsg,queue_size=10)
    rospy.spin()
    
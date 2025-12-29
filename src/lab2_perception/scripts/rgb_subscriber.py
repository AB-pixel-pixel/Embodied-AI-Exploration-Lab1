#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("RGB Image", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('rgb_listener')  # 初始化ROS节点
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)  # 订阅RGB图像话题
    rospy.loginfo("RGB Subscriber Started")
    rospy.spin()  # 保持节点运行

if __name__ == '__main__':
    main()

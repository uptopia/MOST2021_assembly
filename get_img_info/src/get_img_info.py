#!/usr/bin/env python3
import cv2
import sys
sys.path.insert(1, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import PointCloud2 as msg_Cloud
from assembly_srv.srv import ImgInfo, ImgInfoResponse

show_res = True
n = 1
# dir = "./sampling_data/"
class get_img_info_server:
    def __init__(self) -> None:
        rospy.init_node("get_img_info_server")
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", msg_Image, self.imageCallback)
        self.pub = rospy.Publisher('/camera/color/image_raw_workspace', msg_Image, queue_size=10)

    def imageCallback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        # select_ws = cv_image[190:560,400:1000,:] 
        select_ws = cv_image[150:580,370:1030,:] 
        if show_res:
            global n 
            # cv2.imshow("cv", cv_image)
            cv2.imshow("select_workspace", select_ws)
            # cv2.imshow("ori workspace", cv_image)
            key = cv2.waitKey(1)
            if key == ord("s"):
                file_name = "select_workspace_" + str(n) +".jpg"
                cv2.imwrite(file_name, select_ws)
                print("save " + file_name)
                n+=1

        select_ws = self.bridge.cv2_to_imgmsg(select_ws, "bgr8")
        self.pub.publish(select_ws)

def cloudCallback(cloud_msg):

def imageCallback(img_msg):

def handle_img_info(req):
    bridge = CvBridge()
    rospy.Subscriber("/camera/color/image_raw", msg_Image, imgCallback)
    rospy.Subscriber("/camera/depth_registered/points", msg_Cloud, cloudCallback)


    res = ImgInfoResponse
    return res

def get_img_info_server():
    rospy.init_node('get_img_info_server')
    s = rospy.Service('get_img_info', ImgInfo, handle_img_info)
    print("get_img_info_server")
    rospy.spin()


if __name__ == "__main__":
    get_img_info_server()


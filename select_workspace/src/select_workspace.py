#!/usr/bin/env python3
import cv2
import sys
sys.path.insert(1, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from assembly_srv.srv import TakePic, TakePicResponse

show_res = False
n = 1

class Select_workspace_Node:
    def __init__(self) -> None:
        rospy.init_node("Select_workspace_Node")

        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback)
        rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.cloudCallback)

        self.cloud_size = []
        self.img_size = []
        self.organized_cloud_ori = PointCloud2()
        self.rgb_img_ori = Image()
        self.rgb_img_cropped = Image()
        self.crop_init_x = 370
        self.crop_init_y = 150
        self.server = rospy.Service('/take_pic', TakePic, self.handle_take_pic)

        rospy.spin()

    def handle_take_pic(self, req):
        res = TakePic()
        res.organized_cloud_ori = self.organized_cloud_ori
        res.rgb_img_ori = self.rgb_img_ori
        res.rgb_img_cropped = self.rgb_img_cropped
        res.crop_init_x = self.crop_init_x
        res.crop_init_y = self.crop_init_y

        if (req.picture_pose_reached == True) and (self.cloud_size != 0) and (self.img_size != 0):
            res.data_check_correct = True
            print("[Service] cloud and image SEND: cloud_size, img_size", self.cloud_size, self.img_size)
        else:
            res.data_check_correct = False
            print("waiting for cloud and image")

        return res

    def imageCallback(self, img_msg):
        self.rgb_img_ori = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.rgb_img_cropped = self.rgb_img_ori[self.crop_init_y:580, self.crop_init_x:1030,:] #[150:580,370:1030,:] 
        
        img_height, img_width, _ = self.rgb_img_ori.shape 
        self.img_size = img_height*img_width
        
        if show_res:
            global n 
    
            cv2.imshow("select_workspace", self.rgb_img_cropped)
            key = cv2.waitKey(1)
            if key == ord("s"):
                file_name = "select_workspace_" + str(n) +".jpg"
                cv2.imwrite(file_name, self.rgb_img_cropped)
                print("save " + file_name)
                n+=1

        self.rgb_img_cropped = self.bridge.cv2_to_imgmsg(self.rgb_img_cropped, "bgr8")
    
    def cloudCallback(self, cloud_msg):
        self.organized_cloud_ori = cloud_msg
        self.cloud_size = cloud_msg.height*cloud_msg.width

if __name__ == "__main__":
    Select_workspace_Node()


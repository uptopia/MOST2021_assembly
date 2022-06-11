#!/usr/bin/env python3
import cv2
import sys
sys.path.insert(1, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')
from cv_bridge import CvBridge, CvBridgeError

import rospy
import message_filters
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import PointCloud2 as msg_Cloud
from assembly_srv.srv import ImgInfo, ImgInfoResponse

bridge = CvBridge()
show_res = True
n = 1
curr_img_cropped_msg = []
curr_cloud_msg = []

class GetImgInfo_Node:
    def __init__(self): #get_img_info_server
        rospy.init_node('get_img_info_server')
        s = rospy.Service('get_img_info', ImgInfo, self.handle_img_info)
        print("get_img_info_server")
        rospy.spin()

    def handle_img_info(self, req):
        res = ImgInfoResponse()

        if(req.pic_pose_reached == True):
            sub_img = rospy.Subscriber("/camera/color/image_raw", msg_Image)
            sub_cloud = rospy.Subscriber("/camera/depth_registered/points", msg_Cloud)
            sync = message_filters.ApproximateTimeSynchronizer([sub_img, sub_cloud], 10, 1)
            sync.registerCallback(self.multiCallback)
            
            if(req.task_type == "motor"):
                res.grasp_pose = []#motor_grasp_pose

            elif(req.task_type == "screw"):
                res.grasp_pose = []#screw_grasp_pose

            elif(req.task_type == "terminal"):
                res.grasp_pose = []#terminal_grasp_pose

            else:
                print("ERROR!!! Unexpected task_type. [task_type] options: motor, screw, terminal")
        else:
            res = []

        return res

    def multiCallback(self, img_msg, cloud_msg):
        global curr_img_cropped_msg, curr_cloud_msg, n

        #=======================#
        # organized point cloud 
        #=======================#
        curr_cloud_msg = cloud_msg

        #=======================#
        # rgb image -> cropped
        #=======================#
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        img_cropped = cv_image[150:580,370:1030,:] 
        
        if show_res:
            # cv2.imshow("ori workspace", cv_image)
            cv2.imshow("select_workspace", img_cropped)

            key = cv2.waitKey(1)
            if key == ord("s"):
                file_name = "select_workspace_" + str(n) +".jpg"
                cv2.imwrite(file_name, img_cropped)
                print("save " + file_name)
                n += 1

        curr_img_cropped_msg = bridge.cv2_to_imgmsg(img_cropped, "bgr8")


if __name__ == "__main__":
    GetImgInfo_Node()


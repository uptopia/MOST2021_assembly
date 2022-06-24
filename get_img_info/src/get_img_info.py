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
from assembly_srv.srv import Bboxes, BboxesResponse


bridge = CvBridge()
x_shift = 370
y_shift = 150
show_res = True
n = 1

class GetImgInfo_Node:
    def __init__(self, x_shift, y_shift): #get_img_info_server
        self.x_shift = x_shift
        self.y_shift = y_shift
        self.curr_img_cropped_msg = []
        self.curr_cloud_msg = []

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
                #=======================#
                # MOTOR grasp pose client 
                #=======================#
                motor_response = rospy.wait_for_service('obj_detect', Bboxes)
                res.grasp_pose = []#motor_grasp_pose

            elif(req.task_type == "screw"):
                #=======================#
                # SCREW grasp pose client 
                #=======================#
                res.grasp_pose = []#screw_grasp_pose

            elif(req.task_type == "terminal"):
                #=======================#
                # TERMINAL grasp pose client 
                #=======================#
                res.grasp_pose = []#terminal_grasp_pose

            else:
                print("ERROR!!! Unexpected task_type. [task_type] options: motor, screw, terminal")
        else:
            res = []

        return res

    def multiCallback(self, img_msg, cloud_msg):

        #=======================#
        # organized point cloud 
        #=======================#
        self.curr_cloud_msg = cloud_msg

        #=======================#
        # rgb image -> cropped
        #=======================#
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        img_cropped = cv_image[self.y_shift:580, self.x_shift:1030, :] # y, x, channels
        
        if show_res:
            cv2.imshow("select_workspace", img_cropped)
            key = cv2.waitKey(1)

            if key == ord("s"):
                self.save_result(img_cropped)

        self.curr_img_cropped_msg = bridge.cv2_to_imgmsg(img_cropped, "bgr8")

    def save_result(self, img2save):
        global n

        file_name = "select_workspace_" + str(n) +".jpg"
        cv2.imwrite(file_name, img2save)
        print("save " + file_name)
        n += 1

if __name__ == "__main__":
    GetImgInfo_Node(x_shift, y_shift)


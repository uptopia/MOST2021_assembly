#!/usr/bin/env python3
import cv2
from matplotlib import testing
import torch
import argparse
import os
import rospy
import sys
sys.path.insert(1, '/opt/installer/open_cv/cv_bridge/lib/python3/dist-packages/')
sys.path.append(os.path.dirname(os.path.join(os.getcwd(), os.path.pardir))+ "/src")
ws_path = os.path.dirname(os.path.join(os.getcwd(), os.path.pardir)) + "/src/obj_detect/"
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image

from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet

from obj_detect.msg import bbox, bboxes
from assembly_srv.srv import Bboxes, BboxesResponse

bridge = CvBridge()
use_cuda = True
show_res = True
n = 1

def get_args():
    parser = argparse.ArgumentParser('Test your image or video by trained model.')
    parser.add_argument('-namesfile', type=str, default= ws_path + 'data/obj.names',
                        help='path of name file', dest='namesfile')
    parser.add_argument('-cfgfile', type=str, default= ws_path + 'cfg/yolov4-components.cfg',
                        help='path of cfg file', dest='cfgfile')
    parser.add_argument('-weightfile', type=str,
                        default= ws_path + 'weights/yolov4-components_final_20220426.weights',
                        help='path of trained model.', dest='weightfile')
    args, unknown = parser.parse_known_args()
    return args

class Det_Node:
    def __init__(self, cfg, w, namesfile) -> None:
        rospy.init_node("obj_detect_server")
        s = rospy.Service('obj_detect', Bboxes, self.handle_obj_detect)
        print("obj_detect_server")
        self.init_model(cfg, w, namesfile)
        rospy.spin()

    def init_model(self, cfg, w, namesfile):
        m = Darknet(cfg)
        m.print_network()
        m.load_weights(w)
        self.m = m.cuda()
        self.class_names = load_class_names(namesfile)

    def handle_obj_detect(self, req):
        res = BboxesResponse()

        if((req.rgb_img.height > 0) & (req.rgb_img.width > 0)):

            cv_image = self.bridge.imgmsg_to_cv2(req.rgb_img, "bgr8")

            #=======================#
            # object detection boxes 
            #=======================#
            sized = cv2.resize(cv_image, (self.m.width, self.m.height))
            sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)
            boxes = do_detect(self.m, sized, 0.4, 0.6, use_cuda)
            result_img = plot_boxes_cv2(cv_image, boxes[0], savename=None, class_names=self.class_names)

            #=======================#
            # ros service response 
            #=======================#
            bbs_motors, bbs_other_objs = self.srv_response(cv_image, boxes)
            if(req.task_type == "motor"):
                res.bboxes = bbs_motors
            else:
                res.bboxes = bbs_other_objs
            res.organized_cloud = req.organized_cloud
            res.rgb_img = req.rgb_img  
            res.pixel_shift = req.pixel_shift
            
            #=======================#
            # show, save result 
            #=======================#
            if show_res:
                cv2.imshow("obj_detect", result_img)
                key = cv2.waitKey(1)

                if key == ord("s"):
                    boxes = sorted(boxes)
                    self.save_result(result_img, boxes)
            

    def srv_response(self, img, boxes):
        width = img.shape[1]
        height = img.shape[0]
        bbs_other_objs = bboxes()
        bbs_motors = bboxes()

        for i in range(len(boxes)):
            bb = bbox()
            box = boxes[i]
            bb.xmin = int(box[0] * width)
            bb.ymin = int(box[1] * height)
            bb.xmax = int(box[2] * width)
            bb.ymax = int(box[3] * height)
            bb.score = box[4]
            bb.object_name = self.class_names[box[6]]
            if(box[6] == 2):  #motor
                bbs_motors.bboxes.append(bb)
            else:
                bbs_other_objs.bboxes.append(bb)

        return bbs_motors, bbs_other_objs

    def save_result(self, img2save, boxes2save):
        global n

        file_name = "obj_detect_" + str(n)
        f = open(file_name + ".txt", "w")
        cv2.imwrite(file_name + ".jpg", img2save)
        for bb in boxes2save[0]:
            w = bb[2] - bb[0]
            h = bb[3] - bb[1]
            x = bb[0] + w/2
            y = bb[1] + h/2 
            # data = "{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(bb[6], round(x,6), round(y,6), round(w,6), round(h,6))
            data = "{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(self.class_names[bb[6]], x, y, w, h)
            print(data)
            f.write(data)
        n+=1
        f.close

if __name__ == "__main__":
    args = get_args()
    m = Det_Node(args.cfgfile, args.weightfile, args.namesfile)


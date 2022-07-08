#!/usr/bin/env python3
import cv2
import torch
import argparse
import os
# from matplotlib.pyplot import box
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

use_cuda = False
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
        rospy.init_node("Det_Node")
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw_workspace", msg_Image, self.imageCallback)
        self.pub_other_objs = rospy.Publisher('/yolov4_other_objs_bboxes', bboxes, queue_size=10)
        self.pub_motors = rospy.Publisher('/yolov4_motors_bboxes', bboxes, queue_size=10)
        self.init_model(cfg, w, namesfile)
        rospy.spin()

    def init_model(self, cfg, w, namesfile):
        m = Darknet(cfg)
        m.print_network()
        m.load_weights(w)
        if use_cuda == True:
            self.m = m.cuda()
        else:
            self.m = m
        self.class_names = load_class_names(namesfile)

    def imageCallback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        global n 
        if show_res:
            res, boxes = self.det(cv_image)
            boxes = sorted(boxes)
            cv2.imshow("obj_detect", res)
            key = cv2.waitKey(1)
            if key == ord("s"):
                file_name = "obj_detect_" + str(n)
                f = open(file_name + ".txt", "w")
                cv2.imwrite(file_name + ".jpg", cv_image)
                for bb in boxes[0]:
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
        else:
            self.det(cv_image)

    def det(self, img):
        sized = cv2.resize(img, (self.m.width, self.m.height))
        sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)
        boxes = do_detect(self.m, sized, 0.4, 0.6, use_cuda)
        self.msg_publish(img, boxes[0])

        if show_res:
            result_img = plot_boxes_cv2(img, boxes[0], savename=None, class_names=self.class_names)
            return result_img, boxes

    def msg_publish(self, img, boxes):
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

        self.pub_motors.publish(bbs_motors)
        self.pub_other_objs.publish(bbs_other_objs)

if __name__ == "__main__":
    args = get_args()
    m = Det_Node(args.cfgfile, args.weightfile, args.namesfile)


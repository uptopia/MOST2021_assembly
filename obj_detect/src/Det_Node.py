import cv2
from matplotlib.pyplot import box
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image

from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet
import torch
import argparse

from affordance2.msg import bbox, bboxes

use_cuda = True
show_res = True
n = 51

def get_args():
    parser = argparse.ArgumentParser('Test your image or video by trained model.')
    parser.add_argument('-namesfile', type=str, default='./data/obj.names',
                        help='path of name file', dest='namesfile')
    parser.add_argument('-cfgfile', type=str, default='./cfg/yolov4-components.cfg',
                        help='path of cfg file', dest='cfgfile')
    parser.add_argument('-weightfile', type=str,
                        default='./weights/yolov4-components_final_20220426.weights',
                        help='path of trained model.', dest='weightfile')
    args, unknown = parser.parse_known_args()
    return args


class Det_Node:
    def __init__(self, cfg, w, namesfile) -> None:
        rospy.init_node("Det_Node")
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw_workspace", msg_Image, self.imageCallback)
        self.pub = rospy.Publisher('/yolov4_bboxes', bboxes, queue_size=10)
        self.init_model(cfg, w)

    def init_model(self, cfg, w, namesfile):
        m = Darknet(cfg)
        m.print_network()
        m.load_weights(w)
        self.m = m.cuda()
        self.class_names = load_class_names(namesfile)

    def imageCallback(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        global n 
        if show_res:
            res, boxes = self.det(cv_image)
            boxes = sorted(boxes)
            cv2.imshow("cv", res)
            key = cv2.waitKey(1)
            if key == ord("s"):
                f = open("sampling_data/"+str(n)+".txt", "w")
                cv2.imwrite("sampling_data/"+str(n)+".jpg", cv_image)
                for bb in boxes[0]:
                    w = bb[2] - bb[0]
                    h = bb[3] - bb[1]
                    x = bb[0] + w/2
                    y = bb[1] + h/2 
                    # data = "{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(bb[6], round(x,6), round(y,6), round(w,6), round(h,6))
                    data = "{} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(bb[6], x, y, w, h)
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
        bbs = bboxes()
        for i in range(len(boxes)):
            bb = bbox()
            box = boxes[i]
            bb.xmin = int(box[0] * width)
            bb.ymin = int(box[1] * height)
            bb.xmax = int(box[2] * width)
            bb.ymax = int(box[3] * height)
            bb.score = box[4]
            bb.class_ = box[6]
            bbs.bboxes.append(bb)

        self.pub.publish(bbs)

if __name__ == "__main__":
    args = get_args()    
    m = Det_Node(args.cfgfile, args.weightfile, args.namesfile)
    rospy.spin()

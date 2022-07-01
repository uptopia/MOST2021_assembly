#!/usr/bin/env python

from enum import IntEnum
# from Queue import Queue

import rospy
#import queue
import Queue as queue
import copy
import numpy as np
from std_msgs.msg import Bool, Int32, Float32MultiArray
from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status, RobotiqGripper
from get_image_info import GetObjInfo
from math import radians, degrees, sin, cos, pi
from assembly_srv.srv import GraspPose, GraspPoseRequest

picture_pose_reached = False

c_pose = {'left' :[[[0.38,  0.2, -0.25],  [0.0, 65, 0.0]],
                    [[-0.20, 0.1363, -0.38000],  [44.024, -0.005, -44.998]],
                    [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.00, 0.2363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.1807, 0.4337, -0.6569],  [19.196, 0.427, -0.086]],
                    [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]]],
          'right':[[[0.38, -0.2, -0.25],   [0.0, 65, 0.0]],
                    [[-0.16, -0.2863, -0.6500],  [-44.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-44.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-44.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.47500],  [-44.024, 0.005, 4.498]],
                    [[0.60, -0.1820, -0.35500],  [46.024, 20.005, 4.998]],
                    [[0.20, -0.1820, -0.47500],  [46.024, 20.005, 4.998]],
                    [[0.50, 0.1063, -0.35500],  [46.024, 20.005, 4.998]],
                    [[0.20, 0.1063, -0.47500],  [46.024, 20.005, 4.998]],
                    [[0.20, -0.2463, -0.35500], [46.024, 20.005, 4.998]],
                    [[-0.16, -0.1820, -0.6500],  [-42.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-42.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-42.024, 0.005, 4.498]],
                    [[-0.16, -0.2820, -0.6000],  [-42.024, 0.005, 4.498]]],
          'left_indx' : 0, 'right_indx' : 0}

class ObjInfo(dict):
    def __init__(self):
        self['id']      = 0
        self['side_id'] = 'front'         # 'front', 'back', 'side'
        self['name']    = 'scratcher' # 'plum_riceball', 'salmon_riceball', 'sandwich', 'burger', 'drink', 'lunch_box'
        self['state']   = 'new'           # 'new', 'old', 'expired'
        self['pos']     = None
        self['euler']   = None
        self['sucang']  = 0

class State(IntEnum):
    init            = 0
    take_pic        = 1
    approach_motor  = 2
    finish_init     = 3
   
class GraspTask:
    def __init__(self, _name, en_sim):
        rospy.init_node('assembly_grasp')

        # rospy.Subscriber("/motor_grasp_pose", Float32MultiArray, self.motor_grasp_pose)

        self.name = _name
        self.en_sim = en_sim
        self.state = State.init
        print("robot init===============================")
        self.dual_arm = DualArmTask(self.name, self.en_sim)
        # self.camara = GetObjInfo()
        self.left_cpose_queue = queue.Queue()
        self.right_cpose_queue = queue.Queue()
        self.place_pose_queue = queue.Queue()
        self.object_queue = queue.Queue()
        self.object_list = []
        self.left_tar_obj = queue.Queue()
        self.right_tar_obj = queue.Queue()
        self.retry_obj_queue_left = queue.Queue()
        self.retry_obj_queue_right = queue.Queue()
        self.target_obj_queue = {'left' : self.left_tar_obj, 'right' : self.right_tar_obj}
        self.target_obj = {'left': None, 'right': None}
        self.retry_obj_queue = {'left': self.retry_obj_queue_left, 'right': self.retry_obj_queue_right}
        self.obj_done = np.zeros((100), dtype=bool)
        self.obj_retry = np.zeros((100), dtype=bool)
        self.next_level = {'left': False, 'right': False}
        self.motor_grasp_pose = Float32MultiArray()

        #self.init()
    def visiontoArm(self, rot, tvec, arm_ori):
        dx = -0.04  # unit:meter
        dy = 0.032  # unit:meter
        dz = -0.14  # unit:meter
        rad = radians(10)   

        TransMat_EndToImg = np.mat([[0, cos(rad), -sin(rad), dx],
                                    [-1,       0,         0, dy],
                                    [0, sin(rad),  cos(rad), dz],
                                    [0,        0,         0, 1]])

        T0_7 = np.identity(4)
        for i in range(0,4):
            for j in range(0,4):
                T0_7[i][j] = arm_ori[i*4+j]
        Mat_nVec_Pos = np.mat([ [rot[0, 0], rot[0, 1], rot[0, 2],tvec[0]],
                                [rot[1, 0], rot[1, 1], rot[1, 2],tvec[1]],
                                [rot[2, 0], rot[2, 1], rot[2, 2],tvec[2]],
                                [   0,         0,         0,        1   ] ])
    
        Mat_VecPos_ImgToBase = T0_7 * TransMat_EndToImg * Mat_nVec_Pos
        return Mat_VecPos_ImgToBase
    # def motor_grasp_pose(self, grasp_msg):
    #     x = grasp_msg.data[0]
    #     y = grasp_msg.data[1]
    #     z = grasp_msg.data[2]
    #     yaw = grasp_msg.data[3]
    #     pitch = grasp_msg.data[4]
    #     roll = grasp_msg.data[5]
    #     phi = grasp_msg.data[6]
    #     print("motor_grasp_pose Float32MultiArray")
    #     print(x, y, z, yaw, pitch, roll, phi)

    #     self.grasp_pose = grasp_msg

    def state_control(self, state, side):
        global picture_pose_reached

        print('START', state, side)
        if state is None:
            state = State.init

        elif state == State.init:
            state = State.take_pic

        elif state == State.take_pic:
            state = State.approach_motor

        elif state == State.approach_motor:
            state = State.finish_init

        elif state == State.finish_init:
            state = None
        print('END', state)
        return state
     
    def strategy(self, state, side):
        print("strategy", state)
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            print("init____________________________")
            cmd['cmd'] = 'jointMove'
            cmd['gripper_cmd'] = 'active'
            cmd['jpos'] = [0, 0, -1.2, 0, 1.87, 0, -0.87, 0] #[0, 0, 0, 0, 0, 0, 0, 0]
            cmd['state'] = State.init
            cmd['speed'] = 20
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)
            
        elif state == State.take_pic:
            # cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            # # if side == 'left':
            # #     cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            # # else:
            # #     cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            # cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            # cmd_queue.put(copy.deepcopy(cmd))
            # # cmd['cmd'] = 'occupied'
            # cmd['state'] = State.apporach_obj
            # cmd_queue.put(copy.deepcopy(cmd))
            # side = self.dual_arm.send_cmd(side, False, cmd_queue)
            # if side != 'fail':
            #     c_pose[side+'_indx'] +=1
            # else:
            #     print('fuckfailfuckfailfuckfail')

            print("take_pic____________________________")
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.take_pic

            picture_pose_reached = True
            print("hheheheheheheh")
            rospy.wait_for_service('/motor_grasp_pose')
            req = rospy.ServiceProxy('/motor_grasp_pose', GraspPose)
            try:
                print('aaaaaaaaaaaaaaa')
                res = req(picture_pose_reached)
                print('bbbbbbbbbbbbbbbbbbb')
            except rospy.ServiceException as e:
                print('Service call failed: %s'%e)

            self.motor_grasp_pose = res

            print('res',res)
            print(res.grasp_pose[0])
            print(res.grasp_pose[1])
            print(res.grasp_pose[2])
            print(res.grasp_pose[3])
            print(res.grasp_pose[4])
            print(res.grasp_pose[5])
            print(res.grasp_pose[6])
            self.motor_grasp_pose = res.grasp_pose
            x = self.motor_grasp_pose[0]
            y = self.motor_grasp_pose[1]
            z = self.motor_grasp_pose[2]
            yaw = self.motor_grasp_pose[3]
            pitch = self.motor_grasp_pose[4]
            roll = self.motor_grasp_pose[5]
            phi = self.motor_grasp_pose[6]
            print("motor_grasp_pose Float32MultiArray")
            print(x, y, z, yaw, pitch, roll, phi)

            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')

        elif state == State.approach_motor:
            print("approach_motor____________________________")
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            
            pos = [self.motor_grasp_pose[0], self.motor_grasp_pose[1], self.motor_grasp_pose[2]]
            rot = [self.motor_grasp_pose[3], self.motor_grasp_pose[4], self.motor_grasp_pose[5]]
            print('pos', pos)
            print('rot', rot)
            cmd['pos'], cmd['euler'], cmd['phi'] = pos, rot, 0
            cmd_queue.put(copy.deepcopy(cmd))
            #cmd['suc_cmd'] = 0
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.approach_motor
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)

            print("movemovemovemovemovemovemovmeovmeovmoemvoemvomevomeovmoemvoemove")
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')          

        elif state == State.finish_init:
            cmd['cmd'] = 'jointMove'
            cmd['gripper_cmd'] = 'reset'
            cmd['jpos'] = [0, 0, 0, 0, 0, 0, 0, 0]
            cmd['state'] = State.finish_init
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
        return side

    def process(self):
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.dual_arm.shutdown)
        while True:
            # l_status = self.dual_arm.left_arm.status
            # if l_status == Status.idle or l_status == Status.occupied:
            #     l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
            #     self.strategy(l_state, 'left')
            # rate.sleep()
            #==============================================================================
            r_status = self.dual_arm.right_arm.status
            if r_status == Status.idle or r_status == Status.occupied:
                r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
                self.strategy(r_state, 'right')
            # rate.sleep()
            # if l_state is None and r_state is None:
            #     if l_status == Status.idle and r_status == Status.idle:
            #         return
            # if l_state is None :
            #     if l_status == Status.idle:
            #         return
            if r_state is None :
                if r_status == Status.idle:
                    return

if __name__ == '__main__':
    strategy = GraspTask('dual_arm', True)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm



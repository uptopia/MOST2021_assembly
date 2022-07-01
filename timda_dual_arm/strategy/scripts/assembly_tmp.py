#!/usr/bin/env python


import rospy
from assembly_srv.srv import GraspPose, GraspPoseRequest
from std_msgs.msg import Float32MultiArray

class GraspTask:
    def __init__(self):
        rospy.init_node('assembly_grasp')
        self.grasp_pose = Float32MultiArray()
        self.run()
        rospy.spin()

    def run(self):
        print("hheheheheheheh")
        picture_pose_reached = 1
        rospy.wait_for_service('/motor_grasp_pose')
        req = rospy.ServiceProxy('/motor_grasp_pose', GraspPose)
        try:
            print('aaaaaaaaaaaaaaa')
            res = req(picture_pose_reached)
            print('bbbbbbbbbbbbbbbbbbb')
        except rospy.ServiceException as e:
            print('Service call failed: %s'%e)

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


if __name__ == '__main__':
    strategy = GraspTask()



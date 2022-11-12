#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import tf
import tf.transformations as tfm

class shooting():
    def __init__(self, robot_name):

        self.robot_name = robot_name

        # Service
        rospy.Service("/{0}/go_sleep".format(robot_name), Trigger, self.vx300s_sleep)
        rospy.Service("/{0}/go_shoot".format(robot_name), Trigger, self.vx300s_shoot)
        rospy.Service("/{0}/go_scan".format(robot_name), Trigger, self.vx300s_scan)
        rospy.Service("/{0}/go_arm_sleep".format(robot_name), Trigger, self.vx300s_down_arm)
        

        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=robot_name, moving_time=1.5, accel_time=0.3, gripper_pressure=0.75, init_node=False)
        self.arm = robot.arm
        self.gripper = robot.gripper

        # self.init()
    # initital posistion
    # [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper, left_finger,  right_finger]
    # [0.0015339808305725455, -1.9067381620407104, 1.6797089576721191, -0.0076699042692780495, 1.4879614114761353, -0.012271846644580364, 1.5385828018188477, 0.05798161029815674, -0.05798161029815674]

    # sleep mode
    # [0.0, -1.8806605339050293, 1.555456519126892, -0.0015339808305725455, 0.8068739175796509, -0.0015339808305725455, 1.5385828018188477, 0.05798161029815674, -0.05798161029815674]

    def init(self):

        self.gripper.open(2.0)
        self.arm.go_to_sleep_pose()
        rospy.loginfo("initial already!")

    def vx300s_shoot(self, req):
        res = TriggerResponse()
        # self.arm.set_single_joint_position(joint_name="shoulder",position=-1.6)
        self.arm.set_single_joint_position(joint_name="elbow",position=1.5)
        res.success = True
        res.message = "reach shooting position"
        
        return res

    def vx300s_scan(self, req):
        res = TriggerResponse()
        self.arm.set_single_joint_position(joint_name="shoulder",position=-1.6)
        res.success = True
        res.message = "reach scanning position"
        
        return res

    def vx300s_down_arm(self, req):
        res = TriggerResponse()
        self.arm.set_single_joint_position(joint_name="wrist_angle",position=1.45)
        res.success = True
        res.success = "reach arm rest position"
        
        return res

    def vx300s_sleep(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_sleep_pose()
            res.success = True
            res.message = "reach default sleep position"
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

if __name__=='__main__':

    rospy.init_node("arm_control_node", anonymous=False)

    robot_name = rospy.get_param("shoot_arm")
    shoot = shooting(robot_name)

    
    rospy.spin()
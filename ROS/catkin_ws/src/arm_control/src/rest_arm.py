#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import tf
import tf.transformations as tfm

class rest():
    def __init__(self, name):

        self.name = name

        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=name, moving_time=1.5, accel_time=0.3, gripper_pressure=0.75, init_node=False)
        self.arm = robot.arm
        self.gripper = robot.gripper

        self.rest()

    def rest(self):

        self.gripper.open(2.0)
        self.arm.go_to_sleep_pose()
        self.arm.set_single_joint_position(joint_name="wrist_angle",position=1.4)
        rospy.loginfo("rest!")


if __name__=='__main__':

    rospy.init_node("arm_control_node", anonymous=False)

    robot_name = rospy.get_param("shoot_arm")
    shoot = rest(robot_name)

    
    rospy.spin()
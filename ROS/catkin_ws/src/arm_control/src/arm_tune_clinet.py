#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from interbotix_xs_modules.arm import InterbotixManipulatorXS

import tf
import tf.transformations as tfm

class tuning():
    def __init__(self, robot_name):
        self.robot_name = robot_name
        # Sever
        rospy.Service("/{0}/go_tune".format(robot_name), Trigger, self.vx300s_single_joint)


        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=robot_name, moving_time=1.5, accel_time=0.3, gripper_pressure=0.75, init_node=False)
        self.arm = robot.arm
        self.gripper = robot.gripper

        # Clinet
        self.service_name = "/{0}/go_tune".format(robot_name)
        # Call service
        rospy.wait_for_service(self.service_name)
        self.arm_tune()

    def arm_tune(self):

        print("clinet",rospy.get_param('set_joint_name'),rospy.get_param('set_joint_position'))

        arm_tnue_service = rospy.ServiceProxy(self.service_name, Trigger)
        tune = TriggerRequest()
        result = arm_tnue_service(tune)

    def vx300s_single_joint(self, req):
        res = TriggerResponse()
        joint_name = rospy.get_param("set_joint_name")
        joint_position = float(rospy.get_param("set_joint_position"))
        self.arm.set_single_joint_position(joint_name=joint_name, position=joint_position)
        res.success = True
        print("sever",joint_name,joint_position)
        return res




if __name__=='__main__':

    rospy.init_node("arm_tune_mode", anonymous=False)

    robot_name = rospy.get_param("shoot_arm")
    
    tune = tuning(robot_name)

    
    rospy.spin()
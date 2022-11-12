#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from interbotix_xs_modules.arm import InterbotixManipulatorXS

import tf
import tf.transformations as tfm

class tuning():
    def __init__(self, robot_name, joint_name, joint_position):
        self.robot_name = robot_name
        self.name = joint_name
        self.position = joint_position

        # Clinet
        self.service_name = "/{0}/go_tune".format(robot_name)

        # Call service
        rospy.wait_for_service(self.service_name)
        self.arm_tune()

        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=robot_name, moving_time=1.5, accel_time=0.3, gripper_pressure=0.75, init_node=False)
        self.arm = robot.arm
        self.gripper = robot.gripper

    def arm_tune(self):
        
        arm_tnue_service = rospy.ServiceProxy(self.service_name, Trigger)
        tune = TriggerRequest()
        result = arm_tnue_service(tune)




if __name__=='__main__':

    rospy.init_node("arm_tune_mode", anonymous=False)

    robot_name = rospy.get_param("shoot_arm")
    joint_name = rospy.get_param("joint_name")
    joint_position = float(rospy.get_param("joint_position"))
    
    tune = tuning(robot_name,joint_name,joint_position)

    
    rospy.spin()
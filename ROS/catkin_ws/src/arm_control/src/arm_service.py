#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import tf
import tf.transformations as tfm

class shooting():
    def __init__(self, robot_name,single_joint_name,single_joint_position):

        self.robot_name = robot_name
        self.single_joint_name = single_joint_name
        self.single_joint_position = single_joint_position

        # Service
        rospy.Service("/{0}/go_sleep".format(robot_name), Trigger, self.vx300s_sleep)
        rospy.Service("/{0}/go_shoot".format(robot_name), Trigger, self.vx300s_shoot)
        rospy.Service("/{0}/go_scan".format(robot_name), Trigger, self.vx300s_scan)
        rospy.Service("/{0}/go_arm_sleep".format(robot_name), Trigger, self.vx300s_down_arm)
        rospy.Service("/{0}/go_tune".format(robot_name), Trigger, self.vx300s_single_joint)
        

        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=robot_name, moving_time=1.5, accel_time=0.3, gripper_pressure=0.75, init_node=False)
        self.arm = robot.arm
        self.gripper = robot.gripper

        # self.init()

    def init(self):

        self.gripper.open(2.0)
        self.arm.go_to_sleep_pose()
        rospy.loginfo("initial already!")

    def vx300s_shoot(self, req):
        res = TriggerResponse()
        self.arm.set_single_joint_position(joint_name="shoulder",position=-1.6)
        self.arm.set_single_joint_position(joint_name="elbow",position=1.5)
        # self.arm.set_joint_positions([-0.04908738657832146, -0.5660389065742493, 0.5460971593856812, 0.05522330850362778, -0.21629129350185394, -0.012271846644580364])
        res.success = True
        
        return res

    def vx300s_scan(self, req):
        res = TriggerResponse()
        self.arm.set_single_joint_position(joint_name="shoulder",position=-1.6)
        res.success = True
        
        return res

    def vx300s_down_arm(self, req):
        res = TriggerResponse()
        self.arm.set_single_joint_position(joint_name="shoulder",position=-1.8729)
        self.arm.set_single_joint_position(joint_name="elbow",position=1.5582)
        self.arm.set_single_joint_position(joint_name="wrist_angle",position=1.4)
        res.success = True
        
        return res

    def vx300s_sleep(self, req):

        res = TriggerResponse()

        try:
            self.arm.go_to_sleep_pose()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def vx300s_single_joint(self, req):
        res = TriggerResponse()
        self.arm.set_single_joint_position(joint_name=self.single_joint_name, position=self.single_joint_position)
        res.success = True
        
        return res



if __name__=='__main__':

    rospy.init_node("arm_control_node", anonymous=False)

    robot_name = rospy.get_param("shoot_arm")
    single_joint_name = rospy.get_param("single_joint_name")
    single_joint_position = float(rospy.get_param("single_joint_position"))

    shoot = shooting(robot_name,single_joint_name,single_joint_position)

    
    rospy.spin()
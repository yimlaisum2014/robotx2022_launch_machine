#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import tf
import tf.transformations as tfm

class shooting():
    def __init__(self, name):

        self.name = name

        # Service
        rospy.Service("/{0}/go_sleep".format(name), Trigger, self.vx300s_sleep)
        rospy.Service("/{0}/go_shoot".format(name), Trigger, self.vx300s_shoot)

        # Subscriber

        # vx300s setup
        robot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=name, moving_time=1.5, accel_time=0.3, gripper_pressure=0.75, init_node=False)

        self.arm = robot.arm
        self.gripper = robot.gripper

        self.init()

    def init(self):

        self.gripper.open(2.0)
        self.arm.go_to_sleep_pose()
        rospy.loginfo("initial already!")

    def vx300s_shoot(self, req):
        res = TriggerResponse()
        self.arm.set_single_joint_position(joint_name="shoulder",position=-1.6)
        # self.arm.set_joint_positions([-0.04908738657832146, -0.5660389065742493, 0.5460971593856812, 0.05522330850362778, -0.21629129350185394, -0.012271846644580364])
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



if __name__=='__main__':

    rospy.init_node("arm_control_node", anonymous=False)

    robot_name = rospy.get_param("shoot_arm")
    shoot = shooting(robot_name)

    
    rospy.spin()
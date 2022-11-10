#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16, Byte


class shotPub():
    def __init__(self):
        self.pub = rospy.Publisher('/shoot_cmd', Byte, queue_size=1)
        self.inital()
    
    def inital(self):
        shoot = Byte
        shoot = 1
        while not rospy.is_shutdown():
            self.pub.publish(shoot)

if __name__=='__main__':

    rospy.init_node("shoot_pub", anonymous=False)
    shoot = shotPub()

    rospy.spin()
#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int8, Byte


class shotPub():
    def __init__(self):
        self.pub = rospy.Publisher('/wamv/shooting', Byte, queue_size=1)
        self.pub_speed = rospy.Publisher('/shooter/speed', Int8, queue_size=1)
        self.pub_step = rospy.Publisher('/shooter/step', Int8, queue_size=1)

        self.inital()
    
    def inital(self):
    
        shoot = Byte
        shoot = 1

        speed = Int8
        step = Int8

        speed = 60
        step = 10
        self.pub_speed.publish(speed)
        self.pub_step.publish(step)

        while not rospy.is_shutdown():
            # test~~~~
            # self.pub.publish(shoot)
            # rospy.sleep(2)
            # print("shoot")
            # test~~~~
            for x in range(0, 3):
                print(x)
                self.pub.publish(shoot)
                x += 1
                rospy.sleep(3)
            break

                        
if __name__=='__main__':

    rospy.init_node("shoot_pub", anonymous=False)
    shoot = shotPub()

    rospy.spin()
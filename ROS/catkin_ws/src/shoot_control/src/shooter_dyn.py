#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16, Byte
from dynamixel_sdk_examples.msg import SetPosition

class shotPub():
    def __init__(self):

        self.sub_shoot = rospy.Subscriber('/wamv/shoot', Int16, self.shoot_cb)
        self.pub_shoot = rospy.Publisher('/shooter/set_position',SetPosition, queue_size=1)

        self.motor_id = 10

        self.motor_home_position = 0
        self.motor_step1 = 1500
        self.motor_step2 = 3000
        self.motor_step3 = 5000

        self.inital()

        
    def inital(self):
        go_home = SetPosition()

        go_home.id = self.motor_id
        go_home.position =  self.motor_home_position 
        self.pub_shoot.publish(go_home)
        print("go home")
    
    def shoot_cb(self,msg):
        shoot_num = Int16
        shoot_num = msg.data

        shoot_step0 = SetPosition()
        shoot_step1 = SetPosition()
        shoot_step2 = SetPosition()
        shoot_step3 = SetPosition()


        if (shoot_num == 1):
            shoot_step1.id = self.motor_id 
            shoot_step1.position = self.motor_step1
            self.pub_shoot.publish(shoot_step1)
            print("position 1")
        
        elif (shoot_num == 2):
            shoot_step2.id = self.motor_id 
            shoot_step2.position = self.motor_step2
            self.pub_shoot.publish(shoot_step2)
            print("position 2")
        

        elif (shoot_num == 3):
            shoot_step3.id = self.motor_id 
            shoot_step3.position = self.motor_step3
            self.pub_shoot.publish(shoot_step3)
            print("position 3")

        elif (shoot_num == 0):
            shoot_step0.id = self.motor_id
            shoot_step0.position = self.motor_home_position
            self.pub_shoot.publish(shoot_step0)
            print("home position")
        
        else :
            print("out of range")


        

                        
if __name__=='__main__':

    rospy.init_node("shoot_pub", anonymous=False)
    shoot = shotPub()

    rospy.spin()
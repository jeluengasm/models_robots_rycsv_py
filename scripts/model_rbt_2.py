#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np
from rospy.numpy_msg import numpy_msg

class Velocity_publisher:
    
    def __init__(self):

        #----- Robot model
        radius = 0.060

        alpha_right =  0.0
        beta_right  =  np.pi/2 
        l_right     =  0.0

        alpha_left = np.pi/2
        beta_left  = np.pi
        l_left     = 0.707
        
        alpha_front = np.pi/4
        beta_front  = 3*np.pi/4
        l_front     = 0.50

        alpha_back = 3*np.pi/4
        beta_back  = np.pi/4
        l_back     = 0.50

        J1 = np.array( [( np.sin(alpha_right+beta_right), -np.cos(alpha_right+beta_right) , -l_right*np.cos(beta_right)),
                        ( np.sin(alpha_left+beta_left)  , -np.cos(alpha_left+beta_left)   , -l_left*np.cos(beta_left))  ,
                        ( np.sin(alpha_front+beta_front), -np.cos(alpha_front+beta_front) , -l_front*np.cos(beta_front)),
                        ( np.sin(alpha_back+beta_back)  , -np.cos(alpha_back+beta_back)   , -l_back*np.cos(beta_back))  ])

        J2 = radius*np.identity(4)

        self.Jacobian = np.matmul( np.linalg.pinv(J2) , J1 )

        print ("J1: \n", J1)
        print ("J2: \n", J2)
        print ("Jacobian: \n", self.Jacobian)

        #----- Subscribers
        self.cmd_vel_sub_ = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb, queue_size=10)
        
        
        #----- Publishers
        self.pub_front_wheel = rospy.Publisher("/front_wheel_ctrl/command", Float64 , queue_size=10)
        self.pub_back_wheel  = rospy.Publisher("/back_wheel_ctrl/command", Float64 , queue_size=10)
        self.pub_left_wheel  = rospy.Publisher("/left_wheel_ctrl/command", Float64 , queue_size=10)
        self.pub_right_wheel = rospy.Publisher("/right_wheel_ctrl/command", Float64 , queue_size=10)


    #------------------------------------------------------#
    ## Laser callback function
    def cmd_vel_cb(self, cmd_vel): 
        
        command = np.array([0,0,0] , dtype=np.float)

        command[0] = cmd_vel.linear.x
        command[1] = cmd_vel.linear.y
        command[2] = cmd_vel.angular.z

        print("Command : ", command)

        result = np.matmul( self.Jacobian, command)

        print("Result : ", result)

        # Right Wheel
        msgFloat = Float64()
        msgFloat.data = result[0]
        self.pub_right_wheel.publish(msgFloat)

        # Left Wheel
        msgFloat = Float64()
        msgFloat.data = result[1]
        self.pub_left_wheel.publish(msgFloat)
        
        # Front wheel
        msgFloat = Float64()
        msgFloat.data = result[2]
        self.pub_front_wheel.publish(msgFloat)

        # Back wheel
        msgFloat = Float64()
        msgFloat.data = result[3]
        self.pub_back_wheel.publish(msgFloat)
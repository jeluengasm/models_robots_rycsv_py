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
        
        alpha_front = 0.0
        beta_front  = 0.0
        l_front     = 0.433

        alpha_left = np.pi/2
        beta_left  = 0
        l_left     = 0.25

        alpha_right = -np.pi/2
        beta_right  =  np.pi 
        l_right     =  0.25

        J1 = np.array( [( np.sin(alpha_front+beta_front), -np.cos(alpha_front+beta_front) , -l_front*np.cos(beta_front) ),
                        ( np.sin(alpha_left+beta_left)  , -np.cos(alpha_left+beta_left)   , -l_left*np.cos(beta_left)),
                        ( np.sin(alpha_right+beta_right), -np.cos(alpha_right+beta_right) , -l_right*np.cos(beta_right))])

        J2 = radius*np.identity(3)

        self.Jacobian = np.matmul( np.linalg.pinv(J2) , J1 )

        print ("J1: \n", J1)
        print ("J2: \n", J2)
        print ("Jacobian: \n", self.Jacobian)

        #----- Subscribers
        self.cmd_vel_sub_ = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb, queue_size=10)
        
        
        #----- Publishers
        self.pub_front_wheel = rospy.Publisher("/front_wheel_ctrl/command", Float64 , queue_size=10)
        self.pub_left_wheel  = rospy.Publisher("/left_wheel_ctrl/command", Float64 , queue_size=10)
        self.pub_right_wheel = rospy.Publisher("/right_wheel_ctrl/command", Float64 , queue_size=10)

        #------------------------------#
        # Polling to stop robot

        while not rospy.is_shutdown():

            rospy.loginfo("Waiting for command")
            
            # Get a command from keyboard
            command_char = raw_input()
            rospy.loginfo("received: " + command_char)
            
            # If the key pressed is s then stop the robot
            if(command_char == 's'):
                rospy.loginfo("Stopped robot")
                msgFloat = Float64()
                msgFloat = 0.0
                self.pub_front_wheel.publish(msgFloat)
                self.pub_left_wheel.publish(msgFloat)
                self.pub_right_wheel.publish(msgFloat)

            # If the key pressed is q then request a shutdown for the node
            if(command_char == 'q'):
                rospy.loginfo("Exit")
                rospy.signal_shutdown("Request shutdown")


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

        # Front wheel
        msgFloat = Float64()
        msgFloat.data = result[0]
        self.pub_front_wheel.publish(msgFloat)

        # Left Wheel
        msgFloat = Float64()
        msgFloat.data = result[1]
        self.pub_left_wheel.publish(msgFloat)
        
        # Right Wheel
        msgFloat = Float64()
        msgFloat.data = result[2]
        self.pub_right_wheel.publish(msgFloat)
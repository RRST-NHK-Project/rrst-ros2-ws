#!/usr/bin/env python3
## coding: UTF-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self,ps4_msg):
        '''
        LS_X = (-1) * ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        RS_X = (-1) * ps4_msg.axes[2]
        RS_Y = ps4_msg.axes[5]

        print(LS_X, LS_Y, RS_X, RS_Y)
        '''
        CROSS = ps4_msg.buttons[1]
        CIRCLE = ps4_msg.buttons[2]
        TRIANGLE = ps4_msg.buttons[3]
        SQUARE = ps4_msg.buttons[0]
    
        LEFT = ps4_msg.axes[12] == 1.0
        RIGHT = ps4_msg.axes[12] == -1.0
        UP = ps4_msg.axes[13] == 1.0
        DOWN = ps4_msg.axes[13] == -1.0
    
        L1 = ps4_msg.buttons[4]
        R1 = ps4_msg.buttons[5]
    
        L2 = ps4_msg.buttons[6]
        R2 = ps4_msg.buttons[7]
        
        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[12]
    
        L3 = ps4_msg.buttons[10]
        R3 = ps4_msg.buttons[11]



        if SQUARE  == 1:
            print("SAUARE ") 
        
        if CROSS == 1:
            print("CROSS") 
        
        if  CIRCLE== 1:
            print("CIRCLE") 
        
        if  TRIANGLE== 1:
            print("TRIANGLE") 
        
        if  UP == 1:
            print("UP") 
        
        if  DOWN == 1:
            print("DOWN") 
        
        if  LEFT == 1:
            print("LEFT") 
        
        if  RIGHT == 1:
            print("RIGHT") 
        
        if  L1 == 1:
            print("L1") 
        
        if  R1 == 1:
            print("R1") 
        
        if  L2 == 1:
            print("L2") 
        
        if  R2 == 1:
            print("R2") 
        
        if  L3 == 1:
            print("L3") 
        
        if  R3 == 1:
            print("R3") 
        
        if  SHARE == 1:
            print("SHARE") 
        
        if  OPTION == 1:
            print("OPTION") 
        
        if  PS == 1:
            print("PS") 
            
        


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

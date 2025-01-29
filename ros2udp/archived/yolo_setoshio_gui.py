import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

import tkinter as tk
import os
from PIL import ImageTk


obj = [0,0,0,0,0]
color = [0,0,0,0,0]

c1 = "Red"
c2 = "Blue"
c3 = "White"


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Int32MultiArray,'setoshio_pub',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        
        obj[0] = msg.data[0]
        obj[1] = msg.data[1]
        obj[2] = msg.data[2]
        obj[3] = msg.data[3]
        obj[4] = msg.data[4]
        
        print(str(obj[0])+str(obj[1])+str(obj[2])+str(obj[3])+str(obj[4]))

 


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

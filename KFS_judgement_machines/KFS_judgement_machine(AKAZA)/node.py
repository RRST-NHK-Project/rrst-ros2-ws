#!/usr/bin/env python3
## coding: UTF-8
#:)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math

class Publisher(Node):

    def __init__(self):
        super().__init__('responder')
        self.sub = self.create_subscription(Int32MultiArray, '/base_msg', self.msg_callback)
        self.pub = self.create_publisher(String, '/ex_msg', 10)

    def msg_callback(self, msg):
       print(msg) # Subscribeしたメッセージをprint
	    msg.data = msg.data + "!" # メッセージに「！」を付け加える処理
       self.pub.publish(msg) # 処理したメッセージをPublish

    def pub_a_
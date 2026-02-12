
#トチウ


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
        
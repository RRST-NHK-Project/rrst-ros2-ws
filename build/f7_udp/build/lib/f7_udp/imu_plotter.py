#!/usr/bin/env python3
## coding: UTF-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import time
import math
import matplotlib.pyplot as plt
import numpy as np
import quaternion
from transforms3d.euler import quat2euler
import signal

class IMU_Listener(Node):

    def __init__(self):
        super().__init__("imu_listener")

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            PoseStamped, "/wit_ros/imu_pose", self.listener_callback, qos_profile
        )
        self.subscription  # prevent unused variable warning

        # Initialize the plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_ylim(0, 1)
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        plt.ion()
        plt.show()

        # Flag for graceful shutdown
        self.shutdown_flag = False

    def listener_callback(self, imu_msg):
        if self.shutdown_flag:
            return

        # Extract quaternion
        x = imu_msg.pose.orientation.x
        y = imu_msg.pose.orientation.y
        z = imu_msg.pose.orientation.z
        w = imu_msg.pose.orientation.w

        quat = np.quaternion(w, x, y, z)


        # クォータニオンからオイラー角へ変換
        roll,pitch,yaw = quaternion.as_euler_angles(quat)

        # Convert yaw to degrees
        yaw_deg = math.degrees(yaw)

        # Ensure yaw is between 0 and 360 degrees
        yaw_deg = (yaw_deg + 360) % 360

        # Update the plot
        angle = np.radians(yaw_deg)
        
        # Draw new line
        self.line.set_data([0, angle], [0, 1])
        
        self.ax.set_title(f'Yaw: {yaw_deg:.2f}°')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    imu_listener = IMU_Listener()

    exec.add_node(imu_listener)

    def signal_handler(sig, frame):
        print('Ctrl+C pressed. Shutting down...')
        imu_listener.shutdown_flag = True
        exec.shutdown()
        plt.close()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        imu_listener.destroy_node()
        exec.shutdown()
        plt.close()

if __name__ == "__main__":
    main()
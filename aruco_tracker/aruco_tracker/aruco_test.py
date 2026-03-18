import cv2
import numpy as np
from cv2 import aruco
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_pose', 10)
        self.id_pub = self.create_publisher(Int32, 'aruco_id', 10)
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
      
        # Camera
        self.cap = cv2.VideoCapture(4, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("カメラが開けません")
            exit()

        # ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

        # Camera params
        self.camera_matrix = np.array([
            [600,   0, 320],
            [  0, 600, 240],
            [  0,   0,   1]
        ], dtype=np.float32)

        self.dist_coeffs = np.zeros((5, 1))
        self.marker_length = 0.05

        # Timer (30 FPS)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("フレーム取得失敗")
            return

        ret, frame = self.cap.read()
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(
            
            gray, self.aruco_dict, parameters=self.parameters
        )

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)


        if ids is not None:

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            # 1つ目のマーカーだけ publish（複数対応も可能）
            t = tvecs[0][0]
            r = rvecs[0][0]
            marker_id = int(ids[0][0])

            # PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera"

            pose_msg.pose.position.x = float(t[0])
            pose_msg.pose.position.y = float(t[1])
            pose_msg.pose.position.z = float(t[2])

            # rvec → quaternion（OpenCV の Rodrigues）
            R, _ = cv2.Rodrigues(r)
            qw = np.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0
            qx = (R[2,1] - R[1,2]) / (4.0 * qw)
            qy = (R[0,2] - R[2,0]) / (4.0 * qw)
            qz = (R[1,0] - R[0,1]) / (4.0 * qw)

            pose_msg.pose.orientation.x = float(qx)
            pose_msg.pose.orientation.y = float(qy)
            pose_msg.pose.orientation.z = float(qz)
            pose_msg.pose.orientation.w = float(qw)

            # Publish
            self.pose_pub.publish(pose_msg)

            id_msg = Int32()
            id_msg.data = marker_id
            self.id_pub.publish(id_msg)

            # Debug print
            self.get_logger().info(
                f"ID:{marker_id} Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

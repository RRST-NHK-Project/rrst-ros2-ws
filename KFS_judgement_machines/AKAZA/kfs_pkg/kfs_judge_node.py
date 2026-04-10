import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import os
import kfs_judgement  

class KFS_Judge_Node(Node):
    def __init__(self):
        super().__init__("kfs_judge_node")
        self._bridge = CvBridge()
        
        # --- 1. AKAZE & 初期設定 ---
        self.akaze = cv2.AKAZE_create()
        self.ratio = 0.87
        self.point_num_limit = 75
        self.expand_query = 0.5

        # --- 2. マップ画像の準備 (kfs_judgementのロジックを利用) ---
        list_path = "/path/to/KFS_image_list.png"
        self.kp_map, self.des_map, self.h_map, self.w_map = kfs_judgement.prepare_map_data(list_path, self.akaze)

        # --- 3. Topic通信設定 ---
        self.img_sub = self.create_subscription(Image, "/plane_detection/cropped_image", self.image_callback, 10)
        self.info_sub = self.create_subscription(Float32MultiArray, "/cube_detection/info", self.info_callback, 10)
        self.res_pub = self.create_publisher(Float32MultiArray, "/KFS_judge_result", 10)

        self.latest_info = None
        self.get_logger().info("KFS Node (Library Import Mode) 起動完了")

    def info_callback(self, msg):
        self.latest_info = msg.data

    def image_callback(self, msg):
        # cube_detector の情報と同期
        if self.latest_info is None or self.latest_info[0] == 0.0:
            return
        
        # ROS 2画像をOpenCV形式に変換
        query_roi = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        query_gray = cv2.cvtColor(query_roi, cv2.COLOR_BGR2GRAY)
        
        # リサイズ（kfs_judgementの想定に合わせる）
        query_img = cv2.resize(query_gray, None, fx=self.expand_query, fy=self.expand_query)
        h_q, w_q = query_img.shape[:2]

        # 特徴量抽出
        kp_q, des_q = self.akaze.detectAndCompute(query_img, None)
        if des_q is None: return

        # マッチング
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(des_q, self.des_map, k=2)
        good = [m for m, n in matches if m.distance < self.ratio * n.distance]

        if len(good) > 4:
            good = sorted(good, key=lambda x: x.distance)
            p_num = min(len(good), self.point_num_limit)
            q_kp_list = [kp_q[m.queryIdx] for m in good[:p_num]]
            m_kp_list = [self.kp_map[m.trainIdx] for m in good[:p_num]]
            
            # --- 核心：kfs_judgement の関数を呼び出し ---
            deg, size, m1, m2 = kfs_judgement.vote_point(q_kp_list, m_kp_list, p_num)

            if deg is not None:
                # 中心点計算 (ここも kfs_judgement 内に計算関数があるとさらにスッキリします)
                # 今回は元のコードの計算式を適用
                q_x1, q_y1 = q_kp_list[m1].pt
                m_x1, m_y1 = m_kp_list[m1].pt
                q_xc, q_yc = w_q / 2, h_q / 2
                
                q_c_deg = math.atan2(q_yc - q_y1, q_xc - q_x1) * 180 / np.pi
                q_c_len = math.sqrt((q_xc - q_x1)**2 + (q_yc - q_y1)**2)
                
                m_c_deg = q_c_deg - deg
                m_c_len = q_c_len / size
                m_c_rad = m_c_deg * np.pi / 180
                
                m_xc = m_x1 + m_c_len * np.cos(m_c_rad)
                m_yc = m_y1 + m_c_len * np.sin(m_c_rad)

                # マップ範囲内なら結果をパブリッシュ
                if (0 <= m_xc <= self.w_map) and (0 <= m_yc <= self.h_map):
                    res_msg = Float32MultiArray()
                    # [フラグ, マップX, マップY, 深度, 画像内ズレ]
                    res_msg.data = [1.0, float(m_xc), float(m_yc), float(self.latest_info[5]), float(self.latest_info[1])]
                    self.res_pub.publish(res_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KFS_Judge_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

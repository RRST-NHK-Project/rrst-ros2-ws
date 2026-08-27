#!/usr/bin/env python3
"""
z_joint/r_jointは実機ではmotor1/motor2の2モータによる差動機構で駆動される
(z = mix_k*(m1+m2), r = mix_k*(m1-m2))。

本ノードは双方向にミキシングする:
  - motor1_joint/motor2_jointが動いた場合 -> z_joint/r_jointを計算して追従させる
  - z_joint/r_jointが動いた場合           -> motor1_joint/motor2_jointを逆算して追従させる
    (m1 = (z+r)/(2*mix_k), m2 = (z-r)/(2*mix_k))

/joint_states を購読して直近の値との差分からどちら側が操作されたかを判定し、
計算結果を /mixed_joint_states に出力する。joint_state_publisher(_gui)の
source_listにこのトピックを指定すると、スライダー操作(motor1/2側でもz/r側でも)
に応じて反対側のスライダー表示も自動追従する(launch/display.launch.py参照)。

自分がpublishした値がsource_list経由でそのまま/joint_statesにechoされてくるため、
「直前に自分が計算した値」との差分がなければ無視することで、無限ループや
値の奪い合いを防いでいる。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

EPSILON = 1e-6


class MotorMixerNode(Node):

    def __init__(self):
        super().__init__('motor_mixer_node')

        self.declare_parameter('mix_k', 0.5)
        self.declare_parameter('motor1_joint', 'motor1_joint')
        self.declare_parameter('motor2_joint', 'motor2_joint')
        self.declare_parameter('z_joint', 'z_joint')
        self.declare_parameter('r_joint', 'r_joint')
        self.declare_parameter('input_topic', 'joint_states')
        self.declare_parameter('output_topic', 'mixed_joint_states')

        self.mix_k_ = self.get_parameter('mix_k').value
        self.motor1_name_ = self.get_parameter('motor1_joint').value
        self.motor2_name_ = self.get_parameter('motor2_joint').value
        self.z_name_ = self.get_parameter('z_joint').value
        self.r_name_ = self.get_parameter('r_joint').value

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub_ = self.create_publisher(JointState, output_topic, 10)
        self.sub_ = self.create_subscription(
            JointState, input_topic, self.joint_state_callback, 10)

        # 直近の(m1, m2, z, r)。Noneの間は基準値をまだ取得していない状態。
        self.last_ = None

        self.get_logger().info(
            f'motor_mixer_node started (bidirectional): '
            f'{self.motor1_name_}/{self.motor2_name_} <-> {self.z_name_}/{self.r_name_} '
            f'(mix_k={self.mix_k_}), {input_topic} -> {output_topic}')

    def joint_state_callback(self, msg: JointState):
        try:
            m1 = msg.position[msg.name.index(self.motor1_name_)]
            m2 = msg.position[msg.name.index(self.motor2_name_)]
            z = msg.position[msg.name.index(self.z_name_)]
            r = msg.position[msg.name.index(self.r_name_)]
        except ValueError:
            # 4関節が揃うまで(起動直後など)は何もしない
            return

        if self.last_ is None:
            # 基準値を記録するだけで、まだ計算・publishはしない
            self.last_ = {'m1': m1, 'm2': m2, 'z': z, 'r': r}
            return

        motor_changed = (abs(m1 - self.last_['m1']) > EPSILON or
                         abs(m2 - self.last_['m2']) > EPSILON)
        zr_changed = (abs(z - self.last_['z']) > EPSILON or
                     abs(r - self.last_['r']) > EPSILON)

        if not motor_changed and not zr_changed:
            return

        if zr_changed and not motor_changed:
            # z/r側が動いた -> motor1/motor2を逆算
            new_m1 = (z + r) / (2.0 * self.mix_k_)
            new_m2 = (z - r) / (2.0 * self.mix_k_)
            self.last_ = {'m1': new_m1, 'm2': new_m2, 'z': z, 'r': r}
            self.publish_result(self.motor1_name_, new_m1, self.motor2_name_, new_m2)
        else:
            # motor側が動いた(両方同時に変化した稀なケースはmotor側を優先)
            new_z = self.mix_k_ * (m1 + m2)
            new_r = self.mix_k_ * (m1 - m2)
            self.last_ = {'m1': m1, 'm2': m2, 'z': new_z, 'r': new_r}
            self.publish_result(self.z_name_, new_z, self.r_name_, new_r)

    def publish_result(self, name1, value1, name2, value2):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [name1, name2]
        out.position = [value1, value2]
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MotorMixerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

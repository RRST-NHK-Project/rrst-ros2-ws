#!/usr/bin/env python3
"""実機(ros2can)のセンサ値をsokiのjoint_states(root_theta_joint/tip_theta_joint/
z_joint/r_joint)に変換して/mixed_joint_states相当としてpublishするノード。

soki_simのmotor_mixer_nodeがsim側(joint_state_publisher_guiのスライダー操作)を
motor1/2<->z/rで双方向ミキシングするのに対し、本ノードは実機センサ値(一方向)から
最終的な4関節角度を作るのが役割。command_gui_node/trajectory_follower_nodeは
このノードの出力を購読すればsim/実機を区別せず同じインタフェースで扱える。

前提となる伝達経路の詳細・数値根拠はnote/hardware_mapping.txt、
CAN_ID/スロット割当の実測値はnote/can_mapping.txtを参照。値が変わったときは
本ファイルのコードではなく、起動時に読み込むパラメータ(yaml)側を書き換えること。

購読トピック (ros2can/ros_backend.py が配信する Int32MultiArray, 24スロット):
  serial_rx_{cubemars_device_id}_unwrapped
    -> M{n} position (0.1deg/LSB) から root_theta_joint / tip_theta_joint を算出。
       _zeroedではなく_unwrapped(ラップアラウンド解決済みの生の絶対値)を購読する。
       root_theta_jointはCubeMars AK40-10本体へ「Set Origin」CANコマンド
       (control_mode=3、trajectory_follower_nodeの/set_root_theta_origin
       サービス経由、2026-08-27実装)を送ることで実機エンコーダ自体の原点を
       永続的に(フラッシュ保存、電源off/onを跨いでも)真の機械原点に合わせる方式
       にしたため、本ノード側でのオフセット補正(root_theta_offset_rad)は廃止した。
       tip_theta_jointはCubeMars本体側にSet Origin機能を使っておらず、従来通り
       ros2canの/zero_channel(オフセットがGUIプロセスのメモリ上限りで永続化
       されない)には依存せず、本ノード側のパラメータ(tip_theta_offset_rad、
       yamlに保存され再起動しても消えない)で真の原点とのズレを補正する。
       較正手順: 関節を原点センサの位置(真の機械原点)へ物理的に合わせ、
       その瞬間の_unwrapped値でoffsetを逆算してyamlに書き込む。
  serial_rx_{robomas_device_id}_unwrapped
    -> ROBOMAS(C610/M2006)内蔵ロータエンコーダのCAN帰還(M{n} angle)から
       motor1_joint/motor2_joint(線形変位)を算出し、z_joint/r_jointへ合成。
       外付けAMTエンコーダ(ENC1/ENC2、CAN_HOST配下ノード)は2026-08-31の方針転換で
       不使用になった(以前はこちらを位置の真値としていたが、ロボマス内蔵エンコーダに
       統一。note/hardware_mapping.txt参照)。相対(インクリメンタル)エンコーダの
       ため毎起動ホーミングが必須だが、これもros2canの/zero_channel(単体チャンネルを
       ゼロ化するだけで、z=k*(m1+m2)/r=k*(m1-m2)という合成後の量を正しい基準に
       合わせる用途には向かない)には頼らず、z_offset_m/r_offset_mパラメータ
       (homing_node担当、rcl_interfacesのSetParametersサービスで実行時に設定
       される。yamlには保存しないランタイム専用値)で合成後の値を補正する。
       ホーミング未実施の間はz_offset_m=r_offset_m=0.0のまま(起動直後の生値)。
  fallback_topic (2026-09-07追加、パラメータで指定。空文字なら無効):
    -> trajectory_follower_nodeの理想軌道(output_topicとは別トピックに分離
       済み)。まだ実機帰還を一度も受信していない関節群(root_theta/tip_theta、
       z/rの2グループ単位)についてのみ、そのままoutput_topicへ転送する
       (_on_fallback参照)。実機未配線・一部軸のみ実機接続の構成でも、残りの
       軸はsimの理想軌道で動き続けられるようにするためのフォールバックで、
       一度でも実機帰還を受信したグループは以後この転送を無視する
       (実測 -> 理想軌道への逆行はしない)。note/hardware_mapping.txt
       「mixed_joint_statesの真値ソース」参照。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

# 帰還(angle)のスケール。trajectory_follower_node.pyのROBOMAS_FEEDBACK_POSITION_SCALE_DEG
# と一致させること(cubemarsの帰還と同じ0.1deg/LSB、robomas.cpp参照)。
ROBOMAS_FEEDBACK_POSITION_SCALE_DEG = 0.1


class RealJointBridgeNode(Node):

    def __init__(self):
        super().__init__('real_joint_bridge_node')

        # ---- CubeMars (root_theta_joint / tip_theta_joint) ----
        self.declare_parameter('cubemars_device_id', 11)
        self.declare_parameter('cubemars_root_theta_index', 0)   # M1
        self.declare_parameter('cubemars_tip_theta_index', 1)    # M2
        self.declare_parameter('cubemars_position_scale_deg', 0.1)  # LSB -> deg
        self.declare_parameter('root_theta_reduction', 112.0 / 24.0)  # 112/24
        self.declare_parameter('tip_theta_reduction', 1.4)          # 28T/20T
        self.declare_parameter('root_theta_sign', 1.0)
        self.declare_parameter('tip_theta_sign', 1.0)
        # tip_thetaのみ対象(root_thetaはCubeMars本体のSet Originコマンドで原点を
        # 永続化するため、本ノード側のオフセット補正は廃止した。ファイル冒頭コメント参照)。
        # 実機組み立て後の初回較正のみで決める、真の機械原点とのズレ補正値。
        # ros2canの/zero_channelに頼らずここで永続的に持つ(較正手順はファイル冒頭コメント参照)。
        self.declare_parameter('tip_theta_offset_rad', 0.0)

        # ---- ROBOMAS (motor1_joint / motor2_joint、内蔵ロータエンコーダのCAN帰還を
        # 位置の真値として使う。2026-08-31方針転換でENC1/ENC2(外付けAMTエンコーダ)
        # は不使用に変更した) ----
        self.declare_parameter('robomas_device_id', 21)
        self.declare_parameter('robomas_motor1_index', 0)   # M1 angle帰還スロット
        self.declare_parameter('robomas_motor2_index', 1)   # M2 angle帰還スロット
        self.declare_parameter('pulley_pitch_diameter_mm', 22.92)       # ピッチ円直径、確認済み
        self.declare_parameter('mix_k', 0.5)
        self.declare_parameter('motor1_sign', 1.0)
        self.declare_parameter('motor2_sign', 1.0)

        # ---- 出力joint名・トピック ----
        self.declare_parameter('root_theta_joint', 'root_theta_joint')
        self.declare_parameter('tip_theta_joint', 'tip_theta_joint')
        self.declare_parameter('z_joint', 'z_joint')
        self.declare_parameter('r_joint', 'r_joint')
        self.declare_parameter('output_topic', 'mixed_joint_states')
        # trajectory_follower_nodeの理想軌道(output_topicとは別トピックに分離済み、
        # note/hardware_mapping.txt「mixed_joint_statesの真値ソース」参照)を購読し、
        # まだ実機帰還を一度も受信していない関節群についてのみそのまま転送する
        # (2026-09-07追加)。空文字(デフォルト)なら転送しない。実機を配線前・
        # 一部軸のみ実機接続の構成でもsim側の表示・GUI操作が固まらないようにする
        # ためのフォールバックで、一度でも実機帰還を受信した関節群は以後この
        # フォールバックを無視し実測値のみを使う(実測→理想軌道への逆行はしない)。
        self.declare_parameter('fallback_topic', '')

        gp = self.get_parameter
        self.cubemars_device_id_ = gp('cubemars_device_id').value
        self.root_theta_index_ = gp('cubemars_root_theta_index').value
        self.tip_theta_index_ = gp('cubemars_tip_theta_index').value
        self.cubemars_scale_deg_ = gp('cubemars_position_scale_deg').value
        self.root_theta_reduction_ = gp('root_theta_reduction').value
        self.tip_theta_reduction_ = gp('tip_theta_reduction').value
        self.root_theta_sign_ = gp('root_theta_sign').value
        self.tip_theta_sign_ = gp('tip_theta_sign').value
        self.tip_theta_offset_ = gp('tip_theta_offset_rad').value

        self.robomas_device_id_ = gp('robomas_device_id').value
        self.robomas_motor1_index_ = gp('robomas_motor1_index').value
        self.robomas_motor2_index_ = gp('robomas_motor2_index').value
        self.pulley_radius_m_ = (gp('pulley_pitch_diameter_mm').value / 2.0) / 1000.0
        self.mix_k_ = gp('mix_k').value
        self.motor1_sign_ = gp('motor1_sign').value
        self.motor2_sign_ = gp('motor2_sign').value

        # homing_nodeがSetParametersで実行時に更新するランタイム専用オフセット
        # (yamlへの保存値は起動直後の初期値=0.0でよい)。
        self.declare_parameter('z_offset_m', 0.0)
        self.declare_parameter('r_offset_m', 0.0)

        self.root_theta_name_ = gp('root_theta_joint').value
        self.tip_theta_name_ = gp('tip_theta_joint').value
        self.z_name_ = gp('z_joint').value
        self.r_name_ = gp('r_joint').value
        output_topic = gp('output_topic').value
        fallback_topic = gp('fallback_topic').value

        self.pub_ = self.create_publisher(JointState, output_topic, 10)

        # 実機帰還を一度でも受信したかどうか(cubemars=root_theta/tip_theta、
        # robomas=z/rの2グループ単位。_on_fallback参照)。
        self._cubemars_received_ = False
        self._robomas_received_ = False

        self.create_subscription(
            Int32MultiArray, f'serial_rx_{self.cubemars_device_id_}_unwrapped',
            self._on_cubemars, 10)
        self.create_subscription(
            Int32MultiArray, f'serial_rx_{self.robomas_device_id_}_unwrapped',
            self._on_robomas_feedback, 10)
        if fallback_topic:
            self.create_subscription(JointState, fallback_topic, self._on_fallback, 10)

        self.get_logger().info(
            f'real_joint_bridge_node started: '
            f'cubemars(device_id={self.cubemars_device_id_}) + '
            f'robomas(device_id={self.robomas_device_id_}, '
            f'motor1_index={self.robomas_motor1_index_}, '
            f'motor2_index={self.robomas_motor2_index_}) '
            f'-> {output_topic}'
            + (f' (fallback: {fallback_topic})' if fallback_topic else ''))

    def _on_cubemars(self, msg: Int32MultiArray):
        # 以前はrobomas側の帰還も揃うまでpublishを待っていたが、これだとroot_theta/
        # tip_thetaのみの部分実機テスト(real_root_theta_test.launch.py等、ROBOMAS
        # 未接続)ではmixed_joint_statesが永久に出ずsim表示が固まってしまっていた。
        # joint_state_publisherは受信したJointStateに含まれる関節名だけを更新し
        # 他は保持する仕様のため、CubeMars側だけが届いた時点でroot_theta/
        # tip_thetaだけをpublishしても安全(2026-09-07、ユーザー指摘: simは実機の
        # 実測値に追従して表示すべきで、trajectory_follower_nodeの理想軌道と
        # 競合させない設計にする一連の変更の一部)。
        self._cubemars_received_ = True
        root_theta_raw = msg.data[self.root_theta_index_]
        tip_theta_raw = msg.data[self.tip_theta_index_]
        root_theta_deg = root_theta_raw * self.cubemars_scale_deg_
        tip_theta_deg = tip_theta_raw * self.cubemars_scale_deg_
        root_theta = (self.root_theta_sign_ * math.radians(root_theta_deg)
                      / self.root_theta_reduction_)
        tip_theta = (self.tip_theta_sign_ * math.radians(tip_theta_deg)
                     / self.tip_theta_reduction_ + self.tip_theta_offset_)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [self.root_theta_name_, self.tip_theta_name_]
        out.position = [root_theta, tip_theta]
        self.pub_.publish(out)

    def _on_robomas_feedback(self, msg: Int32MultiArray):
        # _on_cubemarsと同じ理由でcubemars側の帰還を待たずに独立してpublishする。
        self._robomas_received_ = True
        m1_deg = msg.data[self.robomas_motor1_index_] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        m2_deg = msg.data[self.robomas_motor2_index_] * ROBOMAS_FEEDBACK_POSITION_SCALE_DEG
        motor1_joint = self.motor1_sign_ * math.radians(m1_deg) * self.pulley_radius_m_
        motor2_joint = self.motor2_sign_ * math.radians(m2_deg) * self.pulley_radius_m_

        z = self.mix_k_ * (motor1_joint + motor2_joint) + self.get_parameter('z_offset_m').value
        r = self.mix_k_ * (motor1_joint - motor2_joint) + self.get_parameter('r_offset_m').value

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [self.z_name_, self.r_name_]
        out.position = [z, r]
        self.pub_.publish(out)

    def _on_fallback(self, msg: JointState):
        """trajectory_follower_nodeの理想軌道(fallback_topic)のうち、まだ実機
        帰還を一度も受信していない関節群だけをそのままoutput_topicへ転送する。
        実機未配線・一部軸のみ実機接続の構成でも、残りの軸はsim(理想軌道)で
        動き続けられるようにするためのフォールバック(2026-09-07追加)。一度でも
        実機帰還を受信した関節群は、以後この転送を無視し実測値のみを使う
        (実測 -> 理想軌道への逆行はしない)。"""
        names = []
        positions = []
        for name, pos in zip(msg.name, msg.position):
            if name in (self.root_theta_name_, self.tip_theta_name_) and self._cubemars_received_:
                continue
            if name in (self.z_name_, self.r_name_) and self._robomas_received_:
                continue
            names.append(name)
            positions.append(pos)
        if not names:
            return
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = positions
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RealJointBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

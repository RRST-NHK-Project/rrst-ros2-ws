#!/usr/bin/env python3
"""z_joint/r_jointのリミットスイッチ(上限・下限、計4個)の位置をrvizへ
MarkerArrayとして表示するノード(2026-09-09追加)。

物理センサはz/r合成後の軸に1個ずつ上限・下限がある(合計4個。homing_node.py
冒頭docstring・note/can_mapping.txt「z/r上限リミットスイッチ」参照)。
下限側はhoming_nodeが原点較正に使うセンサそのもの(homing_node.pyの
z_ref_value_m/r_ref_value_mが、そのセンサ検出位置でのz/r真値)。上限側は
trajectory_follower_nodeが過走防止の安全停止用に別途監視しているだけで、
対応する位置の実測値はどこにも保持されていない(要実測、デフォルトは
soki_sim.urdf.xacroのz_upper/r_upper(ソフトリミットの見積り値)を流用)。

本ノードはtrajectory_follower_node._setup_limit_switches/_limit_triggered
と同じCAN_HOST購読方式(device_id/node_index/local_index、device_id=0は
未配線・無効)を独立に再実装している(重複管理。trajectory_follower_node側の
安全停止ロジックに影響を与えないよう、既存ノードには触れずマーカー表示専用の
別ノードとして追加した)。位置(*_position_m)・配線(*_device_id等)とも
trajectory_follower_node/homing_nodeのyaml設定と値を一致させること
(config/real_joint_bridge.yamlのlimit_switch_marker_node節参照)。

マーカーはz側をturret_link基準、r側をlift_link基準の固定位置に置く
(soki_sim.urdf.xacroのz_joint/r_jointのorigin・axisと一致させること。
z_jointはorigin xyz=(0,0,lift_size_z/2)、axis=(0,0,1)なので、turret_link
原点からのオフセットはZ_JOINT_ORIGIN_OFFSET_M + position_m。r_jointはorigin
xyz=(0,0,0)、axis=(1,0,0)なのでlift_link原点からのオフセットはposition_mそのもの)。
配線が未設定(device_id=0)のスイッチは灰色、配線済みで未検出は緑、検出中は
赤で表示する。
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

# soki_sim.urdf.xacroのlift_size_z/2と一致させること(z_jointのorigin xyz)。
Z_JOINT_ORIGIN_OFFSET_M = 0.04

AXES = ('z', 'r')
DIRECTIONS = ('lower', 'upper')
FRAME_FOR_AXIS = {'z': 'turret_link', 'r': 'lift_link'}
# soki_sim.urdf.xacroのz_lower/z_upper/r_lower/r_upperの見積り値をデフォルトに使う
# (下限側はhoming_nodeのz_ref_value_m/r_ref_value_mが実測できていればそちらへ
# 上書きすること)。
DEFAULT_POSITION_M = {
    ('z', 'lower'): 0.118,
    ('z', 'upper'): 0.508,
    ('r', 'lower'): -0.50975,
    ('r', 'upper'): 0.50975,
}
# 未配線(灰)/未検出(緑)/検出中(赤)。
COLOR_UNWIRED = (0.5, 0.5, 0.5, 0.4)
COLOR_INACTIVE = (0.1, 0.9, 0.1, 0.8)
COLOR_TRIGGERED = (0.9, 0.1, 0.1, 0.95)


class LimitSwitchMarkerNode(Node):

    def __init__(self):
        super().__init__('limit_switch_marker_node')

        self.declare_parameter('can_host_slots_per_node', 5)
        self.declare_parameter('switch_triggered_value', 1)
        self.declare_parameter('marker_topic', 'limit_switch_markers')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('marker_size_m', 0.03)

        for axis, direction in _axis_direction_pairs():
            prefix = f'{axis}_{direction}_limit_switch'
            self.declare_parameter(f'{prefix}_device_id', 0)
            self.declare_parameter(f'{prefix}_node_index', 0)
            self.declare_parameter(f'{prefix}_local_index', 0)
            self.declare_parameter(f'{prefix}_position_m', DEFAULT_POSITION_M[(axis, direction)])

        gp = self.get_parameter
        self.slots_per_node_ = int(gp('can_host_slots_per_node').value)
        self.triggered_value_ = int(gp('switch_triggered_value').value)
        self.marker_size_m_ = float(gp('marker_size_m').value)

        self.switches_ = {}
        self.can_host_data_ = {}
        for axis, direction in _axis_direction_pairs():
            prefix = f'{axis}_{direction}_limit_switch'
            device_id = int(gp(f'{prefix}_device_id').value)
            node_index = int(gp(f'{prefix}_node_index').value)
            local_index = int(gp(f'{prefix}_local_index').value)
            position_m = float(gp(f'{prefix}_position_m').value)
            self.switches_[(axis, direction)] = {
                'device_id': device_id,
                'slot': node_index * self.slots_per_node_ + local_index,
                'position_m': position_m,
            }
            if device_id != 0 and device_id not in self.can_host_data_:
                self.can_host_data_[device_id] = None
                self.create_subscription(
                    Int32MultiArray, f'serial_rx_{device_id}_unwrapped',
                    lambda msg, did=device_id: self._on_can_host(msg, did), 10)

        marker_topic = gp('marker_topic').value
        self.pub_ = self.create_publisher(MarkerArray, marker_topic, 10)

        publish_rate_hz = float(gp('publish_rate_hz').value)
        self.create_timer(1.0 / publish_rate_hz, self._on_tick)

        self.get_logger().info(
            f'limit_switch_marker_node started: publishing to {marker_topic} '
            f'({[f"{a}_{d}" for a, d in _axis_direction_pairs()]})')

    def _on_can_host(self, msg: Int32MultiArray, device_id: int):
        self.can_host_data_[device_id] = msg.data

    def _triggered(self, axis, direction):
        info = self.switches_[(axis, direction)]
        if info['device_id'] == 0:
            return None
        data = self.can_host_data_.get(info['device_id'])
        if data is None:
            return None
        return bool(data[info['slot']] == self.triggered_value_)

    def _on_tick(self):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()
        for marker_id, (axis, direction) in enumerate(_axis_direction_pairs()):
            info = self.switches_[(axis, direction)]
            triggered = self._triggered(axis, direction)
            if info['device_id'] == 0:
                color = COLOR_UNWIRED
            elif triggered:
                color = COLOR_TRIGGERED
            else:
                color = COLOR_INACTIVE

            frame_id = FRAME_FOR_AXIS[axis]
            if axis == 'z':
                x, y, z = 0.0, 0.0, Z_JOINT_ORIGIN_OFFSET_M + info['position_m']
            else:
                x, y, z = info['position_m'], 0.0, 0.0

            cube = Marker()
            cube.header.frame_id = frame_id
            cube.header.stamp = now
            cube.ns = 'limit_switches'
            cube.id = marker_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = z
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = cube.scale.z = self.marker_size_m_
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = color
            cube.lifetime = Duration(seconds=0).to_msg()
            array.markers.append(cube)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = now
            label.ns = 'limit_switch_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + self.marker_size_m_
            label.pose.orientation.w = 1.0
            label.scale.z = self.marker_size_m_ * 1.5
            label.color.r, label.color.g, label.color.b, label.color.a = color
            label.text = f'{axis}_{direction}'
            label.lifetime = Duration(seconds=0).to_msg()
            array.markers.append(label)

        self.pub_.publish(array)


def _axis_direction_pairs():
    return [(axis, direction) for axis in AXES for direction in DIRECTIONS]


def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

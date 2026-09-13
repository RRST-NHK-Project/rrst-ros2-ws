#!/usr/bin/env python3
"""ABU Robocon 2027ルールブックの寸法からGazeboワールドSDFを生成する。

ゾーンの相対配置は、ユーザーがルールブックFigure 1-2を元に作成した3D参照ページ
(Three.js、2026-09-04にURLで共有)のソースコード内の座標(position.set)を
そのまま移植したもの。Three.js座標系(X=東西、Y=高さ、Z=南北)からROS座標系
(X=南北・北が+、Y=西東・東が+、Z=高さ)へ変換している: ROS_x=three_z,
ROS_y=three_x, ROS_z=three_y。地形の絶対寸法(ランプ勾配・階段段差数等)は
ルールブック本文の数値を優先し、参照ページと段数が異なる箇所(階段)は
本文の数値(Ground→L1は4段600mm、L1→L2は2段300mm)を採用している。

移動シミュレーション(ランプ勾配・階段段差の走破性検証)が主目的のため、
ブロック・ムスティカ本体等の動的オブジェクトは含まない(地形+ゾーンマーカーのみ)。

使い方:
    python3 src/nhk27_field/scripts/generate_field_world.py \
        > src/nhk27_field/worlds/nhk27_field.sdf
"""

import math

FIELD_SIZE = 11.0  # ルールブック2章 Game Field: 11000mm x 11000mm

# L1: 6000x6000mm、Ground上600mm、フィールド中央に配置(参照ページ準拠)
L1_SIZE = 6.0
L1_HEIGHT = 0.6
L1_HALF = L1_SIZE / 2  # 3.0

# L2: 3000x3000mm、L1上300mm、L1の中央に乗る(参照ページ準拠)
L2_SIZE = 3.0
L2_HALF = L2_SIZE / 2  # 1.5
L2_TOP_Z = 0.9

CENTRAL_PILLAR_DIA = 0.27
CENTRAL_PILLAR_HEIGHT = 0.8
MUSTIKA_PILLAR_DIA = 0.27
MUSTIKA_PILLAR_HEIGHT = 0.5
MUSTIKA_PAD_X = L1_HALF + 0.5  # 3.5、L1北端中央に接する共有パッド

# ランプ: L1の側面(西=赤/東=青)の外側、南北方向に3500mm登る
RAMP_RUN = 3.5
RAMP_RISE = L1_HEIGHT
RAMP_WIDTH = 1.0
RAMP_Y = L1_HALF + 0.5  # 3.5、L1西/東端の外側
RAMP_X_CENTER = 1.25  # 参照ページ準拠(南北方向の中心位置)

# トランスファーエリア(L1境界の張り出しピア、1000角・高さ600でL1と同高)
TRANSFER_SIZE = 1.0
TRANSFER_X = -1.0  # 参照ページ準拠

# 階段 Ground->L1: ルールブック本文通り4段、300(奥行)x1000(幅)x150(高さ)/段
STAIR_STEP_DEPTH = 0.3
STAIR_STEP_WIDTH = 1.0
STAIR_STEP_HEIGHT = 0.15
STAIRS_G2L1_COUNT = 4  # 4 x 150mm = 600mm

# 階段 L1->L2: ルールブック本文通り2段、L2の西/東端に接続(参照ページ準拠、
# 南北方向ではなく東西方向に登る)
STAIRS_L1L2_COUNT = 2  # 2 x 150mm = 300mm
STAIRS_L1L2_Y = L2_HALF + 0.15  # L2端のすぐ外側

# 色(ルールブック14章RGBを0-1に正規化)
COLOR_GROUND_RED = (240 / 255, 210 / 255, 210 / 255, 1.0)
COLOR_GROUND_BLUE = (170 / 255, 210 / 255, 230 / 255, 1.0)
COLOR_SHARED = (245 / 255, 240 / 255, 200 / 255, 1.0)
COLOR_START_RED = (223 / 255, 34 / 255, 34 / 255, 1.0)
COLOR_START_BLUE = (50 / 255, 0 / 255, 255 / 255, 1.0)
COLOR_PILLAR = (100 / 255, 62 / 255, 0 / 255, 1.0)
COLOR_L1_RED = (235 / 255, 180 / 255, 160 / 255, 1.0)
COLOR_L1_BLUE = (150 / 255, 215 / 255, 220 / 255, 1.0)
COLOR_L2 = (190 / 255, 190 / 255, 190 / 255, 1.0)
COLOR_BUILDING_SPOT = (40 / 255, 100 / 255, 50 / 255, 1.0)
COLOR_TRANSFER_RED = (245 / 255, 170 / 255, 60 / 255, 1.0)
COLOR_TRANSFER_BLUE = (60 / 255, 170 / 255, 245 / 255, 1.0)
COLOR_RAMP_RED = (200 / 255, 150 / 255, 140 / 255, 1.0)
COLOR_RAMP_BLUE = (130 / 255, 180 / 255, 200 / 255, 1.0)
COLOR_BARRIER = (190 / 255, 190 / 255, 185 / 255, 1.0)
COLOR_BOUNDARY = (100 / 255, 62 / 255, 0 / 255, 1.0)


def _rgba(c):
    return f"{c[0]} {c[1]} {c[2]} {c[3]}"


def static_box(name, sx, sy, sz, x, y, z, roll=0.0, pitch=0.0, yaw=0.0,
               color=COLOR_GROUND_RED, collision=True):
    coll = ""
    if collision:
        coll = f"""
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>"""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="link">{coll}
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{_rgba(color)}</ambient>
            <diffuse>{_rgba(color)}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def static_cylinder(name, radius, length, x, y, z, color=COLOR_PILLAR):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          <material>
            <ambient>{_rgba(color)}</ambient>
            <diffuse>{_rgba(color)}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def flat_marker(name, sx, sy, x, y, z, color, layer=0):
    """衝突なしの薄い色マーカー(ゾーン表示専用、走行の障害にならない)。

    同じ高さ(base z)に複数のマーカーを重ねて置くとZ-fighting(同一平面での
    描画競合、ギザギザ・チラつきの原因)になるため、layerで高さを8mmずつ
    ずらす。0=ベース色(ground_red/level1_red等)、1=その上のゾーン塗り
    (start_zone/storage/shared area等)、2=さらにその上のビルディングスポット。"""
    return static_box(name, sx, sy, 0.01, x, y, z + 0.006 + layer * 0.008, color=color,
                       collision=False)


def ramp(name, y_center, x_center, color):
    """南北方向(X方向)に登るランプ。x_center-RAMP_RUN/2(北寄り、ムスティカ側)がZ=0の
    低い端、x_center+RAMP_RUN/2(南寄り、トランスファーエリア側)がZ=RAMP_RISEの
    高い端になるよう傾斜させる(参照ページのrampBase/rampTop座標で確認済み:
    ランプの高い側はトランスファーエリアに接する南寄りのX≈-0.5、低い側は
    ムスティカ寄りの北側X≈3.0)。"""
    length = math.hypot(RAMP_RUN, RAMP_RISE)
    pitch = math.atan2(RAMP_RISE, RAMP_RUN)
    return static_box(name, length, RAMP_WIDTH, 0.05, x_center, y_center, RAMP_RISE / 2,
                       pitch=pitch, color=color)


def stairs_along_x(name_prefix, y_center, x_top, count, z_base, color):
    """X方向(南北)に登る階段。x_topが最上段(z_base+count*step_heightに接続する側)。"""
    parts = []
    for i in range(count):
        step_top_z = z_base + STAIR_STEP_HEIGHT * (i + 1)
        step_x = x_top - STAIR_STEP_DEPTH * (count - 1 - i)
        parts.append(static_box(
            f"{name_prefix}_step{i+1}", STAIR_STEP_DEPTH, STAIR_STEP_WIDTH, step_top_z,
            step_x, y_center, step_top_z / 2, color=color))
    return "".join(parts)


def stairs_along_y(name_prefix, x_center, y_edge, count, z_base, color, climbing_toward_positive_y):
    """Y方向(東西)に登る階段。L1からL2の側面へ登る用。"""
    parts = []
    for i in range(count):
        step_top_z = z_base + STAIR_STEP_HEIGHT * (i + 1)
        if climbing_toward_positive_y:
            step_y = y_edge - STAIR_STEP_DEPTH * (count - 1 - i)
        else:
            step_y = y_edge + STAIR_STEP_DEPTH * (count - 1 - i)
        parts.append(static_box(
            f"{name_prefix}_step{i+1}", STAIR_STEP_WIDTH, STAIR_STEP_DEPTH, step_top_z,
            x_center, step_y, step_top_z / 2, color=color))
    return "".join(parts)


def build_world():
    parts = []
    b = FIELD_SIZE / 2  # 5.5

    # 地面(11m x 11m、西=赤/東=青で二分)
    parts.append(static_box("ground_base", FIELD_SIZE, FIELD_SIZE, 0.05,
                             0, 0, -0.025, color=(0.23, 0.18, 0.13, 1.0)))
    parts.append(flat_marker("ground_red", FIELD_SIZE, b, 0, -b / 2, 0, COLOR_GROUND_RED))
    parts.append(flat_marker("ground_blue", FIELD_SIZE, b, 0, b / 2, 0, COLOR_GROUND_BLUE))

    # フィールド境界
    parts.append(static_box("field_boundary_north", 0.15, FIELD_SIZE + 0.3, 0.16,
                             b + 0.05, 0, 0.08, color=COLOR_BOUNDARY))
    parts.append(static_box("field_boundary_south", 0.15, FIELD_SIZE + 0.3, 0.16,
                             -b - 0.05, 0, 0.08, color=COLOR_BOUNDARY))
    parts.append(static_box("field_boundary_east", FIELD_SIZE + 0.3, 0.15, 0.16,
                             0, b + 0.05, 0.08, color=COLOR_BOUNDARY))
    parts.append(static_box("field_boundary_west", FIELD_SIZE + 0.3, 0.15, 0.16,
                             0, -b - 0.05, 0.08, color=COLOR_BOUNDARY))

    # L1プラットフォーム(フィールド中央、西=赤/東=青)
    parts.append(static_box("level1_structure", L1_SIZE, L1_SIZE, L1_HEIGHT,
                             0, 0, L1_HEIGHT / 2, color=COLOR_L2))
    parts.append(flat_marker("level1_red", L1_SIZE, L1_HALF, 0, -L1_HALF / 2, L1_HEIGHT,
                              COLOR_L1_RED))
    parts.append(flat_marker("level1_blue", L1_SIZE, L1_HALF, 0, L1_HALF / 2, L1_HEIGHT,
                              COLOR_L1_BLUE))

    # L2プラットフォーム(L1の中央に乗る、共有)
    parts.append(static_box("level2_structure", L2_SIZE, L2_SIZE, L2_TOP_Z - L1_HEIGHT,
                             0, 0, (L1_HEIGHT + L2_TOP_Z) / 2, color=COLOR_L2))

    # 中央ピラー(L2中央、ムスティカ奉納先)
    parts.append(static_cylinder("central_pillar", CENTRAL_PILLAR_DIA / 2,
                                  CENTRAL_PILLAR_HEIGHT, 0, 0,
                                  L2_TOP_Z + CENTRAL_PILLAR_HEIGHT / 2))

    # ムスティカ共有パッド+ピラー(L1北端中央、センターライン上=どちらのチームも到達可)
    parts.append(flat_marker("mustika_pad", 1.0, 1.0, MUSTIKA_PAD_X, 0.0, 0.0, COLOR_SHARED,
                              layer=1))
    parts.append(static_cylinder("mustika_pillar", MUSTIKA_PILLAR_DIA / 2,
                                  MUSTIKA_PILLAR_HEIGHT, MUSTIKA_PAD_X, 0,
                                  MUSTIKA_PILLAR_HEIGHT / 2))

    # ランプ+トランスファーエリア+階段(西=赤/東=青、ミラー配置)
    for sign, side, l1_color, ramp_color, transfer_color in (
        (-1, "red", COLOR_L1_RED, COLOR_RAMP_RED, COLOR_TRANSFER_RED),
        (1, "blue", COLOR_L1_BLUE, COLOR_RAMP_BLUE, COLOR_TRANSFER_BLUE),
    ):
        y = sign * RAMP_Y
        parts.append(ramp(f"ramp_{side}", y, RAMP_X_CENTER, ramp_color))
        parts.append(static_box(f"transfer_area_{side}", TRANSFER_SIZE, TRANSFER_SIZE,
                                 L1_HEIGHT, TRANSFER_X, y, L1_HEIGHT / 2,
                                 color=transfer_color))
        parts.append(stairs_along_x(f"stairs_g2l1_{side}", y,
                                     TRANSFER_X - TRANSFER_SIZE / 2, STAIRS_G2L1_COUNT, 0.0,
                                     l1_color))

    # 階段 L1->L2(L2の西/東端、赤=西から登る/青=東から登る)
    parts.append(stairs_along_y("stairs_l1l2_red", 0.0, -L2_HALF, STAIRS_L1L2_COUNT,
                                 L1_HEIGHT, COLOR_L1_RED, climbing_toward_positive_y=True))
    parts.append(stairs_along_y("stairs_l1l2_blue", 0.0, L2_HALF, STAIRS_L1L2_COUNT,
                                 L1_HEIGHT, COLOR_L1_BLUE, climbing_toward_positive_y=False))

    # L1周縁バリア(トランスファーエリア接続部分は開口)
    barrier_h = 0.15
    bz = L1_HEIGHT + barrier_h / 2
    parts.append(static_box("l1_barrier_north", 0.06, L1_SIZE + 0.12, barrier_h,
                             L1_HALF, 0.0, bz, color=COLOR_BARRIER))
    parts.append(static_box("l1_barrier_south", 0.06, L1_SIZE + 0.12, barrier_h,
                             -L1_HALF, 0.0, bz, color=COLOR_BARRIER))
    # 東西の側面バリア: トランスファー接続部(x:[-1.5,-0.5]、幅TRANSFER_SIZE)だけ
    # 開口させ南北2分割。南側1.5m(x:[-3.0,-1.5])・北側3.5m(x:[-0.5,3.0])。
    gap_min = TRANSFER_X - TRANSFER_SIZE / 2  # -1.5
    gap_max = TRANSFER_X + TRANSFER_SIZE / 2  # -0.5
    south_len = gap_min - (-L1_HALF)  # 1.5
    north_len = L1_HALF - gap_max  # 3.5
    for sign, side in ((-1, "west"), (1, "east")):
        y = sign * L1_HALF
        parts.append(static_box(f"l1_barrier_{side}_south", south_len, 0.06, barrier_h,
                                 -L1_HALF + south_len / 2, y, bz, color=COLOR_BARRIER))
        parts.append(static_box(f"l1_barrier_{side}_north", north_len, 0.06, barrier_h,
                                 gap_max + north_len / 2, y, bz, color=COLOR_BARRIER))

    # L2 Side Wall(西/東の階段接続部を除く周囲)
    l2z = L2_TOP_Z + barrier_h / 2
    parts.append(static_box("l2_wall_north", 0.06, L2_SIZE + 0.12, barrier_h,
                             L2_HALF, 0.0, l2z, color=COLOR_BARRIER))
    parts.append(static_box("l2_wall_south", 0.06, L2_SIZE + 0.12, barrier_h,
                             -L2_HALF, 0.0, l2z, color=COLOR_BARRIER))

    # センターディバイダ(Y=0沿い。共有パッド・共有グリッド・L2は開口)
    # X方向(南北)に長く、Y方向(東西)に薄い壁にする(sx=長さ,sy=厚み0.06)。
    # 旧版はsx/syが逆(厚み0.06が長さ側)になっており90度回転して見えるバグが
    # あったため修正済み(2026-09-04)。
    divider_h = 0.3
    dz = divider_h / 2
    # 地上: ムスティカパッド(x:[3.0,4.0])の北側(storage側)
    parts.append(static_box("center_divider_ground_north", 1.5, 0.06, divider_h,
                             4.75, 0.0, dz, color=COLOR_BOUNDARY))
    # 地上: L1南端からGround Shared Areaまで
    parts.append(static_box("center_divider_ground_south1", 0.5, 0.06, divider_h,
                             -3.3, 0.0, dz, color=COLOR_BOUNDARY))
    # 地上: Ground Shared Areaからフィールド境界まで
    parts.append(static_box("center_divider_ground_south2", 0.4, 0.06, divider_h,
                             -5.3, 0.0, dz, color=COLOR_BOUNDARY))
    # L1上: 共有パッド(x=±2.5)とL2(x=±1.5)の間
    l1_divider_z = L1_HEIGHT + 0.1
    parts.append(static_box("center_divider_l1_north", 0.5, 0.06, 0.2,
                             1.75, 0.0, l1_divider_z, color=COLOR_BOUNDARY))
    parts.append(static_box("center_divider_l1_south", 0.5, 0.06, 0.2,
                             -1.75, 0.0, l1_divider_z, color=COLOR_BOUNDARY))

    # Start Zone(南端角、赤/青それぞれTR用・BR用の2枠)。ground_red/blue(layer0)の
    # 上に乗るのでlayer=1。
    for sign, side, color in ((-1, "red", COLOR_START_RED), (1, "blue", COLOR_START_BLUE)):
        parts.append(flat_marker(f"start_zone_{side}_a", 0.7, 0.7, -b + 0.35, sign * 5.15, 0.0,
                                  color, layer=1))
        parts.append(flat_marker(f"start_zone_{side}_b", 0.7, 0.7, -b + 0.35, sign * 4.35, 0.0,
                                  color, layer=1))

    # Storage Area(北端角、赤/青)。ground_red/blueの上なのでlayer=1。
    for sign, side, color in ((-1, "red", COLOR_GROUND_RED), (1, "blue", COLOR_GROUND_BLUE)):
        parts.append(flat_marker(f"storage_{side}", 1.0, 2.0, 5.0, sign * 4.5, 0.0, color,
                                  layer=1))

    # Ground Shared Area(1200x1200mmの5x5グリッド、中央マス空き。センターライン跨ぎ)。
    # ground_red/blueの上なのでlayer=1。
    parts.append(flat_marker("ground_shared_area", 1.2, 1.2, -4.3, 0.0, 0.0, COLOR_SHARED,
                              layer=1))

    # L1 Retry Zone(赤/青)。level1_red/blue(layer0)の上なのでlayer=1。
    for sign, side, color in ((-1, "red", COLOR_START_RED), (1, "blue", COLOR_START_BLUE)):
        parts.append(flat_marker(f"l1_retry_zone_{side}", 0.7, 0.7, 2.65, sign * 1.95,
                                  L1_HEIGHT, color, layer=1))

    # Building Spot: L1専用(北の角・南の角、level1_red/blueの上でlayer=1)、
    # L1共有(センターパッド上2箇所。l1_shared_pad(layer1)のさらに上なのでlayer=2)、
    # L2共有(4隅、L2構造の上でlayer=1)
    for sign, side in ((-1, "red"), (1, "blue")):
        parts.append(flat_marker(f"building_spot_l1_{side}_north", 0.5, 0.5, 2.65, sign * 2.65,
                                  L1_HEIGHT, COLOR_BUILDING_SPOT, layer=1))
        parts.append(flat_marker(f"building_spot_l1_{side}_south", 0.5, 0.5, -2.65, sign * 2.6,
                                  L1_HEIGHT, COLOR_BUILDING_SPOT, layer=1))
    parts.append(flat_marker("l1_shared_pad_north", 1.0, 1.0, 2.5, 0.0, L1_HEIGHT, COLOR_SHARED,
                              layer=1))
    parts.append(flat_marker("l1_shared_pad_south", 1.0, 1.0, -2.5, 0.0, L1_HEIGHT, COLOR_SHARED,
                              layer=1))
    parts.append(flat_marker("building_spot_l1_shared_north", 0.5, 0.5, 2.5, 0.0, L1_HEIGHT,
                              COLOR_BUILDING_SPOT, layer=2))
    parts.append(flat_marker("building_spot_l1_shared_south", 0.5, 0.5, -2.5, 0.0, L1_HEIGHT,
                              COLOR_BUILDING_SPOT, layer=2))
    for xs in (-0.95, 0.95):
        for ys in (-0.95, 0.95):
            parts.append(flat_marker(f"building_spot_l2_{'n' if xs > 0 else 's'}"
                                      f"{'e' if ys > 0 else 'w'}", 0.5, 0.5, xs, ys, L2_TOP_Z,
                                      COLOR_BUILDING_SPOT, layer=1))

    world_body = "".join(parts)

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="nhk27_field">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
{world_body}
  </world>
</sdf>
"""


if __name__ == "__main__":
    print(build_world())

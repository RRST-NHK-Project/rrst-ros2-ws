# NHK2026ロボコン
# 経路探索システム
# 2026/03/11

'''
 MAIN
 ^GUI --> plot3D
 ^^A*   --> Move ((→ ROS2通信へ))
 ^^^[Node]

 ^MapData
'''

# ノードの定義
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
# NHK2026ロボコン
# 経路探索システム
# 2026/03/11

'''
 MAIN
 ^GUI --> plot3D
 ^^[A*]
 ^^^Node
 
 ^MapData
'''
# 分割用
from path_system_node import Node

# A* 実装
def astar(maze, height_map, start, end):
    start_node = Node(None, start)
    end_node = Node(None, end)
    open_list, closed_list = [start_node], []

    while open_list:
        current_node = min(open_list, key=lambda node: node.f)
        open_list.remove(current_node)
        closed_list.append(current_node)

        if current_node == end_node:
            total_cost = current_node.g
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            path = path[::-1]
            return path, total_cost

        for move in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            new_pos = (current_node.position[0] + move[0], current_node.position[1] + move[1])
            if not (0 <= new_pos[0] < len(maze) and 0 <= new_pos[1] < len(maze[0])):
                continue
            if maze[new_pos[0]][new_pos[1]] in [1, 2, 3]:
                continue

            child = Node(current_node, new_pos)
            if child in closed_list:
                continue

            height_pos = height_map[current_node.position[0]][current_node.position[1]]
            height_new = height_map[new_pos[0]][new_pos[1]]
            move_cost = calc_step_height(height_pos, height_new, height=0.5)

            if move_cost == "wall":
                continue
            child.g = current_node.g + move_cost
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            if any(open_node for open_node in open_list if child == open_node and child.g > open_node.g):
                continue
            open_list.append(child)

    return None, None

# 高さを考慮
def calc_step_height(height_pos, height_new, height):
    step = height_new - height_pos
    if step == 0:
        return 1
    if abs(step) > 3:
        return "wall"
    return 1 + abs(step) * height

# NHK2026ロボコン
# 経路探索システム
# 2026/03/11

'''
 MAIN
 ^GUI --> [plot3D]
 ^^A*   --> Move ((→ ROS2通信へ))
 ^^^Node

 ^MapData
'''


import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# 迷路の3D表示
def plot_3d_maze_path(height_map, maze, path, start_pos, goal_pos):
    rows, cols = height_map.shape
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    dx = dy = 0.9
    for r in range(rows):
        for c in range(cols):
            z = 0
            h = height_map[r, c]
            color = 'yellowgreen'
            if h == 1.0:
                color = 'darkgreen'
            if h == 2.0:
                color = 'green'
            if maze[r][c] == 1:
                color = 'hotpink'
                h = max(h, 0.5)
            if maze[r][c] == 3:
                color = 'black'
                h = max(h, 0.5)
            if path and (r, c) in path:
                color = 'red'

            ax.bar3d(c, r, z, dx, dy, h, color=color, alpha=0.7)
            ax.set_box_aspect((1, 1, 0.1))

    if path:
        xs = [c + 0.45 for r, c in path]
        ys = [r + 0.45 for r, c in path]
        zs = [height_map[r, c] + 0.5 for r, c in path]
        ax.plot(xs, ys, zs, color='blue', linewidth=2)

    # StartとGoalの位置にテキストを追加
    if start_pos:
        start_x, start_y = start_pos
        ax.text(start_y + 0.5, start_x + 0.5, height_map[start_x, start_y] + 0.5, 'Start', color='green', fontsize=10)

    if goal_pos:
        goal_x, goal_y = goal_pos
        ax.text(goal_y + 0.5, goal_x + 0.5, height_map[goal_x, goal_y] + 0.5, 'Goal', color='red', fontsize=10)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')
    ax.set_title("NHK2026 MMF PathfindingMAP")
    plt.show(block=False)
    plt.pause(5)


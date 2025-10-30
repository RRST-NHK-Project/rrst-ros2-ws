# NHKロボコン2026 
# R2
# 経路生成　A*
# 3次元平面コスト追加
# 3D平面可視化

# matplotのインポート
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_maze_path(height_map, path):
    rows, cols = height_map.shape
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 各ブロックのサイズ
    dx = dy = 0.9
    

    for r in range(rows):
        for c in range(cols):
            z = 0
            h = height_map[r, c]
            color = 'yellowgreen'  # 薄緑
            if h == 1.0:
                color = 'darkgreen'  # 緑
            if h == 2.0:
                color = 'green'  # 緑
            if path and (r, c) in path:
                color = 'yellow'
             
            ax.bar3d(c, r, z, dx, dy, h, color=color, alpha=0.7)
            ax.set_box_aspect((1,1, 0.2))
       # 経路を線で描く
  
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Height')
    ax.set_title("3D Maze with Blocks")
    plt.show()

class Node: 
    def __init__(self, parent = None, position = None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

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
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        for move in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            new_pos = (current_node.position[0] + move[0], current_node.position[1] + move[1]) # オープンリストの中でF値が一番小さいノードを選ぶ
             # 迷路内の移動に限る
            if not (0 <= new_pos[0] < len(maze) and 0 <= new_pos[1] < len(maze[0])):
                continue
             # 移動できる位置に限る（障害物は移動できない）
            if maze[new_pos[0]][new_pos[1]] != 0:
                continue

            child = Node(current_node, new_pos)
            if child in closed_list:
                continue

            # 高さの追加 (height = 段差の高さ)
            height_pos = height_map[current_node.position[0]][current_node.position[1]]
            height_new = height_map[new_pos[0]][new_pos[1]]            
            move_cost = calc_step_height(height_pos, height_new, height=0.5)  
            
            if move_cost == "wall":
                continue # 壁、通行不可       
            print(f"{current_node.position} -> {new_pos}, cost={move_cost}")

            # child.h＝マンハッタン距離
            child.g = current_node.g + move_cost # gのみ高さを考慮
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h


            if any(open_node for open_node in open_list if child == open_node and child.g > open_node.g):
                continue
            open_list.append(child)

    return None

# 高さを考慮
def calc_step_height(height_pos, height_new, height):
            step = height_new - height_pos
            # 段差なし
            if step == 0:
                return  1
            # 段差あり
            if step != 0:    
                if abs(step) > 3:
                   return "wall" # 今回は3段以上を壁とする  
            return  1 + abs(step) * height # 高さ分コストを加える

# 経路を＊で表示
def print_maze_path(maze, path):
    maze_copy = [row[:] for row in maze]
    for (r, c) in path:
        maze_copy[r][c] = '*'
    for row in maze_copy:
        print(' '.join(str(x) for x in row))
    print()

def print_height_map(maze, path):
    maze_copy = [row[:] for row in maze]
    for (r, c) in path:
        maze_copy[r][c] = '*'
    for row in maze_copy:
        print(' '.join(str(x) for x in row))
    print()

    
#　番号の振り分け
def create_numbering(start_row, start_col, end_row, end_col):
    number_to_pos = {}
    pos_to_number = {}
    num = 1
    for r in range(start_row, end_row + 1):
        for c in range(start_col, end_col + 1):
            number_to_pos[num] = (r, c)
            pos_to_number[(r, c)] = num
            num += 1
    return number_to_pos, pos_to_number


 # メイン関数、変更可
def main():

    # 0行0列は不使用
    # 障害物を1とする
    maze = [
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 0, 0, 1],
        [1, 0, 1, 0, 1],
        [1, 0, 0, 0, 1] 
    ]

    # 各マスの高さ
    height_map = np.array([
        [0, 0, 0, 0, 0],
        [0, 2, 1, 2, 0],
        [0, 1, 2, 3, 0],
        [0, 2, 3, 2, 0],
        [0, 1, 2, 1, 0]
    ])

    start = (1, 1)

    # (1,1) 〜 (3,4) に番号を振る
    number_to_pos, pos_to_number = create_numbering(1, 1, 4, 3)

    print("番号と座標の対応表:")
    for num in sorted(number_to_pos.keys()):
        print(f"{num}: {number_to_pos[num]}")
    print()

    current = start
    print(f"スタート位置: {current} (番号 {pos_to_number.get(current, 'なし')})")

    while True:
        try:
            inp = input("目的地の番号を入力してください(終了はq): ")
            if inp.lower() == 'q':
                print("終了します。")
                break

            goal_num = int(inp)
            if goal_num not in number_to_pos:
                print("範囲外の番号です。もう一度入力してください。")
                continue

            goal = number_to_pos[goal_num]
            print(f"目的地: {goal} (番号 {goal_num})")

            path = astar(maze, height_map, current, goal)
            if path:
                # 経路の座標リスト path から対応する番号リストを作成
                path_numbers = [pos_to_number.get(pos, '-') for pos in path]
                print("経路:", path)
                print("経路の番号:", path_numbers)
                print_maze_path(maze, path)
                print("matplotを閉じてください。次の実行を行います。")
                plot_3d_maze_path(height_map, path)
                plt.pause(3)
                plt.close()
                current = goal
            else:
                print("経路が見つかりませんでした。")

        except ValueError:
            print("有効な番号を入力してください。")


if __name__ == "__main__":
    main()

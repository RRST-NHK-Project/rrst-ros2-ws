class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    start_node = Node(None, start)
    end_node = Node(None, end)
    open_list, closed_list = [start_node], []

    while open_list:
        current_node = min(open_list, key=lambda node: node.f)       # オープンリストの中でF値が一番小さいノードを選ぶ
        open_list.remove(current_node)
        closed_list.append(current_node)
　　　　　
　　　　　 # 目的地に到達していれば経路(Path)を表示して終了
        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        for move in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            new_pos = (current_node.position[0] + move[0], current_node.position[1] + move[1])
　　　　　　　　 # 迷路内の移動に限る
            if not (0 <= new_pos[0] < len(maze) and 0 <= new_pos[1] < len(maze[0])):
                continue
             # 移動できる位置に限る（障害物は移動できない）
            if maze[new_pos[0]][new_pos[1]] != 0:
                continue

            child = Node(current_node, new_pos)
            if child in closed_list:
                continue
　　　　　　　　
            # child.h＝マンハッタン距離
            child.g = current_node.g + 1
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
            child.f = child.g + child.h

            if any(open_node for open_node in open_list if child == open_node and child.g > open_node.g):
                continue
            open_list.append(child)

    return None

# 経路の表示(メイン関数からの地図コピー→経路＊で表示)
def print_maze_path(maze, path):
    maze_copy = [row[:] for row in maze]
    for (r, c) in path: 
        maze_copy[r][c] = '*'
    for row in maze_copy:
        print(' '.join(str(value) for value in row))
    print()

# 番号の設定
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


def main():
    maze = [
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 0, 0, 1],
        [1, 0, 1, 0, 1],
        [1, 0, 0, 0, 1]
    ]

    start = (1, 2)

    # (1,1) 〜 (3,4) に番号を振る-> MF1〜12
    number_to_pos, pos_to_number = create_numbering(1, 1, 4, 3)

    print("番号と座標の対応表:")
    for num in sorted(number_to_pos.keys()):
        print(f"{num}: {number_to_pos[num]}")
    print()

    current = start
    print(f"スタート位置: {current} (番号 {pos_to_number.get(current, 'なし')})")

    while True:
        try:
            inp = input("目的地の番号を入力してください（終了はq）: ")
            if inp.lower() == 'q':
                print("終了します。")
                break

            goal_num = int(inp)
            if goal_num not in number_to_pos:
                print("範囲外の番号です。もう一度入力してください。")
                continue

            goal = number_to_pos[goal_num]
            print(f"目的地: {goal} (番号 {goal_num})")

            path = astar(maze, current, goal)
            if path:
                # 経路の座標リスト path から対応する番号リストを作成
                path_numbers = [pos_to_number.get(pos, '-') for pos in path]
                print("経路:", path)
                print("経路の番号:", path_numbers)
                print_maze_path(maze, path)
                current = goal
            else:
                print("経路が見つかりませんでした。")

        except ValueError:
            print("有効な番号を入力してください。")


if __name__ == "__main__":
    main()
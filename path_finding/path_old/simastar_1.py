# 試作版 Astar アルゴリズム (R2戦術を組み込み)
# NHKロボコン2026
# R2
# KFSの配置シュミュレーション実装

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import random
import collections
import time
import heapq # A* のパフォーマンス最適化

# --- 定数 ---
ROWS = 4
COLS = 3
# KFSの種類を細分化
EMPTY = 0      # 0: 空（回収後）
R2_KFS = 1     # 1: R2 KFS（回収ターゲット）
OBSTACLE_R1 = 2  # 2: 障害物 (R1 KFS)
OBSTACLE_FAKE = 3 # 3: 障害物 (Fake KFS)
UNKNOWN = -1   # -1: ロボットがまだスキャンしていない未知のエリア

# 高さ (メートル単位)
HEIGHT_200 = 0.2
HEIGHT_400 = 0.4
HEIGHT_600 = 0.6
# 200H (0.2m) の段差まで許可
ALLOWED_STEP_HEIGHT = HEIGHT_200 

# (dr, dc)
DIRECTIONS = [
    (1, 0),  # 0: 下 (行+)
    (0, 1),  # 1: 右 (列+)
    (-1, 0), # 2: 上 (行-)
    (0, -1)  # 3: 左 (列-)
]
TURN_COST = 1
MOVE_COST = 1

BLOCK_COORDS = {
    1: (0, 0), 2: (0, 1), 3: (0, 2),
    4: (1, 0), 5: (1, 1), 6: (1, 2),
    7: (2, 0), 8: (2, 1), 9: (2, 2),
    10: (3, 0), 11: (3, 1), 12: (3, 2),
}
COORDS_TO_BLOCK = {v: k for k, v in BLOCK_COORDS.items()}
ENTRANCE_BLOCKS_NUMS = {1, 2, 3}
ENTRANCE_BLOCKS_COORDS = {BLOCK_COORDS[b] for b in ENTRANCE_BLOCKS_NUMS}
EXIT_BLOCKS_NUMS = {10, 11, 12}
EXIT_BLOCKS_COORDS = {BLOCK_COORDS[b] for b in EXIT_BLOCKS_NUMS}
BOUNDARY_BLOCKS_NUMS = {1, 2, 3, 4, 6, 7, 9, 10, 11, 12} 

# --- ユーティリティ: 番号付け ---
def create_numbering():
    pos_to_number = {}
    for num, pos in BLOCK_COORDS.items():
        pos_to_number[pos] = num
    return pos_to_number

# --- ユーティリティ: 隣接ノード取得 ---
def get_adjacent_nodes(row, col):
    nodes = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for dr, dc in directions:
        next_row, next_col = row + dr, col + dc
        if (0 <= next_row < ROWS) and (0 <= next_col < COLS):
            nodes.append((next_row, next_col))
    return nodes

# --- ユーティリティ: 座標からブロック番号取得 ---
def get_block_num(row, col):
    return COORDS_TO_BLOCK.get((row, col), "不明")

# --- 3Dプロット関数 ---
def plot_3d_maze_path(height_map, robot_grid_map_for_plot, path, title="3D Maze"):
    rows, cols = height_map.shape
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection='3d')
    dx = dy = 0.9
    pos_to_number = create_numbering()

    for r in range(rows):
        for c in range(cols):
            z = 0
            h = height_map[r, c]
            state = robot_grid_map_for_plot[r][c]
            
            if h == HEIGHT_200: color = 'yellowgreen'
            elif h == HEIGHT_400: color = 'darkgreen'
            elif h == HEIGHT_600: color = 'green'
            else: color = 'lightgrey'
            
            if state == R2_KFS: color = 'blue'
            elif state == OBSTACLE_R1: color = 'red'
            elif state == OBSTACLE_FAKE: color = 'purple'
            elif state == UNKNOWN: color = 'grey'
            
            if path and (r, c) in path:
                color = 'yellow'
             
            plot_h = h if h > 0 else 0.01
            ax.bar3d(c, r, z, dx, dy, plot_h, color=color, alpha=0.8, edgecolor='black')
            
            block_num = pos_to_number.get((r,c))
            if block_num:
                 ax.text(c + dx/2, r + dy/2, plot_h + 0.05, f"{block_num}", 
                         color='black', ha='center', va='bottom', fontsize=9)
    if path:
        path_x = [pos[1] + dx/2 for pos in path]
        path_y = [pos[0] + dy/2 for pos in path]
        path_z = [height_map[pos[0], pos[1]] + 0.1 for pos in path]
        ax.plot(path_x[0:1], path_y[0:1], path_z[0:1], 'o', markersize=12, color='cyan', label='Start of Path') 
        ax.plot(path_x[-1:], path_y[-1:], path_z[-1:], 'X', markersize=12, markeredgewidth=3, color='magenta', label='End of Path')
        ax.plot(path_x, path_y, path_z, '-', color='yellow', linewidth=4, label='Path')

    ax.set_xlabel('Column (X)')
    ax.set_ylabel('Row (Y)')
    ax.set_zlabel('Height')
    ax.set_xticks(np.arange(cols) + dx/2)
    ax.set_xticklabels(np.arange(cols))
    ax.set_yticks(np.arange(rows) + dy/2)
    ax.set_yticklabels(np.arange(rows))
    ax.set_title(title)
    ax.legend(loc='upper left', bbox_to_anchor=(0.0, 0.95))
    ax.view_init(elev=35, azim=-120)
    plt.show(block=False)

# --- A* アルゴリズム (heapq使用) ---
class Node: 
    def __init__(self, parent = None, position = None, orientation = None):
        self.parent = parent
        self.position = position
        self.orientation = orientation
        self.g = 0; self.h = 0; self.f = 0
    def __eq__(self, other):
        return self.position == other.position and self.orientation == other.orientation
    def __lt__(self, other):
        return self.f < other.f
    def __hash__(self):
        return hash((self.position, self.orientation))

def astar(robot_grid_map, height_map, start_state, end_pos):
    start_node = Node(None, (start_state[0], start_state[1]), start_state[2])
    end_node_pos = end_pos
    
    open_list = []
    heapq.heappush(open_list, start_node)
    closed_list = set() 
    open_list_lookup = {start_node: start_node.f} 
    
    start_height = height_map[start_node.position[0], start_node.position[1]]

    while open_list:
        current_node = heapq.heappop(open_list)
        if current_node not in open_list_lookup:
             continue
        open_list_lookup.pop(current_node)

        current_pos = current_node.position
        current_ori = current_node.orientation
        
        if current_pos == end_node_pos:
            path_states = []
            temp = current_node
            while temp:
                path_states.append((temp.position[0], temp.position[1], temp.orientation))
                temp = temp.parent
            return path_states[::-1] 

        closed_list.add(current_node)
        current_height = height_map[current_pos[0], current_pos[1]]
        
        possible_moves = []
        (dr, dc) = DIRECTIONS[current_ori]
        move_pos = (current_pos[0] + dr, current_pos[1] + dc)
        possible_moves.append(('move', move_pos, current_ori, MOVE_COST))
        turn_left_ori = (current_ori + 1) % 4
        possible_moves.append(('turn', current_pos, turn_left_ori, TURN_COST))
        turn_right_ori = (current_ori - 1 + 4) % 4
        possible_moves.append(('turn', current_pos, turn_right_ori, TURN_COST))

        for move_type, new_pos, new_ori, move_cost in possible_moves:
            
            if not (0 <= new_pos[0] < ROWS and 0 <= new_pos[1] < COLS):
                continue
            
            new_pos_state = robot_grid_map[new_pos[0]][new_pos[1]]
            
            if new_pos_state == OBSTACLE_FAKE or new_pos_state == UNKNOWN:
                continue
            
            if move_type == 'move':
                new_height = height_map[new_pos[0], new_pos[1]]
                height_diff = abs(new_height - current_height)
                epsilon = 0.01 
                if height_diff > (ALLOWED_STEP_HEIGHT + epsilon):
                    continue 
            
            child_node = Node(current_node, new_pos, new_ori)

            if child_node in closed_list:
                continue
            
            child_node.g = current_node.g + move_cost
            child_node.h = abs(child_node.position[0] - end_node_pos[0]) + abs(child_node.position[1] - end_node_pos[1])
            child_node.f = child_node.g + child_node.h

            if child_node in open_list_lookup and child_node.g >= open_list_lookup[child_node]:
                continue
                
            open_list_lookup[child_node] = child_node.g
            heapq.heappush(open_list, child_node)
                
    return None

# --- シミュレーション: KFS配置 ---
def opponent_places_kfs_and_heights(force_trap=False):
    print("--- [シミュレーション] 相手チームがKFSを配置中... ---")
    if force_trap:
        print("--- [シミュレーション] 妨害戦略：1,2,3 に R2 KFS を配置しない ---")
        
    ground_truth_map = [[EMPTY for _ in range(COLS)] for _ in range(ROWS)]
    available_blocks = list(BLOCK_COORDS.keys())
    
    # R1 KFS (2)
    r1_placement_pool = [b for b in available_blocks if b in BOUNDARY_BLOCKS_NUMS]
    if force_trap:
        r1_choices = []
        trap_blocks = [b for b in r1_placement_pool if b in ENTRANCE_BLOCKS_NUMS]
        r1_choices.extend(trap_blocks)
        r1_placement_pool = [b for b in r1_placement_pool if b not in ENTRANCE_BLOCKS_NUMS]
        r1_choices.extend(random.sample(r1_placement_pool, 3 - len(r1_choices)))
    else:
        r1_choices = random.sample(r1_placement_pool, 3)
    for b in r1_choices:
        ground_truth_map[BLOCK_COORDS[b][0]][BLOCK_COORDS[b][1]] = OBSTACLE_R1
        available_blocks.remove(b)
        
    # Fake KFS (3)
    fake_placement_pool = [b for b in available_blocks if b not in ENTRANCE_BLOCKS_NUMS]
    fake_choice = random.choice(fake_placement_pool)
    ground_truth_map[BLOCK_COORDS[fake_choice][0]][BLOCK_COORDS[fake_choice][1]] = OBSTACLE_FAKE
    available_blocks.remove(fake_choice)
    
    # R2 KFS (1)
    r2_placement_pool = available_blocks
    if force_trap:
        r2_placement_pool = [b for b in available_blocks if b not in ENTRANCE_BLOCKS_NUMS]
    r2_choices = random.sample(r2_placement_pool, 4)
    for b in r2_choices:
        ground_truth_map[BLOCK_COORDS[b][0]][BLOCK_COORDS[b][1]] = R2_KFS
        available_blocks.remove(b)

    fixed_height_map = np.array([
        [HEIGHT_400, HEIGHT_200, HEIGHT_400], # 1(0.4), 2(0.2), 3(0.4)
        [HEIGHT_200, HEIGHT_400, HEIGHT_600], # 4(0.2), 5(0.4), 6(0.6)
        [HEIGHT_400, HEIGHT_600, HEIGHT_400], # 7(0.4), 8(0.6), 9(0.4)
        [HEIGHT_200, HEIGHT_400, HEIGHT_200]  # 10(0.2), 11(0.4), 12(0.2)
    ])
    
    print("--- [シミュレーション] 配置完了 ---")
    return ground_truth_map, fixed_height_map

# --- R2の動作 ---
def scan_surroundings(current_pos, robot_grid_map, target_list, ground_truth_map):
    print(f"  [スキャン] ブロック {get_block_num(current_pos[0], current_pos[1])} から周囲をスキャン...")
    for node in get_adjacent_nodes(current_pos[0], current_pos[1]):
        r, c = node
        if robot_grid_map[r][c] == UNKNOWN:
            ground_truth_state = ground_truth_map[r][c]
            
            if ground_truth_state == EMPTY: robot_state_to_save = EMPTY
            elif ground_truth_state == R2_KFS: robot_state_to_save = R2_KFS
            elif ground_truth_state == OBSTACLE_R1: robot_state_to_save = OBSTACLE_R1
            elif ground_truth_state == OBSTACLE_FAKE: robot_state_to_save = OBSTACLE_FAKE
            
            robot_grid_map[r][c] = robot_state_to_save
            
            if robot_state_to_save == R2_KFS and node not in target_list:
                target_list.append(node)
                print(f"    -> 新ターゲット(R2 KFS)発見！ (ブロック {get_block_num(r,c)})")

def dummy_pickup_kfs(target_node):
    print(f"  [回収] 隣接するブロック {get_block_num(target_node[0], target_node[1])} のKFSを回収しました。")


# --- メインの戦術実行ロジック ---
def run_r2_strategy(ground_truth_map, height_map, pos_to_number):
    
    # --- 0. 初期化 ---
    GOAL_KFS_COUNT = 2
    PLOT_PAUSE_TIME = 3.0
    
    robot_grid_map = [[UNKNOWN for _ in range(COLS)] for _ in range(ROWS)]
    target_list = []
    
    collected_kfs_count = 0
    first_kfs_obligation = None 
    first_kfs_collected = False 
    
    current_state = None 
    
    max_iterations = 100
    iteration_count = 0
    
    # --- 1. スタート位置の動的決定 ---
    valid_start_pos = None
    START_BLOCK_NUM = -1
    
    for block_num in [1, 2, 3]:
        pos = BLOCK_COORDS[block_num]
        height = height_map[pos[0], pos[1]]
        if height <= (ALLOWED_STEP_HEIGHT + 0.01): 
            valid_start_pos = pos
            START_BLOCK_NUM = block_num
            print(f"[R2] スタート地点探索：ブロック {block_num} (高さ {height}m) は登頂可能。")
            break 
        else:
             print(f"[R2] スタート地点探索：ブロック {block_num} (高さ {height}m) は登頂不可。")
            
    if valid_start_pos is None:
        print(f"\n--- [R2] タスク失敗 ---")
        print(f"理由: ブロック 1, 2, 3 がすべて登れる段差 ({ALLOWED_STEP_HEIGHT}m) を超えています。")
        plot_title = f"TASK FAILED: No valid start block (<= {ALLOWED_STEP_HEIGHT}m)"
        plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], None, plot_title)
        plt.pause(PLOT_PAUSE_TIME * 2); plt.close('all'); return
        
    start_orientation = 0 
    current_state = (valid_start_pos[0], valid_start_pos[1], start_orientation)
    current_position = valid_start_pos
    
    scan_surroundings(current_position, robot_grid_map, target_list, ground_truth_map)
    start_state_on_map = robot_grid_map[current_position[0]][current_position[1]]
    if start_state_on_map == OBSTACLE_FAKE: 
        print(f"\n--- [R2] タスク失敗 ---")
        print(f"理由: スタートブロック (ブロック {START_BLOCK_NUM}) が通行不可(FAKE)です。")
        plt.close('all'); return
    if start_state_on_map == R2_KFS:
         target_list.remove(current_position) 
    robot_grid_map[current_position[0]][current_position[1]] = EMPTY 
    
    print(f"[R2] ブロック {START_BLOCK_NUM} (高さ {height_map[current_position[0], current_position[1]]}m) に移動...")
    print(f"\n--- 戦術開始 (目標: {GOAL_KFS_COUNT}個のKFS回収) ---")

    plot_title = f"1. Moved to Block {START_BLOCK_NUM}. First Scan."
    plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], [current_position], plot_title)
    plt.pause(PLOT_PAUSE_TIME)
    
    # --- 2. メイン探索ループ ---
    while True:
        iteration_count += 1
        if iteration_count > max_iterations:
            print(f"[R2] エラー: 最大試行回数 ({max_iterations}) を超えました。無限ループの可能性があります。")
            break

        current_position = (current_state[0], current_state[1]) 
        
        # --- [Phase 1] 義務を確認 ---
        if first_kfs_obligation is None:
            print("\n[R2] 戦術：ルール4.4.15（入口ブックの収集義務）を確認中...")
            priority_targets_found = False
            for pos in ENTRANCE_BLOCKS_COORDS:
                if robot_grid_map[pos[0]][pos[1]] == R2_KFS:
                    priority_targets_found = True
                    break
            
            if priority_targets_found:
                first_kfs_obligation = True
                print("[R2] 結果：ブロック 1, 2, 3 にR2 KFSを発見")
                print("[R2] 戦術：最初のKFSを 1, 2, 3 から回収します。")
            else:
                all_entrance_scanned = all(robot_grid_map[pos[0]][pos[1]] != UNKNOWN for pos in ENTRANCE_BLOCKS_COORDS)
                if all_entrance_scanned:
                    first_kfs_obligation = False
                    first_kfs_collected = True 
                    print("[R2] 結果：ブロック 1, 2, 3 を全てスキャンしましたがR2 KFSはありません。")
                    print("[R2] 戦術：MFの任意の場所から収集を開始します。")
                else:
                    print("[R2] 結果：入口エリアにまだ未知のブロックがあります。探索を続行します。")

        # --- [Phase 2] 入口の義務を遂行 (または入口を探索) ---
        elif first_kfs_obligation is True and first_kfs_collected is False:
            print(f"\n[R2] 戦術：(義務) 入口のKFSを探索中... (現在地: ブロック {get_block_num(current_position[0], current_position[1])})")

            priority_targets = [t for t in target_list if t in ENTRANCE_BLOCKS_COORDS]
            
            pickup_locations = set()
            for target in priority_targets:
                for node in get_adjacent_nodes(target[0], target[1]):
                    state = robot_grid_map[node[0]][node[1]]
                    if (state != OBSTACLE_FAKE and state != UNKNOWN) and (node in ENTRANCE_BLOCKS_COORDS):
                        pickup_locations.add(node)
            
            path_states = None
            if pickup_locations:
                shortest_path_len = float('inf')
                best_path_states = None
                for goal_pos in pickup_locations:
                    path = astar(robot_grid_map, height_map, current_state, goal_pos)
                    if path and len(path) < shortest_path_len:
                        shortest_path_len = len(path)
                        best_path_states = path
                path_states = best_path_states
            
            if path_states:
                goal_block_num = get_block_num(path_states[-1][0], path_states[-1][1])
                print(f"[R2] [回収モード] ブロック {goal_block_num} へ移動し、隣のKFSを回収します。")
                
                plot_path = [pos[0:2] for pos in path_states]
                if len(plot_path) > 1:
                    plot_title = f"1a. Moving to Block {goal_block_num} (neighbor of KFS)"
                    plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], plot_path, plot_title)
                    plt.pause(PLOT_PAUSE_TIME)
                
                current_state = path_states[-1]
                current_position = (current_state[0], current_state[1])
                scan_surroundings(current_position, robot_grid_map, target_list, ground_truth_map)
                
                target_to_pickup = None
                for adj_node in get_adjacent_nodes(current_position[0], current_position[1]):
                    if adj_node in priority_targets:
                        target_to_pickup = adj_node
                        break
                
                if target_to_pickup:
                    dummy_pickup_kfs(target_to_pickup)
                    collected_kfs_count += 1
                    first_kfs_collected = True 
                    
                    robot_grid_map[target_to_pickup[0]][target_to_pickup[1]] = EMPTY 
                    target_list.remove(target_to_pickup)
                    
                    plot_title = f"1b. FIRST KFS collected! (from {goal_block_num})"
                    plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], [current_position], plot_title)
                    plt.pause(PLOT_PAUSE_TIME)
                    continue 

            else:
                print("[R2] 戦術：入口 (1, 2, 3) にR2 KFSが見つからないか、到達できません。")
                
                explorable_entrance_blocks = []
                for pos in ENTRANCE_BLOCKS_COORDS:
                    state = robot_grid_map[pos[0]][pos[1]]
                    if state != OBSTACLE_FAKE and state != UNKNOWN and pos != current_position:
                        explorable_entrance_blocks.append(pos)
                
                path_states = None
                if explorable_entrance_blocks:
                    goal_node_pos = explorable_entrance_blocks[0]
                    path_states = astar(robot_grid_map, height_map, current_state, goal_node_pos)

                if path_states:
                    goal_block_num = get_block_num(path_states[-1][0], path_states[-1][1])
                    print(f"[R2] 戦術：入口の未スキャン領域を探すため、ブロック {goal_block_num} へ移動します。")
                    
                    plot_path = [pos[0:2] for pos in path_states]
                    plot_title = f"1c. Exploring Entrance Area. Moving to {goal_block_num}"
                    plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], plot_path, plot_title)
                    plt.pause(PLOT_PAUSE_TIME)
                    
                    current_state = path_states[-1]
                    current_position = (current_state[0], current_state[1])
                    scan_surroundings(current_position, robot_grid_map, target_list, ground_truth_map)
                    continue 
                else:
                    print("[R2] 戦術：入口 (1, 2, 3) の探索できる場所がもうありません。")
                    all_entrance_scanned = all(robot_grid_map[pos[0]][pos[1]] != UNKNOWN for pos in ENTRANCE_BLOCKS_COORDS)
                    
                    if all_entrance_scanned and not priority_targets:
                        print("\n--- [R2] タスク失敗 ---")
                        print("理由: ブロック 1, 2, 3 をすべてスキャンしましたが、R2 KFSが見つかりませんでした。")
                        print("相手チームの妨害戦略が成功しました。MFタスクを中断します。")
                        plot_title = "TASK FAILED: No R2 KFS found in blocks 1, 2, or 3."
                        plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], [current_position], plot_title)
                        plt.pause(PLOT_PAUSE_TIME * 2); plt.close('all'); break
        
        # --- [Phase 3] 2個目のKFS (どこでも可) を回収する (義務免除 or 義務遂行済み) ---
        elif first_kfs_collected is True and collected_kfs_count < GOAL_KFS_COUNT:
            print(f"\n[R2] 戦術：{collected_kfs_count + 1}個目のKFSを探索中... (現在地: ブロック {get_block_num(current_position[0], current_position[1])})")

            pickup_locations = set()
            for target in target_list:
                for node in get_adjacent_nodes(target[0], target[1]):
                    state = robot_grid_map[node[0]][node[1]]
                    if state != OBSTACLE_FAKE and state != UNKNOWN:
                        pickup_locations.add(node)
            
            path_states = None
            if pickup_locations:
                shortest_path_len = float('inf')
                best_path_states = None
                for goal_pos in pickup_locations:
                    path = astar(robot_grid_map, height_map, current_state, goal_pos)
                    if path and len(path) < shortest_path_len:
                        shortest_path_len = len(path)
                        best_path_states = path
                path_states = best_path_states
            
            if path_states:
                goal_block_num = get_block_num(path_states[-1][0], path_states[-1][1])
                print(f"[R2] [回収モード] ブロック {goal_block_num} へ移動し、隣のKFSを回収します。")
                
                plot_path = [pos[0:2] for pos in path_states]
                plot_title = f"2a. Moving to neighbor of KFS (at {goal_block_num})"
                plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], plot_path, plot_title)
                plt.pause(PLOT_PAUSE_TIME)
                
                current_state = path_states[-1]
                current_position = (current_state[0], current_state[1])
                scan_surroundings(current_position, robot_grid_map, target_list, ground_truth_map)
                
                target_to_pickup = None
                for adj_node in get_adjacent_nodes(current_position[0], current_position[1]):
                    if adj_node in target_list:
                        target_to_pickup = adj_node
                        break
                
                if target_to_pickup:
                    dummy_pickup_kfs(target_to_pickup)
                    collected_kfs_count += 1
                    robot_grid_map[target_to_pickup[0]][target_to_pickup[1]] = EMPTY
                    target_list.remove(target_to_pickup)
                    
                    plot_title = f"2b. Collected KFS {collected_kfs_count}. (from {goal_block_num})"
                    plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], [current_position], plot_title)
                    plt.pause(PLOT_PAUSE_TIME)
                    continue
            
            else:
                print("[R2] [探索モード] 既知のターゲットがないか経路がありません。未知のエリアを探索します。")
                
                explorable_neighbors = set()
                for r in range(ROWS):
                    for c in range(COLS):
                        state = robot_grid_map[r][c]
                        if state != OBSTACLE_FAKE and state != UNKNOWN:
                            for node in get_adjacent_nodes(r, c):
                                if robot_grid_map[node[0]][node[1]] == UNKNOWN:
                                    explorable_neighbors.add((r, c)) 
                                    break
                
                explorable_neighbors.discard(current_position)

                if not explorable_neighbors:
                    print("[R2] マップをすべて探索しましたが、新しいターゲットが見つかりません。")
                    break 

                shortest_path_len = float('inf')
                best_path_states = None
                for goal_pos in explorable_neighbors:
                    path = astar(robot_grid_map, height_map, current_state, goal_pos)
                    if path and len(path) < shortest_path_len:
                        shortest_path_len = len(path)
                        best_path_states = path
                path_states = best_path_states
                
                if path_states:
                    goal_block_num = get_block_num(path_states[-1][0], path_states[-1][1])
                    plot_title = f"2c. [Exploring] Moving to {goal_block_num}"
                    plot_path = [pos[0:2] for pos in path_states]
                    plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], plot_path, plot_title)
                    plt.pause(PLOT_PAUSE_TIME)
                    
                    current_state = path_states[-1]
                    current_position = (current_state[0], current_state[1])
                    scan_surroundings(current_position, robot_grid_map, target_list, ground_truth_map)
                    continue
                else:
                    print("[R2] 探索モード失敗: 未知エリアへの経路もありません。")
                    break 
        
        # --- [Phase 4] 脱出 ---
        elif first_kfs_collected is True and collected_kfs_count >= GOAL_KFS_COUNT:
            print(f"\n--- ステップ3: 脱出シーケンス開始 ---")
            print(f"[R2] 目標の{GOAL_KFS_COUNT}個を回収完了。脱出します。")
            
            movable_exit_cells = set()
            for pos in EXIT_BLOCKS_COORDS:
                state = robot_grid_map[pos[0]][pos[1]]
                if state != OBSTACLE_FAKE and state != UNKNOWN:
                    movable_exit_cells.add(pos)
            
            # 変数名を path_to_exit に統一
            path_to_exit = None 
            if movable_exit_cells:
                shortest_path_len = float('inf')
                best_path_states = None
                for goal_pos in movable_exit_cells:
                    path = astar(robot_grid_map, height_map, current_state, goal_pos)
                    if path and len(path) < shortest_path_len:
                        shortest_path_len = len(path)
                        best_path_states = path
                path_to_exit = best_path_states
            
            # 変数名を path_to_exit に統一
            if path_to_exit: 
                exit_block_num = get_block_num(path_to_exit[-1][0], path_to_exit[-1][1])
                plot_title = f"3. Escaping via Block {exit_block_num}"
                plot_path = [pos[0:2] for pos in path_to_exit] # (r,c)のみ抽出
                plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], plot_path, plot_title)
                print(f"\n--- [R2] 戦術完了: {exit_block_num} 番ブロックへ脱出しました ---")
                print("シミュレーション完了。最後のプロットウィンドウを閉じてください。")
                plt.show() # 最後のプロットを表示したまま保持
                break # while ループを終了

            else:
                # 既知の出口がない(UNKNOWN)か、経路がない場合 -> [探索モード] へ
                print("[R2] [探索モード] 既知の出口がないか経路がありません。出口エリアを探索します。")
                
                explorable_neighbors = set()
                for r in range(ROWS):
                    for c in range(COLS):
                        state = robot_grid_map[r][c]
                        # 通行可能なブロックから
                        if state != OBSTACLE_FAKE and state != UNKNOWN:
                            for node in get_adjacent_nodes(r, c):
                                # UNKNOWN を探す
                                if robot_grid_map[node[0]][node[1]] == UNKNOWN:
                                    explorable_neighbors.add((r, c)) 
                                    break
                
                explorable_neighbors.discard(current_position)

                if not explorable_neighbors:
                    print("[R2] マップをすべて探索しましたが、出口への経路が見つかりません。")
                    plt.close('all'); break

                # 最も出口(Row 3)に近い探索ゴールを選ぶ
                # (出口に隣接しているか、出口エリア(Row 3)にある探索ゴール)
                explorable_exit_neighbors = [pos for pos in explorable_neighbors if pos[0] == 3 or any(n in EXIT_BLOCKS_COORDS for n in get_adjacent_nodes(pos[0], pos[1]))]
                
                best_path_states = None
                if explorable_exit_neighbors:
                     # 出口に最も近い探索ゴール
                     goal_node_pos = max(explorable_exit_neighbors, key=lambda pos: pos[0]) 
                     best_path_states = astar(robot_grid_map, height_map, current_state, goal_node_pos)

                # もし出口付近に探索ゴールがない、または経路がない場合、他の探索ゴールを探す
                if not best_path_states:
                    shortest_path_len = float('inf')
                    for goal_pos in explorable_neighbors:
                        path = astar(robot_grid_map, height_map, current_state, goal_pos)
                        if path and len(path) < shortest_path_len:
                            shortest_path_len = len(path)
                            best_path_states = path
                
                path_states = best_path_states # path_to_unknown の代わりに
                
                if path_states:
                    goal_block_num = get_block_num(path_states[-1][0], path_states[-1][1])
                    plot_title = f"3a. [Exploring Exit] Moving to {goal_block_num}"
                    plot_path = [pos[0:2] for pos in path_states]
                    plot_3d_maze_path(height_map, [row[:] for row in robot_grid_map], plot_path, plot_title)
                    plt.pause(PLOT_PAUSE_TIME)
                    
                    current_state = path_states[-1]
                    current_position = (current_state[0], current_state[1])
                    scan_surroundings(current_position, robot_grid_map, target_list, ground_truth_map)
                    continue # ループの最初に戻り、再度脱出を試みる (Phase 4)
                else:
                    print("[R2] 戦術失敗: 出口エリアへの探索経路が見つかりませんでした。")
                    plt.close('all'); break
        
        else:
             print("[R2] 致命的なロジックエラー。停止します。")
             break

# --- 実行 ---
if __name__ == "__main__":
    pos_to_number = create_numbering()
    
    # ----------------------------------------------------
    # シミュレーションのシナリオを選択
    # True = 妨害戦略 (1,2,3にR2 KFSなし)
    # False = 通常 (ランダム配置)
    RUN_TRAP_SCENARIO = True
    # ----------------------------------------------------
    
    ground_truth_map, height_map = opponent_places_kfs_and_heights(force_trap=RUN_TRAP_SCENARIO)
    
    plt.ion() 
    run_r2_strategy(ground_truth_map, height_map, pos_to_number)
    plt.ioff()
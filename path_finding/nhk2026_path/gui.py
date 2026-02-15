#　スタートとゴール、R1、FAKEを順にマップをクリックすると
#　Astarアルゴリズムに基づいた経路が生成されます


import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


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

     
def create_tooltip(widget, text):
    tooltip = tk.Label(widget, text=text, background="yellow", relief="solid", borderwidth=1)
    tooltip.place_forget()  # 最初は表示しない

    def on_enter(event):
     tooltip.place(x=event.x_root, y=event.y_root)

    def on_leave(event):
     tooltip.place_forget()

    widget.bind("<Enter>", on_enter)
    widget.bind("<Leave>", on_leave)
    return tooltip


# 迷路を描画してクリックでスタートとゴールを選択
class PathfindingGUI:
    def __init__(self, master, maze, height_map, number_to_pos, pos_to_number):
        self.master = master
        self.maze = maze
        self.height_map = height_map
        self.number_to_pos = number_to_pos
        self.pos_to_number = pos_to_number
        self.rows = len(maze)
        self.cols = len(maze[0])

        self.canvas = tk.Canvas(master, width=self.cols * 200, height=self.rows * 200)
        self.canvas.pack()
       # 変数current_selectionの初期化
        self.current_selection = "start"
        self.r1_count = 0  # r1ブロックの配置カウントを初期化
        

        self.start_pos = None
        self.goal_pos = None
        

        self.canvas.bind("<Button-1>", self.on_click)

        self.instruction_label = tk.Label(master, text="クリックでスタート位置を選択してください。")
        self.instruction_label.pack()

        # ステータスバーの設置
        self.status_label = tk.Label(master, text="現在の状態: スタート位置", font=('Helvetica', 10, 'italic'))
        self.status_label.pack()


        self.switch_button = tk.Button(
                master, 
                text="Start >>> Goal >>> R1 >>> Fake", 
                command=self.switch_selection,
                bg="lightblue", 
                font=('Helvetica', 12, 'bold'),
                relief="raised", 
                bd=5
            )
        self.switch_button.pack()

        # ツールチップの作成
        create_tooltip(self.switch_button, "スタートとゴールを設定した後に探索を開始します")

        # 迷路の描画
        self.draw_maze()
    
    def switch_selection(self):
        """ 選択肢を切り替えるメソッド """
        if self.current_selection == "start":
            self.current_selection = "goal"
            self.instruction_label.config(text="ゴール位置をクリックしてください。")
        elif self.current_selection == "goal":
            self.current_selection = "r1"
            self.instruction_label.config(text="R1ブロックをクリックしてください。")
        elif self.current_selection == "r1":
            self.current_selection = "fake"
            self.instruction_label.config(text="Fakeブロックをクリックしてください。")
        elif self.current_selection == "fake":
            self.current_selection = None
            self.instruction_label.config(text="すべてのブロックが設定されました。経路探索を開始します。")
            self.find_path()
    
    def draw_maze(self):
        """ 迷路をキャンバスに描画する """
        for r in range(self.rows):
            for c in range(self.cols):
                # セルの色を決定
                color = 'white'  # 通行可能
                if self.maze[r][c] == 1:  # 障害物
                    color = 'hotpink'
                elif self.maze[r][c] == 2:  # 壁
                    color = 'gray'
                elif self.maze[r][c] == 3:  # Fake
                    color = 'black'


                # 各セルを描画
                self.canvas.create_rectangle(c * 200, r * 200, (c + 1) * 200, (r + 1) * 200, fill=color, outline="black")


    def on_click(self, event):
        col = event.x // 200  # 1セルの幅を100pxと仮定
        row = event.y // 200  # 1セルの高さを100pxと仮定

        if self.current_selection == "start":
            self.start_pos = (row, col)
            self.canvas.create_text(col * 200 + 100, row * 200 + 100, text="START", font=('Helvetica', 24, 'bold'))
            self.canvas.create_oval(col * 200 + 50, row * 200 + 50, col * 200 + 150, row * 200 + 150, outline="green", width=3)
            self.instruction_label.config(text="ゴール位置をクリックしてください。")
            self.current_selection = "goal"  # 次はgoalを設定

        elif self.current_selection == "goal":
            self.goal_pos = (row, col)
            self.canvas.create_text(col * 200 + 100, row * 200 + 100, text="GOAL", font=('Helvetica', 24, 'bold'))
            self.canvas.create_oval(col * 200 + 50, row * 200 + 50, col * 200 + 150, row * 200 + 150, outline="red", width=3)
            self.instruction_label.config(text="R1ブロックをクリックしてください。")
            self.current_selection = "r1"  # 次はR1を設定

        elif self.current_selection == "r1":
             if self.r1_count < 3:  # r1ブロックは最大3つまで配置
                self.maze[row][col] = 1  # R1は障害物として設定
                self.canvas.create_rectangle(col * 200, row * 200, (col + 1) * 200, (row + 1) * 200, fill="hotpink", outline="black")
                self.canvas.create_text(col * 200 + 100, row * 200 + 100, text="R1",font=('Helvetica', 24, 'bold'))
                self.r1_count += 1
                if self.r1_count == 3:  # 3つ配置後にFakeブロック設定へ
                    self.instruction_label.config(text="Fakeブロックをクリックしてください。")
                    self.current_selection = "fake"  # 次はFakeを設定
        

        elif self.current_selection == "fake":
            self.maze[row][col] = 3  # Fakeブロックを設定
            
            self.canvas.create_rectangle(col * 200, row * 200, (col + 1) * 200, (row + 1) * 200, fill="black", outline="black")
            self.canvas.create_text(col * 200 + 100, row * 200 + 100, text="R1",font=('Helvetica', 24, 'bold'))
     
            self.instruction_label.config(text="すべてのブロックが設定されました。経路探索を開始します。")
            self.current_selection = None  # 最後の選択後は終了
            self.find_path()

         # すべての選択が終わったら、経路探索を開始
        



    def find_path(self):
        
        if self.start_pos and self.goal_pos:
            path, total_cost = astar(self.maze, self.height_map, self.start_pos, self.goal_pos)
            
            if path:
                print(f"経路: {path}")
                print(f"総コスト: {total_cost}")
                self.show_path(path)
        
                plot_3d_maze_path(self.height_map, self.maze, path,self.start_pos, self.goal_pos)

            else:
                messagebox.showerror("エラー", "経路が見つかりませんでした。R1ブックを削除して再探索しましょう。")
                """経路探索が失敗した後にR1ブロック削除モードに切り替える"""
                self.current_selection = "remove_r1"  # 新しいモードに切り替え
                self.canvas.bind("<Button-1>", self.on_remove_r1_click)  # 削除用のクリックイベントをバインド
                
                    # 経路が見つかった後にr1ブロックを削除するオプション

    

    def on_remove_r1_click(self, event):
        """R1ブロック削除用のクリックイベント"""
        col = event.x // 200  # 1セルの幅を100pxと仮定
        row = event.y // 200  # 1セルの高さを100pxと仮定

        if self.maze[row][col] == 1:  # R1ブロックがクリックされた場合
            # R1ブロックを削除する確認ダイアログを表示
            user_input = messagebox.askyesno(
                "R1ブロック削除確認",
                f"R1ブロック {self.pos_to_number[(row, col)]} ({row}, {col}) を取り除きますか？"
            )
            if user_input:  # ユーザーが「Yes」を選択した場合
                self.maze[row][col] = 0  # R1ブロックを取り除く
                self.canvas.create_rectangle(col * 200, row * 200, (col + 1) * 200, (row + 1) * 200, fill="white", outline="black")
                self.r1_count -= 1  # R1ブロックの数を減らす

                # 削除後にメッセージを表示
                messagebox.showinfo("削除完了", f"R1ブロック {self.pos_to_number[(row, col)]} が取り除かれました。")
                
                # 削除後に再探索を試みる
                self.find_path()  # 経路探索を再実行
            else:
                # ユーザーが「No」を選択した場合、何もしない
                messagebox.showinfo("キャンセル", "R1ブロックの削除はキャンセルされました。")
       
    
        



    def show_path(self, path):
        """ 経路をキャンバスに矢印で描画 """
        for i in range(len(path)-1):
            r1, c1 = path[i]
            r2, c2 = path[i+1]

            # セル中心の座標
            x1 = c1 * 200 + 100  # 200pxセルの中心
            y1 = r1 * 200 + 100
            x2 = c2 * 200 + 100
            y2 = r2 * 200 + 100

            # 矢印を描画
            self.canvas.create_line(x1, y1, x2, y2, arrow=tk.LAST, fill="blue", width=3)

        self.instruction_label.config(text="経路探索が完了しました。")
   

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

# メイン関数
def main():
    maze = [
        [2, 2, 2, 2, 2],
        [2, 0, 0, 0, 2],
        [2, 0, 0, 0, 2],
        [2, 0, 0, 0, 2],
        [2, 0, 0, 0, 2]
    ]
    
    height_map = np.array([
        [0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 2, 1, 0, 0],
        [0, 1, 2, 1, 0],
        [0, 0, 1, 0, 0]
    ])

    number_to_pos, pos_to_number = create_numbering(1, 1, 4, 3)

    root = tk.Tk()
    root.title("NHKロボコン2026 経路探索")
    app = PathfindingGUI(root, maze, height_map, number_to_pos, pos_to_number)
    root.mainloop()

if __name__ == "__main__":
    main()

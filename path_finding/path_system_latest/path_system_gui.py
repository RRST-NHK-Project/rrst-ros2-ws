# NHK2026ロボコン
# 経路探索システム
# 2026/03/11

'''
 MAIN
 ^[GUI] --> plot3D
 ^^A*
 ^^^Node
 
 ^MapData
'''

import tkinter as tk
from tkinter import messagebox

# 分割用
from path_system_astar import astar, calc_step_height
from path_system_plot3d import plot_3d_maze_path
from path_system_map import maze, height_map

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

# GUIクラス
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
            self.canvas.create_text(col * 200 + 100, row * 200 + 100, text="",font=('Helvetica', 24, 'bold'))
     
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
   
# NHK2026ロボコン
# 経路探索システム
# 2026/03/11

'''
 [MAIN]
 ^GUI --> plot3D
 ^^A*   --> Move ((→ ROS2通信へ))
 ^^^Node

 ^MapData
'''
# 分割用
from path_system_node import Node
from path_system_astar import astar, calc_step_height
from path_system_move import Movement
from path_system_gui import PathfindingGUI

from path_system_map import maze, height_map
 

import tkinter as tk
from tkinter import messagebox
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


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
 
    number_to_pos, pos_to_number = create_numbering(1, 1, 4, 3)

    root = tk.Tk()
    root.title("NHKロボコン2026 経路探索")
    app = PathfindingGUI(root, maze, height_map, number_to_pos, pos_to_number)
    root.mainloop()

if __name__ == "__main__":
    main()





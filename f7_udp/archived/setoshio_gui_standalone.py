import tkinter as tk
import os
from PIL import ImageTk

root = tk.Tk()
root.geometry("600x400")

# Canvasの作成
canvas = tk.Canvas(root, bg = "Grey")
# Canvasを配置
canvas.pack(fill = tk.BOTH, expand = True)

canvas.create_rectangle(75, 175, 125, 225, fill="Red") 
canvas.create_rectangle(175, 175, 225, 225, fill="Red") 
canvas.create_rectangle(275, 175, 325, 225, fill="Blue") 
canvas.create_rectangle(375, 175, 425, 225, fill="White") 
canvas.create_rectangle(475, 175, 525, 225, fill="White") 

root.mainloop()
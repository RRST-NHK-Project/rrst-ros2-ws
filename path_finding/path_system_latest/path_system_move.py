# NHK2026ロボコン
# 経路探索システム
# 2026/03/19(intに変更)

'''
 MAIN
 ^GUI --> plot3D
 ^^A*   --> [Move] ((→ ROS2通信へ))
 ^^^Node

 ^MapData
'''


# ロボットの進行方向クラス
class Movement:
    def __init__(self, path, height_map):
        self.path = path  # A*のpath(r,c)
        self.height_map = height_map
        self.moves = []

    def generate_moves(self):
        self.moves = []

        for i in range(len(self.path)-1):
            r1, c1 = self.path[i]
            r2, c2 = self.path[i+1]
        
            # 次のマスとの距離の差
            dr = r2 - r1
            dc = c2 - c1
            dh = self.height_map[r2][c2] - self.height_map[r1][c1]

            # 0=前、1=後ろ、2=右、3=左、4=登る、5=降りる、6=止まる
            # (方向、上り下り)
            ## 方向
            if dr == -1 and dc == 0:    
                direction = 1   #前
            elif dr == 1 and dc == 0:
                direction = 2    #後
            elif dr == 0 and dc == -1:    
                direction = 3   #右
            elif dr == 0 and dc == 1:
                direction = 4    #左
            else:
                direction = 0 #止まる

            ##上り下り
            if dh > 0:
                status = 1   #上
            elif dh < 0:
                status = 2  #下
            else:
                status = 0   #水平
            

            move = direction + status * 10 #(前->1,前方にのぼる->11) 
            self.moves.append(move)  
        
        return self.moves
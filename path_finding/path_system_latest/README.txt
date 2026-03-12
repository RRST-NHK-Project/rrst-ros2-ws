NHKロボコン2026 経路探索システム

スタートとゴールを設定し、R1とFakeブロックを配置することで、
マップ内の障害物を避けながら、進む最適経路を見つけます。
最適経路を3DマップとGUI上に表示します。

================================

[実行方法]
cd ros2_ws/src/path_finding/path_system_latest
python path_system_main.py


[フォルダ構成]
path_system_latest/
    |
    ├─  path_system_main.py         // 全体
    ├─  path_system_gui.py          // 画面操作など
    ├─  path_system_plot3d.py       // 3D図描画
    ├─  path_system_astar.py        // 経路アルゴリズムA*の計算
    ├─  path_system_node.py         // Nodeクラス(A*で使用)
    ├─  path_system_map.py          // マップ
    └─  README.txt  


[使用ライブラリ] 
・ Python3
・ numpy
・ matplotlib
・ tkinter (画面用)
・ mpl_toolkits.mplot3d (3D図用)

    ※ インストール
       pip install numpy matplotlib
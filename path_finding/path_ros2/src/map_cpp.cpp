/*NHK2026ロボコン
  経路探索システム
  2026/03/22 */

/*
 MAIN
 ^GUI --> plot3D
 ^^A*   --> Move ((→ ROS2通信へ))
 ^^^Node

 ^[MapData]
*/

#include <vector>
#include <iostream>

// 迷路マップ
std::vector<std::vector<int>> maze(){
  return {
    {2, 2, 2, 2, 2},
    {2, 0, 0, 0, 2},
    {2, 0, 0, 0, 2},
    {2, 0, 0, 0, 2},
    {2, 0, 0, 0, 2}   
  };
}

// 高さマップ
std::vector<std::vector<int>> height_map(){
  return {
    {0, 0, 0, 0, 0},
    {0, 1, 0, 1, 0},
    {0, 2, 1, 0, 0},
    {0, 1, 2, 1, 0},
    {0, 0, 1, 0, 0}    
  };
}




/*NHK2026ロボコン
  経路探索システム
  2026/03/30(みかんせい)*/

/*
 MAIN
 ^GUI --> plot3D
 ^^A*   --> Move ((→ ROS2通信へ))
 ^^^[Node]

 ^MapData
*/

#include <iostream>
#include <vector>
#include <algorithm> //min_element

struct pos{
    int row,col; //マップ行列  
    int g,h,f; //astarコスト
};

int calc_h(pos a, pos b){
  return abs(a.row - b.row) + abs(a.col - b.col);
}

void astar(pos start,pos end){

  //リスト作る
  std::vector<pos> open_list; //探索  
  std::vector<pos> closed_list; //探索DONE

  open_list = {start}; closed_list = {};
  start.g = 0; start.h = calc_h(start, end);
  start.f = start.g + start.h;
  

  while(!open_list.empty()){
    
    //openリスト内で最小のfを探す
    auto min_f = std::min_element(
      open_list.begin(), open_list.end(), 
      [](const pos &a, const pos &b) {
        return a.f < b.f; // fが小さい方を優先
      }
    );
    pos current = *min_f;
    
    //ゴールの座標かどうか
    if (current.row == end.row && current.col == end.col){
      std::cout << "経路探索成功" << std::endl;
      return;
    };

    // 再度計算
    if (current.row != end.row || current.col != end.col){
      open_list.erase(min_f);
      closed_list.push_back(current);
    };
  
  }


    


  }


  




int main(){
  pos start = {0,0};
  pos end = {1,3};
  astar(start,end);
   
  


  
  

  return 0;
}




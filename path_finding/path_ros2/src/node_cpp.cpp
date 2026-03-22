/*NHK2026ロボコン
  経路探索システム
  2026/03/22 */

/*
 MAIN
 ^GUI --> plot3D
 ^^A*   --> Move ((→ ROS2通信へ))
 ^^^[Node]

 ^MapData
*/


class Node
{
  public:
        int x,y;
        int g,h,f; 
        Node* parent;  

        // 初期化
        Node(int x,int y,Node* parent=nullptr)
        : x(x),y(y),g(0),h(0),f(0),parent(parent) {}

        // 座標の確認
        bool operator==(const Node& other) const {
            return this->x == other.x && this->y == other.y;
        }
};
 








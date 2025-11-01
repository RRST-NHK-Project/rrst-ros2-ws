#include "AstarPlanner.h"
#include <algorithm> // std::reverse

// Pythonの astar 関数
Path AstarPlanner::findPath(
    const GridMap& robotGridMap,
    const HeightMap& heightMap,
    const State& startState,
    const Position& endPos) 
{
    auto startNode = std::make_shared<AstarNode>(nullptr, startState);
    startNode->h = heuristic(startState.pos, endPos);
    startNode->f = startNode->g + startNode->h;
    
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNode> openList;
    openList.push(startNode);

    std::set<State> closedList; 
    std::map<State, double> openListLookup;
    openListLookup[startState] = startNode->g;

    double startHeight = heightMap[startNode->state.pos.r][startNode->state.pos.c];

    while (!openList.empty()) {
        NodePtr currentNode = openList.top();
        openList.pop();

        if (openListLookup.find(currentNode->state) == openListLookup.end()) {
            continue; 
        }
        openListLookup.erase(currentNode->state);

        const Position& currentPos = currentNode->state.pos;
        const int& currentOri = currentNode->state.orientation;

        closedList.insert(currentNode->state);

        if (currentPos == endPos) {
            return reconstructPath(currentNode);
        }

        double currentHeight = heightMap[currentPos.r][currentPos.c];
        
        // 1. 前進
        const auto& dir = FieldData::DIRECTIONS[currentOri];
        Position movePos = {currentPos.r + dir.first, currentPos.c + dir.second};
        processMove(currentNode, movePos, currentOri, MOVE_COST, 
                    endPos, robotGridMap, heightMap, currentHeight, 
                    openList, closedList, openListLookup);
        
        // 2. 左旋回
        int turnLeftOri = (currentOri + 1) % 4;
        processMove(currentNode, currentPos, turnLeftOri, TURN_COST, 
                    endPos, robotGridMap, heightMap, currentHeight, 
                    openList, closedList, openListLookup);

        // 3. 右旋回
        int turnRightOri = (currentOri - 1 + 4) % 4;
        processMove(currentNode, currentPos, turnRightOri, TURN_COST, 
                    endPos, robotGridMap, heightMap, currentHeight, 
                    openList, closedList, openListLookup);
    }

    return {}; // Path not found
}

// ヒューリスティック (マンハッタン距離)
double AstarPlanner::heuristic(const Position& a, const Position& b) {
    return std::abs(a.r - b.r) + std::abs(a.c - b.c);
}

// パスを再構築 (Pythonの while temp:)
Path AstarPlanner::reconstructPath(NodePtr endNode) {
    Path path;
    auto temp = endNode;
    while (temp != nullptr) {
        path.push_back(temp->state);
        temp = temp->parent;
    }
    std::reverse(path.begin(), path.end()); // Pythonの [::-1]
    return path;
}

// 探索の共通処理
void AstarPlanner::processMove(
    NodePtr currentNode,
    const Position& newPos, int newOri, int moveCost,
    const Position& endPos,
    const GridMap& robotGridMap,
    const HeightMap& heightMap,
    double currentHeight,
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNode>& openList,
    std::set<State>& closedList,
    std::map<State, double>& openListLookup)
{
    State newState = {newPos, newOri};

    if (!FieldData::isPositionValid(newPos)) {
        return;
    }

    KfsType newPosState = robotGridMap[newPos.r][newPos.c];
    if (newPosState == KfsType::OBSTACLE_FAKE || newPosState == KfsType::UNKNOWN) {
        return;
    }

    bool isMove = (newPos != currentNode->state.pos);
    if (isMove) {
        double newHeight = heightMap[newPos.r][newPos.c];
        double heightDiff = std::abs(newHeight - currentHeight);
        if (heightDiff > (ALLOWED_STEP_HEIGHT + EPSILON)) {
            return;
        }
    }

    if (closedList.count(newState)) {
        return;
    }

    double new_g = currentNode->g + moveCost;
    
    if (openListLookup.count(newState) && new_g >= openListLookup[newState]) {
         return;
    }
    
    auto childNode = std::make_shared<AstarNode>(currentNode, newState);
    childNode->g = new_g;
    childNode->h = heuristic(newPos, endPos);
    childNode->f = childNode->g + childNode->h;
    
    openList.push(childNode);
    openListLookup[newState] = new_g;
}
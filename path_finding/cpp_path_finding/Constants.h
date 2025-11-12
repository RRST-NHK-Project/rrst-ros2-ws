#pragma once

#include <vector>
#include <map>
#include <set>
#include <string>
#include <cmath>        // std::abs
#include <numeric>      // std::iota
#include <algorithm>    // std::find
#include <memory>       // std::shared_ptr

// --- 基本定数 ---
constexpr int ROWS = 4;
constexpr int COLS = 3;

// --- KFSの状態 ---
// 型 enum class を使用
enum class KfsType {
    EMPTY = 0,
    R2_KFS = 1,
    OBSTACLE_R1 = 2,
    OBSTACLE_FAKE = 3,
    UNKNOWN = -1
};

// --- 高さ ---
constexpr double HEIGHT_200 = 0.2;
constexpr double HEIGHT_400 = 0.4;
constexpr double HEIGHT_600 = 0.6;
constexpr double ALLOWED_STEP_HEIGHT = HEIGHT_200;
constexpr double EPSILON = 0.01; // 浮動小数点数の比較用

// --- 動作コスト ---
constexpr int TURN_COST = 1;
constexpr int MOVE_COST = 1;

// --- 座標 (r, c) ---
struct Position {
    int r = 0;
    int c = 0;

    // std::map や std::set のキーとして使うための比較演算子
    bool operator<(const Position& other) const {
        if (r != other.r) return r < other.r;
        return c < other.c;
    }
    bool operator==(const Position& other) const {
        return r == other.r && c == other.c;
    }
    bool operator!=(const Position& other) const {
        return !(*this == other);
    }
};

// --- R2の状態 (r, c, orientation) ---
struct State {
    Position pos;
    int orientation = 0; // 0:下, 1:右, 2:上, 3:左

    bool operator<(const State& other) const {
        if (pos.r != other.pos.r) return pos.r < other.pos.r;
        if (pos.c != other.pos.c) return pos.c < other.pos.c;
        return orientation < other.orientation;
    }
     bool operator==(const State& other) const {
        return pos == other.pos && orientation == other.orientation;
    }
};

// --- 共通の型エイリアス ---
using GridMap = std::vector<std::vector<KfsType>>;
using HeightMap = std::vector<std::vector<double>>;


// --- フィールドデータとユーティリティ ---
// (Pythonのグローバル辞書やユーティリティ関数をnamespaceに集約)
namespace FieldData {
    // (dr, dc)
    const std::vector<std::pair<int, int>> DIRECTIONS = {
        {1, 0},  // 0: 下
        {0, 1},  // 1: 右
        {-1, 0}, // 2: 上
        {0, -1}  // 3: 左
    };

    // Pythonの DICT を C++の map で初期化
    inline std::map<int, Position> getBlockCoords() {
        return {
            {1, {0, 0}}, {2, {0, 1}}, {3, {0, 2}},
            {4, {1, 0}}, {5, {1, 1}}, {6, {1, 2}},
            {7, {2, 0}}, {8, {2, 1}}, {9, {2, 2}},
            {10, {3, 0}}, {11, {3, 1}}, {12, {3, 2}},
        };
    }

    inline std::map<Position, int> getCoordsToBlock() {
        std::map<Position, int> m;
        for (const auto& pair : getBlockCoords()) {
            m[pair.second] = pair.first;
        }
        return m;
    }

    // Pythonの set を C++の set で初期化
    inline std::set<Position> getEntranceBlocksCoords() {
        return {{0, 0}, {0, 1}, {0, 2}}; // 1, 2, 3
    }
    
    inline std::set<Position> getExitBlocksCoords() {
        return {{3, 0}, {3, 1}, {3, 2}}; // 10, 11, 12
    }
    
    inline std::set<int> getBoundaryBlocksNums() {
        return {1, 2, 3, 4, 6, 7, 9, 10, 11, 12};
    }

    // --- ユーティリティ関数 ---

    // Pythonの get_adjacent_nodes
    inline std::vector<Position> getAdjacentNodes(Position pos) {
        std::vector<Position> nodes;
        const std::vector<std::pair<int, int>> adjDirections = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}
        };
        for (const auto& dir : adjDirections) {
            Position nextPos = {pos.r + dir.first, pos.c + dir.second};
            if (nextPos.r >= 0 && nextPos.r < ROWS && 
                nextPos.c >= 0 && nextPos.c < COLS) {
                nodes.push_back(nextPos);
            }
        }
        return nodes;
    }

    // Pythonの get_block_num
    inline std::string getBlockNum(Position pos) {
        static const auto coordsMap = getCoordsToBlock();
        auto it = coordsMap.find(pos);
        if (it != coordsMap.end()) {
            return std::to_string(it->second);
        }
        return "不明";
    }

    inline bool isPositionValid(Position pos) {
        return pos.r >= 0 && pos.r < ROWS && pos.c >= 0 && pos.c < COLS;
    }
} // namespace FieldData
#pragma once
#include "Constants.h"

// A*アルゴリズムで使用するノード
struct AstarNode {
    std::shared_ptr<AstarNode> parent; // 親ノードへのポインタ
    State state; // 状態 (位置 + 向き)
    double g = 0.0; // スタートからの実コスト
    double h = 0.0; // ゴールまでの推定コスト
    double f = 0.0; // f = g + h

    AstarNode(std::shared_ptr<AstarNode> p, State s)
        : parent(p), state(s) {}

    bool operator==(const AstarNode& other) const {
        return state == other.state;
    }

    // std::priority_queue (最小ヒープ) で使うための比較
    bool operator>(const AstarNode& other) const {
        return f > other.f;
    }
};

// std::priority_queue のための比較ファンクタ
struct CompareNode {
    bool operator()(const std::shared_ptr<AstarNode>& a, const std::shared_ptr<AstarNode>& b) {
        return *a > *b;
    }
};
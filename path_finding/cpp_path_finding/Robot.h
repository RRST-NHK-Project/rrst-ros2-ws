#pragma once
#include "Constants.h"
#include "Field.h"
#include "AstarPlanner.h"
#include <optional> // C++17 (Pythonの None の代わり)

class Robot {
public:
    Robot(const Field& field);
    void runStrategy(); // Pythonの run_r2_strategy

private:
    // --- 内部状態 ---
    const Field& m_field; // 参照するフィールド(Ground Truth)
    GridMap m_robotGridMap; // R2が認識しているマップ
    std::vector<Position> m_targetList; // 発見したR2 KFS
    int m_collectedKfsCount = 0;
    
    // Pythonの `first_kfs_obligation = None` を C++17 の optional で表現
    std::optional<bool> m_firstKfsObligation; 
    bool m_firstKfsCollected = false;
    State m_currentState;
    AstarPlanner m_planner; // R2はA*plannerを持つ

    const int GOAL_KFS_COUNT = 2;
    const int MAX_ITERATIONS = 100;

    // --- 内部ロジック ---
    
    // Python: 1. スタート位置の動的決定
    std::optional<State> findValidStartPosition();
    
    // Python: scan_surroundings
    void scanSurroundings();
    
    // Python: dummy_pickup_kfs
    void dummyPickupKfs(Position targetNode);

    // Python: Phase 1
    void checkKfsObligation();
    
    // Python: Phase 2 (true: 続行, false: 失敗/中断)
    bool handleObligationPhase();
    
    // Python: Phase 3 (true: 続行, false: 失敗/中断)
    bool handleCollectionPhase();
    
    // Python: Phase 4 (true: 脱出成功, false: 失敗/探索続行)
    bool handleExitPhase();

    // --- 探索ヘルパー ---
    std::set<Position> findPickupLocations(const std::vector<Position>& targets);
    Path findShortestPathTo(const std::set<Position>& goals);
    Path findExplorationPath(bool exitOnly);
};
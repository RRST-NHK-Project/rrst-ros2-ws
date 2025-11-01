#include "Robot.h"
#include "Log.h"
#include <limits> // std::numeric_limits

Robot::Robot(const Field& field)
    : m_field(field) 
{
    // m_robotGridMap を UNKNOWN で初期化
    m_robotGridMap.resize(ROWS, std::vector<KfsType>(COLS, KfsType::UNKNOWN));
}

void Robot::runStrategy() {
    
    // --- 1. スタート位置の動的決定 ---
    std::optional<State> startStateOpt = findValidStartPosition();
    if (!startStateOpt) {
        Log::R2("タスク失敗");
        Log::info(std::string("理由: ブロック 1, 2, 3 がすべて登れる段差 (") + 
                  std::to_string(ALLOWED_STEP_HEIGHT) + "m) を超えています。");
        return;
    }
    
    m_currentState = *startStateOpt;
    Position currentPos = m_currentState.pos;
    
    // 最初のスキャン
    scanSurroundings();
    
    KfsType startStateOnMap = m_robotGridMap[currentPos.r][currentPos.c];
    if (startStateOnMap == KfsType::OBSTACLE_FAKE) {
        Log::R2("タスク失敗");
        Log::pos("理由: スタートブロックが通行不可(FAKE)です。", currentPos);
        return;
    }
    // スタート地点がターゲットだった場合、リストから削除
    auto it = std::find(m_targetList.begin(), m_targetList.end(), currentPos);
    if (it != m_targetList.end()) {
        m_targetList.erase(it);
    }
    m_robotGridMap[currentPos.r][currentPos.c] = KfsType::EMPTY; // 踏んだのでEMPTY
    
    Log::pos(std::string("[R2] ブロック (高さ ") + 
             std::to_string(m_field.getHeight(currentPos)) + "m) に移動...", currentPos);
    Log::R2(std::string("\n--- 戦術開始 (目標: ") + 
            std::to_string(GOAL_KFS_COUNT) + "個のKFS回収) ---");

    // --- 2. メイン探索ループ ---
    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        
        currentPos = m_currentState.pos;

        // [Phase 1] 義務を確認
        if (!m_firstKfsObligation.has_value()) { // Pythonの is None
            checkKfsObligation();
        }
        // [Phase 2] 入口の義務を遂行
        else if (m_firstKfsObligation.value() && !m_firstKfsCollected) {
            if (!handleObligationPhase()) break; // 失敗 or 中断
        }
        // [Phase 3] 2個目のKFS (どこでも可) を回収
        else if (m_collectedKfsCount < GOAL_KFS_COUNT) {
            if (!handleCollectionPhase()) break; // 失敗 or 中断
        }
        // [Phase 4] 脱出
        else {
            if (handleExitPhase()) {
                Log::R2("\n--- [R2] 戦術完了: 脱出しました ---");
                break; // 脱出成功
            }
            // handleExitPhaseがfalseを返した場合、探索モードを実行したのでループ続行
        }
        
        if (iter == MAX_ITERATIONS - 1) {
            Log::R2(std::string("エラー: 最大試行回数 (") + 
                    std::to_string(MAX_ITERATIONS) + ") を超えました。");
        }
    }
}

// --- プライベートメソッドの実装 ---

// Python: 1. スタート位置の動的決定
std::optional<State> Robot::findValidStartPosition() {
    const auto BLOCK_COORDS = FieldData::getBlockCoords();
    for (int blockNum : {1, 2, 3}) {
        Position pos = BLOCK_COORDS.at(blockNum);
        double height = m_field.getHeight(pos);
        if (height <= (ALLOWED_STEP_HEIGHT + EPSILON)) {
            Log::pos(std::string("[R2] スタート地点探索：ブロック (高さ ") + 
                     std::to_string(height) + "m) は登頂可能。", pos);
            return State{pos, 0}; // 向き0 (下) でスタート
        } else {
            Log::pos(std::string("[R2] スタート地点探索：ブロック (高さ ") + 
                     std::to_string(height) + "m) は登頂不可。", pos);
        }
    }
    return std::nullopt; // C++17 (Pythonの None)
}

// Python: scan_surroundings
void Robot::scanSurroundings() {
    Log::pos(std::string("[スキャン] ブロック から周囲をスキャン..."), m_currentState.pos);
    for (Position node : FieldData::getAdjacentNodes(m_currentState.pos)) {
        if (m_robotGridMap[node.r][node.c] == KfsType::UNKNOWN) {
            // Ground Truth (正解マップ) から情報をコピー
            KfsType groundTruthState = m_field.getGroundTruth(node);
            m_robotGridMap[node.r][node.c] = groundTruthState;
            
            if (groundTruthState == KfsType::R2_KFS) {
                // target_list に既になければ追加
                if (std::find(m_targetList.begin(), m_targetList.end(), node) == m_targetList.end()) {
                    m_targetList.push_back(node);
                    Log::pos("    -> 新ターゲット(R2 KFS)発見！", node);
                }
            }
        }
    }
}

// Python: dummy_pickup_kfs
void Robot::dummyPickupKfs(Position targetNode) {
    Log::pos("[回収] 隣接するブロック のKFSを回収しました。", targetNode);
    m_collectedKfsCount++;
    m_robotGridMap[targetNode.r][targetNode.c] = KfsType::EMPTY;
    // targetListから削除
    m_targetList.erase(
        std::remove(m_targetList.begin(), m_targetList.end(), targetNode),
        m_targetList.end()
    );
}

// Python: Phase 1
void Robot::checkKfsObligation() {
    Log::R2("\n戦術：ルール4.4.15（入口ブックの収集義務）を確認中...");
    bool priorityTargetsFound = false;
    for (const auto& pos : FieldData::getEntranceBlocksCoords()) {
        if (m_robotGridMap[pos.r][pos.c] == KfsType::R2_KFS) {
            priorityTargetsFound = true;
            break;
        }
    }
    
    if (priorityTargetsFound) {
        m_firstKfsObligation = true;
        Log::R2("結果：ブロック 1, 2, 3 にR2 KFSを発見");
        Log::R2("戦術：最初のKFSを 1, 2, 3 から回収します。");
    } else {
        bool allEntranceScanned = true;
        for (const auto& pos : FieldData::getEntranceBlocksCoords()) {
            if (m_robotGridMap[pos.r][pos.c] == KfsType::UNKNOWN) {
                allEntranceScanned = false;
                break;
            }
        }
        
        if (allEntranceScanned) {
            m_firstKfsObligation = false;
            m_firstKfsCollected = true; // 義務免除
            Log::R2("結果：ブロック 1, 2, 3 を全てスキャンしましたがR2 KFSはありません。");
            Log::R2("戦術：MFの任意の場所から収集を開始します。");
        } else {
            Log::R2("結果：入口エリアにまだ未知のブロックがあります。探索を続行します。");
            // m_firstKfsObligation は std::nullopt のまま (次回もPhase 1)
        }
    }
}

// Python: Phase 2
bool Robot::handleObligationPhase() {
    Log::pos("\n[R2] 戦術：(義務) 入口のKFSを探索中...", m_currentState.pos);

    // 1. 入口 (1,2,3) のターゲットを探す
    std::vector<Position> priorityTargets;
    const auto ENTRANCE_COORDS = FieldData::getEntranceBlocksCoords();
    std::copy_if(m_targetList.begin(), m_targetList.end(), 
                 std::back_inserter(priorityTargets),
                 [&](const Position& p){ return ENTRANCE_COORDS.count(p); });

    std::set<Position> pickupLocations;
    if (!priorityTargets.empty()) {
        // 回収可能な場所 (入口エリア内の通行可能な隣接マス) を探す
        for (const auto& target : priorityTargets) {
            for (const auto& node : FieldData::getAdjacentNodes(target)) {
                KfsType state = m_robotGridMap[node.r][node.c];
                if (state != KfsType::OBSTACLE_FAKE && state != KfsType::UNKNOWN &&
                    ENTRANCE_COORDS.count(node)) { // 入口エリア内
                    pickupLocations.insert(node);
                }
            }
        }
    }
    
    Path path = findShortestPathTo(pickupLocations);
    
    if (!path.empty()) {
        // 1a. 回収パスを実行
        Log::pos("[R2] [回収モード] ブロック へ移動し、隣のKFSを回収します。", path.back().pos);
        m_currentState = path.back();
        scanSurroundings();
        
        // 隣接するKFSを探して回収
        for (const auto& adjNode : FieldData::getAdjacentNodes(m_currentState.pos)) {
            if (std::find(priorityTargets.begin(), priorityTargets.end(), adjNode) != priorityTargets.end()) {
                dummyPickupKfs(adjNode);
                m_firstKfsCollected = true;
                break; // 1個回収したら終わり
            }
        }
        return true; // 続行
    } 
    else {
        // 1b. 回収パスがない -> 入口エリアを探索
        Log::R2("戦術：入口 (1, 2, 3) に到達可能なKFSがありません。");
        
        std::set<Position> explorableEntranceBlocks;
        for(const auto& pos : ENTRANCE_COORDS) {
            KfsType state = m_robotGridMap[pos.r][pos.c];
             if (state != KfsType::OBSTACLE_FAKE && state != KfsType::UNKNOWN && 
                 pos != m_currentState.pos) {
                explorableEntranceBlocks.insert(pos);
             }
        }

        Path explorationPath = findShortestPathTo(explorableEntranceBlocks);
        
        if (!explorationPath.empty()) {
            Log::pos("[R2] 戦術：入口の未スキャン領域を探すため、ブロック へ移動します。", explorationPath.back().pos);
            m_currentState = explorationPath.back();
            scanSurroundings();
            return true; // 続行
        } else {
             // 1c. 探索もできない -> 妨害戦略と判断
             Log::R2("戦術：入口 (1, 2, 3) の探索できる場所がもうありません。");
             Log::R2("\n--- [R2] タスク失敗 ---");
             Log::info("理由: ブロック 1, 2, 3 をすべてスキャンしましたが、R2 KFSが見つからないか到達できませんでした。");
             return false; // 中断
        }
    }
}

// Python: Phase 3
bool Robot::handleCollectionPhase() {
    Log::pos(std::string("\n[R2] 戦術：") + std::to_string(m_collectedKfsCount + 1) + 
             "個目のKFSを探索中...", m_currentState.pos);

    // 1. 回収モード
    std::set<Position> pickupLocations = findPickupLocations(m_targetList);
    Path path = findShortestPathTo(pickupLocations);

    if (!path.empty()) {
        Log::pos("[R2] [回収モード] ブロック へ移動し、隣のKFSを回収します。", path.back().pos);
        m_currentState = path.back();
        scanSurroundings();
        
        // 隣接するKFSを探して回収
        for (const auto& adjNode : FieldData::getAdjacentNodes(m_currentState.pos)) {
            if (std::find(m_targetList.begin(), m_targetList.end(), adjNode) != m_targetList.end()) {
                dummyPickupKfs(adjNode);
                break; 
            }
        }
        return true; // 続行
    }
    
    // 2. 探索モード
    Log::R2("[探索モード] 既知のターゲットがないか経路がありません。未知のエリアを探索します。");
    Path explorationPath = findExplorationPath(false);

    if (!explorationPath.empty()) {
        Log::pos("[R2] [探索] ブロック へ移動します。", explorationPath.back().pos);
        m_currentState = explorationPath.back();
        scanSurroundings();
        return true; // 続行
    }

    Log::R2("探索モード失敗: 未知エリアへの経路もありません。");
    return false; // 中断
}

// Python: Phase 4
bool Robot::handleExitPhase() {
    Log::R2(std::string("\n--- ステップ3: 脱出シーケンス開始 ---"));
    Log::R2(std::string("目標の") + std::to_string(GOAL_KFS_COUNT) + "個を回収完了。脱出します。");

    // 1. 脱出モード
    std::set<Position> movableExitCells;
    for (const auto& pos : FieldData::getExitBlocksCoords()) {
        KfsType state = m_robotGridMap[pos.r][pos.c];
        if (state != KfsType::OBSTACLE_FAKE && state != KfsType::UNKNOWN) {
            movableExitCells.insert(pos);
        }
    }
    
    Path pathToExit = findShortestPathTo(movableExitCells);

    if (!pathToExit.empty()) {
        Log::pos("[R2] 脱出します。", pathToExit.back().pos);
        m_currentState = pathToExit.back();
        return true; // 脱出成功
    }

    // 2. 出口探索モード
    Log::R2("[探索モード] 既知の出口がないか経路がありません。出口エリアを探索します。");
    Path explorationPath = findExplorationPath(true); // true = 出口付近を優先

    if (!explorationPath.empty()) {
        Log::pos("[R2] [出口探索] ブロック へ移動します。", explorationPath.back().pos);
        m_currentState = explorationPath.back();
        scanSurroundings();
        return false; // 脱出はまだ -> ループ続行
    }

    Log::R2("戦術失敗: 出口エリアへの探索経路が見つかりませんでした。");
    return true; // ループを抜ける (脱出失敗だが処理は終了)
}


// --- ヘルパーメソッド ---

// Pythonの pickup_locations を作るロジック
std::set<Position> Robot::findPickupLocations(const std::vector<Position>& targets) {
    std::set<Position> pickupLocations;
    for (const auto& target : targets) {
        for (const auto& node : FieldData::getAdjacentNodes(target)) {
            KfsType state = m_robotGridMap[node.r][node.c];
            if (state != KfsType::OBSTACLE_FAKE && state != KfsType::UNKNOWN) {
                pickupLocations.insert(node);
            }
        }
    }
    return pickupLocations;
}

// 複数のゴール候補から最短のパスを探す
Path Robot::findShortestPathTo(const std::set<Position>& goals) {
    Path bestPath;
    double shortestPathLen = std::numeric_limits<double>::infinity();

    for (const auto& goalPos : goals) {
        Path path = m_planner.findPath(m_robotGridMap, m_field.getHeightMap(), m_currentState, goalPos);
        if (!path.empty() && path.size() < shortestPathLen) {
            shortestPathLen = path.size();
            bestPath = path;
        }
    }
    return bestPath;
}

// 探索モード (Pythonの explorable_neighbors を探す)
Path Robot::findExplorationPath(bool exitOnly) {
    std::set<Position> explorableGoals; // 未知に隣接する、訪問済みのセル
    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            KfsType state = m_robotGridMap[r][c];
            if (state != KfsType::OBSTACLE_FAKE && state != KfsType::UNKNOWN) {
                for (const auto& node : FieldData::getAdjacentNodes({r, c})) {
                    if (m_robotGridMap[node.r][node.c] == KfsType::UNKNOWN) {
                        explorableGoals.insert({r, c});
                        break;
                    }
                }
            }
        }
    }
    explorableGoals.erase(m_currentState.pos); // 現在地は除く

    if (explorableGoals.empty()) {
        Log::R2("マップをすべて探索しました。");
        return {};
    }
    
    if (exitOnly) {
        // 出口(Row 3)に近い探索ゴールを優先
        std::set<Position> exitExplorationGoals;
        std::copy_if(explorableGoals.begin(), explorableGoals.end(),
                     std::inserter(exitExplorationGoals, exitExplorationGoals.begin()),
                     [](const Position& p){ return p.r == (ROWS - 1); }); // 最後の行
        
        if (!exitExplorationGoals.empty()) {
            return findShortestPathTo(exitExplorationGoals);
        }
    }
    
    // 通常の探索 (または出口付近に探索ゴールがなかった場合)
    return findShortestPathTo(explorableGoals);
}
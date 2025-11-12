#include "Field.h"
#include "Log.h"
#include <algorithm> // std::shuffle, std::remove, std::copy_if
#include <iterator>  // std::back_inserter
#include <chrono>    // 実行のたびにシードを変える

Field::Field(bool forceTrap)
    // m_rng を現在の時刻で初期化
    : m_rng(std::chrono::high_resolution_clock::now().time_since_epoch().count())
{
    Log::info("--- [シミュレーション] 相手チームがKFSを配置中... ---");
    if (forceTrap) {
        Log::info("--- [シミュレーション] 妨害戦略：1,2,3 に R2 KFS を配置しない ---");
    }
    
    // マップを初期化
    m_groundTruthMap.resize(ROWS, std::vector<KfsType>(COLS, KfsType::EMPTY));
    
    initializeMaps(forceTrap);

    Log::info("--- [シミュレーション] 配置完了 ---");
}

KfsType Field::getGroundTruth(Position p) const {
    if (FieldData::isPositionValid(p)) {
        return m_groundTruthMap[p.r][p.c];
    }
    return KfsType::UNKNOWN; // 範囲外
}

double Field::getHeight(Position p) const {
    if (FieldData::isPositionValid(p)) {
        return m_heightMap[p.r][p.c];
    }
    return 0.0;
}

const HeightMap& Field::getHeightMap() const {
    return m_heightMap;
}

const GridMap& Field::getGroundTruthMap() const {
    return m_groundTruthMap;
}

// Pythonの opponent_places_kfs_and_heights のロジック
void Field::initializeMaps(bool forceTrap) {
    // 高さマップを固定値で設定
    m_heightMap = {
        {HEIGHT_400, HEIGHT_200, HEIGHT_400}, // 1(0.4), 2(0.2), 3(0.4)
        {HEIGHT_200, HEIGHT_400, HEIGHT_600}, // 4(0.2), 5(0.4), 6(0.6)
        {HEIGHT_400, HEIGHT_600, HEIGHT_400}, // 7(0.4), 8(0.6), 9(0.4)
        {HEIGHT_200, HEIGHT_400, HEIGHT_200}  // 10(0.2), 11(0.4), 12(0.2)
    };
    
    // 利用可能なブロック番号 (1-12)
    std::vector<int> availableBlocks(12);
    std::iota(availableBlocks.begin(), availableBlocks.end(), 1); // 1から12まで
    
    const auto BLOCK_COORDS = FieldData::getBlockCoords();
    const auto BOUNDARY_BLOCKS_NUMS = FieldData::getBoundaryBlocksNums();
    const auto ENTRANCE_BLOCKS_NUMS = std::set<int>{1, 2, 3};

    // --- R1 KFS (2) ---
    std::vector<int> r1PlacementPool;
    std::copy_if(availableBlocks.begin(), availableBlocks.end(), 
                 std::back_inserter(r1PlacementPool),
                 [&](int b){ return BOUNDARY_BLOCKS_NUMS.count(b); });

    std::vector<int> r1Choices;
    if (forceTrap) {
        // ... (force_trap時のR1配置ロジック) ...
        // (簡略化のため、Python版のロジックをそのまま適用)
        std::vector<int> trapBlocks;
        std::copy_if(r1PlacementPool.begin(), r1PlacementPool.end(), 
                     std::back_inserter(trapBlocks),
                     [&](int b){ return ENTRANCE_BLOCKS_NUMS.count(b); });
        r1Choices = trapBlocks;
        // (残りをシャッフルして追加するロジック)
        std::shuffle(r1PlacementPool.begin(), r1PlacementPool.end(), m_rng);
        for(int b : r1PlacementPool) {
            if (r1Choices.size() >= 3) break;
            if (std::find(r1Choices.begin(), r1Choices.end(), b) == r1Choices.end()) {
                r1Choices.push_back(b);
            }
        }
    } else {
        std::shuffle(r1PlacementPool.begin(), r1PlacementPool.end(), m_rng);
        r1Choices.assign(r1PlacementPool.begin(), r1PlacementPool.begin() + 3);
    }
    
    for (int b : r1Choices) {
        Position pos = BLOCK_COORDS.at(b);
        m_groundTruthMap[pos.r][pos.c] = KfsType::OBSTACLE_R1;
        // availableBlocks から b を削除 (erase-removeイディオム)
        availableBlocks.erase(
            std::remove(availableBlocks.begin(), availableBlocks.end(), b),
            availableBlocks.end()
        );
    }

    // --- Fake KFS (3) ---
    std::vector<int> fakePlacementPool;
    std::copy_if(availableBlocks.begin(), availableBlocks.end(), 
                 std::back_inserter(fakePlacementPool),
                 [&](int b){ return !ENTRANCE_BLOCKS_NUMS.count(b); });
    
    std::shuffle(fakePlacementPool.begin(), fakePlacementPool.end(), m_rng);
    int fakeChoice = fakePlacementPool[0];
    Position fakePos = BLOCK_COORDS.at(fakeChoice);
    m_groundTruthMap[fakePos.r][fakePos.c] = KfsType::OBSTACLE_FAKE;
    availableBlocks.erase(
        std::remove(availableBlocks.begin(), availableBlocks.end(), fakeChoice),
        availableBlocks.end()
    );

    // --- R2 KFS (1) ---
    std::vector<int> r2PlacementPool = availableBlocks;
    if (forceTrap) {
        r2PlacementPool.clear();
        std::copy_if(availableBlocks.begin(), availableBlocks.end(), 
                     std::back_inserter(r2PlacementPool),
                     [&](int b){ return !ENTRANCE_BLOCKS_NUMS.count(b); });
    }
    std::shuffle(r2PlacementPool.begin(), r2PlacementPool.end(), m_rng);
    std::vector<int> r2Choices(r2PlacementPool.begin(), r2PlacementPool.begin() + 4);
    
    for (int b : r2Choices) {
        Position pos = BLOCK_COORDS.at(b);
        m_groundTruthMap[pos.r][pos.c] = KfsType::R2_KFS;
        availableBlocks.erase(
            std::remove(availableBlocks.begin(), availableBlocks.end(), b),
            availableBlocks.end()
        );
    }
}
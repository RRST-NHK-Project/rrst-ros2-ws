#pragma once
#include "Constants.h"
#include <random> // C++の乱数生成

class Field {
public:
    // Pythonの __init__ と opponent_places... を兼ねる
    Field(bool forceTrap);

    // ゲッター
    KfsType getGroundTruth(Position p) const;
    double getHeight(Position p) const;
    const HeightMap& getHeightMap() const; // A*に渡すため
    const GridMap& getGroundTruthMap() const; // Robotに渡すため

private:
    void initializeMaps(bool forceTrap);
    
    GridMap m_groundTruthMap;
    HeightMap m_heightMap;

    // 乱数生成器 (Pythonの random.sample/choice の代わり)
    std::mt19937 m_rng; 
};
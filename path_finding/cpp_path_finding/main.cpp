#include "Constants.h"
#include "Field.h"
#include "Robot.h"
#include "Log.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr int kRows = 4;
constexpr int kCols = 3;

Position toPosition(const int cellNumber) {
    const int zeroBased = cellNumber - 1;
    return Position{zeroBased / kCols, zeroBased % kCols};
}

int toCellNumber(const Position& pos) {
    return pos.r * kCols + pos.c + 1;
}

bool isValidCellNumber(const int cell) {
    return cell >= 1 && cell <= (kRows * kCols);
}

std::vector<int> parseCellsCsv(const std::string& csv) {
    std::vector<int> cells;
    std::stringstream ss(csv);
    std::string token;

    while (std::getline(ss, token, ',')) {
        if (token.empty()) {
            continue;
        }
        try {
            const int value = std::stoi(token);
            if (isValidCellNumber(value)) {
                cells.push_back(value);
            }
        } catch (...) {
            continue;
        }
    }
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    return cells;
}

std::vector<Position> getNeighbors(const Position& pos) {
    std::vector<Position> neighbors;
    for (const auto& d : FieldData::DIRECTIONS) {
        Position next{pos.r + d.first, pos.c + d.second};
        if (FieldData::isPositionValid(next)) {
            neighbors.push_back(next);
        }
    }
    return neighbors;
}

std::vector<int> shortestPathCells(const int startCell, const int goalCell) {
    if (startCell == goalCell) {
        return {startCell};
    }

    const Position start = toPosition(startCell);
    const Position goal = toPosition(goalCell);

    std::queue<Position> q;
    std::set<Position> visited;
    std::map<Position, Position> parent;

    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        Position current = q.front();
        q.pop();

        if (current == goal) {
            break;
        }

        for (const auto& next : getNeighbors(current)) {
            if (visited.count(next)) {
                continue;
            }
            visited.insert(next);
            parent[next] = current;
            q.push(next);
        }
    }

    if (!visited.count(goal)) {
        return {startCell};
    }

    std::vector<int> path;
    Position cursor = goal;
    path.push_back(toCellNumber(cursor));

    while (cursor != start) {
        cursor = parent[cursor];
        path.push_back(toCellNumber(cursor));
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> solveVisitAllCellsOptimal(const int startCell, const std::vector<int>& requiredCells) {
    std::vector<int> targets = requiredCells;
    std::sort(targets.begin(), targets.end());
    targets.erase(std::remove(targets.begin(), targets.end(), startCell), targets.end());

    if (targets.empty()) {
        return {startCell};
    }

    const int n = static_cast<int>(targets.size());
    const int maxMask = 1 << n;
    const int kInf = std::numeric_limits<int>::max() / 4;

    std::vector<int> startToTarget(n, kInf);
    std::vector<std::vector<int>> targetToTarget(n, std::vector<int>(n, kInf));

    for (int i = 0; i < n; ++i) {
        startToTarget[i] = static_cast<int>(shortestPathCells(startCell, targets[i]).size()) - 1;
        for (int j = 0; j < n; ++j) {
            targetToTarget[i][j] = (i == j)
                ? 0
                : static_cast<int>(shortestPathCells(targets[i], targets[j]).size()) - 1;
        }
    }

    std::vector<std::vector<int>> dp(maxMask, std::vector<int>(n, kInf));
    std::vector<std::vector<int>> parent(maxMask, std::vector<int>(n, -1));

    for (int i = 0; i < n; ++i) {
        dp[1 << i][i] = startToTarget[i];
    }

    for (int mask = 1; mask < maxMask; ++mask) {
        for (int last = 0; last < n; ++last) {
            if ((mask & (1 << last)) == 0 || dp[mask][last] >= kInf) {
                continue;
            }
            for (int next = 0; next < n; ++next) {
                if (mask & (1 << next)) {
                    continue;
                }
                const int nextMask = mask | (1 << next);
                const int nextCost = dp[mask][last] + targetToTarget[last][next];
                if (nextCost < dp[nextMask][next]) {
                    dp[nextMask][next] = nextCost;
                    parent[nextMask][next] = last;
                }
            }
        }
    }

    const int fullMask = maxMask - 1;
    int endIndex = -1;
    int bestCost = kInf;
    for (int i = 0; i < n; ++i) {
        if (dp[fullMask][i] < bestCost) {
            bestCost = dp[fullMask][i];
            endIndex = i;
        }
    }

    if (endIndex == -1) {
        return {startCell};
    }

    std::vector<int> orderIndices;
    int mask = fullMask;
    int current = endIndex;
    while (current != -1) {
        orderIndices.push_back(current);
        const int prev = parent[mask][current];
        mask &= ~(1 << current);
        current = prev;
    }
    std::reverse(orderIndices.begin(), orderIndices.end());

    std::vector<int> fullPath{startCell};
    int cursor = startCell;
    for (const int index : orderIndices) {
        const int nextCell = targets[index];
        const std::vector<int> segment = shortestPathCells(cursor, nextCell);
        if (segment.size() > 1) {
            fullPath.insert(fullPath.end(), std::next(segment.begin()), segment.end());
        }
        cursor = nextCell;
    }

    return fullPath;
}

void printPathCsv(const std::vector<int>& path) {
    std::cout << "PATH:";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) {
            std::cout << ",";
        }
        std::cout << path[i];
    }
    std::cout << std::endl;
}

bool tryRunMarkerPathCli(const int argc, char* argv[]) {
    if (argc < 3 || std::string(argv[1]) != "--solve-markers") {
        return false;
    }

    const std::vector<int> requiredCells = parseCellsCsv(argv[2]);
    const std::vector<int> path = solveVisitAllCellsOptimal(1, requiredCells);
    printPathCsv(path);
    return true;
}

} // namespace

int main(int argc, char* argv[]) {
    if (tryRunMarkerPathCli(argc, argv)) {
        return 0;
    }

    // ----------------------------------------------------
    // シミュレーションのシナリオを選択
    // true = 妨害戦略 (1,2,3にR2 KFSなし)
    // false = 通常 (ランダム配置)
    constexpr bool RUN_TRAP_SCENARIO = true;
    // ----------------------------------------------------

    // 1. フィールドを初期化
    Field field(RUN_TRAP_SCENARIO);

    // 2. ロボットを初期化
    Robot robot(field);

    // 3. 戦術を実行
    robot.runStrategy();

    Log::info("\nシミュレーション完了。");
    
    return 0;
}
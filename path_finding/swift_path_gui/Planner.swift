import Foundation

enum MarkerType: String, CaseIterable, Hashable {
    case logo
    case circle
    case cross

    var symbol: String {
        switch self {
        case .logo: return "logo"
        case .circle: return "❍"
        case .cross: return "✖"
        }
    }

    var maxCount: Int {
        switch self {
        case .logo: return 3
        case .circle: return 4
        case .cross: return 1
        }
    }
}

struct GridPosition: Hashable {
    let row: Int
    let col: Int
}

enum MoveDirection {
    case up
    case down
    case left
    case right

    var arrowSymbol: String {
        switch self {
        case .up: return "↑"
        case .down: return "↓"
        case .left: return "←"
        case .right: return "→"
        }
    }
}

struct ArrowStep: Hashable {
    let cellNumber: Int
    let direction: MoveDirection
}

final class CppPathFinderBridge {
    private let executablePath = "/home/ubuntu/rrst-ros2-ws/path_finding/cpp_path_finding/cpp_path_finding"

    func solve(requiredCells: [Int]) -> [Int]? {
#if os(macOS)
        let csv = requiredCells.sorted().map(String.init).joined(separator: ",")
        let process = Process()
        process.executableURL = URL(fileURLWithPath: executablePath)
        process.arguments = ["--solve-markers", csv]

        let pipe = Pipe()
        process.standardOutput = pipe
        process.standardError = pipe

        do {
            try process.run()
            process.waitUntilExit()
            guard process.terminationStatus == 0 else { return nil }
            let data = pipe.fileHandleForReading.readDataToEndOfFile()
            guard let output = String(data: data, encoding: .utf8) else { return nil }
            return parsePath(from: output)
        } catch {
            return nil
        }
#else
        return nil
#endif
    }

    private func parsePath(from output: String) -> [Int]? {
        guard let line = output.split(whereSeparator: \.isNewline).first(where: { $0.hasPrefix("PATH:") }) else {
            return nil
        }
        let csv = line.replacingOccurrences(of: "PATH:", with: "")
        let cells = csv.split(separator: ",").compactMap { Int($0) }
        return cells.isEmpty ? nil : cells
    }
}

final class GridPathPlanner {
    private let rows = 4
    private let cols = 3

    func solveOptimalPath(startCell: Int = 1, requiredCells: [Int]) -> [Int] {
        let uniqueTargets = Array(Set(requiredCells)).sorted()
        let targets = uniqueTargets.filter { $0 != startCell }
        if targets.isEmpty { return [startCell] }

        let n = targets.count
        let startToTarget = targets.map { shortestPathCells(from: startCell, to: $0).count - 1 }
        var targetToTarget = Array(repeating: Array(repeating: 0, count: n), count: n)

        for i in 0..<n {
            for j in 0..<n {
                targetToTarget[i][j] = i == j ? 0 : shortestPathCells(from: targets[i], to: targets[j]).count - 1
            }
        }

        var dp: [Int: [Int: Int]] = [:]
        var parent: [Int: [Int: Int]] = [:]

        for i in 0..<n {
            let mask = 1 << i
            dp[mask, default: [:]][i] = startToTarget[i]
            parent[mask, default: [:]][i] = -1
        }

        for mask in 1..<(1 << n) {
            for last in 0..<n {
                guard (mask & (1 << last)) != 0 else { continue }
                guard let cost = dp[mask]?[last] else { continue }

                for next in 0..<n {
                    guard (mask & (1 << next)) == 0 else { continue }
                    let nextMask = mask | (1 << next)
                    let nextCost = cost + targetToTarget[last][next]
                    if nextCost < (dp[nextMask]?[next] ?? Int.max) {
                        dp[nextMask, default: [:]][next] = nextCost
                        parent[nextMask, default: [:]][next] = last
                    }
                }
            }
        }

        let fullMask = (1 << n) - 1
        var bestEnd = -1
        var bestCost = Int.max
        for i in 0..<n {
            if let cost = dp[fullMask]?[i], cost < bestCost {
                bestCost = cost
                bestEnd = i
            }
        }
        if bestEnd == -1 { return [startCell] }

        var order: [Int] = []
        var mask = fullMask
        var current = bestEnd
        while current != -1 {
            order.append(current)
            let prev = parent[mask]?[current] ?? -1
            mask &= ~(1 << current)
            current = prev
        }
        order.reverse()

        var fullPath = [startCell]
        var cursor = startCell
        for idx in order {
            let target = targets[idx]
            let segment = shortestPathCells(from: cursor, to: target)
            fullPath.append(contentsOf: segment.dropFirst())
            cursor = target
        }
        return fullPath
    }

    func makeArrowSteps(from cellPath: [Int]) -> [ArrowStep] {
        guard cellPath.count > 1 else { return [] }
        var steps: [ArrowStep] = []
        for i in 0..<(cellPath.count - 1) {
            let from = cellPath[i]
            let to = cellPath[i + 1]
            if let direction = directionBetween(from: from, to: to) {
                steps.append(ArrowStep(cellNumber: from, direction: direction))
            }
        }
        return steps
    }

    private func shortestPathCells(from start: Int, to goal: Int) -> [Int] {
        if start == goal { return [start] }
        let startPos = toPosition(cell: start)
        let goalPos = toPosition(cell: goal)

        var queue = [startPos]
        var head = 0
        var visited: Set<GridPosition> = [startPos]
        var parent: [GridPosition: GridPosition] = [:]

        while head < queue.count {
            let current = queue[head]
            head += 1
            if current == goalPos { break }

            for next in neighbors(of: current) {
                guard !visited.contains(next) else { continue }
                visited.insert(next)
                parent[next] = current
                queue.append(next)
            }
        }

        guard visited.contains(goalPos) else { return [start] }

        var path = [goalPos]
        var cursor = goalPos
        while let p = parent[cursor] {
            path.append(p)
            cursor = p
        }
        return path.reversed().map { toCellNumber(position: $0) }
    }

    private func neighbors(of position: GridPosition) -> [GridPosition] {
        let cands = [
            GridPosition(row: position.row + 1, col: position.col),
            GridPosition(row: position.row - 1, col: position.col),
            GridPosition(row: position.row, col: position.col + 1),
            GridPosition(row: position.row, col: position.col - 1)
        ]
        return cands.filter { $0.row >= 0 && $0.row < rows && $0.col >= 0 && $0.col < cols }
    }

    private func directionBetween(from: Int, to: Int) -> MoveDirection? {
        let a = toPosition(cell: from)
        let b = toPosition(cell: to)
        if b.row == a.row + 1 && b.col == a.col { return .down }
        if b.row == a.row - 1 && b.col == a.col { return .up }
        if b.row == a.row && b.col == a.col + 1 { return .right }
        if b.row == a.row && b.col == a.col - 1 { return .left }
        return nil
    }

    private func toPosition(cell: Int) -> GridPosition {
        let z = cell - 1
        return GridPosition(row: z / cols, col: z % cols)
    }

    private func toCellNumber(position: GridPosition) -> Int {
        position.row * cols + position.col + 1
    }
}

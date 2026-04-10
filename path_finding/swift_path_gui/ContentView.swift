import SwiftUI

struct ContentView: View {
    private enum Theme {
        static let bgBase = Color(red: 0.95, green: 0.95, blue: 0.95)
        static let bgSoft = Color(red: 0.89, green: 0.88, blue: 0.88)
        static let accent = Color(red: 0.80, green: 0.11, blue: 0.08)
        static let accentStrong = Color(red: 0.56, green: 0.07, blue: 0.05)
        static let textMain = Color(red: 0.10, green: 0.10, blue: 0.10)
        static let textSub = Color(red: 0.34, green: 0.34, blue: 0.34)
    }

    @State private var selectedMarker: MarkerType? = nil
    @State private var placedMarkers: [Int: MarkerType] = [:]
    @State private var warningMessage = ""
    @State private var isShowingWarning = false
    @State private var animateBackground = false

    @State private var plannedCells: [Int] = []
    @State private var arrowSteps: [ArrowStep] = []
    @State private var revealedStepCount = 0

    private let planner = GridPathPlanner()
    private let cppBridge = CppPathFinderBridge()

    private let gridColumns = [
        GridItem(.flexible(), spacing: 12),
        GridItem(.flexible(), spacing: 12),
        GridItem(.flexible(), spacing: 12)
    ]

    private var totalPlacedCount: Int { placedMarkers.count }

    private var countByType: [MarkerType: Int] {
        Dictionary(grouping: placedMarkers.values, by: { $0 }).mapValues { $0.count }
    }

    var body: some View {
        GeometryReader { geo in
            ZStack {
                LinearGradient(
                    colors: [Theme.bgBase, Theme.bgSoft, Theme.bgBase],
                    startPoint: .topLeading,
                    endPoint: .bottomTrailing
                )
                .ignoresSafeArea()

                Circle()
                    .fill(Theme.accent.opacity(0.22))
                    .frame(width: 280, height: 280)
                    .blur(radius: 12)
                    .offset(
                        x: animateBackground ? geo.size.width * 0.22 : geo.size.width * 0.28,
                        y: animateBackground ? -geo.size.height * 0.28 : -geo.size.height * 0.20
                    )

                Circle()
                    .fill(Color(red: 0.10, green: 0.55, blue: 0.84).opacity(0.24))
                    .frame(width: 230, height: 230)
                    .blur(radius: 14)
                    .offset(
                        x: animateBackground ? -geo.size.width * 0.32 : -geo.size.width * 0.24,
                        y: animateBackground ? geo.size.height * 0.30 : geo.size.height * 0.22
                    )

                VStack(spacing: 14) {
                    headerView

                    HStack(spacing: 16) {
                        boardView
                            .frame(width: geo.size.width * 0.60)

                        controlPanelView
                            .frame(width: geo.size.width * 0.34)
                    }
                }
                .padding(16)
                .background(
                    RoundedRectangle(cornerRadius: 18)
                        .fill(Color.white.opacity(0.90))
                        .overlay(
                            RoundedRectangle(cornerRadius: 18)
                                .stroke(Color.black.opacity(0.14), lineWidth: 1)
                        )
                        .shadow(color: .black.opacity(0.16), radius: 22, x: 0, y: 12)
                )
                .padding(8)
            }
        }
        .onAppear {
            withAnimation(.easeInOut(duration: 7).repeatForever(autoreverses: true)) {
                animateBackground = true
            }
        }
        .alert("警告", isPresented: $isShowingWarning) {
            Button("OK", role: .cancel) {}
        } message: {
            Text(warningMessage)
        }
    }

    private var headerView: some View {
        HStack(spacing: 12) {
            ZStack {
                RoundedRectangle(cornerRadius: 10)
                    .fill(Theme.accent.opacity(0.14))
                Image(systemName: "atom")
                    .font(.system(size: 24, weight: .bold))
                    .foregroundStyle(Theme.accent)
            }
            .frame(width: 48, height: 48)

            VStack(alignment: .leading, spacing: 2) {
                Text("Path Finding Console")
                    .font(.system(size: 23, weight: .heavy, design: .rounded))
                    .foregroundStyle(Theme.textMain)
                Text("8個配置後に最適経路を矢印で表示")
                    .font(.system(size: 12, weight: .medium))
                    .foregroundStyle(Theme.textSub)
            }

            Spacer(minLength: 0)

            Text(totalPlacedCount == 8 ? "READY" : "SETUP")
                .font(.system(size: 12, weight: .bold, design: .monospaced))
                .padding(.horizontal, 10)
                .padding(.vertical, 6)
                .background(totalPlacedCount == 8 ? Color.green.opacity(0.18) : Theme.accent.opacity(0.14))
                .clipShape(Capsule())
                .foregroundStyle(totalPlacedCount == 8 ? Color.green : Theme.accentStrong)
        }
    }

    private var boardView: some View {
        ZStack {
            RoundedRectangle(cornerRadius: 16)
                .fill(Color.white.opacity(0.78))
                .overlay(
                    RoundedRectangle(cornerRadius: 16)
                        .stroke(Color.black.opacity(0.10), lineWidth: 1)
                )

            LazyVGrid(columns: gridColumns, spacing: 12) {
                ForEach(1...12, id: \.self) { cell in
                    cellView(cell)
                }
            }
            .padding(12)
        }
    }

    private var controlPanelView: some View {
        VStack(spacing: 18) {
            VStack(spacing: 12) {
                markerButton(.logo)
                markerButton(.circle)
                markerButton(.cross)
            }
            .padding(10)
            .background(Color.white.opacity(0.70))
            .clipShape(RoundedRectangle(cornerRadius: 14))

            Button {
                if totalPlacedCount == 8 {
                    runPathFindingAfterPlacement()
                } else {
                    showWarning("8個配置すると経路探索を開始できます。")
                }
            } label: {
                Text("経路を再生")
                    .font(.system(size: 22, weight: .heavy))
                    .foregroundStyle(Color.white)
                    .frame(maxWidth: .infinity)
                    .frame(height: 62)
                    .background(
                        LinearGradient(
                            colors: [Theme.accent, Theme.accentStrong],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        )
                    )
                    .clipShape(RoundedRectangle(cornerRadius: 14))
            }

            Button {
                clearAll()
            } label: {
                Text("リセット")
                    .font(.system(size: 22, weight: .bold))
                    .foregroundStyle(Color.white)
                    .frame(maxWidth: .infinity)
                    .frame(height: 58)
                    .background(
                        LinearGradient(
                            colors: [Color(red: 0.32, green: 0.42, blue: 0.50), Color(red: 0.22, green: 0.30, blue: 0.36)],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        )
                    )
                    .clipShape(RoundedRectangle(cornerRadius: 14))
            }

            statusView
            Spacer(minLength: 0)
        }
    }

    private var statusView: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("配置: \(totalPlacedCount)/8")
            Text("logo: \(countByType[.logo] ?? 0)/3")
            Text("❍: \(countByType[.circle] ?? 0)/4")
            Text("✖: \(countByType[.cross] ?? 0)/1")
            Text(totalPlacedCount == 8 ? "最適経路計算済み" : "マークを配置してください")
                .foregroundStyle(totalPlacedCount == 8 ? Color.green : Theme.textSub)
        }
        .font(.system(size: 19, weight: .semibold, design: .rounded))
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(12)
        .background(Color.white.opacity(0.72))
        .clipShape(RoundedRectangle(cornerRadius: 14))
    }

    private func markerButton(_ marker: MarkerType) -> some View {
        let isSelected = selectedMarker == marker

        return Button {
            selectedMarker = marker
        } label: {
            HStack {
                Text(marker.symbol)
                    .font(.system(size: marker == .logo ? 28 : 40, weight: .bold, design: .rounded))
                Spacer(minLength: 8)
                Text("\(countByType[marker] ?? 0)/\(marker.maxCount)")
                    .font(.system(size: 17, weight: .heavy, design: .rounded))
            }
            .padding(.horizontal, 16)
            .foregroundStyle(isSelected ? Color.white : Theme.textMain)
            .frame(maxWidth: .infinity)
            .frame(height: 84)
            .background(
                isSelected
                    ? AnyView(LinearGradient(colors: [Theme.accent, Theme.accentStrong], startPoint: .topLeading, endPoint: .bottomTrailing))
                    : AnyView(Color.white)
            )
            .clipShape(RoundedRectangle(cornerRadius: 16))
            .overlay(
                RoundedRectangle(cornerRadius: 16)
                    .stroke(isSelected ? Color.clear : Color.black.opacity(0.12), lineWidth: 1)
            )
            .shadow(color: .black.opacity(0.10), radius: 5, x: 0, y: 2)
        }
    }

    private func cellView(_ cell: Int) -> some View {
        let marker = placedMarkers[cell]
        let arrow = arrowForCell(cell)

        return Button {
            placeMarker(at: cell)
        } label: {
            ZStack {
                RoundedRectangle(cornerRadius: 12)
                    .fill(Color(red: 0.98, green: 0.98, blue: 0.99))
                    .overlay(
                        RoundedRectangle(cornerRadius: 12)
                            .stroke(Color.black.opacity(0.15), lineWidth: 1.5)
                    )

                VStack(spacing: 3) {
                    Text("\(cell)")
                        .font(.system(size: 15, weight: .semibold))
                        .foregroundStyle(Theme.textSub)

                    Text(marker?.symbol ?? "")
                        .font(.system(size: marker == .logo ? 20 : 30, weight: .bold, design: .rounded))
                        .foregroundStyle(Theme.textMain)
                        .frame(height: 34)

                    Text(arrow?.direction.arrowSymbol ?? "")
                        .font(.system(size: 24, weight: .bold))
                        .foregroundStyle(Theme.accent)
                        .opacity(arrow != nil ? 1 : 0)
                        .scaleEffect(arrow != nil ? 1.0 : 0.6)
                        .animation(.spring(response: 0.35, dampingFraction: 0.75), value: revealedStepCount)
                }
            }
            .frame(height: 82)
        }
        .buttonStyle(.plain)
    }

    private func placeMarker(at cell: Int) {
        guard let selectedMarker else {
            showWarning("先に右側のlogo / ❍ / ✖ ボタンを選択してください。")
            return
        }

        if totalPlacedCount >= 8, placedMarkers[cell] == nil {
            showWarning("8個すべて配置済みです。リセットして再配置してください。")
            return
        }

        if selectedMarker == .circle && (cell == 5 || cell == 8) {
            showWarning("❍は5番と8番に配置できません。")
            return
        }

        let currentCount = countByType[selectedMarker] ?? 0
        if placedMarkers[cell] != selectedMarker && currentCount >= selectedMarker.maxCount {
            showWarning("\(selectedMarker.symbol)は最大\(selectedMarker.maxCount)個までです。")
            return
        }

        if let current = placedMarkers[cell], current != selectedMarker {
            let newCount = countByType[selectedMarker] ?? 0
            if newCount >= selectedMarker.maxCount {
                showWarning("\(selectedMarker.symbol)は最大\(selectedMarker.maxCount)個までです。")
                return
            }
        }

        placedMarkers[cell] = selectedMarker

        if totalPlacedCount == 8 {
            runPathFindingAfterPlacement()
        }
    }

    private func runPathFindingAfterPlacement() {
        let required = placedMarkers.keys.sorted()
        if let cppPath = cppBridge.solve(requiredCells: required), !cppPath.isEmpty {
            plannedCells = cppPath
        } else {
            plannedCells = planner.solveOptimalPath(startCell: 1, requiredCells: required)
        }
        arrowSteps = planner.makeArrowSteps(from: plannedCells)
        revealedStepCount = 0
        animateArrows()
    }

    private func animateArrows() {
        guard !arrowSteps.isEmpty else { return }

        for i in arrowSteps.indices {
            let delay = Double(i) * 0.35
            DispatchQueue.main.asyncAfter(deadline: .now() + delay) {
                withAnimation(.easeInOut(duration: 0.28)) {
                    revealedStepCount = i + 1
                }
            }
        }
    }

    private func arrowForCell(_ cell: Int) -> ArrowStep? {
        let visible = arrowSteps.prefix(revealedStepCount)
        return visible.last(where: { $0.cellNumber == cell })
    }

    private func clearAll() {
        selectedMarker = nil
        placedMarkers.removeAll()
        plannedCells.removeAll()
        arrowSteps.removeAll()
        revealedStepCount = 0
    }

    private func showWarning(_ text: String) {
        warningMessage = text
        isShowingWarning = true
    }
}

#Preview {
    ContentView()
}

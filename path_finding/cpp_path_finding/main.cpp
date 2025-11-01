#include "Constants.h"
#include "Field.h"
#include "Robot.h"
#include "Log.h"

int main() {
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
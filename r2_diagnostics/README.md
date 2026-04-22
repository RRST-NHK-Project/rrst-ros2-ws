# R2 Diagnostics Package

R2ロボットの自己診断パッケージです。ハードウェアコンポーネント、センサー、通信の健全性をチェックします。

## 機能

### 1. ハードウェア診断
- **モータテスト** (MD1-MD8): モーター応答確認
- **サーボテスト** (SERVO1-SERVO8): サーボ応答確認
- **ソレノイドテスト** (TR1-TR7): ソレノイド応答確認

### 2. センサー診断
- **エンコーダ監視**: 回転値の読み取り確認
- **スイッチ監視**: マイクロスイッチ状態確認
- **IMU/LiDAR対応**: 拡張可能な構造

### 3. 出力形式
- **ターミナルレポート**: CLI出力
- **ROSトピック**: DiagnosticArray形式で配信
- **ファイル保存**: JSON/YAML形式のレポート
- **UI統合**: rrst-consoleへの統合可能

### 4. 起動方法
- コマンドライン実行
- ROS2サービス呼び出し
- ジョイスティック（LB+RB）トリガー
- ロボット起動時に自動実行

## インストール

```bash
cd ~/ros2_ws
colcon build --packages-select r2_diagnostics
source install/setup.bash
```

## 使用方法

### 1. CLI実行（手動実行）

#### ハードウェア診断のみ実行
```bash
ros2 run r2_diagnostics run_diagnostics hardware
```

#### センサー診断のみ実行
```bash
ros2 run r2_diagnostics run_diagnostics sensors
```

#### 完全診断実行
```bash
ros2 run r2_diagnostics run_diagnostics full
```

#### レポート生成（JSON）
```bash
ros2 run r2_diagnostics run_diagnostics report -o diagnostics_report.json
```

#### レポート生成（YAML）
```bash
ros2 run r2_diagnostics run_diagnostics report -o diagnostics_report.yaml
```

#### 詳細出力
```bash
ros2 run r2_diagnostics run_diagnostics full -v
```

### 2. ROS2ノード実行

#### ノード起動
```bash
ros2 launch r2_diagnostics diagnostics.launch.py
```

#### サービス呼び出し

完全診断を実行：
```bash
ros2 service call /r2/run_diagnostics std_srvs/srv/Trigger
```

ハードウェア診断のみ実行：
```bash
ros2 service call /r2/run_hardware_diagnostics std_srvs/srv/Trigger
```

診断レポート保存：
```bash
ros2 service call /r2/save_diagnostics_report std_srvs/srv/Trigger
```

### 3. ロボット起動時に自動実行

`bringup.launch.py` に診断ノードを追加：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ... other nodes ...
    
    diagnostics_node = Node(
        package='r2_diagnostics',
        executable='diagnostics_node',
        name='r2_diagnostics',
        output='log',  # ログのみ、コンソール出力なし
    )
    
    return LaunchDescription([
        # ... other nodes ...
        diagnostics_node,
    ])
```

### 4. UIボタンから実行

`App.js` に診断ボタンを追加：

```javascript
// 診断タブを追加
const runDiagnostics = async () => {
  if (!rosRef.current?.ros?.isConnected) return;
  
  const client = new ROSLIB.Service({
    ros: rosRef.current.ros,
    name: '/r2/run_diagnostics',
    serviceType: 'std_srvs/srv/Trigger',
  });
  
  const request = new ROSLIB.ServiceRequest({});
  client.callService(request, (result) => {
    console.log('Diagnostics result:', result);
  });
};
```

## 出力例

### ターミナル出力
```
=== Hardware Diagnostics ===

[       PASS] Motor[1]: Motor MD1 responding
[       PASS] Motor[2]: Motor MD2 responding
...
[PASS] Servo[1]: Servo SERVO1 responding
[PASS] Servo[2]: Servo SERVO2 responding
...

Summary: 23/23 passed (100.0%), 0 failed, 0 warnings
```

### ROSトピック出力
```
/diagnostics (diagnostic_msgs/DiagnosticArray):
  - name: r2_motor_1
    level: OK (0)
    message: Motor MD1 responding
    value: 0.0
  
  - name: r2_servo_1
    level: OK (0)
    message: Servo SERVO1 responding
    value: 0.0

/r2/diagnostics_text (std_msgs/String):
  "Diagnostics Summary: 23/23 passed (100.0%), 0 failed, 0 warnings
   [       PASS] Motor[1]: Motor MD1 responding
   ..."
```

### ファイル出力
```json
{
  "timestamp": 1713607200.123,
  "timestamp_readable": "2024-04-20 12:00:00.123",
  "summary": {
    "total_tests": 23,
    "passed": 23,
    "failed": 0,
    "warnings": 0,
    "success_rate_percent": 100.0
  },
  "results": [
    {
      "component": "Motor",
      "component_id": 1,
      "status": "PASS",
      "message": "Motor MD1 responding"
    },
    ...
  ]
}
```

## 設定ファイル

`config/diagnostics_config.yaml` で動作を設定できます：

```yaml
diagnostics:
  publish_interval: 10.0  # 診断パブリッシュ間隔（秒）
  hardware:
    motors:
      enabled: true
      count: 8
    servos:
      enabled: true
      count: 8
    solenoids:
      enabled: true
      count: 7
  
  report:
    save_reports: true
    report_directory: "~/.ros/diagnostics_reports"
    report_format: "json"
```

## 拡張方法

### 新しい診断を追加

```python
from r2_diagnostics.hardware_diagnostics import HardwareDiagnostics, DiagnosticStatus

hw_diag = HardwareDiagnostics()

# カスタム診断
status = DiagnosticStatus.PASS  # or FAIL, WARNING
hw_diag.add_result(
    component="CustomComponent",
    component_id=1,
    status=status,
    value=42.0,
    expected=40.0,
    message="Custom test passed"
)
```

### 新しいセンサーを監視

```python
from r2_diagnostics.sensor_diagnostics import SensorDiagnostics, SensorType

sensor_diag = SensorDiagnostics()

# センサー登録
sensor_diag.register_sensor(
    SensorType.ENCODER, 
    "custom_encoder",
    min_limit=0,
    max_limit=10000
)

# 値更新
sensor_diag.update_reading(
    SensorType.ENCODER,
    "custom_encoder",
    5000,
    "pulses"
)

# 健全性チェック
ok, msg = sensor_diag.check_sensor_health(
    SensorType.ENCODER,
    "custom_encoder"
)
```

## トラブルシューティング

### サービスが見つからない
```bash
ros2 service list
# /r2/run_diagnostics が表示されていることを確認
```

### トピックが表示されない
```bash
ros2 topic list
# /diagnostics と /r2/diagnostics_text を確認
```

### レポートが保存されない
```bash
mkdir -p ~/.ros/diagnostics_reports
# ディレクトリ作成と権限確認
```

## 今後の拡張計画

- [ ] I2C/CAN通信診断
- [ ] バッテリー監視
- [ ] 温度監視
- [ ] 通信レイテンシー測定
- [ ] 自動修復機能
- [ ] Webダッシュボード表示
- [ ] 履歴管理とトレンド分析

## ライセンス

Apache License 2.0

## 作者

Development Team

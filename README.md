# ddsm115_ros2_driver (最強版 / Ultimate Edition)

Waveshare DDSM115 ダイレクトドライブサーボモーター用の、堅牢で高性能な ROS 2 ドライバパッケージです。
これは、元となる `ddsm115_ros2_driver` をベースに大幅にリファクタリングし、実用性と信頼性を極限まで高めた **「最強の ROS 2 DDSM115 ドライバ」** です。

An advanced, high-performance, and robust ROS 2 driver package for the Waveshare DDSM115 direct-drive servo motor. Relies on low-latency serial communication and integrates seamlessly with `ros2_control`.

---

## 🌟 主な特徴 / Key Features

1. **純粋なC++非同期通信ライブラリ (Pure C++ Client Library)**:
   - `Boost.ASIO` を用いた完全な非同期シリアル読込。ROS 2 と完全に疎結合化され、堅牢に動作します。
   - スライディングバッファを用いたフレームアライメントと、厳格な **CRC-8/MAXIM** チェックサム検証。
   
2. **`ros2_control` への完全対応 (Full ros2_control Integration)**:
   - `hardware_interface::SystemInterface` プラグインとして動作し、`diff_drive_controller` や `joint_state_broadcaster` などの標準コントローラと直接結合可能です。
   - `ros2_control` のリアルタイムループをブロックしない非同期のシリアル送信（制御スレッド遅延ゼロ）。

3. **マルチモーターの衝突回避ポーリング (Collision-free RS485 Polling)**:
   - バックグラウンドで順次送信・受信時間をズラすポーリングアルゴリズムにより、単一の RS485 バス上で複数モーターを動かした際のパケット衝突（Collision）を完璧に防ぎます。

4. **オドメトリ用連続角度トラッキング (Continuous Angle Tracking)**:
   - モーターの 0〜360度 の一回転絶対角度フィードバックをトラッキングし、ラップアラウンド（一回転時の境界ジャンプ）を自動補正して **連続的な回転角（ラジアン）** を出力します。これにより、差動2輪ロボットのオドメトリが正しく動作します。

5. **安全対策用ウォッチドッグ (Safety Watchdog & Timeout)**:
   - ROS 2 トピック制御時、一定時間コマンドが途絶えた場合に自動でブレーキ（停止コマンド）をかける安全機能を搭載。

6. **詳細な診断機能 (Diagnostic Updater Integration)**:
   - モーターのエラービット（過熱保護、過電流、過電圧・低電圧、センサー異常、ストール検出）を解析し、ROS 2 Diagnostics 経由でステータスを発信します。

---

## 🛠️ ハードウェア接続 / Hardware Setup

1. モーターと USB-to-RS485 アダプタを接続します。
   - DDSM115 `DATA+` (Yellow) ➔ RS485 `A+`
   - DDSM115 `DATA-` (Blue) ➔ RS485 `B-`
   - DDSM115 `GND` (Black) ➔ RS485 `GND` (ノイズ防止のためシリアルグラウンドも接続することを推奨)
2. 電源 (12V ~ 24V DC, 6A以上推奨) を接続します。

---

## 🚀 ビルドとインストール / Building and Installation

### 1. ワークスペースへの配置
本リポジトリを ROS 2 ワークスペースの `src` ディレクトリにクローンまたは配置します。
Place this directory under your ROS 2 workspace `src/` (e.g. `main_ws/src/drivers/actuators/ddsm115_ros2_driver`).

### 2. ビルドの実行
ワークスペースのルートディレクトリにて、以下を実行します：
```bash
colcon build --symlink-install --packages-select ddsm115_ros2_driver
source install/setup.bash
```

---

## ⚙️ モーターIDの設定ツール / Motor ID Config Tool

RS485バス上に複数のモーターを配置する場合、各モーターに一意のID（1〜253）を書き込む必要があります。
このパッケージには、**正しい CRC-8/MAXIM 計算を実装した** ID 設定スクリプトが付属しています。

> [!IMPORTANT]
> IDを設定する際は、**必ず書き換えたいモーター1台のみ**を RS485 バスに接続した状態で実行してください。複数接続していると、全てのモーターが同じIDに書き換わってしまいます。

```bash
# 例: /dev/ttyUSB0 に接続したモーターのIDを「2」に書き換える場合
ros2 run ddsm115_ros2_driver set_motor_id.py --ros-args -p serial_port:=/dev/ttyUSB0 -p id:=2

# 完了後、モーターの電源を一度切り、再投入（Power Cycle）することで設定が反映されます。
```

---

## 運行モード 1: スタンドアロン ROS 2 ノード / Standalone ROS 2 Node Mode

トピック通信を利用して簡易的にモーターを動作・モニタリングしたい場合に使用します。

### 1. 起動方法
```bash
ros2 launch ddsm115_ros2_driver ddsm115_node.launch.py
```

### 2. トピック仕様 / ROS Topics
各モーターのIDごとにトピックが自動生成されます。

*   **購読トピック (Subscribed)**
    *   `/motor_[ID]/command` ([ddsm115_ros2_driver/msg/Ddsm115Command])
        - `mode`: 制御モード (1: 電流ループ, 2: 速度ループ, 3: 位置ループ)
        - `value`: コマンド値 (A, RPM, 度)
        - `brake_mode`: ブレーキ制御 (0: 解除, 1: ロック)
*   **配信トピック (Published)**
    *   `/motor_[ID]/status` ([ddsm115_ros2_driver/msg/Ddsm115Status])
        - `current`: 実測電流 (A)
        - `velocity`: 実測速度 (RPM)
        - `position`: 単回転実測位置 (度: 0.0 ~ 360.0)
        - `error_code`: エラーコード (診断ステータス)
    *   `/diagnostics` ([diagnostic_msgs/msg/DiagnosticArray])
        - 各モーターおよび通信状態の診断データ

---

## 運行モード 2: `ros2_control` ハードウェアインターフェース / ros2_control Mode

移動ロボット（デファレンシャルドライブなど）で高精度・標準的なオドメトリとナビゲーション（Nav2）を利用するための推奨モードです。

### 1. URDF (xacro) への設定例
URDFに以下の記述を追加することで、`ros2_control` の `controller_manager` から直接モーターを呼び出せます。

```xml
<ros2_control name="DDSM115HardwareInterface" type="system">
  <hardware>
    <plugin>ddsm115_ros2_driver/DDSM115SystemHardware</plugin>
    <param name="usb_path">/dev/ttyUSB0</param>
    <param name="serial_baud">115200</param>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">1</param>
    <param name="invert_direction">false</param>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="motor_id">2</param>
    <param name="invert_direction">true</param> <!-- 右輪を反転する場合 -->
  </joint>
</ros2_control>
```

### 2. デモ起動方法
以下のローンチファイルを実行すると、ダミーのURDFを生成し、`ros2_control_node` と `diff_drive_controller` を起動してデモ運行を行います。

```bash
ros2 launch ddsm115_ros2_driver ddsm115_control.launch.py motor_id_left:=1 motor_id_right:=2
```

---

## 📄 ライセンス / License
本パッケージは [MIT License](LICENSE) の下で配布されています。
Licensed under the MIT License.

# RobStride モーター制御ライブラリ（ESP32 用）

ESP32 プラットフォームで CAN バスを介して RobStride EDULITE05 モーターを制御する Arduino ライブラリです。

> **注意**: 現在このライブラリは製作中です。EDULITE05 用のパラメータのみ対応しています。他の RobStride モーターを使用する場合は、パラメータ範囲（位置、速度、トルク等）の調整が必要です。

## 機能

- RobStride モーターの全制御モードに対応:
  - **MIT モーションコントロール**: 位置/速度の直接制御（Kp/Kd によるインピーダンス制御）
  - **位置制御（PP）**: ポイント・ツー・ポイント位置決めモード
  - **位置制御（CSP）**: 連続位置ストリーミングモード
  - **速度制御**: 電流制限付き速度制御
  - **電流/トルク制御**: 直接電流指令
- 遅延なしのアトミック操作による高速な API 設計
- 安全なモード切替手順
- ESP32-TWAI-CAN ライブラリをベースに構築

## 必要なハードウェア

- ESP32-S3 または ESP32 ボード（M5Stack AtomS3 で動作確認済み）
- RobStride EDULITE05 モーター
- CAN トランシーバ（M5Stack Unit Mini CAN で動作確認済み）

## 配線

```
ESP32        CANトランシーバ
GPIO1   -->  CRX (RX)
GPIO2   -->  CTX (TX)
3.3V    -->  VCC
GND     -->  GND
```

CAN トランシーバの CAN-H と CAN-L を RobStride モーターの CAN バスに接続してください。

## インストール

### PlatformIO

`platformio.ini`に以下を追加:

```ini
lib_deps =
    handmade0octopus/ESP32-TWAI-CAN@^1.0.1
    https://github.com/necobit/RobStride_ESP32_Library.git
```

### Arduino IDE

1. このライブラリを ZIP でダウンロード
2. Arduino IDE: スケッチ → ライブラリをインクルード → .ZIP 形式のライブラリをインストール
3. ライブラリマネージャから ESP32-TWAI-CAN ライブラリをインストール

## クイックスタート

```cpp
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <RobStride.h>

RobStrideMotor motor(1);  // モーターID = 1

void setup() {
  Serial.begin(115200);

  // CANバス初期化: 1Mbps, TX=GPIO2, RX=GPIO1
  ESP32Can.begin(TWAI_SPEED_1000KBPS, 2, 1);

  delay(100);

  // モーター初期化
  motor.stop();
  delay(50);
  motor.clear_fault();
  delay(50);

  // モーションコントロールモードに設定
  motor.set_run_mode(RS_MODE_MOTION);
  delay(20);

  motor.enable();
}

void loop() {
  // MIT モーションコントロール: 位置, 速度, Kp, Kd, トルク
  motor.send_motion_command(1.0, 0, 10.0, 0.5, 0);
  delay(10);
}
```

## 制御モード

### MIT モーションコントロール（RS_MODE_MOTION）

位置、速度、インピーダンスパラメータによる直接制御:

```cpp
motor.set_run_mode(RS_MODE_MOTION);
delay(20);
motor.enable();

// 位置, 速度, Kp, Kd, フィードフォワードトルク
motor.send_motion_command(1.5, 0, 10.0, 0.5, 0);
```

### 位置制御 - ポイント・ツー・ポイント（RS_MODE_POS_PP）

```cpp
motor.set_limit_current(3.0);
delay(10);
motor.set_pp_limits(5.0, 5.0);  // 速度, 加速度
delay(10);
motor.set_run_mode(RS_MODE_POS_PP);
delay(20);
motor.enable();

motor.set_position_reference(2.0);  // 目標位置（ラジアン）
```

### 位置制御 - 連続（RS_MODE_POS_CSP）

```cpp
motor.set_limit_speed(5.0);
delay(10);
motor.set_limit_current(3.0);
delay(10);
motor.set_run_mode(RS_MODE_POS_CSP);
delay(20);
motor.enable();

motor.set_position_reference(1.0);  // 位置を連続更新
```

### 速度制御（RS_MODE_SPEED）

```cpp
motor.set_limit_current(3.0);
delay(10);
motor.set_limit_accel_rad(10.0);
delay(10);
motor.set_run_mode(RS_MODE_SPEED);
delay(20);
motor.enable();

motor.set_speed_reference(5.0);  // rad/s
```

### 電流/トルク制御（RS_MODE_CURRENT）

```cpp
motor.set_run_mode(RS_MODE_CURRENT);
delay(20);
motor.enable();

motor.set_current_reference(0.5);  // アンペア
```

## API リファレンス

### 基本コマンド

```cpp
motor.enable();              // モーター有効化
motor.stop();                // モーター停止（無効化）
motor.clear_fault();         // フォルトステータスをクリア
motor.set_zero_position();   // 現在位置をゼロに設定
```

### モード設定

```cpp
motor.set_run_mode(mode);    // 制御モードを設定
// モード: RS_MODE_MOTION, RS_MODE_POS_PP, RS_MODE_POS_CSP,
//        RS_MODE_SPEED, RS_MODE_CURRENT
```

### 制御コマンド

```cpp
// MITモーションコントロール
motor.send_motion_command(position, velocity, kp, kd, torque);

// 位置/速度/電流指令値
motor.set_position_reference(position);
motor.set_speed_reference(speed);
motor.set_current_reference(current);
```

### リミット設定

```cpp
motor.set_limit_current(current);      // 電流リミット設定 (A)
motor.set_limit_speed(speed);          // 速度リミット設定 (rad/s)
motor.set_limit_accel_rad(accel);      // 速度モード用加速度リミット
motor.set_pp_limits(velocity, accel);  // PP位置モード用リミット
```

### ステータス読み出し

モーターのステータス（位置、速度、トルク、温度など）を読み取ります:

```cpp
// 手動でステータスを読み取る（タイムアウト指定可能）
RobStrideStatus status = motor.read_status(timeout_ms);

if (status.valid) {
  // status.position (rad)
  // status.velocity (rad/s)
  // status.torque (Nm)
  // status.temperature (°C)
  // status.mode (0=Reset, 1=Cali, 2=Motor)
  // status.fault (0=正常)
}
```

**注**: `read_status()`は制御コマンド送信後にモーターから自動的に送られるフィードバックフレーム（通信タイプ2）を読み取ります。

### 自動報告モード

モーターが定期的に自動でステータスを送信するようにできます:

```cpp
// 自動報告を有効化
motor.enable_auto_report();

// setup()で一度だけ有効化すれば、モーターが定期的にステータスを送信
// read_status(0)で非ブロッキングで読み取り可能

void loop() {
  // 制御コマンド送信
  motor.send_motion_command(pos, vel, kp, kd, torque);

  // ステータスを読み取る（すぐに返る）
  RobStrideStatus status = motor.read_status(0);
  if (status.valid) {
    Serial.printf("Pos: %.3f\n", status.position);
  }

  delay(10);
}

// 自動報告を無効化
motor.disable_auto_report();
```

**推奨**: 自動報告モードを使用することで、ブロッキングなしでリアルタイムにモーターステータスを監視できます。

### パラメータ範囲

| パラメータ | 最小値 | 最大値 | 単位  |
| ---------- | ------ | ------ | ----- |
| 位置       | -12.57 | 12.57  | rad   |
| 速度       | -50.0  | 50.0   | rad/s |
| Kp         | 0.0    | 500.0  | -     |
| Kd         | 0.0    | 5.0    | -     |
| トルク     | -6.0   | 6.0    | Nm    |

## 重要な注意事項

### 安全なモード切替

制御モードを切り替える際は、必ず以下の手順に従ってください:

1. モーターを停止
2. フォルトをクリア（2 回推奨）
3. 新しいモードを設定する**前に**パラメータリミットを設定
4. 実行モードを設定
5. モーターを有効化

```cpp
void switch_mode_safely(uint8_t new_mode) {
  motor.stop();
  delay(50);
  motor.clear_fault();
  delay(10);
  motor.clear_fault();
  delay(50);

  // モードに応じたリミット設定
  if (new_mode == RS_MODE_SPEED) {
    motor.set_limit_current(3.0);
    delay(10);
    motor.set_limit_accel_rad(10.0);
    delay(10);
  }

  motor.set_run_mode(new_mode);
  delay(20);
  motor.enable();
}
```

### タイミング要件

- パラメータ書き込み間: 10ms
- モード変更後、有効化前: 20ms
- 停止/フォルトクリア操作後: 50ms

### ライブラリ設計

すべてのライブラリメソッドは**アトミック**（内部遅延なし）です。コマンド間の適切なタイミングはアプリケーションコード側で管理してください。

## サンプル

`examples/`フォルダに完全なサンプルがあります:

- **BasicControl**: シンプルな位置制御の例
- **DemoLoop**: 全制御モードを順番に実行する包括的なデモ

## ライセンス

MIT License

## クレジット

ESP32-TWAI-CAN ライブラリを使用して RobStride EDULITE05 モーター向けに開発されました。

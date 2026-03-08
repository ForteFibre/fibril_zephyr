# RoboMaster Mini V1 Zephyr Porting Plan

## Goal

`CanMotorMbed/src/targets/robomaster_miniv1/main.cpp` の機能を、`fibril_zephyr` 配下で段階的に再構成し、最終的に RoboMaster Mini V1 基板向けの Zephyr ファームウェアとして置き換える。

ここでの主目的は「Mbed 実装の忠実な行単位移植」ではなく、以下の 3 層へ責務を分離した Zephyr ネイティブ実装へ移すこと。

1. board 層: SoC/ピン/周辺機器定義
2. driver 層: モータ CAN と GPIO/LED/スイッチ等のハード抽象
3. subsys 層: モータ制御、watchdog、所有権管理、制御ループ
4. app 層: control CAN protocol と board 固有の組み合わせ

## Current Findings

### 1. Mbed 側の `robomaster_miniv1` は 3 系統の CAN を持つ

`CanMotorMbed/src/targets/robomaster_miniv1/main.cpp` では以下の構成になっている。

- モータ用 CAN x2
  - `MOTCAN0_RX/TX`
  - `MOTCAN1_RX/TX`
- 通信用 CAN x1
  - `CAN_RX/TX`

モータ CAN は `fibril::RoboMasterRouter` に束ねられ、8 モータ分の `RoboMasterMotor` / `RoboMasterEncoder` / `RoboMasterTorqueSensor` を生成している。通信用 CAN は別の `fibril::can::Node` として `DriverProtocol` / `SimpleProtocol` に使われている。

加えて、新要件として USB 接続時にホスト PC から CAN デバイスとして認識される機能も必要である。Zephyr 側ではこれは独自実装より `cannectivity` module の `gs_usb` を使うのが第一候補になる。

### 2. `robomaster_miniv1` は内蔵 RoboMaster エンコーダ前提

`CanMotorMbed/custom_targets.json` の `FORTEFIBRE_ROBOMASTER_MINIV1` には以下が含まれる。

- `NUM_MOTORS=8`
- `EXTADC_COUNT=8`
- `ROBOMASTER_ENCODER`

そのため、Mbed 側 `main.cpp` でも実際に使われるエンコーダは外付け A/B 相ではなく `fibril::RoboMasterEncoder` である。外部クアドラチャ配線は初期移植では必須ではない。

### 3. 制御ロジックは `main.cpp` ではなく `MotorWorker` / `Protocol` 群にある

Mbed 側の主要責務は以下。

- `MotorWorker`
  - 1 kHz 周期更新
  - duty / speed / position / position+speed 制御
  - 速度 LPF
  - encoder safety
- `MotorArbiter` / `MotorProxy`
  - モータ所有権管理
- `DriverProtocol`
  - 詳細設定・状態通知・bulk telemetry
- `SimpleProtocol`
  - duty/speed/position 指令の簡易制御
- `BoardWatchdog`
  - 通信断時の全停止

つまり、Zephyr 移植の難所は RoboMaster CAN 通信そのものではなく、上位の状態機械と周期制御である。

### 4. Zephyr 側には既に RoboMaster transport driver の土台がある

`fibril_zephyr/drivers/motor/robomaster.c` は既に以下を実装済み。

- 複数 CAN バス対応
- motor ID ごとの `dji,robomaster-motor` 子ノード
- 電流指令の group frame 化
- フィードバック受信
  - orientation
  - velocity
  - current
  - temperature
  - accumulated position
- ztest による CAN fake テスト

このため、Mbed の `RoboMasterRouter` 相当をゼロから作る必要はない。既存 driver を transport 層の基礎として使うべき。

## Recommended Architecture

### A. board port を先に作る

新規 board を `fibril_zephyr/boards/fibril/robomaster_miniv1/` に追加する。

最低限必要なファイル:

- `board.yml`
- `board.cmake`
- `robomaster_miniv1.dts`
- `robomaster_miniv1-pinctrl.dtsi`
- `robomaster_miniv1_defconfig`
- 必要なら `Kconfig.board`

初期 DTS で定義すべきもの:

- `chosen`
  - flash / sram / console
- `&fdcan1`
  - control CAN
- `&fdcan2`
  - motor CAN 0
- `&fdcan3`
  - motor CAN 1
- `gpio-leds` もしくは独自 RGB LED ノード
- ロータリスイッチ入力 GPIO x4
- USB FS device controller
- `gs_usb` 用 devicetree node

Mbed `PinNames.h` から読み取れる主要ピンは以下。

- control CAN: `PD0` / `PD1`
- motor CAN0: `PB12` / `PB13`
- motor CAN1: `PA8` / `PA15`
- rotary: `PB1` / `PA3` / `PB0` / `PA4`
- RGB LED: `PC10` / `PC11` / `PC12`

USB については STM32G474 の USB FS device を有効化し、VBUS sensing / D+ / D- 配線とクロック要件を board bring-up で先に確認する。

### B. transport driver と subsys/app を分離する

既存 `drivers/motor/robomaster.c` はそのまま「RoboMaster motor transport」として維持する。

Mbed の `DriverProtocol` / `SimpleProtocol` に対応する CAN protocol は driver に入れず、`app` 層として実装する。`MotorWorker` 相当の制御ロジックは `subsys` に置く。理由は以下。

- 制御対象が複数 motor device にまたがる
- watchdog と ownership は system policy でありハードドライバ責務ではない
- protocol version 変更の影響を board/driver から切り離せる
- CAN protocol は製品都合で変わりやすく、Zephyr 共通機能として固定しない方がよい

推奨ディレクトリ例:

- `fibril_zephyr/subsys/motor_control/`
- `fibril_zephyr/app/src/protocol/`

役割分割:

- `drivers/motor/robomaster.c`
  - CAN frame I/O
  - motor feedback cache
  - `motor_enable/set_output/get_feedback`
- `subsys/motor_control/arbiter.c`
  - 所有権管理
  - watchdog timeout
  - 全 motor の enable/disable policy
- `subsys/motor_control/loop.c`
  - 1 kHz 制御周期
  - PID / ramp / safety
- `app/src/protocol/robomaster_protocol.cpp`
  - control CAN の受信
  - Mbed 互換コマンド実装
  - subsys API 呼び出し
- `app/src/usb/gs_usb_bridge.c` または設定コード
  - USB CAN device としての列挙
  - host から見える CAN channel の公開
  - 制御用 CAN を USB 側へ公開

この分割では「motor control は reusable subsys」「CAN protocol と USB product policy は application policy」という整理になる。

### C. serde は `fibril_common` をそのまま流用する

`fibril_common/include/fibril/data/serde.hpp` に既存の C++ serde 実装があり、`read` / `write` / `read_callback` まで揃っている。Mbed 側 protocol もこの API を使っているため、Zephyr 側で serde を作り直さない方針が妥当。

推奨方針:

1. protocol 実装は C++ で書く
2. `fibril_common` を Zephyr module もしくはローカル library として取り込む
3. wire format は Mbed と同じ `fibril::serde` を使って維持する
4. protocol 部だけを無理に C 化しない

注意点:

- `fibril_zephyr/app/CMakeLists.txt` は現状 `project(app LANGUAGES C)` なので、protocol を app に置くなら C++ 有効化が必要
- `subsys` は C でも C++ でもよいが、controller core を `fibril_common` の既存 C++ 資産に寄せるなら C++ に揃えた方が素直

### D. Mbed 互換 CAN protocol は「まず互換維持」で入る

既存の ROS 2 側や上位ツールが `DriverProtocol` / `SimpleProtocol` を前提にしている可能性が高い。初期移植段階では protocol を再設計せず、まず wire format 互換を維持する方が安全。

推奨方針:

1. `CmdId=0x170` / `CallbackId=0x160` の互換維持
2. `TEST`, `RESET`, `CMD_SET_DUTY`, `CMD_SET_SPEED`, `CMD_SET_POSITION`, `CMD_SUB_SETTING` など主要コマンドを優先実装
3. bulk telemetry は必要最小限から開始
4. 互換性が不要と確認できるまで protocol 差し替えはしない

### E. USB CAN device 機能は `gs_usb` を優先利用する

`west.yml` には既に `cannectivity` が含まれており、module 内に `gs_usb` の device class 実装とテストがある。そのため USB-CAN bridge 機能は自前実装せず、以下の方針を推奨する。

推奨方針:

1. USB 側の CAN device 列挙は `gs_usb` を使う
2. USB に公開するのは制御用 CAN のみとする
3. RoboMaster motor CAN は USB 公開対象にしない
4. control protocol 用 CAN と USB-CAN bridge は同一物理 CAN を共有する
5. `fdcan1` で受信した CAN message は原則すべて USB 側へ流せるようにする

設計上の注意:

- `gs_usb` は host からは一般的な USB-CAN アダプタとして見える
- これは `DriverProtocol` / `SimpleProtocol` の代替ではなく並列機能である
- ただし今回の要件では、USB 側で見せるのは control CAN であり RoboMaster motor bus ではない
- `gs_usb` の class 実装は流用対象だが、control CAN protocol と同居させる都合で integration は `fibril_zephyr/app` 側で整理する必要がある
- `fdcan1` の RX は monitor/bridge を兼ねるため、特定 ID だけを USB 転送する前提にしない
- Mbed 互換 protocol で消費する frame も含めて、少なくとも受信観測は USB 側から可能にする

初期推奨マッピング:

- `fdcan1`: USB `gs_usb` 公開対象かつ board control protocol 用
- `fdcan2`, `fdcan3`: RoboMaster motor transport 専用

この構成なら PC からは通常の USB-CAN アダプタとして制御用 CAN の全受信トラフィックを観測でき、RoboMaster motor bus はファームウェア内部専用バスとして閉じられる。

### F. 制御ループは Zephyr kernel primitive へ置換する

Mbed の `EventQueue::call_every()` を Zephyr では以下へ置き換える。

- 1 kHz 制御ループ
  - `k_timer` + dedicated thread
  - または cooperative thread + `k_sleep(K_USEC(1000))`
- 10 ms / 100 ms / 1000 ms の周期処理
  - `k_work_delayable`
  - `k_timer`

推奨は以下。

- 制御ループ: 専用 thread
  - 理由: CAN callback と独立に deterministic に回したい
- telemetry / watchdog / status publish:
  - `k_work_delayable`

### G. MotorWorker は「subsys と controller ライブラリ」に分割する

`MotorWorker` は Mbed 依存と制御ロジックが混ざっている。Zephyr 移植時は次のように分けるとよい。

- controller core
  - 純粋 C/C++ ロジック
  - PID
  - acceleration limiter
  - position/speed state machine
- zephyr adapter
  - device から feedback 取得
  - `motor_set_output()` へ反映
  - watchdog / ownership 連携

この分割により `native_sim` でも controller 単体テストが書ける。

## Phased Migration Plan

### Phase 0: build baseline

達成条件:

- 新 board `robomaster_miniv1` が `west build -b ...` で通る
- console 出力できる
- 3 CAN device が `device_is_ready()` を返す
- USB device controller が初期化できる

実施内容:

- board 追加
- pinctrl 追加
- `app/prj.conf` で STM32 CAN, GPIO, LOG, USB を有効化

### Phase 1: motor transport bring-up

達成条件:

- DTS 上で 8 個の `dji,robomaster-motor` を定義
- 2 つの motor CAN で group current frame を送信できる
- 実機で feedback を受信できる
- USB 接続時に `gs_usb` として列挙できる

実施内容:

- `robomaster_controller` ノード定義
- 子 motor ノード 1..8 を定義
- `gs_usb` ノード定義
- `cannectivity` channel ノードで control CAN を公開
- 必要なら `robomaster.c` に timestamp / stale 判定改善を追加

### Phase 2: control CAN protocol app

達成条件:

- control CAN から `TEST` / `ENABLE_FEATURE` / `CMD_SET_DUTY` が通る
- 通信断で停止する

実施内容:

- control CAN 用受信フィルタ
- BoardWatchdog 相当の software watchdog
- 所有権管理
- `fibril_common::serde` を使った Mbed 互換 frame 実装
- 同一 control CAN 上で `gs_usb` と Mbed 互換 protocol をどう共存させるか確認
- `fdcan1` 受信 frame の USB 転送経路を確認

### Phase 3: closed-loop control

達成条件:

- speed / position 制御が動作
- 1 kHz ループで既存 Mbed と同等の応答を得る

実施内容:

- `MotorWorker` 相当 service 実装
- 必要なら `fibril_common` の controller 資産流用
- PID と acceleration limiter 移植
- encoder safety 相当導入

### Phase 4: auxiliary features

達成条件:

- status RGB LED
- rotary switch board ID
- torque sensor / calibration / bulk telemetry
- USB 機能の要否判断

備考:

`cannectivity` module は USB-CAN bridge 系には流用余地があるが、Mbed 実装の control protocol をそのまま置き換える用途ではない。

## Features To Port First

優先度高:

- board definition
- motor CAN transport
- USB `gs_usb` 列挙
- control CAN receive/transmit
- watchdog
- duty mode
- speed mode

優先度中:

- position mode
- bulk status/telemetry
- RGB LED
- rotary switch

後回し可:

- ADC calibration
- RS485/AMT21x 関連

## Main Gaps And Risks

### 1. `motor` driver API は closed-loop 制御にはまだ薄い

現状の `include/drivers/motor.h` は low-level API としては十分だが、Mbed `MotorWorker` が必要とする設定項目はまだ表現していない。

例:

- encoder reset / set value
- safety timeout
- controller gains
- mode state

対策:

- これらを driver API に追加しない
- app/service 側の state として保持する

### 2. Mbed 側 protocol は float と C++ serde に依存する

wire format の再現には以下の確認が必要。

- endian
- frame size
- CAN FD 利用有無
- コマンドごとの payload layout

対策:

- 最初に `DriverProtocol` / `SimpleProtocol` のコマンドセットを表形式で洗い出す
- `fibril_common` の `fibril::serde` を Zephyr 側へ直接取り込む
- loopback テストを追加する

### 3. USB と CAN の同時成立で board bring-up の複雑さが上がる

STM32G4 では USB クロックと FDCAN の立ち上げ確認を同時に進めると切り分けが難しい。

対策:

- USB 列挙単体確認
- `fdcan1` 単体確認
- `fdcan2/3` 単体確認
- 最後に併用確認

### 4. control CAN を `gs_usb` と独自 protocol が共有する設計整理が必要

`gs_usb` は通常 USB-CAN アダプタとして CAN controller をそのまま host に公開する。一方で今回の control CAN には既存 Mbed 互換 protocol も載せたい。さらに `fdcan1` は受信した frame を全て USB 側から観測できることが要件になる。

対策:

- control CAN 上のメッセージは host 主体で扱う前提にする
- firmware 内の protocol service は control CAN controller を直接専有せず、`fdcan1` の全受信経路を前提に共存させる
- `cannectivity` app 全体を流用するのではなく、`gs_usb` class 実装を中心に統合方式を詰める
- `fdcan1` RX callback で受けた frame が USB 転送対象から漏れない構成にする
- protocol service 側で frame を消費しても、USB 監視経路は独立して維持する
- `fdcan1` では protocol 専用の `rx_filter` に依存しない
- protocol 判定は全受信 frame を見た上で software 側で行う

### 5. 実機 bring-up 前に board pinmux ミスを埋め込みやすい

特に STM32G4 の FDCAN は pinctrl と clock の食い違いで無反応になりやすい。

対策:

- Phase 0 で CAN 単体疎通確認を先に行う
- 3 本の CAN を同時に有効化する前に 1 本ずつ確認する

### 6. 外部 ADC とトルクセンサは後段で再検討が必要

Mbed 側 `adc_calibration` は `RoboMasterTorqueSensor` を `AnalogInExt` 扱いで再利用している。Zephyr ではこのモデルをそのまま再現するより、トルクセンサを motor feedback の一部として扱う方が自然な可能性がある。

## Concrete DTS Shape

初期イメージ:

```dts
/ {
    model = "Fibril RoboMaster Mini V1";
    compatible = "fibril,robomaster-miniv1";

    aliases {
        motor-controller = &robomaster_controller;
    };

    rgb_led {
        compatible = "gpio-leds";
    };

    rotary_sw: rotary-sw {
        compatible = "gpio-keys";
    };

    robomaster_controller: robomaster {
        compatible = "dji,robomaster";
        cans = <&fdcan2 &fdcan3>;
        feedback-timeout-ms = <100>;

        motor0: motor@1 {
            compatible = "dji,robomaster-motor";
            reg = <1>;
            can-bus = <0>;
            model = "c620";
        };

        /* ... motor@8 まで ... */
    };

    cannectivity: cannectivity {
        compatible = "cannectivity";

        channel {
            compatible = "cannectivity-channel";
            can-controller = <&fdcan1>;
        };
    };

    gs_usb0: gs_usb0 {
        compatible = "gs_usb";
    };
};
```

control CAN は motor transport に含めず、`&fdcan1` を USB 公開と Mbed 互換 protocol の両方で扱う前提で app 層に統合する。

このとき `fdcan1` の全受信 message を USB へ流すことを要件とし、protocol 用 ID のみを個別に抜き出す構成にはしない。

## Implementation Recommendation

実装順は以下が最も安全。

1. board 追加
2. USB `gs_usb` を列挙させる
3. DTS で 2 motor CAN + 8 motors を定義
4. 既存 `robomaster` driver の実機疎通
5. control CAN service を追加
6. watchdog + ownership
7. duty mode
8. speed / position 制御
9. 周辺機能

逆に避けるべき進め方:

- board 未確定のまま protocol 実装から始める
- `MotorWorker` をそのまま巨大な 1 ファイル移植する
- motor transport と control protocol を 1 driver に混ぜる

## Suggested File Layout

```text
fibril_zephyr/
  boards/fibril/robomaster_miniv1/
  subsys/motor_control/
    CMakeLists.txt
    Kconfig
    motor_arbiter.cpp
    motor_loop.cpp
    software_watchdog.cpp
  app/src/protocol/
    robomaster_protocol.cpp
    simple_protocol.cpp
  include/fibril/
    motor_control.hpp
  tests/
    subsys/motor_control/
    app/protocol/
```

## Immediate Next Task

次に着手すべき作業は設計より具体的で、実装としても独立している。

1. `robomaster_miniv1` board 定義を追加する
2. `app.overlay` ではなく board DTS に motor transport を定義する
3. `native_sim` 向けに protocol service の unit test を先に用意する

この 3 点まで進めば、以後の移植は「ハード起動」と「上位互換制御」に分離して進められる。

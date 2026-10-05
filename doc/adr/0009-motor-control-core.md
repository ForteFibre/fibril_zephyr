---
status: Proposed
date: 2026-10-05
---

# 0009. CanMotorMbed の制御のコアを、1 kHz 固定の tick で回る C++ ライブラリとして移す

## Status

Proposed

`lib/motor_control/` とそのテストはある。
fibril_can のブロック型から使うのは次の段階で、まだ実機で回していない。

## Context

CanMotorMbed（Mbed OS）の `MotorWorker` は、モータ 1 台ぶんの閉ループ制御を 1 ms ごとに回している。
その内容は次のとおりである。

- モードは DUTY、SPEED、POSITION、SPEED_POSITION（位置の PID の出力を速度の目標にするカスケード）の 4 つ。
- 速度にはローパスフィルタを通す。
- duty には変化率の上限を掛け、エンコーダの失速ブレーカを通す。
- `feature/ff-friction` ブランチは、SPEED_POSITION に速度と duty のフィードフォワード、クーロン摩擦と粘性摩擦の補償を足している。

salamander の機体は、このブランチを実運用で使っている。

fibril_zephyr では、ROS 2 側から見たときに ros2_can_toolbox の `can_md_controller` と同じに見えるブロック型を作る。
ホストが持っているゲインの表と、その意味をそのまま使えることが前提になる。

[ADR 0002](0002-encoder-feedback-layering.md) は encoder の速度を counts/s にした。
そのとき「Mbed 側の閾値とゲインは制御 tick あたりの量に対して調整されているので、制御周期ぶんスケールし直しになる」と書いた。
[ADR 0008](0008-fibril-common-integration.md) は fibril_common を取り込む前提を調べ、Phase 1 を「最初の利用者で使う」とした。

## Decision

### 1. 制御のコアを `lib/motor_control/` に、Zephyr を知らない C++17 のクラスとして置く

`motor_control::MotorControl` はモータ 1 台ぶんの状態を持つ。
入力は `EncoderSample`（有効か、積算位置の counts、速度の counts/s）、出力は duty である。
呼び出し側が tick ごとに `update()` を 1 回呼び、返った duty をモータに書く。

`<zephyr/*.h>` を含まない。
時間は tick の数で数える（フィードフォワードの失効と失速の判定）。
そのため native_sim の ztest で、デバイスを用意せずに試せる。
CAN、devicetree、モータとエンコーダのデバイスを結ぶのは、これを使うブロック型の仕事である。

PID と変化率の上限には、fibril_common の `PIDController<float>` と `AccelerationLimit` を使う。
CanMotorMbed が使っているものと同じ実装である。
fibril_common を基板で使う最初の利用者になる（ADR 0008 の Phase 1）。

### 2. tick は 1 kHz に固定し、PID のゲインは tick あたりのまま運ぶ

`PIDController` は経過時間を受け取らない。
積分項と微分項は、`update()` 1 回あたりの量である。
Mbed も 1 ms 固定で回していたので、tick を 1 kHz に固定すれば、ki と kd はそのまま運べる。
`MotorControl::tick_us` がこの約束である。

速度の PID に渡す値は `encoder_gain × counts/s` にする。
Mbed は `gain × (tick あたりの counts) / 0.001` を渡していたので、同じ量になる。
ADR 0002 が心配した速度のゲインのスケールし直しは、要らない。
Mbed が速度を PID に入れる前に、既に dt で割っていたためである。

位置は `encoder_gain × 積算 counts` を `float` にしたものである。
Mbed と同じく、24 bit を超える counts では分解能が落ちる。

### 3. Mbed の振る舞いのうち、ホストが頼っているものは残す

- エンコーダが無効になると、DUTY 以外のモードは DUTY の 0 に落ちる。エンコーダが戻っても、モードは DUTY のままで、ホストが目標を送り直す。
- モードに入るときだけ、そのモードの PID の積分と微分をリセットする。
- フィードフォワード（`set_velocity_ff`、`set_duty_ff`）は SPEED_POSITION でだけ効く。最後に設定してから 100 tick で失効する。失効した値も、設定し直せば生き返る。モードは変えない。
- 摩擦補償は、実速度ではなく目標速度 r_v の符号と大きさで掛ける。不感帯 0.01 は r_v の単位（ユーザ単位/s）なので、`encoder.gain` に依存する。この定数は Mbed と同じ値のままにする。
- SPEED_POSITION の duty は、すべての項を足した後に ±1 で 1 回だけ丸める。
- 失速ブレーカは既定で無効で、タイムアウトを 0 にすると無効になる。duty が 0.1 を超えているのに、エンコーダが動かない状態がタイムアウトより長く続くと、出力を 0 にする。`reset_safety()` まで戻らない。判定した tick の出力は変えず、次の tick から効く。
  ブレーカを有効にするか無効にするかを切り替えたときだけ、トリップを解く。タイムアウトの長さを変えるだけでは、トリップしたままになる。

### 4. Mbed の不具合は写さない

- **失速ブレーカが逆転を失速と判定していた。** `speed() > 1e-6` を絶対値を取らずに比べていたためである。速度の絶対値で比べる。
  さらに、ゲインを掛けない生の counts で判定する。負のゲインで向きを反転すると、判定まで反転していたからである。
  閾値の「1e-6」は tick あたりの counts に対する値で、意味は「0 でない」だった。counts/s でも 0 と比べる。
- **SPEED_POSITION だけが、フィルタを通さない速度を使っていた。** SPEED と同じく、フィルタを通した速度を使う。
  既定の係数 0.9 では、調整済みのゲインに対して振る舞いが少し変わる。`velocity.filter_coe = 1.0` にすれば Mbed と同じになる。

### 5. `reset_all()` はすべての設定を既定に戻す

Mbed の `reset_all()` は、ゲインを 0 にした。
一方で、積分の飽和値、フィルタ係数、エンコーダのゲイン、失速ブレーカの設定は残していた。
ホストは接続のたびに設定を送り直すので、残す理由が無い。
起動直後と同じ状態に戻す。

## Consequences

- ブロック型は、tick を 1 kHz で回し続ける責任を負う。`apps/node` の tick はもともと 1 kHz である。
- 制御周期を変えるなら、ゲインの意味が変わる。そのときは、この ADR を置き換える。
- `lib/motor_control/` は fibril_common を `CONFIG_FIBRIL_COMMON` で取り込む。fibril_common のソースは twister の `-Werror` で `-Wdouble-promotion` が出ないものが要る。
- 電流ベースライン、キャリブレーション、bulk のテレメトリは、ここに含めない。

## 却下した選択肢

### 経過時間を受け取る PID に替える

tick の周期が揺れても、ゲインの意味が保たれる。
しかし、ホストが持っているゲインの表がすべて換算し直しになる。
fibril_common の `PIDController` を ROS 2 側と共有する利点も失う。

### 速度を tick あたりの counts のまま制御層に渡す

Mbed の式をそのまま写せる。
しかし ADR 0002 が退けたのと同じ理由で、encoder の標本化の周期と制御の周期が結び付いてしまう。

### Mbed の `MotorWorker` をそのまま移す

`EncoderSafety` は `rtos::Kernel::Clock` を、フィードフォワードの watchdog は `SoftwareWatchdog` を直接使っている。
時間の源をクラスの中に持つと、テストで時間を進められない。

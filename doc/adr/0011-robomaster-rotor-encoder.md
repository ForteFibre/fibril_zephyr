---
status: Proposed
date: 2026-10-09
---

# 0011. RoboMaster のロータ角を、受信経路から通知を受ける encoder device として見せる

## Status

Proposed

## Context

CanMotorMbed の `robomaster_miniv4` には、モータを基板のエンコーダ入力で閉じる構成と、RoboMaster 自身のロータ角で閉じる構成（`ROBOMASTER_ENCODER`）があった。
後者は最大 8 台を、基板にエンコーダを付けずに制御できる。

MdMotor（ADR 0010）は `motors` と `encoders` を組にするので、ロータ角を encoder クラスの device として見せれば、後者もそのまま組める。
ADR 0002 はこの shim を「encoder クラスの形が固まってから別に決める」として範囲外にしていた。

encoder クラスが制御ループに約束しているのは、積算位置、実際に測った間隔で割った速度（`sample_interval_us`）、積算をやり直したことを知らせる `position_epoch` である（ADR 0002）。
MdMotor は `POSITION` と `VELOCITY` の両方が立たないサンプルを無効とみなし、`position_epoch` が変わると出力を止める。

モータドライバ（`drivers/motor/robomaster.c`）はロータ角の折り返しを解いて `motor_feedback.position` に積算している（ADR 0005 で 64 bit）。
しかし速度は ESC が返す rpm の生値だけで、受信の時刻も、途絶えたかどうかも、スナップショットからは分からない。

## Decision

1. **`dji,robomaster-encoder` を、モータを phandle で指す独立した device にする。** 実装は `drivers/encoder/robomaster_encoder.c`。1 つの device は 1 つの API しか持てないので、モータの device に encoder の API を足すことはできない。

2. **モータドライバの受信経路から、フレームごとに通知を受ける。** `include/drivers/motor/robomaster.h` に `robomaster_motor_set_rotor_callback()` を足す。受け手はモータ 1 台につき 1 つで、ロータ角、受信時刻（`k_cycle_get_32()`）、直前から続いているか（`continuous`）を受け取る。積算は他のエンコーダドライバと同じ `encoder_accum` に任せる。

3. **速度はロータ角の差分を、測った間隔で割って作る。** ESC の rpm は使わない。CanMotorMbed がロータ角の差分を速度にしていたので、Mbed で調整したゲインがそのまま合う（2026-10-09、ユーザーが選んだ）。

4. **`feedback-timeout-ms` より長く途絶えたら、積算をやり直して `position_epoch` を進める。** 判定はモータドライバが `continuous` として行う。`motor_get_feedback()` が stale を返す境界と同じなので、エンコーダの `-EAGAIN` とやり直しが食い違わない。

5. **それより短い取りこぼしは、やり直さずに差分で解く。** 1 フレームの取りこぼしで epoch を進めると、MdMotor が出力を止めてしまう。

6. **積算の始まりは、最初のフレームのロータ角とする。** ロータ角は 1 回転の中では絶対値なので、ADR 0002 の決定 10（絶対値エンコーダは最初の絶対位置から始める）に従う。モータドライバの `motor_feedback.position` とも揃う。

## Consequences

MdMotor の `encoders` に `dji,robomaster-encoder` を並べるだけで、ロータで閉じる構成になる（snippet `miniv4-md-rotor8`）。
制御のコアと block type には触らない。

counts は減速前のロータの counts（8192/rev）になる。
減速比は `encoder/gain` に含める。

ロータが 1 フレームの間に半回転（4096 counts）以上回ると、向きを取り違える。
9000 rpm で約 1229 counts/ms なので、3 フレーム続けて落とすと起きうる。
CanMotorMbed も同じ前提で動いていた。

速度の量子化は 1 count あたり約 1000 counts/s（1 ms 間隔）で、低速では粗い。
MdMotor の速度のローパスで均す前提である。

モータドライバの受信コールバックが、ロックを外した後に他のドライバのコードを呼ぶようになる。
受け手は割り込み文脈で呼ばれうるので、ブロックしてはいけない。

初期化の順序は devicetree の依存で決まる。
encoder のノードが `motor` の phandle を持つので、同じ優先度でもモータが先に初期化される（Zephyr は同じ優先度の device を devicetree の序数の順に並べる。`Z_DEVICE_INIT_SUB_PRIO`）。
`CONFIG_ENCODER_INIT_PRIORITY` を `CONFIG_MOTOR_INIT_PRIORITY` より小さくするとこの順序が崩れるので、`BUILD_ASSERT` で止める。
モータの初期化は自分の data を 0 で埋めるので、順序が逆になると登録した受け手が消える。
encoder 側は `device_is_ready()` でモータを確かめてから登録する。

## 却下した選択肢

**encoder が `motor_get_feedback()` をポーリングする。**
モータドライバに手を入れずに済む。
しかし速度を割る間隔が、受信ではなくポーリングの時刻になる。
ポーリングとフレームの位相がずれると、1 間隔に 0 フレームや 2 フレームが入り、速度が 0 と 2 倍を行き来する。
途絶えたことも、`-EAGAIN` の境界でしか分からない。

**`motor_feedback.position` を encoder の位置としてそのまま写す。**
積算をモータドライバに任せられる。
しかし `encoder_set_position()` のオフセットと `position_epoch` を encoder 側で別に持つことになり、モータドライバの積算がやり直したとき（今は無いが）に気付けない。
速度の問題も残る。

**ESC の rpm を速度にする。**
低速の量子化が細かい（1 rpm が約 137 counts/s）。
しかし ESC の中のフィルタの遅れが乗り、Mbed で調整したゲインと値が変わる。

**取りこぼしのたびに積算をやり直す。**
向きの取り違えは起きなくなる。
しかしフレームを 1 つ落とすたびに MdMotor が出力を止めるので、実用にならない。

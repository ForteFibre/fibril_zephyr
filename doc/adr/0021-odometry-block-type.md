---
status: Proposed
date: 2026-10-26
---

# 0021. 自己位置推定の入力を、PCMainFW の host が出していた topic と同じ形のブロック型にする

## Status

Proposed

## Context

PCMainFW（Main 2024 V1）は、R6093U と 2 本の接地エンコーダの値を、独自の CAN フレーム（0x300〜0x308）にして USB で PC に送っていた。
PC の `pc_main_interface` がそれを 2 本の topic にし、fibril_localization が読む。

| topic | 型 | 読む側 |
| --- | --- | --- |
| `encoder` | `fibril_localization/msg/OdometryEncoder`（int64 のカウント 2 つ、roll、pitch、yaw、`is_connecting`） | `basic_deadreckoning` |
| `imu/raw_data` | `sensor_msgs/msg/Imu` | `imu_covariance`（covariance と `frame_id` を埋めて `imu` に出す） |

`basic_deadreckoning` は、yaw を R6093U の向きのまま受け取り、`-yaw` にして使う。

RC26 MainAir V01 には、エンコーダの入力 2 本（J11、J12）と R6093U のソケット（J25）がある。
この基板に役目を移すとき、0x300 台のフレームは再現できない。
fibril_can の std ID はマスターが動的に割り当てるので、hub が PC に見せるバスの上で衝突しうる（ADR 0015）。

## Decision

1. **ブロック型 `Odometry` を足し、`encoder` と `imu` の 2 本の topic を、今の host の driver と同じ型と中身で出す。** RobstrideMotor（ADR 0006）と同じく、基板に移しても読む側を変えずに済むことを目的にする。ROS 側は remap でつなぐ。

2. **yaw はジャイロの向きのまま出す。** 符号を基板で反転すると、`basic_deadreckoning` の `-yaw` と二重になる。向きを直すのは、読む側を直すときにまとめて行う。

3. **カウントは int64 のまま運ぶ。** PCMainFW の host は int64 を float に入れて戻していたので、2^24 を超えると下位の桁が落ちた。fibril_can は i64 のフィールドを持てる。

4. **カウントはエンコーダの `position` を差分で追い、積算のやり直し（`position_epoch`）で跳ばない。** 自己位置推定はカウントの差を移動量として読むので、跳びはそのまま位置の誤りになる。

5. **1 つのノードに 1 インスタンスだけを持たせる。** `OdometryEncoder` はエンコーダを 2 つだけ持ち、自己位置推定は機体に 1 つである。

6. **snippet `rc26-odom` は `NODE_ROLE` を確保せず、どのデプロイ snippet にも重ねられるようにする。** RC26 の機体では、同じ MainAir が電磁弁や RobStride も駆動する（`rc26-main`）。自己位置推定のために、組み合わせごとのデプロイ snippet を増やさない（`miniv4-adc` と同じ形）。

## Consequences

`pc_main_interface` の driver を外し、bridge の topic を `encoder` と `imu/raw_data` に remap すれば、fibril_localization はそのまま動く見込みである。
ただし、fibril_can の bridge は topic の `header` を埋めない（v0.4.2）。
`basic_deadreckoning` は受けた `header.stamp` を TF の時刻にするので、bridge が stamp を埋めるまでは、この topic で TF を出す構成にできない。

`is_connecting` は、ジャイロの値が新しいかで決まる。
PCMainFW の host は 100 ms 以内に届いたかで決めていたが、ここでは IMU のドライバの stale の閾値（R6093U の既定で 50 ms）になる。

ジャイロから値が来ていない間、orientation は回転なしになる。
PCMainFW は全成分が 0 のクオータニオンを出していた。

`rc26-odom` は重ねる側のスキーマの `max_frames` を変えない。
2 本の topic は 1 本ずつフレームを使う見込みで、`rc26-robstride` に重ねると 16 のうち約 15 になる。

## 却下した選択肢

**エンコーダと IMU を別のブロック型にする。**
ほかの基板でも、エンコーダだけ、ジャイロだけを出せる。
しかし `OdometryEncoder` はエンコーダと yaw を 1 つのメッセージに持つので、今の読む側に合わせるには、どちらか一方の型に他方を混ぜることになる。

**`sensor_msgs/msg/Imu` と `nav_msgs/msg/Odometry` のような標準の型にする。**
robot_localization のような一般のノードが読める。
しかし fibril_localization を直す必要があり、基板を移す作業と、自己位置推定を作り直す作業が混ざる。

**基板でデッドレコニングを回し、位置を出す。**
PC の負荷と遅れが減る。
しかし `encoder_gain` などの param と TF の出し方を作り直すことになり、PCMainFW が担っていた範囲を超える。

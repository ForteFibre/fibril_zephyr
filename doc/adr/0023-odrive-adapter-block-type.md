---
status: Proposed
date: 2026-10-26
---

# 0023. ODrive を fibril_odrive_adapter の後継のブロック型として載せる

## Status

Proposed

## Context

ODrive は、これまで PC 上の 2 つのプロセスで駆動してきた。
ros_odrive の `odrive_can_node` が軸ごとに SocketCAN を CANSimple に訳し、fibril_ros の `fibril_odrive_adapter` がそれを `fibril_control_msgs` の `target` と `feedback` に訳す。
adapter は状態機械も持つ。起動したらキャリブレーションし、closed loop に入れ、指令の種類が変わったら IDLE を経由して制御モードを切り替え、エラーが出たら消して入れ直す。

[ADR 0022](0022-odrive-control-api-layering.md) で CANSimple の解釈は Zephyr のドライバに移った。
残っているのは、そのドライバをバスの向こうから使えるようにすることである。
RobStride では同じ問いに [ADR 0006](0006-robstride-fcan-node-parity.md) が答えた。ブロック型を置き、ROS 2 実装のインタフェースをそのまま名乗る。

ODrive で違うのは、ROS 2 実装が 2 層あることである。
どちらの層の顔を名乗るかで、載せる中身が変わる。

## Decision

### 1. adapter の顔を名乗り、`odrive_can_node` の顔は名乗らない

ブロック型 `ODriveMotor` は、モータごとに `target`（`fibril_control_msgs/msg/Target`）と `feedback`（`Feedback`）を出す。
adapter の param（`encoder_gain`、`invert`、`use_full_calibration`、`switch_stop_velocity_threshold`、`use_input_vel_ramp`）は同じ名前と同じ意味で持つ。
`fcan-ns = "motor{i}"` と bridge の `name_remap` で、adapter の `motorN` と同じ名前にできる。

ForteFibre で `odrive_can_node` を使っていたのは adapter だけで、`target` と `feedback` を使う側は adapter の存在を知らない。
ADR 0006 の 1 と同じ考え方（購読側が移動を知らなくてよい）を、実際に購読されている層に当てた。

### 2. adapter の状態機械を基板に移す

状態機械は `lib/odrive_sequencer` に置き、fibril_can にもドライバにも依存させない。
ブロック型は tick ごとにドライバのスナップショットを渡し、求められた操作をドライバに伝えるだけにする。
状態の名前と順序は adapter と同じで、`diagnostics.state` の数は adapter の `State` と同じ意味になる。

要求への答えの読み方も adapter に合わせる。
adapter は ros_odrive の `request_axis_state` の応答で判断していて、その応答は要求から 1 s 以上たった最初の heartbeat だった（closed loop 以外は `procedure_result` が BUSY でなくなるまで待つ）。
sequencer も同じ時刻に同じ値で判断する。ただし closed loop だけは、見えた時点で受け入れる。

ADR 0022 は「ドライバは故障から自動で戻さず、上位が判断する」とした。
その上位がこの状態機械である。エラーを 500 ms ごとに消し、消えたら入れ直すのは adapter と同じで、ADR 0022 とは矛盾しない。

### 3. 制御モードの切り替えは IDLE を経由する

`target` の `type` が変わったら、IDLE にして速度が `switch_stop_velocity_threshold` を下回るのを待ち、closed loop に入れ直してから新しい指令を送る。
ドライバは arm したまま Set_Controller_Mode を送り直して切り替えられるが、使わない。
adapter がこの手順を入れた理由は残っていない（fibril_ros の `121fb0a`「fix switch control mode」）。実機で確かめるまでは、adapter の振る舞いを変えない。

### 4. arm は指令が来てからにする

adapter は、キャリブレーションが済むと指令を待たずに closed loop に入れていた。
`ODriveMotor` は、キャリブレーションまでは進み、closed loop には `target` が来てから入れる。キャリブレーションも、master が RUNNING の間にしか始めない。

[ADR 0016](0016-command-loss-stop.md) で止めた後と同じ状態から始めるためである。
ODrive は、closed loop に入るときに前の `input_vel` と `input_torque` を残すかどうかを確かめていない（fibril_zephyr#19 のレビュー）。
基板だけが再起動して ODrive が生きていると、指令を待たずに入れれば前の速度で回りだしうる。
`feedback.is_ready` は closed loop に入るまで false になる。RobstrideMotor と同じで、指令を 10 ms ごとに出す側（`fibril_control` の steering controller など）には影響しない。

### 5. 指令の途絶で disarm する

ADR 0016 のとおり 2 段で止める。master が途絶えたら disarm し、`command_timeout_ms`（既定 200 ms）`target` が来なければ disarm する。
次の `target` で 4 と同じ手順で入れ直す。
adapter には途絶で止める仕組みが無かった。PC のプロセスが落ちれば `odrive_can_node` も落ち、ODrive の watchdog が止めていたからである。
基板では、ドライバが watchdog の餌を送り続けるので、止めないと最後の指令のまま回り続ける。

### 6. DUTY は RobstrideMotor と同じ無次元にし、全尺は param に置く

adapter は DUTY の値を Nm として、スケールせずにトルク指令にしていた。
`ODriveMotor` では、DUTY を -1..1 の無次元の指令として扱い、`duty_full_scale`（Nm）を掛けてトルク指令にする。
既定の 0 では DUTY を断る。

RobstrideMotor は全尺を devicetree の `max-current-ma` にしている。モータ自身に書く上限と兼ねるからである。
ODrive の上限は ODrive 自身の設定にあり（ADR 0022）、CANSimple では電流の上限もトルク定数も読めない。
DUTY を使う関節ごとに値が要り、ROS 側で調整できるほうが都合がよいので、param にした。

## Consequences

- PC から `odrive_can_node` と adapter の 2 つのプロセスが消える。ODrive のメッセージ型（`odrive_can`）にも依存しなくなる。
- adapter の param は、bridge の node に `<node>.motorN.encoder_gain` の形で載る。値はそのまま使える。
- diagnostics は `diagnostic_updater` ではなく `diagnostics` topic になる（RobstrideMotor と同じ）。
- `feedback` の周期は adapter の 50 ms から 10 ms になる。`output` には、adapter が 0 にしていたトルクの推定値が入る。
- ODrive 自身の `*_msg_rate_ms` で、heartbeat、encoder、iq、torques、error を出させる必要がある。どれかが止まっていると、対応する値が 0 になるか、状態機械が進まない。
- adapter の切り替えの手順の理由は分からないままである。実機で確かめて理由が分かれば、ここを直す。

## 却下した選択肢

### `odrive_can_node` の顔を名乗るブロック型にし、adapter は PC に残す

置き換える範囲が最小になる。
却下したのは 2 つの理由からである。
1 つ目に、ODrive がオフラインになったことを adapter に見せられない。`odrive_can_node` はフレームが来なければ publish しないが、fibril_can の S→M の topic は最後の値を周期的に送り続ける。adapter は status の途絶で電源断を判断していた。
2 つ目に、`set_configs` の `string` は、決まった形のフレームに載らない。
さらに、adapter の状態機械が PC に残るので、制御が USB とホストのスケジューリングを挟むままになる（ADR 0006 がゲートウェイ案を退けた理由と同じ）。

### arm したまま制御モードを切り替える

ドライバはそれができ、切り替えが速くなる。
adapter が IDLE を経由させた理由が分からないうちに変えると、実機で同じ問題を踏み直すおそれがある。

---
status: Proposed
date: 2026-10-06
---

# 0010. CanMotorMbed の後継を MdMotor ブロック型として載せ、can_md_controller に寄せる

## Status

Proposed

`lib/fibril_can_node/md_motor/` と、RoboMaster Mini V4 向けの snippet（`miniv4-can`、`miniv4-md`）がある。
ビルドだけを確かめていて、bridge に繋いだ確認も実機での確認もまだしていない。
とくに、bridge が `JointTrajectoryPoint` の `float64[]` を長さ 1 の `f32` に詰められることは、まだ通しで確かめていない。

## Context

CanMotorMbed（Mbed OS）は、エンコーダで閉ループにしたモータを、独自の CAN プロトコル（`0x170+id`）で ros2_can_toolbox の `can_md_controller` から駆動していた。
[ADR 0009](0009-motor-control-core.md) で、その制御のコアを `lib/motor_control/` に移した。
残るのは、それを fibril_can のバスに出すことである。

`fibril_can_bridge` は、スレーブのスキーマにある topic、service、param を、そのまま ROS の実体にする。
そのため、スキーマを `can_md_controller` の ROS インタフェースに合わせれば、利用側の多くをそのまま使える。
[ADR 0006](0006-robstride-fcan-node-parity.md) の RobstrideMotor も同じやり方を取った。

ただし、bridge だけでは揃えられないものがある。

- bridge は param を自分の node に `<node>.<ns>.<param>` の形で宣言する。`can_md_controller` は自分の node に `motorN.velocity.kp` を持っていた。
- topic の接頭辞は `/<schema の node>/motorN/` になる。`can_md_controller` は `/motor_driver/motorN/` だった。
- bridge は、ROS の配列がスキーマの長さより短いメッセージを捨てる。

## Decision

### 1. 完全な互換は狙わず、型、名前、ゲインの単位を揃える

メッセージの型（`fibril_control_msgs` の Target、Feedback、ResetEncoder）、`motorN` の下の topic と param の名前、ゲインの意味を `can_md_controller` に合わせる。
param の置き場所と topic の接頭辞の違いは受け入れる。
完全な互換が要るなら、ROS 側の adapter で埋める（別のリポジトリで扱う）。

### 2. host でしていた処理のうち、node で閉じるものは param にする

`invert`、`use_cascade_position`、`accel_ff_gain` は、`can_md_controller` が host で処理していた。
これらを node の param にする。
adapter を通さずに bridge から直接使っても、同じ動きになるようにするためである。

`invert` の符号は、エンコーダの入力とモータへの出力の 2 か所でだけ反転する。
指令、feedback、制御のコアは利用者側の向きのままになる。
PID が奇関数なので、host で指令と feedback に ±1 を掛けていた `can_md_controller` と同じ結果になる。

### 3. trajectory は `JointTrajectoryPoint` を直接受ける

`can_md_controller` は `JointTrajectoryPoint` の 1 点を、位置（カスケード）、速度の FF、duty の FF（`accel_ff_gain × 加速度`）に分けて送っていた。
node がこれを直接受け、同じ分け方で制御のコアに渡す。
スキーマは 3 つの配列を長さ 1 で宣言するので、どの配列も 1 要素以上埋める必要がある。
`can_md_controller` では速度と加速度を省略できたが、ここでは省略すると bridge が捨てる。

FF は 100 ms で切れる（ADR 0009）ので、trajectory は流し続ける。

### 4. 新しく届いた指令だけを当てる

M2S の topic は、最後に届いた値を返し続ける。
毎 tick 当て直すと、`target` と `trajectory` がモードを奪い合い、FF の期限が切れなくなる。
値を前回と比べるだけでは、エンコーダを見失って DUTY 0 に落ちた後に同じ指令を送り直しても、新しい指令だと分からない。

fibril_can の `read_<topic>(seq)` が返す世代は、フレームを受けるたびに変わる。
これを前回の値と比べ、変わった topic の指令だけを当てる。
同じ tick で両方が変わったときは、流し続ける側の `trajectory` を当てる。

世代はフレームごとに進むので、topic が複数のフレームに分かれて届くと、半分だけ新しい指令を当ててしまう。
bridge の frame planner は 64 バイト以下の topic を 1 つのフレームに収める（fibril_can の `frame_planner.hpp`）。
`target`（5 バイト）と `trajectory`（12 バイト）はこれに当たるので、分かれて届くことはない。
この 2 つにフィールドを足すときは、64 バイトを超えないようにする。

### 5. duty 1.0 は生の電流値 10000 にする

制御のコアは duty を返す。
モータには `MOTOR_OUTPUT_MODE_CURRENT` で、`duty × duty-full-scale` を送る。
`duty-full-scale` の既定は 10000 である。
CanMotorMbed は、機種によらず RoboMaster に `duty × 10000` を送っていた。
同じ値にしておけば、そのファームウェアで調整したゲインをそのまま使える。
モータの `max-current` は満量ではなく上限として効く。
上限を下げても、すべてのゲインの意味が変わることはない。

### 6. エンコーダの位置が作り直されたら止める

エンコーダの `position_epoch` が変わった tick は、無効なサンプルとして制御のコアに渡す。
閉ループのモードは DUTY 0 に落ち、master が指令を送り直すまで待つ。
跳んだ位置を追いかけて動き出すことを避けるためである。

ただし、`trajectory` を流している master は数 ms のうちに次の点を送ってくるので、そのまま新しい位置を追い始める。
止まったままにしたいなら、master が feedback の位置の跳びを見て止める。

## Consequences

- ゲインの表と、`target` と `feedback` を使うコードは、そのまま移せる。param を設定するコードは、bridge の node と名前に合わせて直すか、adapter を通す。
- 指令が途絶えたときに出力を止める仕組みはまだ無い。fibril_can の `master_lost_us` は S2M の送信を止めるだけで、ブロック型からは見えない。決めるまでは、master が止まっても最後の指令を保持し続ける。
- 制御のコアは 1 kHz の tick を前提にしている（ADR 0009）。direct のトランスポートは 1 ms の `k_timer` で tick を回すが、hub のトランスポートは `tick(); k_sleep(K_MSEC(1))` なので周期が 1 ms より長くなる。hub の基板でこのブロック型を使う前に直す。
- ADC のキャリブレーション（`sensor`、`sensor_state`、`trigger`、`trigger_cancel`）、feedback の `current`、診断はまだ無い。
- RoboMaster 自身のエンコーダを使う構成は、encoder class の shim ができるまで組めない。

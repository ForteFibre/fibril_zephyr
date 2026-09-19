---
status: Proposed
date: 2026-09-18
---

# 0006. RobStride を fibril_can のブロック型として載せ、ROS 2 実装のインタフェースに合わせる

## Status

Proposed

## Context

RobStride のアクチュエータは、これまで PC 上の `robstride_actuator_bridge_ros2` から SocketCAN 経由で駆動してきた。
モータ 1 台につき Topic `target` と `motion_target`、Topic `feedback`、Service `enable` / `fault_clear` / `reset_encoder` / `save_parameters`、そして ROS パラメータの一式が ROS グラフに出る。
台数分の diagnostic status も出る。

[ADR 0004](0004-robstride-control-api-layering.md) の判断でこのプロトコル解釈は Zephyr のドライバに移り、[doc/drivers/robstride.md](../drivers/robstride.md) の `robstride,bus` として実機で動く状態にある。
残っているのは、そのドライバをバスの向こうから使えるようにすることである。

ここで決めるのは 2 つある。
**どこに置くか**と、**ROS グラフから見た姿をどうするか**である。

置き場は [ADR 0003](0003-multi-app-structure.md) がすでに決めている。
fibril_can スレーブが担う機能は `lib/fibril_can_node/<type>/` のブロック型で、devicetree にノードを置くことだけがそれを image に載せる手段である。
RobStride も例外にする理由がない。

姿の方は自由度がある。
スキーマは自分で書くので、ROS 型を新しく起こすことも、既存のパッケージの型を名指しすることもできる。

そして ADR 0004 は、**ギア比と回転方向をドライバから意図的に外した**。
どちらも制御層が counts を物理量に変えるときのスケールに吸収されるもので、ドライバと制御層に割ると積算位置が追いにくくなる、というのがその理由だった。
その「制御層」が、いま作ろうとしているブロック型である。

## Decision

### 1. `RobstrideMotor` ブロック型を 1 つ置き、ROS 2 実装のインタフェースをそのまま名乗る

`target` は `fibril_control_msgs/msg/Target`、`feedback` は `fibril_control_msgs/msg/Feedback`、`reset_encoder` は `fibril_control_msgs/srv/ResetEncoder`、`enable` と `fault_clear` は `std_srvs/srv/SetBool`、`save_parameters` は `std_srvs/srv/Trigger` を名乗る。
`motion_target` だけは `robstride_actuator_bridge_ros2` 自身の `MotionTarget` を名乗る。

**モータが PC の SocketCAN から基板の上に移ったことを、購読側が知らなくてよい**のが狙いである。
新しい型を起こせばそのパッケージへの依存は消えるが、代わりに既存のノードと launch を全部書き換えることになる。
ブリッジは解決できない ROS 型を警告して飛ばすだけなので、`MotionTarget` を持たないマスターでもプロビジョニングは通る。

### 2. ギア比と反転はこのブロック型が持つ

スキーマパラメータ `gear_ratio` と `invert` を置き、指令は割ってモータ側に、feedback は掛けて負荷側に戻す。
バスに出る量はすべて関節の単位になる。

ADR 0004 がドライバから外したものの行き先がここである。
ROS 2 実装でも同じ位置（ノードのパラメータ）にあり、同じ式を使う。

### 3. モータ自身が守る上限は devicetree に残す

`max-current-ma` などはパラメータにしない。
機械に組み付けた時点で決まる値であり、ROS 2 実装がノードのパラメータに置いていたのは devicetree を持たなかったからにすぎない。

その帰結として、`Target` の `DUTY`（-1..1 の無次元指令）の全尺は **`max-current-ma` が決める**。
ROS 2 実装は機種の定格電流を全尺にしていたので、`max-current-ma` を定格に設定した deployment でのみ同じ意味になる。
機種表をこちら側に複製しないための割り切りで、binding とドライバ文書の両方に書いてある。

### 4. `TORQUE` は電流に変換せず、運動制御モードのトルク項で出す

ROS 2 実装は Nm をトルク定数で割って電流指令にしていた。
ドライバは運動制御モードを持っているので、kp と kd を 0 にしたフレームがそのままトルク指令になる。
トルク定数という機種依存の定数をもう 1 つ複製せずに済み、精度も落ちない。

### 5. 指令途絶でサーボを切る

パラメータ `command_timeout_ms`（既定 200 ms）を置き、この時間 m2s の指令が来なければ出力を落とす。

ROS 2 実装には対応物がない。
あちらではリンクが切れればプロセスごと消えるので、モータを止める者が必ずいた。
スレーブはマスターより長く生き残るので、**止める約束を明示的に書かない限り、最後の指令を握ったまま回り続ける**。
fibril_can はフェイルセーフをフレームワークの外に置いているので、ここが書く場所になる。

### 6. モータに触るのは tick だけにする

Service ハンドラは要求フラグを立てて返し、tick がそれを消費する。

ハンドラが走るのは `fcan_poll` を回すスレッドで、CAN hub 構成ではそれがドライバのスレッドになる（[doc/apps.md](../apps.md)）。
ハンドラ側がどのスレッドにいるかを知らずに済ませるための分離である。

## Consequences

- モータを基板に移す作業が、overlay を 1 つ書くことに縮む。購読側は変わらない。
- `fibril_control_msgs` と `robstride_actuator_bridge_ros2` の msg 定義が、このリポジトリの外にある契約になる。
  向こうでフィールド名が変われば、こちらのスキーマも同じ PR で追う必要がある。
- ゲインはモード別に持てない。ドライバが `robstride_gains` を 1 組しか持たないためで、ROS 2 実装が速度モードと位置モードで別の速度ループ Ki を書いていた分だけ再現しない。
  モード切り替えのたびにゲインを書き直す実装に踏み込むより、1 組で足りるかを実機で確かめるほうが先である。
- feedback の `current` を埋めない。ドライバが報告するのはトルクで、相電流ではない。
  ブリッジがスキーマにないフィールドを既定値で埋めるので ROS 型は満たされるが、値は 0 のままになる。
- 電源投入時の位置範囲（ROS 2 実装の `zero_sta`）は書き込まない。
  モータが自分の不揮発メモリに持つ値のままになるので、**同じ姿勢が個体によって違う値で読める**。
  合わせるには `robstride_set_parameter()` にプロトコルのインデックスを直接渡すことになり、ドライバの外に索引表を複製する。
  再ゼロが `reset_encoder` で足りるかを実機で見てから決める。
- 診断は `diagnostics` Topic 1 本になり、`diagnostic_updater` の集約は付いてこない。
  `diagnostic_msgs` は値を文字列の写像で運ぶので、固定レイアウトのフレームからは組めない。

## 却下した選択肢

### ROS 2 実装をそのまま使い、基板は CAN のゲートウェイに徹する

gs_usb で RobStride のバスを PC に見せれば、`robstride_actuator_bridge_ros2` は無改造で動く。
却下したのは、制御周期が USB とホストのスケジューリングを挟むままになるからである。
基板の上に置けば、指令と feedback は 5 ms のドライバ周期で閉じる。

### 新しい msg パッケージを起こして ROS 型を定義し直す

`fibril_robstride_adapter/msg/MotionTarget` への依存を消せる。
却下したのは、それが消す依存より、既存の購読側を書き換える手間のほうが大きいからである。
`MotionTarget` は 5 つの f32 を並べただけの型で、名前を変える価値がない。

### `Target` の `DUTY` に機種ごとの定格電流表を持つ

ROS 2 実装と同じ全尺にできる。
却下したのは、機種表の複製がドライバとこちらの 2 か所になるからで、[doc/drivers/robstride.md](../drivers/robstride.md) が「`model` を間違えると失敗せずに全部の物理量がずれる」と書いている表をもう 1 つ増やすことになる。
`max-current-ma` を全尺にすれば、表はドライバの中に 1 つで済む。

### Service ハンドラから直接モータを叩く

ドライバは spinlock で自分の状態を守っているので、これ自体は安全である。
却下したのは安全性ではなく、**どのスレッドから呼ばれるかがトランスポートの選択で変わる**ことを機能の実装が意識せずに済むほうがよいからである。
要求フラグ 1 つで済む値段なら、そちらを払う。

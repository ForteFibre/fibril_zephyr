---
status: Proposed
date: 2026-10-14
---

# 0016. 指令の途絶を、master 全体の途絶と topic ごとの途絶の 2 段で捉えて出力を止める

## Status

Proposed

## Context

fibril_can は、指令が途絶えたときにサーボを切ることをアプリの責務にしている。
fibril_can の `master_lost_us` は、ノードを SUSPENDED にして S2M の送信を止めるだけで、出力には触れない。
そのため、MdMotor は master が止まっても最後の指令を握ったまま回り続けていた（ADR 0010 の Consequences）。

RobstrideMotor は、ADR 0006 の 5 で `command_timeout_ms`（既定 200 ms）を置いた。
しかし、生成ラッパの `read_target()` は topic の最後の値を毎 tick 返すので、tick は指令を 1 度受けた後、毎 tick「指令が来た」と数えていた。
タイムアウトは一度も切れず、ADR 0006 の約束は守られていなかった。

fibril_can v0.3.4 と、fibril_zephyr の `fibril_fcan_node_state()` で、ブロック型の tick から次の 2 つが読めるようになった。

- master 全体の途絶：ノードの状態が `FCAN_STATE_RUNNING` でなくなる。`apps/node` の `master_lost_us` は 300 ms である。
- topic ごとの新しい指令：`read_<topic>(seq)` の世代が前回から変わる。

CanMotorMbed は、ホストの heartbeat が 1 s 途絶えると全モータを `reset_all()` にし（DUTY 0、ゲイン 0、エンコーダのリセット）、DUTY と SPEED の指令が 200 ms 来ないと `set_duty(0)` にしていた。
位置制御は「定期的に値が来るわけではない」ので、指令の途絶の対象から外していた（`src/simple_protocol.cpp`）。

## Decision

1. **master 全体の途絶では、すべてのモードの出力を止める。** tick の先頭でノードの状態を 1 回読み、RUNNING から外れた tick で止める。
   - MdMotor は、制御のコアの `stop()` で DUTY 0 に落とす。slew 制限を越えてその場で落とし、フィードフォワードも捨てる。ゲインとエンコーダは残す。param はノードが持ち続けるので、Mbed のように 0 にしない。
   - RobstrideMotor は、指令を受けていない状態に戻し、出力を切る。`command_timeout_ms` が 0 の構成も止まる。
   - RUNNING に戻っても動かさず、新しい指令を待つ。
2. **止めるのは RUNNING から外れたエッジの 1 回だけにし、その後に届いた指令は捨てない。** hub 構成では、ノードの状態と topic を `fcan_poll` のスレッドが書き、tick は別のスレッドで読む。そのため、RUNNING に戻す frame の指令を、状態がまだ SUSPENDED の tick で見ることがある。非 RUNNING の間の指令を捨てると、1 度だけ送られた位置指令が永久に失われる。止める処理は、その tick の指令の適用より前に置く。
3. **topic ごとの途絶は、型ごとに対象を決める。**
   - MdMotor の `command_timeout_ms`（既定 200 ms）は、DUTY と VELOCITY にだけ効かせる。POSITION と trajectory は位置を保持する指令なので、master が生きている限り保持する（Mbed と同じ）。trajectory のフィードフォワードは、もともと 100 tick で失効する（ADR 0009）。
   - RobstrideMotor の `command_timeout_ms` は、ADR 0006 のとおり、位置を含むすべての指令に効かせる。
   - 途絶で止めたら、新しい指令まで止めたままにする。止める処理は 1 回だけ行う。
   - 拒否した指令（値が NaN、扱えない `type`）は、指令として数えない。数えると、無効な指令を流し続けるだけで古い出力が止まらない。
5. **RobstrideMotor は、出力が切れた後は `enable` で戻さない。** master の途絶でも `command_timeout_ms` でも同じである。ドライバは切れる前の目標を持ったままなので、`enable` で戻すと古い目標へ動く。ADR 0006 の「`enable` も通信として数える」は、出力が切れる前に期限を延ばす意味にだけ残る。
4. **新しい指令は topic の世代で判断し、1 度だけ適用する。** 保持中の指令を毎 tick 当て直さない。RobstrideMotor では、これで `motion_target` を 1 度受けた後の `target` が上書きされ続ける不具合も消える。

## Consequences

- MdMotor の DUTY と VELOCITY は 200 ms で、POSITION と trajectory は master の途絶（300 ms）で止まる、2 段の構成になる。速度を流す master は、200 ms より短い間隔で送り続ける。
- RobstrideMotor は、今まで一度も効いていなかった `command_timeout_ms` が効くようになる。位置を 1 度送って保持させていた master は、200 ms より短い間隔で送り続けるか、`command_timeout_ms` を 0 にする（その場合も master の途絶では止まる）。
- RobstrideMotor の保持中の目標は当て直さないので、`reset_encoder`、`gear_ratio`、`invert` の変更は次の指令から効く。今までは、変えた tick から保持中の目標が新しい座標系で読み直され、モータが動いた。MdMotor の `reset_encoder` は、制御のコアが新しい位置を読むので、保持中の目標がすぐ別の場所を指す。2 つの型でこの振る舞いは揃っていない。
- Solenoid は止めない。弁の状態を勝手に変えると危ない機構があるので、master が途絶えても保持する。
- 停止は slew 制限を越える。Mbed の topic ごとの途絶は `set_duty(0)` で slew に沿って下げていた。

## 却下した案

- **非 RUNNING の間は毎 tick 出力を抑え、指令を適用しない。** 上の 2 の理由で、復帰の frame の指令を落とす。
- **MdMotor の `command_timeout_ms` をすべてのモードに効かせる。** `can_md_controller` の利用側は位置を 1 度送って保持させるので、ファームウェアを入れ替えると振る舞いが変わる。master の途絶は 1 で捉えられる。
- **Mbed と同じく、master の途絶でゲインを 0 にし、エンコーダをリセットする。** Mbed はホストが接続のたびに設定を送り直す前提だった。fibril_can の param はノードが保持するので、0 にすると復帰の後に master が設定し直す手順が別に要る。エンコーダのリセットは、復帰後の位置の原点を変える。

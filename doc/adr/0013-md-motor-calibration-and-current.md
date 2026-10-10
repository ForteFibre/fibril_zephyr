---
status: Proposed
date: 2026-10-10
---

# 0013. MdMotor の ADC キャリブレーションを AdcPort ブロック型で入れ替え可能にし、電流ベースラインは測り終えてから応答する

## Status

Proposed

ビルドと、判定のロジックの ztest（`tests/lib/motor_control` の `sensing`）だけを確かめた。
bridge に繋いだ確認と、実機での確認はまだしていない。

## Context

[ADR 0010](0010-md-motor-block-type.md) は、CanMotorMbed の ADC キャリブレーションと feedback の `current` を後回しにした。
CanMotorMbed の該当部分は次のとおりである（`CanMotorMbed@b3f1cb4`、`@165fab5`）。

- **キャリブレーション**：5 ms ごとに各ポートの |値| をしきい値と比べ、またいだら、そのときのエンコーダの位置を送る。位置は変えない。miniv4 のポートは 13 本で、0〜7 は RoboMaster のトルク（生の値 / 10000）、8〜12 は基板のアナログ入力 ADC__0〜ADC__4 である。どのモータがどのポートを見るかは、ホストが設定する。
- **電流ベースライン**（`tmp/saramander/robomaster` ブランチ）：DUTY 0 で静止している間の電流を 0.5〜5 s 平均し、電流の零点にする。動いたら黙って中止する。ROS 側（ros2_can_toolbox の `feature/current-baseline-v2`）は、待ち時間だけ sleep してから、無条件に成功を返していた。

Mbed の実装には直すべき点が 2 つあった。

- ポートの前回値を、モータのループの中で更新していた。そのため、同じポートを 2 台のモータが見ると、2 台目はエッジを取りこぼす。
- エッジが無いと前回値を更新しないので、一括送信の値が古いまま残る。

`fibril_can_bridge` は service の応答を `service_timeout_ms`（既定 1.5 s）待つ。
fibril_can の ACCEPTED（§7.2）を返すと、bridge はその時点から待ち時間を数え直し、最終応答を待つ。

## Decision

### 1. 入力は `AdcPort` ブロック型にし、devicetree の子ノードで並べる

`fibril,fcan-adc-port` ノードの子 1 つが、1 つのポート（1 つのインスタンス）になる。
子は、ADC のチャネル（`io-channels`）か、motor class の device（`motor`）のどちらか 1 つを指す。
ADC は満量を 1 とする 0〜1 に、モータは報告する電流 / 10000 に正規化する。
しきい値はインスタンスの param `adcN/threshold` に置く。`can_md_controller` と CanMotorMbed の `feat/fibril-can` も、この名前を使っていた。

ポートの並びは基板ではなくデプロイ snippet が決める（2026-10-10、ユーザーの指示）。
基板の devicetree は、引き出しているアナログピンのチャネルを定義するだけで、ADC を有効にしない。
miniv4 には、CanMotorMbed と同じ並びの `miniv4-adc-torque8-ext5` と、基板の入力だけを並べる `miniv4-adc-ext5` を置く。

### 2. 判定はモータごとに持ち、ポートは一括でサンプリングする

`adc_port` の tick が、5 ms ごとにすべてのポートをサンプリングし、世代を進める。
`md_motor` は、世代が進んだ tick で、自分が見るポートを 1 回だけ判定する。
前のレベルはポートではなくモータの判定器（`motor_control::LevelWatch`）が持つ。
そのため、同じポートを見るモータは、どれもすべてのエッジを受け取る。
Mbed と同じく、ヒステリシスもデバウンスも入れない。

### 3. `trigger` と `trigger_cancel` はファームウェアに入れない

`WaitSensorTrigger` は、`sensor` を待って最初に一致したエッジを返す、ホスト側の複合的な service だった。
fibril_can の service は bridge のタイムアウトに縛られるので、いつ来るか分からないエッジを待つのには向かない。
ROS 側の adapter が `sensor` を購読して作る（2026-10-06 の決定）。

CanMotorMbed の `feat/fibril-can` にある `trigger` は、位置に着いたかを判定するもので、名前が同じだけの別物である。
これも引き継がない。

### 4. `feedback.current` は生の値 / 1000 にする

CanMotorMbed の `read_current()` と同じ値にする（2026-10-10、ユーザーが選んだ）。
ホストが持っている offset やしきい値を、そのまま使うためである。
物理量（A）ではない。

符号と零点は次の順に掛ける。

1. `current/polarity` の符号を掛ける。
2. `current/offset` を引く。
3. 最後に測ったベースラインを引く。
4. `invert` を掛ける。

Mbed は `polarity` と `offset` を同じ 1 つの変数で持ち、ベースラインでそれを上書きしていた。
ここでは、ベースラインを `current/offset` を引いた後の残りとして測る。
こうすると、測り終えた後の零点は Mbed と同じになる。
そのうえ、スレーブが param を書き換えられない（fibril_can §8）という制約とも衝突しない。

### 5. 電流ベースラインは ACCEPTED を返し、測り終えてから応答する

`calibrate_current_baseline`（`std_srvs/Trigger`）は、受け付けると ACCEPTED を返す。
測り終えると成功、静止していなかったか電流が読めなかったら APP_ERROR を、`fcan_svc_complete` で返す（2026-10-10、ユーザーが選んだ）。
測る時間（`current/baseline_duration_s`）は 0.05〜1 s に丸める。
bridge の既定のタイムアウト（1.5 s）に収めるためである。

測っている間に重ねて呼ぶと、BUSY を返す。
静止の判定は、エンコーダが有効で、DUTY モード、|duty| < 1e-3、|速度| < 0.1 / 1 ms tick（Mbed の値を 1 秒あたりに直したもの）のすべてを満たすことである。

## Consequences

- 最終応答は tick から送る。direct のトランスポートでは、tick は `fcan_poll` と同じスレッドで動くので問題ない。hub のトランスポートでは別のスレッドになり、hub スレッドと同時に `hal.send` を呼ぶことになる。miniv4 は direct なので影響しない。fibril_can v0.3.2 の `fcan_svc_complete` はその場で送るためで、ForteFibre/fibril_can#64 で直す（応答を預け、`fcan_poll` が送る）。その版に上げるまで、MdMotor を hub の基板に載せない。
- ベースラインの時間を 1 s より長くするには、bridge の `service_timeout_ms` を延ばしたうえで、上限を変える。
- ポートは、`fcan_poll` を回すスレッドではなく tick で読む。ADC は 5 ms ごとに数チャネルを順に読み、そのたびに tick を少しブロックする。
- Mbed のように、ホストとの接続が切れたら監視を止める処理は無い。fibril_can の master の喪失は、ブロック型からは見えないためである（ADR 0010）。
- `adc/port` は u8 で、255 を「モータと同じ番号」に使う。256 本以上のポートは持てない。
- `sensor` の topic を足したので、モータ 8 台の構成は topic が 32 本になる。`snippets/miniv4/schema/md.yaml` の `max_frames` を 32 から 48 に上げ、master の frame planner に余裕を残した。

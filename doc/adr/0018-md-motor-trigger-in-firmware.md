---
status: Proposed
date: 2026-10-16
---

# 0018. MdMotor の trigger をファームウェアに入れる

## Status

Proposed

ビルドだけを確かめていて、bridge に繋いだ確認も実機での確認もまだしていない。

## Context

`can_md_controller` の `trigger`（`fibril_control_msgs/srv/WaitSensorTrigger`）は、センサのエッジが来るまで応答を返さない service である。
[ADR 0013](0013-md-motor-calibration-and-current.md) の 3 節は、これをファームウェアに入れず、ROS 側の adapter が `sensor` を購読して作るとした。
fibril_can の service は bridge のタイムアウト（`service_timeout_ms`、既定 1.5 s）に縛られ、いつ来るか分からないエッジを待てなかったためである。

fibril_can の ADR-0018（SPEC §7.6）で、ACCEPTED の後の呼び出しにはタイムアウトを掛けず、ノードの状態で終わらせることになった。
スレーブの runtime は STOP、再 DISCOVER、FAULT のときに受理した呼び出しを破棄し、master は HEARTBEAT の node_state と boot_id を見て同じ呼び出しを失敗で返す。SUSPENDED では、どちらも終わらせない。
これで、いつ終わるか分からない呼び出しをファームウェアの service として書ける。

## Decision

`trigger` と `trigger_cancel` を MdMotor の service にし、待つ向きを param `adc/direction` に置く。

- `trigger` は ACCEPTED を返し、向きが一致するエッジの tick で `{triggered_position, success: true}` を返す。
- 数えるのは待ち始めた後のエッジだけで、待ち始めたときの向きでは返らない。`sensor_state` が有効にした直後に出す向きと同じものを一致とみなすと、すでにセンサの上にいる機構が動かないまま原点を決めてしまう。
- 監視は `sensor_state` と別に持つ。`sensor` の配信を止めたり、向きの報告を動かしたりしない。
- `trigger_cancel` と 2 回目の `trigger` は、待っている呼び出しを `success=false` で終わらせる。`can_md_controller` と同じである。
- `adc/direction` は 0 が FALL、1 が RISE で、`can_md_controller` の `adc.direction` と同じ値である。2 はどちらの向きでも一致する。`can_md_controller` の 2（BOTH）はどの向きとも一致せず、待ちが終わらなかったので、値の意味を変えても困る利用者はいない。
- 待つ時間に上限は持たない。ROS の service は呼び出し側が待つのをやめたことを伝えないので、待ちをやめたい利用者は `trigger_cancel` を呼ぶ。
- runtime が呼び出しを破棄したら、`fibril_fcan_call_live()` が false になったのを tick で見て、待つのをやめる。master を見失って SUSPENDED になっても、`trigger` は待ち続ける（モータは ADR 0016 のとおり止まる）。

## Consequences

- `can_md_controller` の `trigger` を使う host（rc25 の calibration_helper）は、adapter を介さずに bridge から使える。
- 最終応答は tick から runtime に預ける。預け先が空いていなければ（`FCAN_E_BUSY`）、次の tick にやり直す。待っている `trigger` を 2 回目の `trigger` が置き換えたとき、置き換えられた方と新しい方の応答が同時に待つことがあるので、モータごとに 2 つまで持つ。応答が残っている間に届いた `trigger` は BUSY で断る。
- `adc/direction` を param の最後に足したので、ほかの param の on_params_changed のビットは動かない。

## Related

- [ADR 0013](0013-md-motor-calibration-and-current.md) の 3 節（この ADR が置き換える）
- fibril_can の ADR-0018 と SPEC §7.6

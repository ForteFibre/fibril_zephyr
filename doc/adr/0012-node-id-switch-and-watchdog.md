---
status: Proposed
date: 2026-10-09
---

# 0012. node_id を基板の ID スイッチの値そのままにし、watchdog を tick で叩く

## Status

Proposed

## Context

ADR 0003 の決定 7 は、同じイメージを複数の基板に焼けるように node_id を実行時に読むと決め、`lib/node_id/` ができるまでは Kconfig の整数で与えるとしていた。
RoboMaster Mini V4（と V3、CanMotor Tourobo 2023）は 4 ビットのロータリスイッチを持ち、CanMotorMbed はこの値で個体を区別していた。
CanMotorMbed の fibril_can 版（`feat/fibril-can`）は、スイッチの値をそのまま node_id にしていた。

板の devicetree はこのスイッチを `gpio-keys` として書いていたが、入力イベント源として使うものではなかった。

同じ板は IWDG を持つ。
CanMotorMbed は 1 s の timeout で起動し、イベントキューから 100 ms ごとに叩いていた。

## Decision

1. **スイッチを `fibril,id-switch` の binding で書き、`chosen` の `fibril,node-id` で指す。** `gpios` に最下位ビットから並べる。miniv4_4 の 3 ビットの DIP スイッチも同じ binding で書ける。

2. **スイッチの値をそのまま node_id にする。** ロータリスイッチなら 0〜15 になる。CanMotorMbed の fibril_can 版と同じである（2026-10-09、ユーザーが選んだ）。

3. **`chosen` があれば `CONFIG_FIBRIL_NODE_ID_SWITCH` が立ち、`CONFIG_FIBRIL_NODE_ID` は消える。** スイッチの無い板は、今までどおりデプロイ snippet の Kconfig で決める。

4. **watchdog は `apps/node` が、機能の tick を回すたびに叩く。** 起動は transport に処理を渡す直前、timeout は既定 1000 ms（`CONFIG_APP_WATCHDOG_TIMEOUT_MS`）。`CONFIG_WATCHDOG` と `watchdog0` の alias が揃ったときだけ有効になる。デバッガで止めている間は watchdog も止める。

5. **tick を持つ機能が無いイメージでは、watchdog を起動しない。** 叩く場所が無いからである。

## Consequences

同じ snippet で焼いた miniv4 が、スイッチを変えるだけで別のノード（`md_controller<スイッチの値>`）になる。

node_id の帯はスイッチの幅で決まり、0〜15 に限られる。
同じバスに、スイッチの値が同じ別の役割の基板を載せると衝突する。
fibril_can は HEARTBEAT で衝突を検出して後発を FAULT に落とす（fibril_can の SPEC §5.3）ので、黙って混ざることはない。
帯を分けたくなったら、snippet が base を足す形に広げられる（今回は採らなかった）。

watchdog は制御ループ（とそれを回すスレッド）が止まったことを捉える。
hub 構成では tick が `fcan_poll` と別のスレッドで回るので、`fcan_poll` だけが止まっても watchdog は叩かれ続ける。

STM32 の IWDG は `wdt_setup` を呼ぶまで動かないので、`CONFIG_WATCHDOG` を立てただけでリセットがかかることはない。
一度動かした IWDG は止められない。

## 却下した選択肢

**node_id = base + スイッチの値（base は snippet）。**
役割ごとに帯を分けられる。
しかし CanMotorMbed の fibril_can 版と割り当てが変わり、今のところ帯を分ける必要のある機体が無い。

**node_id の読み方を Kconfig で選ぶ。**
板が ID スイッチを持つことは配線の事実であり、それを書く場所は devicetree にある（ADR 0003 の「機能を Kconfig で選ぶ」を却下したのと同じ理由）。

**watchdog を独立したタイマで叩く（CanMotorMbed と同じ）。**
起動の順序を気にしなくてよい。
しかし示せるのはタイマ割り込みが生きていることだけで、制御ループが止まってもモータに最後の出力が残る。

**watchdog をブロック型（MdMotor）の中で叩く。**
叩く場所が制御そのものになる。
しかし watchdog は基板 1 枚に 1 つで、ブロック型は複数載りうる。
どのブロック型が叩くかを決める必要があり、ADR 0003 の責務分け（アプリケーションがブロック型を知らない）とも合わない。

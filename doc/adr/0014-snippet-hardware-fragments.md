---
status: Proposed
date: 2026-10-11
---

# 0014. 基板に繋がるハードウェアを共有の devicetree 断片に切り出し、未接続を許すデバイスは最大の構成で書く

## Status

Proposed

## Context

ADR 0003 は、焼く単位を snippet で表し、トランスポート層とデプロイ層の 2 層に分けた。
miniv4 のデプロイ snippet が 4 つになった時点で、overlay には次の 3 種類のものが混ざっていた。

- 基板に繋がりうるハードウェア：RoboMaster のバスとモータ、ロータ角のエンコーダ、qdec、ADC。
- バスに名乗る中身：`fibril,fcan-*` のノード。
- 基板が常に必要とする設定：CAN トランシーバの STB の gpio-hog、RNG。

その結果、次のことが起きていた。

- RoboMaster のバスとモータ 8 台は `miniv4-md-qdec4` と `miniv4-md-rotor8` の両方に書かれていた。ADC の 5 チャネルの割り当ては 2 つの `miniv4-adc-*` に書かれていた。`md-qdec4.conf` と `md-rotor8.conf` は同じ内容だった。
- `miniv4-adc-torque8-ext5` は、デプロイ snippet が定義する `rm_motor1..8` を参照していた。そのため `miniv4-md-rotor8` としか組めず、ADR 0003 の「overlay は基板のラベルしか参照しない」にも反していた。
- STB の hog と RNG は経路ごとに書かれていた。P5 で miniv4 に hub の経路を足すと、もう一度書くことになる。rc26 の CAN1 の hog はデプロイ snippet（`rc26-robstride`）にあった。

基板の種類と、基板に繋ぐデバイスの組み合わせは、これからも増える。
組み合わせを 1 つ足すたびに、ハードウェアの記述を写すことになっていた。

## Decision

1. **基板に繋がりうるハードウェアは、`dts/fibril/` の断片に書く。** モジュールの `dts_root` が `.` なので、`dts/` は devicetree の include path に入っている。overlay は `#include <fibril/...>` で取り込む。
   - 基板に依存しない断片：`robomaster-x8.dtsi`（ESC 8 台とロータ角、`cans` は書かない）、`can-bitrate.h`（fibril_can のバスとアクチュエータのバスのビットレート）。
   - 基板ごとの断片：`dts/fibril/<board>/`。miniv4 には `robomaster.dtsi`（バスを FDCAN2 と FDCAN3 に割り当てる）、`qdec.dtsi`、`adc.dtsi` がある。さらに、`fcan-adc-port` の子ノードを並べる `adc-ports-torque8.dtsi` と `adc-ports-ext5.dtsi` がある。この 2 つはノードの中で include する。
   - 断片には include guard を置く。overlay はすべて 1 回の C プリプロセッサの実行で処理されるので、2 つの snippet が同じ断片を include しても 1 回しか展開されない。

2. **デプロイ snippet の overlay には、`fibril,fcan-*` のノードだけを書く。** `motors` と `encoders` の並び、`fcan-adc-port` の子の並びは、バス上のインスタンス番号であり、ROS 側との約束である。断片には入れず、ハードウェアを最大の構成にしてもこの数は変えない。

3. **未接続を許し、空回りのコストが小さいデバイスは、断片の中で基板が出せる最大の構成で常に定義する。** 判定には次の 3 つを使う。
   - ハードウェアが無くても init が成功するか。
   - 未接続の間、周期ごとにバスの待ちやタイムアウトが起きるか。
   - timer や CAN コントローラのような資源を占有するか。

   これに当たるのは、RoboMaster の ESC 8 台とロータ角、miniv4 の qdec 6 本（ENC0〜ENC5）、ADC のチャネルである。
   - RoboMaster：繋がっていない ESC は feedback を返さないだけで、モータは `-ENODATA`、ロータ角はデータ無しになる。ドライバは feedback で所在を知ったモータにしか指令のフレームを送らないので、モータの CAN のトラフィックも増えない。バスに電源が無くても `can_start` を後から再試行する。
   - qdec：外部に依存しない。基板の文書の表の割り当てなら、6 本の timer は衝突しない。
   - ADC：読むまでサンプリングしない。

   AMT21 は RS485 で応答を待ってタイムアウトし、RobStride は周期的に指令を送って応答を待つ。この 2 つは、繋いだものだけを書く。

4. **STB の gpio-hog と RNG は、基板の DTS に置く。** どちらも、どの経路でも同じ配線の事実である（2026-10-11、ユーザーが選んだ）。

5. **役割（ノードヘッダと Kconfig）は `snippets/common/<役割>/` に置き、基板ごとのデプロイ snippet が `snippet.yml` の相対パスで取り込む。** `MdMotor` を持つ snippet は `snippets/common/md/md.conf` を使う。この conf が `snippets/common/md/schema.yaml` を指す。

## Consequences

デプロイ snippet の overlay は、include とペアの表だけになった。
同じ基板でモータとエンコーダの組み合わせを足すときは、`motors` と `encoders` を並べた 20 行ほどの overlay を書けば済む。
別の基板に RoboMaster を載せるときは、`dts/fibril/<board>/robomaster.dtsi` に `cans` を書くだけで済む。

`miniv4-adc-torque8-ext5` は、どの `miniv4-md-*` とも組めるようになった。
駆動しないモータのトルクのポートは、サンプル無しとして読める。
`-S` の順序を入れ替えても、ノードの並びが変わるだけで、devicetree の中身と schema の blob は変わらない。

最大の構成の分だけ、イメージが大きくなる。
`miniv4-md-qdec4` では、text が約 1.7 KB、bss が約 1.5 KB 増えた（ESC 4 台、ロータ角 8 個、qdec 2 本の分）。
`miniv4-md-rotor8` は変わらない。
qdec の断片を使うイメージは、TIM1〜TIM5 と TIM8 をすべて押さえる。
miniv4 でそれらの timer を PWM などに使うデプロイが現れたら、`qdec.dtsi` を include せず、使う入力だけを overlay に書く。

STB を基板が持つので、トランシーバはどのビルドでも動作状態になる。
`app/` の動作確認のビルドも含む。
使わないトランシーバは、動作時の電流を流すだけである。

## 却下した選択肢

**最大の構成を基板の DTS に `status = "okay"` で書く。**
include も要らなくなる。
しかし、RoboMaster の ESC やエンコーダは、基板の外のコネクタに繋ぐものである。
基板の DTS に書くと、`app/` やサンプルのビルドにまで、モータのドライバと timer の占有が入る。

**断片を `snippets/<board>/` に置き、相対パス（`#include "../hw/x.dtsi"`）で取り込む。**
2 つの snippet が同じ断片を別の相対パスで include すると、CMake が受け取る依存の一覧に、同じファイルが 2 つの名前で載る。
ninja が `multiple rules generate` で止まる。
include path から `<...>` で取り込めば、名前は 1 つになる。

**部品ごとの断片（モータ 1 台、qdec 1 本）にして、デプロイが使う分だけ include する。**
資源は最小になる。
しかし、組み合わせごとに include の並びを書くことになり、今回減らしたかった写しが戻る。
未接続を許すデバイスに限れば、最大の構成にするコストは小さい。

**`fcan-*` のインスタンスも最大の構成にする（`miniv4-md-qdec4` を motor0..7 にする）。**
ROS 側から見えるモータの数が変わり、`can_md_controller` との約束が崩れる。

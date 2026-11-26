---
status: Proposed
date: 2026-10-15
---

# 0017. CAN の健全性は、devicetree に書かない組み込みの機能として全てのイメージで出す

## Status

Proposed

## Context

fibril_can v0.3.5 は、基板の CAN の健全性を普通の S→M topic として bridge に渡すブロック型 `CanPortDiag` と `CanHubDiag` を持つ（fibril_can の ADR-0017）。
ブロック型の定義（`zephyr/schema/can_diag.yaml`）と、値を読む関数（`fibril_can_zephyr/can_diag.h`、`can_hub.h`）は fibril_can にあり、topic を埋める処理はアプリが書く。

このリポジトリの機能（ブロック型の実装）は、devicetree の `fibril,fcan-*` ノードから instances を作り、ノードがあるときだけコンパイルされる（ADR 0003、[doc/apps.md](../apps.md) の「schema は 3 種類の断片から組む」）。
しかし、CAN の健全性を出す対象は、機能のハードウェアではなく、トランスポートが使うコントローラである。

- direct のトランスポートは、`zephyr,canbus` の 1 つのコントローラを使う。
- hub のトランスポートは、`fibril,can-hub` の peer を使い、hub 自体の転送カウンタも持つ。

どの基板も、どちらかのトランスポートで fibril_can のバスに出る。

## Decision

CAN の健全性を出す機能 `can_diag`（`lib/fibril_can_node/can_diag/`）を、devicetree に書かない組み込みの機能として、全てのイメージに既定で入れる。

- **instances はトランスポートから決める。** direct なら `CanPortDiag` を 1 つ、hub なら peer の数だけの `CanPortDiag` と `CanHubDiag` を 1 つ作る。ns は `can{i}` と `can_hub` で、ROS 側は `/<node>/can{i}/diag`、`/<node>/can_hub/diag` になる。hub の `CanPortDiag` の i 番目は、`peers` の i 番目（hub の port i + 1）である。
- **instances の断片は、devicetree 由来のものとは別のプロパティ（`fibril_can_node_builtin_instance_schemas`）に積む。** `apps/node` の「devicetree が何も有効にしていないイメージを拒む」検査は、devicetree 由来の断片だけで行う。
- **ブロック型の `type.yaml` は fibril_can のものを使う。** bridge、`fibril_can_diagnostics`、各 firmware が共有する約束なので、ROS の msg と同じ場所にある。`ZEPHYR_FIBRIL_CAN_MODULE_DIR` から参照する。
- **`CONFIG_FIBRIL_CAN_NODE_CAN_DIAG`（既定 y）で外せる。** `STATS` と `CAN_STATS` は `imply` で入れ、RAM の足りない基板はエラーカウンタだけを外せる。
- 値は tick で 1 秒ごとに読む。CAN_STATS は `fcan_zephyr_can_stats_totals` でコントローラごとに持ち越す。bus-off の回数は、direct では `fcan_zephyr_bus_off_counter` で数え（このリポジトリで直結のコントローラに state change callback を置くものは無い）、hub では hub のドライバが数えた値を読む。
- `CanHubDiag` の `fwd_count` は 8 port 分なので、`BUILD_ASSERT(CONFIG_FCAN_HUB_MAX_PORTS <= FCAN_ZEPHYR_CAN_HUB_DIAG_PORTS)` を置く。

## Consequences

- **得るもの**：基板を足すだけで、その基板の CAN の健全性が ROS に出る。snippet にも overlay にも何も書かなくてよい。
- **失うもの**：どのイメージも 1 つ以上の tick を持つ。`tick` を持つ機能が無かったイメージ（rc26_air のように Solenoid だけのもの）でも、周期の tick が回り、`watchdog0` のある基板では watchdog が起動する。`can_diag` の tick は 1 秒に 1 度しか仕事をしないが、毎 ms 呼ばれる。
- **失うもの**：topic と Standard ID を、direct で 1 つ、hub で peer の数 + 1 使う。copy entry を `CanPortDiag` 1 つにつき 13、`CanHubDiag` で 5 使う。MdMotor 8 個の hub の構成で 114 / 128 になる。
- **失うもの**：RAM が増える（STATS、CAN_STATS、topic の保存域）。miniv4 の直結の MdMotor 4 個で 39940 B、hub（USB）で 54384 B。
- **守るべき約束への影響**：「このイメージが何を担うかは devicetree だけが決める」は、駆動するものについては変わらない。`can_diag` は、devicetree が選んだトランスポートについて報告するだけである。

## Alternatives considered

- **devicetree のノード（`fibril,fcan-can-diag`）で有効にする**：他の機能と形がそろうが、全ての snippet に同じノードを足すことになり、足し忘れた基板だけが報告しない。報告の対象はトランスポートが決まった時点で決まっている。
- **トランスポートのソース（`lib/fcan_transport/`）に入れる**：トランスポートは生成ラッパに依存せず、どの schema でも同じに組める。topic を埋めるには `schema_gen.hpp` が要るので、機能のライブラリに置いた。
- **`CanHubDiag` を `count: 1` で書く**：codegen はどちらも受け付けるが、他の機能と同じく `max_count` にそろえた。

## Related

- fibril_can の ADR-0017（CAN のメトリクスは topic で出す）
- [ADR 0003](0003-multi-app-structure.md)
- [ADR 0015](0015-hub-joins-fibril-can-only.md)
- [doc/apps.md](../apps.md) の「CAN の健全性（can_diag）」

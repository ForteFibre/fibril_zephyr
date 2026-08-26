# fibril_can latency probe slave (Zephyr sample)

`fibril_can_benchmark/docs/firmware-requirements.md` の MCU 側 contract を
Zephyr で満たす参考実装。ROS 側の probe launch から見ると、reference host slave
(`fcan_latency_slave`) と wire semantics が一致する。

契約側の変更管理は `firmware-requirements.md` の `contract_version` に従う。
本 README は **contract_version: 1** に対する実装。

## Build

west build は sandbox 外で行う。

```sh
# native_sim (smoke test)
west build -b native_sim \
    fibril_zephyr/samples/lib/fibril_can/latency_probe_node \
    -- -DFCAN_CODEGEN=/abs/path/to/fcan_codegen

# 実機
west build -b fibril_robomaster_miniv1 \
    fibril_zephyr/samples/lib/fibril_can/latency_probe_node \
    -- -DFCAN_CODEGEN=/abs/path/to/fcan_codegen
```

`fcan_codegen` は `modules/lib/fibril_can/fibril_can_codegen` を先に host build
しておき、生成される CLI の絶対パスを `-DFCAN_CODEGEN=...` で渡す (PATH 経由でも可)。

## §3.2 — no-ping tick の扱い

`ping` を一度も受け取っていない tick では
`seq = 0, t_master_send_ns = 0, t_slave_*_us = 0` を emit する。probe 側は
seq 変化で重複除外するので、warmup と実サンプルは seq==0 で分離できる。
前回値の再送は選択しない。

## §3.4 / §3.4.1 — 周期送出と poll 順

- 1 kHz 周期の `echo` は `latency_probe.yaml` の `period_us: 1000` に従い
  常に flush される (ping 未着でも継続)。
- Main loop は `drain_rx → probe_logic_tick (commit) → fcan_poll (emit) → sleep`。
  §3.4.1 の recommendation どおり change-driven `echo_change` を同 tick で
  emit する構成。
- Tick rate は build option `TICK_HZ` で上書き可 (§3.4.3):

  ```sh
  west build ... -- -DCONFIG_EXTRA_CFLAGS="-DTICK_HZ=5000"
  ```

## §4 — スレーブタイムスタンプ

- 実装: `k_cyc_to_us_floor32(k_cycle_get_32())`。実 MCU 上では DWT CYCCNT に
  bind され CPU 周波数の分解能を持つ。
- 32-bit low を渡すのみ。ラップ差分は probe 側で吸収 (§4.2)。
- `native_sim` は cycle rate が粗く §4.2 の 1 μs 精度を満たさない。実測用ではない。

## §5 — 決定性

- Heap は `fcan_init` の中で 1 回のみ確保 (`K_HEAP_DEFINE(fcan_heap, 8*1024)`)。
  hot path から `k_heap_alloc` は呼ばれない。
- Hot path から `LOG_*` は呼ばない。1 Hz status は separate low-priority
  thread (prio 7) が発火する。
- Echo commit dwell の最悪ケース: 100 μs 以内 (tick 周期 1 ms の 1/10、
  §5) を狙う。`CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000` で sleep_until 粒度
  100 μs。

## §6 — Node ID / warmup

- 既定 node ID: **0x20** (probe launch の `--node-id 0x20` と一致)。
  `-DCONFIG_EXTRA_CFLAGS="-DNODE_ID=0x??"` で上書き可。
- Boot から `echo` 周期送信開始までの想定時間: **≦ 1 s**
  (T_listen 300 ms + boot_id ジッタ + fcan_init + fcan_register_all)。
  probe launch は `--slave-warmup-s 1` を推奨。

## §7 — master heartbeat

`fcan_config_t.master_lost_us = 0` (無効) を default とする。
FAULT / silent 化を避け、計測中 echo が止まらないことを優先する。

## §8 — Router 併用

このサンプルは **single-bus 構成** (`zephyr,canbus` を 1 つ chosen する) の
実装。router を通した実験は対象外。

## §9 — CAN 物理層

CAN FD 固定。fibril_robomaster_miniv1 overlay の既定値:

| 項目 | 値 |
| :--- | :--- |
| arb bitrate | 1 Mbps |
| data bitrate | 5 Mbps |
| ping wire size | 14 B → DLC 16 |
| echo wire size | 20 B → DLC 20 |

他 bitrate で実験する場合は overlay を書き換える。実測 wire frame 長は
実験時に probe 側 CSV へメタデータとして残す。

## §10 — 測定メタデータ (fibril_robomaster_miniv1 build)

- MCU family: STM32H723 (Cortex-M7)
- Clock 源: 外付け XO + PLL_Q
- Clock 精度: ±30 ppm (HSE 依存)
- CAN peripheral: FDCAN1
- CAN transceiver: 実装者側で board schematic を参照
- `t_slave_*_us` 実装手段: `k_cyc_to_us_floor32(k_cycle_get_32())`
  (Zephyr が DWT CYCCNT に mapping)
- Router 併用: なし
- Firmware git SHA: 手動申告 (`git rev-parse HEAD` を実験ログに添付)

`native_sim` build は smoke test 専用のためメタデータの意味を持たない。

## §11 — チェックリスト

- [ ] T_listen 後 1 秒以内に ANNOUNCE が観測される
- [ ] ping 未送信状態でも echo が 1 kHz で周期送出される
- [ ] 1000 seq 送って `echo.seq` の重複や欠落が ±1 tick 相当
- [ ] `t_slave_recv_us < t_slave_send_us` が同 tick で常に成立
- [ ] Heap 使用量が起動時のみ確保され、hot path で増えない
- [ ] `master_lost_us = 0` で連続 24 時間 echo 継続
- [ ] Router 併用なし ゆえ `fcan_router_on_rx` は未使用
- [ ] arb/data bitrate 切替後も上記が成立

## 対応 board

- `native_sim` — `zephyr,can-loopback` による smoke test
- `fibril_robomaster_miniv1` — 実機 (FDCAN1, PLL_Q @ 80 MHz)

他 board を足すときは overlay で `chosen { zephyr,canbus = &... }` と
bitrate/bitrate-data を設定する。`prj.conf` の書き換えは不要。

# router_gs_usb_latency_probe

`fibril_can_benchmark/docs/firmware-requirements.md` の contract を満たす MCU
firmware を、`fcan_router` の **self node** として置き、uplink を
CANnectivity の `gs_usb` device class で PC に露出する構成の Zephyr sample。
`samples/lib/fibril_can/latency_probe_node` と同じ wire semantics だが、
物理層が「単一 CAN bus」ではなく「USB gs_usb + downlink FDCAN1」になる。

契約側は **contract_version: 1**。

```text
PC (SocketCAN / fibril_can_web) --USB FS--> zephyr_udc0 (&usb)
                                                ├─ gs_usb subsys
                                                ↓
                                         fcan_router (uplink)
                                            ├── self node (schema=latency_probe)
                                            └── downlink: fdcan1  (物理 CAN)
```

## いつこれを使うか

- 同一 MCU で「latency probe slave」と「他の物理 CAN slave の USB ゲートウェイ」を
  同居させたい (bench 上でスキーマ動作確認と実測を並行させたい) とき
- probe を USB 一本で PC に繋ぎ、bring-up を手軽にしたいとき

**latency 最短を狙う本命計測** は `latency_probe_node` (single-bus) を推奨。
理由は下の「latency components」節参照。

## Build

west build は sandbox 外で実行。

```sh
FCAN_CODEGEN=/abs/path/fcan_codegen \
  west build -b fibril_robomaster_miniv1 \
    fibril_zephyr/samples/lib/fibril_can/router_gs_usb_latency_probe
```

Twister:

```sh
FCAN_CODEGEN=/abs/path/fcan_codegen \
  twister -T fibril_zephyr/samples/lib/fibril_can/router_gs_usb_latency_probe
```

## Linux で立ち上げる

1. `west flash` して USB-C を PC に挿す。
2. `dmesg | grep gs_usb` — `1209:ca01` の device が enumerate する。
   product string は `"CANnectivity USB to CAN adapter (latency probe)"` で、
   `router_gs_usb_self` firmware と区別できる。
3. `ip link` に `canX` が生える。CAN FD で 1M/5M に上げてから up:

   ```sh
   sudo ip link set canX down
   sudo ip link set canX type can bitrate 1000000 dbitrate 5000000 fd on
   sudo ip link set canX up
   ```

4. `candump canX` で HEARTBEAT / echo 1 kHz を確認。
5. probe launch を起動:

   ```sh
   ros2 launch fibril_can_benchmark probe.launch.py \
       iface:=canX slave_warmup_s:=1
   ```

## latency components (§8 — probe が router のどちら側にいるか)

このサンプルは **probe を router の uplink 側 (self node) に置く**。
物理 CAN wire を経由しないため:

| 経路 | 単一 bus (`latency_probe_node`) | router+gs_usb (このサンプル) |
| :--- | :---: | :---: |
| PC → MCU | CAN wire | USB FS (bulk) |
| MCU → PC | CAN wire | USB FS (bulk) |
| commit → emit dwell 最悪 | ≈ scheduler inspect | ≈ router poll interval |

commit → emit dwell について:
- **単一 bus**: main loop で
  `drain_rx → probe_logic_tick (commit) → fcan_poll (emit) → sleep` の順に
  running するので、commit と emit が同 tick に載る。§3.4.1 recommendation。
- **router**: `probe_logic_tick` は app thread、`fcan_poll(self)` は router
  driver thread。両者は `CONFIG_CAN_FCAN_ROUTER_POLL_INTERVAL_US` (default 1000 µs)
  ごとに独立に動く。したがって commit → emit の待ちは平均 0.5 ms、最悪 1 ms。

  この値は `prj.conf` の `CONFIG_CAN_FCAN_ROUTER_POLL_INTERVAL_US` で
  調整できる (range 1..1_000_000)。実効分解能は
  `CONFIG_SYS_CLOCK_TICKS_PER_SEC` で決まる kernel tick に丸められる
  (このサンプルは 10 kHz tick なので 100 µs)。

USB FS の bulk 転送は 1 ms frame 単位。したがって minimum RTT は
`~1 ms (USB in) + ~1 ms (router dwell) + ~1 ms (USB out) ≒ 3 ms` オーダー。
物理 CAN 経由の RTT (1M arb / 20 B DLC で 300 μs 程度) より 1 桁大きい点は
覚えておくべき制約。

## §3.2 — no-ping tick の扱い

`seq = 0, t_master_send_ns = 0, t_slave_*_us = 0` を emit する。前回値の
再送は選択しない。probe 側は seq==0 で warmup サンプルを弾ける。

## §5 — 決定性

- Heap は `fcan_init` の中で 1 回のみ確保 (`K_HEAP_DEFINE(fcan_heap, 16*1024)`)。
- Hot path (main の `probe_logic_tick` と router thread の `fcan_poll(self)`)
  から LOG / printf / malloc を呼ばない。1 Hz status は独立 thread (prio 7)。
- Main tick は `k_sleep(K_TIMEOUT_ABS_TICKS(next))` で phase noise を
  最小化。`CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000` で 100 μs 粒度。

## §6 — Node ID / warmup

- 既定 node ID: **0x20**。
  `-DCONFIG_EXTRA_CFLAGS="-DNODE_ID=0x??"` で上書き可。
- Boot から echo 周期送信開始まで:
  T_listen 300 ms + boot_id ジッタ + USB enumerate/bind (~500 ms)
  ≒ **≦ 2 s** 見積もり。probe launch は `--slave-warmup-s 2` 推奨。

## §7 — master heartbeat

`fcan_config_t.master_lost_us = 0` (無効)。silent 化を避け echo を止めない。

## §8 — Router 併用時の probe 位置

- **probe は router の uplink (self node) 側**。ROS 側は
  `probe が router のどちら側にいたか` を「uplink (self)」として実験ログに書く。
- fdcan1 downlink に別の slave (例: `example_node`) を繋ぐと、その topic は
  同じ `canX` に載って ROS 側に到達する。実験時 downlink の負荷が
  self の echo emit に干渉しないかは `fcan_router_get_diag` の
  `drop_send_failed` / `ingress_drops` で監視する (status thread が 1 Hz で
  ログする)。

## §9 — CAN 物理層 (downlink 側)

fibril_robomaster_miniv1 overlay 既定値:

| 項目 | 値 |
| :--- | :--- |
| downlink | FDCAN1 |
| arb bitrate | 1 Mbps |
| data bitrate | 5 Mbps |
| ping wire size (M→S on wire, uplink 側) | USB bulk — CAN 物理層を経由しない |
| echo wire size (S→M on wire, uplink 側) | USB bulk — 同上 |

uplink 側の wire time は USB bulk (1 ms frame) が支配。§9 の「bitrate 切替」
は downlink 側の実験を対象にする。

## §10 — 測定メタデータ (fibril_robomaster_miniv1 build)

- MCU family: STM32H723 (Cortex-M7)
- Clock 源: HSI16 → PLL (overlay で HSE 依存を外している)
- Clock 精度: HSI16 ≈ ±1 % — production 実測用途では HSE に戻すこと
- CAN peripheral: FDCAN1 (downlink)、gs_usb (uplink)
- USB: full-speed, VID 0x1209 / PID 0xca01 (CANnectivity 互換)
- `t_slave_*_us` 実装: `k_cyc_to_us_floor32(k_cycle_get_32())`
- Router 併用: **あり** — probe は uplink (self) 側
- Firmware git SHA: 実験ログに手動添付

## §11 — チェックリスト

- [ ] T_listen 後 1 秒以内に ANNOUNCE が観測される (USB enumerate 後)
- [ ] ping 未送信状態でも echo が 1 kHz で周期送出される
      (`ros2 topic hz /latency_probe/echo`)
- [ ] `echo.seq` 重複/欠落が ±1 tick + 1 router-poll 相当
- [ ] `t_slave_recv_us < t_slave_send_us` が同 tick で常に成立
- [ ] Heap 使用量が起動時のみ確保され、hot path で増えない
- [ ] `master_lost_us = 0` で連続 24 時間 echo 継続
- [ ] `fcan_router_on_rx` は router driver thread 1 本からのみ呼ばれる
      (driver 実装が保証。app thread で追加で呼ばない)
- [ ] downlink bitrate 切替後も上記が成立

## Troubleshooting

- **gs_usb が enumerate しない**: usbd_setup が USBD_CONFIGURATION_DEFINE 系
  で失敗しているか、USB clock (HSI48) が不安定。`prj.conf` の
  `CONFIG_UDC_DRIVER_LOG_LEVEL_DBG=y` / `CONFIG_USBD_GS_USB_LOG_LEVEL_DBG=y`
  を有効化して USART2 (115200 8N1) のログを確認。
- **provisioning が完了しない**: `fcan_router_attach_self` が
  `usbd_setup_enable` の前に呼ばれているか main.c の順序を確認 (詳細は
  `fibril_can/docs/09-router.md`)。
- **VID/PID が実 CANnectivity dongle と衝突**: udev rule を分けたい場合は
  `usbd_setup.c` の VID/PID/product を編集する。
- **echo latency が単一 bus 版より大きい**: 上記 "latency components" 節
  参照。設計上の下限は USB FS の 1 ms frame + router poll。

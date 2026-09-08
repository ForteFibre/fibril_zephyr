# fibril_can の codegen スレーブノード

ビルド時に `fcan_codegen` がスキーマ YAML から生成した型付きラッパを使い、fibril_can のスレーブを立ち上げるサンプル。

スキーマは fibril_can 側の `fibril_can_example/schema/example_node.yaml` を参照する。
`FCAN_EXAMPLE_SCHEMA` を渡せば別のスキーマに差し替えられる。

## このサンプルが確かめること

- `fcan_codegen` の出力（`schema_gen.{h,c}` と `schema_blob.c`）が Zephyr ターゲット上でランタイムの ABI と一致すること
- `fcan_init` が codegen の `FCAN_*` マクロを受け取って `FCAN_OK` を返すこと
- `fcan_register_all` が全 topic、param、service をリンクエラーなしで結線できること。`app_logic.c` の service ハンドラが 1 つでも欠けるとリンクが通らない
- `fcan_poll` が HEARTBEAT の送出と状態機を進め、その裏で app のティックが S2M テレメトリを publish すること

マスターがいなくてもスモークテストとして意味を持つ。
ループバックバスには ANNOUNCE_ACK が来ないので、スレーブは UNPROVISIONED のまま state machine を回す。

逆に、このサンプルでは `fibril_can_bridge` とのプロビジョニング往復と、実際の FDCAN ハードウェア上のタイミングは確かめられない。

## 前提

`fcan_codegen` の実行ファイルの場所をビルドシステムに教える必要がある。

```shell
west build -b native_sim samples/lib/fibril_can/example_node -- \
  -DFCAN_CODEGEN=/abs/path/fcan_codegen
```

Twister から走らせる場合は環境変数でも渡せる。

```shell
FCAN_CODEGEN=/abs/path/fcan_codegen \
  west twister -T samples/lib/fibril_can/example_node
```

`fibril_can` モジュールが west manifest に載っていない、または `CONFIG_FIBRIL_CAN` が立っていない場合は、CMake が `ZEPHYR_FIBRIL_CAN_MODULE_DIR` 未設定として止まる。

## 実行

`native_sim` は `zephyr,can-loopback` を使うので追加のハードウェアが要らない。

```shell
west build -b native_sim samples/lib/fibril_can/example_node -- \
  -DFCAN_CODEGEN=/abs/path/fcan_codegen
./build/zephyr/zephyr.exe
```

実機向けの overlay は `boards/` に 4 ボードぶん置いてある（`fibril_canmotor_tourobo2023`、`fibril_robomaster_miniv1`、`fibril_robomaster_miniv3`、`fibril_robomaster_miniv4`）。
いずれも `zephyr,canbus` の chosen ノードを与えることが条件になる。

## 実装の分担

| ファイル | 役割 |
| --- | --- |
| `src/main.c` | `fcan_init`、`fcan_register_all`、`scheduler_wait` と `drain_rx` と `fcan_poll` の 3 点セットによるメインループ |
| `src/app_logic.c` | 生成された service ハンドラの実装と、S2M テレメトリを publish するティック |
| `src/rgb_state.c` | ノードの状態を RGB LED に出す |

メインループはイベント駆動で回る（`CONFIG_FCAN_ZEPHYR_EVENT_WAKE`）。
送信待ちの通知、RX のエンキュー、送信完了に加えて、app のティックを約 1 kHz に保つための `k_timer` が wake ビットを立てる。

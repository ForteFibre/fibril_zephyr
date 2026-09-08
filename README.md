# fibril_zephyr

ForteFibre のロボット用基板で動く Zephyr ファームウェアを、1 つの workspace にまとめたリポジトリ。
自作ボードの定義、out-of-tree のモータドライバとエンコーダドライバ、CAN FD 通信フレームワーク [fibril_can](https://github.com/ForteFibre/fibril_can) を使ったサンプルが入っている。

Zephyr の [T2 topology][west_t2] に沿った **workspace application** で、このリポジトリ自身が west manifest（[west.yml](west.yml)）を提供し、Zephyr 本体と依存モジュールを取り込む。
Zephyr のバージョンは `west.yml` で v4.4.1 に固定してある。

構成の全体像は [doc/overview.md](doc/overview.md) にある。

## 対応ボード

いずれも STM32 系である。

| ボード | `west build -b` に渡す名前 | MCU | CAN | 文書 |
| --- | --- | --- | --- | --- |
| CanMotor Tourobo 2023 | `fibril_canmotor_tourobo2023` | STM32F407VG | MCP2517FD（SPI 外付け）1 本 | [doc](boards/fibril/canmotor_tourobo2023/doc/index.rst) |
| RoboMaster Mini V1 | `fibril_robomaster_miniv1` | STM32G474VE | FDCAN1 | [doc](boards/fibril/robomaster_miniv1/doc/index.rst) |
| RoboMaster Mini V3 | `fibril_robomaster_miniv3` | STM32G474ME（LQFP80） | FDCAN1 | [doc](boards/fibril/robomaster_miniv3/doc/index.rst) |
| RoboMaster Mini V4 | `fibril_robomaster_miniv4` | STM32G474VE | FDCAN1 / 2 / 3 | [doc](boards/fibril/robomaster_miniv4/doc/index.rst) |
| RC26 MainAir V01 | `fibril_rc26_mainair_v01` | STM32G474RE（LQFP64） | FDCAN2 / 3 | [doc](boards/fibril/rc26_mainair_v01/doc/index.rst) |

CAN の欄は、そのボードの devicetree がピンとクロックを設定しているコントローラである。
STM32G4 のボードは FDCAN を無効のままにしてあり、有効化とビットレートの指定は使う側の overlay に委ねている。
CanMotor の MCP2517FD は既定で有効である。

CanMotor と RoboMaster Mini の 4 枚は、ピン割り当てを先行ファームウェア CanMotorMbed の同名ターゲットから移してある。
既存のハードウェアを配線変更なしで動かせる。
RC26 MainAir V01 は KiCad の回路図から起こしてある。

Zephyr 標準のボードでも、必要な overlay を与えれば動く（`app/boards/nucleo_g474re.overlay` が例）。

## セットアップ

Zephyr の開発環境が未構築なら、先に公式の [Getting Started Guide][zephyr_gsg] に従う。
環境が既にあるなら、workspace の初期化は次のとおり。

```shell
west init -m https://github.com/ForteFibre/fibril_zephyr --mr main fibril_zephyr_ws
cd fibril_zephyr_ws
west update
```

`west update` は private リポジトリである `fibril_can` を clone する。
先に GitHub の認証を通しておく（`gh auth login` の後に `gh auth setup-git` を実行するか、SSH 鍵を登録して `insteadOf` を張る）。

## コーディングエージェントを開く場所

**workspace のルート**（このリポジトリの 1 つ上）で開く。

ドライバやサンプルを読むには、Zephyr 本体（`zephyr/`）と `modules/lib/fibril_can` のソースを参照する必要がある。
どちらもこのリポジトリの外にあるため、`fibril_zephyr/` の中で開くと読めない。

`west` 自身は `.west/` を上に辿るので、`fibril_zephyr/` の中からでもコマンドは通る。
制約になるのは、エージェントが読めるファイルの範囲である。

このリポジトリの中を触るときの約束事は [CLAUDE.md](CLAUDE.md) にある。

## ビルドと書き込み

```shell
cd fibril_zephyr
west build -b <ボード名> app
west flash
```

診断用の Kconfig をまとめた `app/debug.conf` を追加で適用できる。

```shell
west build -b <ボード名> app -- -DEXTRA_CONF_FILE=debug.conf
```

## ドライバ

| クラス | 実装 | 文書 |
| --- | --- | --- |
| エンコーダ | AMT21x アブソリュートエンコーダ（RS485） | [doc/drivers/amt21.md](doc/drivers/amt21.md) |
| モータ | DJI RoboMaster C610 / C620（CAN） | [doc/drivers/robomaster.md](doc/drivers/robomaster.md) |

## サンプル

| パス | 内容 |
| --- | --- |
| [samples/drivers/amt21](samples/drivers/amt21) | AMT21x アブソリュートエンコーダの読み出し |
| [samples/drivers/qdec](samples/drivers/qdec) | STM32 タイマでデコードした直交エンコーダの読み出し |
| [samples/drivers/can_router](samples/drivers/can_router) | fibril_can のルータを Zephyr の CAN デバイスとして使う |
| [samples/lib/fibril_can/example_node](samples/lib/fibril_can/example_node) | codegen 出力を使った fibril_can スレーブノード |
| [samples/lib/fibril_can/hub_gs_usb_self](samples/lib/fibril_can/hub_gs_usb_self) | CAN hub と self ノードを CANnectivity の `gs_usb` 経由で PC に見せる |
| [samples/lib/fibril_can/latency_probe_node](samples/lib/fibril_can/latency_probe_node) | E2E レイテンシ計測用のスレーブ |
| [samples/lib/fibril_can/hub_gs_usb_latency_probe](samples/lib/fibril_can/hub_gs_usb_latency_probe) | レイテンシ計測スレーブを hub + `gs_usb` 構成で動かす |

## テスト

```shell
west twister -T tests --integration
```

テストの構成と Kconfig の組み合わせは [doc/testing.md](doc/testing.md) にある。

## ドキュメント

| 知りたいこと | 参照先 |
| --- | --- |
| 全体像と workspace のどこに何があるか | [doc/overview.md](doc/overview.md) |
| ボードごとのピン配置とクロック | 上のボード表の「文書」列 |
| ドライバの使い方と配線の要件 | [doc/drivers/](doc/drivers) |
| テストの構成 | [doc/testing.md](doc/testing.md) |
| 設計判断の理由 | [doc/adr/](doc/adr) |

HTML 版と API リファレンス（Doxygen）は `doc/` でビルドする。

```shell
cd doc
pip install -r requirements.txt
doxygen
make html
```

出力は `doc/_build_doxygen` と `doc/_build_sphinx` に入る。

## ライセンス

Apache-2.0（[LICENSE](LICENSE)）

[west_t2]: https://docs.zephyrproject.org/latest/develop/west/workspaces.html#west-t2
[zephyr_gsg]: https://docs.zephyrproject.org/latest/develop/getting_started/index.html

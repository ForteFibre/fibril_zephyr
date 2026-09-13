# 全体像

`fibril_zephyr` は ForteFibre のロボット用基板で動く Zephyr ファームウェアを 1 か所にまとめたリポジトリである。
自作ボードの定義、out-of-tree のドライバ、CAN FD 通信フレームワーク fibril_can を使ったサンプルが入る。

Zephyr の [T2 topology](https://docs.zephyrproject.org/latest/develop/west/workspaces.html#west-t2) に沿った **workspace application** で、このリポジトリ自身が west manifest（`west.yml`）を提供する。
Zephyr 本体はこのリポジトリの外に clone され、`west.yml` がその revision を固定する。

## workspace の構成

`west init -m https://github.com/ForteFibre/fibril_zephyr <workspace>` を実行すると、次の配置ができる。
`<workspace>` の名前は init の引数で決まる。

```text
<workspace>/
├── .west/config                  manifest = fibril_zephyr/west.yml、zephyr base = zephyr
├── zephyr/                       Zephyr 本体（v4.4.1 に固定）
├── modules/
│   ├── hal/stm32                 STM32 の HAL
│   ├── hal/cmsis_6               Cortex-M ポートが要求する CMSIS
│   ├── lib/cmsis-dsp             CMSIS-DSP
│   ├── lib/fibril_can            CAN FD 通信フレームワーク（private repo）
│   └── third_party/cannectivity  gs_usb による USB-CAN ゲートウェイ
└── fibril_zephyr/                このリポジトリ
```

`west.yml` が直接宣言しているのは `zephyr`、`cannectivity`、`fibril_can` の 3 つだけである。
`modules/hal` と `modules/lib` 以下の残りは、`zephyr` プロジェクトの `import` に `name-allowlist` を与えて必要なものだけ取り込んでいる。
workspace のディレクトリに他のものが置かれていても、この manifest が管理する対象ではない。

サンプルが参照する fibril_can はこのリポジトリの中ではなく `modules/lib/fibril_can` にある。
private repository なので、`west update` を通すには先に GitHub の認証が必要である。

fibril_can の revision はブランチではなくコミットで固定してある。
codegen が決めるブロック配列の並びはワイヤの契約の一部で、`main` を追いかけていると `fcan_config_t::instance_counts` の割り当てが黙って変わる。
上げるときは意図して上げ、必要な移行を同じ変更に含める。

## Zephyr module としての入口

このリポジトリは Zephyr module でもある。
`zephyr/module.yml` がビルドシステムから見た入口を宣言している。

| 宣言 | 指す先 | 効果 |
| --- | --- | --- |
| `build.kconfig` | `Kconfig` | Kconfig ツリーの Zephyr → Modules 以下に生える |
| `build.cmake` | `.`（`CMakeLists.txt`） | `drivers/` と `lib/` を `add_subdirectory` する |
| `settings.board_root` | `.` | `boards/` を追加のボード検索パスにする |
| `settings.dts_root` | `.` | `dts/` と `dts/bindings/` を追加の検索パスにする |
| `settings.snippet_root` | `.` | `snippets/` を snippet の検索パスにする |
| `runners` | `scripts/example_runner.py` | `west flash` の runner を追加する |
| `name` | `fibril_zephyr` | `ZEPHYR_FIBRIL_ZEPHYR_MODULE_DIR` を clone 先の名前から切り離す |

`CMakeLists.txt` は `include/` をインクルードパスに加え、`zephyr_syscall_include_directories()` も呼ぶ。
`include/drivers/motor.h` などが `__syscall` を使うため、これがないとシステムコールが生成されない。

## リポジトリの構成

| ディレクトリ | 内容 |
| --- | --- |
| `apps/` | 実機に焼くアプリケーション。`apps/node/` が fibril_can スレーブ |
| `app/` | ボード持ち込みの動作確認用アプリケーション |
| `snippets/` | 焼く単位ごとの schema、Kconfig、overlay の組 |
| `boards/fibril/` | 自作ボードの定義。ボードごとの詳細は各 `doc/index.rst` |
| `drivers/` | out-of-tree ドライバの実装 |
| `dts/bindings/` | 上記ドライバと機能の devicetree binding |
| `include/` | 公開ヘッダ。ドライバクラスの API はここが正本 |
| `lib/` | out-of-tree ライブラリ。`lib/fibril_can_node/` にブロック型の実装、`lib/fcan_transport/` にバスへの繋ぎ方 |
| `samples/` | ドライバ単体および fibril_can と組み合わせたサンプル |
| `tests/` | Twister から走る ztest |
| `scripts/` | west の拡張コマンドと runner |
| `doc/` | このドキュメントと Doxygen の設定 |

`apps/` と `lib/fibril_can_node/` と `snippets/` の関係は [アプリケーションと機能](apps.md) にある。
どの機能を載せるかは snippet が選ぶノードの schema が決め、アプリケーションはブロック型の名前を持たない。

## ドライバ

| クラス | 実装 | ドキュメント |
| --- | --- | --- |
| エンコーダ（`include/drivers/encoder.h`） | `drivers/encoder/amt21.c` | [AMT21x](drivers/amt21.md) |
| エンコーダ（`include/drivers/encoder.h`） | `drivers/encoder/qdec_stm32.c` | [直交エンコーダ](drivers/qdec_stm32.md) |
| モータ（`include/drivers/motor.h`） | `drivers/motor/robomaster.c` | [RoboMaster](drivers/robomaster.md) |

ラップ解決・速度・オフセットはエンコーダドライバに共通なので、`drivers/encoder/encoder_accum.c` に切り出してある。
公開 API ではなく `drivers/encoder/` の内部ヘッダである。

ドライバクラスの API は `include/` に置き、実装は `drivers/` に置く。
ドライバ固有の診断のように、クラスの API に載せられないものは `include/drivers/<class>/<driver>.h` に分ける。
AMT21x の統計取得がその例で、トランザクションの失敗の分類は RS485 プロトコルの性質であってエンコーダ一般の性質ではないため、エンコーダクラスには載せていない。

## Zephyr のバージョン

`west.yml` の `zephyr` プロジェクトの `revision` が正本である。
現在は v4.4.1 に固定してある。

上げるときは `west.yml` を書き換えて `west update` を実行し、README のバージョン記述も合わせて直す。

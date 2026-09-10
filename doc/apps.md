# アプリケーションと機能

実機に焼くファームウェアは `apps/` にある。
そこに何が載るかは、機能ライブラリと snippet の組み合わせで決まる。
この構成を選んだ理由と却下した案は [ADR 0003](adr/0003-multi-app-structure.md) にある。

## アプリケーションの一覧

| パス | 内容 |
| --- | --- |
| `apps/node/` | fibril_can スレーブ。バス上のスレーブはすべてこれ 1 つで焼く |
| `app/` | ボード持ち込みの動作確認。fibril_can を使わない |

`apps/node/` はブロック型の名前を 1 つも持たない。
何を担うかは、ビルド時に選ぶ snippet が決める。

## イメージが決まる 4 つの軸

```shell
west build -b fibril_rc26_mainair_v01 apps/node -S rc26-air
```

| 軸 | 決めるもの | 置き場 |
| --- | --- | --- |
| 基板 | ペリフェラルの実体 | `boards/fibril/<board>/` |
| 機能 | ブロック型とその実装 | `lib/fibril_can_node/<type>/` |
| トランスポート | フレームがバスに届く経路 | `lib/fcan_transport/`（devicetree が選ぶ） |
| 焼く単位 | schema、配線の割り当て、node_id | `snippets/<大会>/<名前>/` |
| 個体 | node_id | 実行時（当面は snippet の Kconfig） |

snippet 1 つが 1 つのイメージである。
機能の組み合わせと配線が同じ基板は、何枚あっても同じイメージで動き、実行時の node_id だけが違う。

## snippet の中身

```text
snippets/rc26/air/
├── snippet.yml                        name と、何を append するか
├── air.conf                           CONFIG_FIBRIL_NODE_SCHEMA と node_id
├── fibril_rc26_mainair_v01.overlay    配線の割り当て、CAN、RNG
└── schema/air.yaml                    node、limits、instances
```

`snippet.yml` の `boards:` キーは正規表現にする。
比較の対象が `<board>/<qualifiers>` なので、ボード名そのままの完全一致キーは、SoC 名を持つボードに当たらない。

```yaml
boards:
  /fibril_rc26_mainair_v01.*/:
    append:
      EXTRA_DTC_OVERLAY_FILE: fibril_rc26_mainair_v01.overlay
```

ノードの schema は `types:` を持たない。
実装済みのブロック型はアプリケーションがすべて codegen に渡し、インスタンス化しない型は blob から落ちる。
schema が言うのは、このノードがどの機能をいくつ動かすかだけである。

```yaml
node: rc26_air{i}
limits:
  max_frames: 16
  max_copy_entries: 128
instances:
  - { type: Solenoid, max_count: 6, ns: "air{i}" }
```

ノード名の `{i}` は起動時の node_id で置換される。
schema はロボット 1 台の役割を述べるもので、基板 1 枚を述べるものではない。

## 機能を 1 つ足す

`lib/fibril_can_node/<type>/` に 4 つのファイルを置く。
アプリケーションには触らない。

| ファイル | 内容 |
| --- | --- |
| `type.yaml` | ブロック型の定義。バスから見た姿 |
| `impl.c` | 生成されるハンドラの実装 |
| `Kconfig` | devicetree にノードがあるときだけ y になる真偽値 |
| `CMakeLists.txt` | ソースの登録と、`type.yaml` をグローバルプロパティに積む 1 行 |

`impl.c` は本体を生成マクロで囲む。

```c
#include "schema_gen.h"

#if defined(FCAN_SOLENOID_MAX_COUNT)
/* ... */
#endif
```

**このガードが、機能の有無を schema 1 か所で決めている仕掛けである。**
schema が型をインスタンス化しなければ生成マクロが出ず、ファイルは空になる。
だから機能ごとの Kconfig を作らない。作れば schema と 2 か所で同じことを決めることになる。

インスタンスとハードウェアの対応は devicetree が持つ。
phandle 配列の並びがそのままバス上のインスタンス番号になるので、並べ替えると全部の名前が変わる。

```c
static const struct gpio_dt_spec valves[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))
};

BUILD_ASSERT(ARRAY_SIZE(valves) <= FCAN_SOLENOID_MAX_COUNT,
             "more gpios wired than the schema's Solenoid max_count");
```

配線の数と schema の上限は別のファイルに書かれていて、ほかに両者を結ぶものがない。
この `BUILD_ASSERT` を省くと、余ったバルブがバスから触れないまま黙って残る。

最後に自分を登録する。

```c
FIBRIL_FCAN_FUNC_DEFINE(
  solenoid,
  .array = FCAN_ARRAY_SOLENOID,
  .count = (uint8_t)ARRAY_SIZE(valves),
  .init = solenoid_init,
  .start = solenoid_start);
```

`init` はハードウェアを掴む（`fcan_init` の前）、`start` はバスに最初に触る（`fcan_register_all` の後）、`tick` は約 1 kHz で呼ばれる。
どれも省略できる。

## 周期送信を持たせない

状態がサービス呼び出しでしか変わらない機能は、`tick` を書かずに `period_us: 0` を使う。
このトピックはスケジューラの周期送信から外れ、`commit` を呼んだ回だけ 1 発送られる。

```yaml
state:
  dir: s2m
  period_us: 0
  fields: { on: bool }
```

publish はハンドラの中と `start` の 2 か所からになる。
`start` の時点ではノードがまだ RUNNING でないが、送信要求は保持され、送信できる最初の `fcan_poll` で出る。
だからマスターの最初の読み取りが既定値ではなく実際の状態になる。

`tick` を持つ機能が 1 つも無いイメージでは、アプリケーションが 1 kHz のタイマを起動しない。
`fcan_poll` はランタイム自身の期限（ハートビート、ANNOUNCE、周期 S2M スロット）と、RX・TX 完了・commit の 3 つのイベントで回るので、タイマが無くても止まらない。

## トランスポートを差し替える

ノードのフレームがバスに届く経路は 2 通りある。
CAN コントローラに直結する形と、複数のセグメントを橋渡しする hub の self port になる形である。

**選ぶのは devicetree で、アプリケーションも機能も書き換えない。**

| overlay に書くもの | 選ばれるバックエンド |
| --- | --- |
| `chosen { zephyr,canbus = &fdcanN; }` | `lib/fcan_transport/direct.c` |
| `compatible = "fibril,can-hub"` のノード | `lib/fcan_transport/hub.c` |

`fibril,can-hub` のノードを置くと `CONFIG_CAN_FCAN_HUB` が自動で立ち、それが
`CONFIG_FCAN_TRANSPORT_HUB` を選ぶ。conf に書き足すことはない。

2 つの違いは 2 点に尽きる。

**HAL の出どころ。** 直結は `fcan_zephyr_can_hal_get()`、hub は
`fcan_hub_get_self_hal()` である。後者は Zephyr の CAN device のラッパではない。
hub の self は「自分も port の 1 つ」の対称モデルに従い、送信を全 live port に
broadcast する。方向を知らずに済むので、USB が繋がっている port を静的に決めなくてよい。

**`fcan_poll` を回すスレッド。** 直結はアプリケーションのループが回す。
hub は **ドライバのスレッドが回す**（`fcan_hub_poll` が attach 済みの self を駆動する）。
`fcan_hub_on_rx` は内部ロックを持たず、呼ぶスレッドを 1 本に保つ約束なので、
hub 構成でアプリケーションが並行して poll してはいけない。
`fcan_transport_run()` が hub では即座に返るのはこのためである。

hub 構成では `can_start()` を呼ばない。
外部 port を開けるのはゲートウェイの仕事で、peer はドライバの init で既に上がっている。
ホストを繋がない基板でも peer セグメントにハートビートが流れる。

### tick の契約

hub 構成では、周期処理が `fcan_poll` とは別のスレッドで走る。
ドライバのループにアプリケーション用のフックが無いためである。

publish は安全側にある。トピックの commit は、commit する側とスケジューラが別の
コンテキストで走る前提で作られている。
ただし **`tick` は `fcan_poll` と同じスレッドにいると仮定してはいけない。**
`func.h` の `tick` の説明はこの前提で書いてある。

## `instance_counts` の埋め方

ブロック配列の並びは schema から導かれる。
位置指定の初期化子は、並びが変わってもコンパイルが通ったまま、別のブロックに数を渡す。
`FCAN_ARRAY_<TYPE>` を添字に使う。

```c
static const uint8_t counts[FCAN_NUM_BLOCK_ARRAYS] = {
  [FCAN_ARRAY_MOTORDRIVER] = 1U,
  [FCAN_ARRAY_IMU] = 1U,
};
```

`apps/node/` では機能が自分の枠を埋めるので、この配列を手で書く場所は残っていない。
`samples/lib/fibril_can/` のサンプルは自前の schema を持つので、こちらの形で書く。

## codegen の場所

`fcan_codegen` は `ros-jazzy-fibril-can-codegen` パッケージが `/opt/ros/jazzy/lib/fibril_can_codegen/` に置く。
このパスは既定で探されるので、パッケージが入っている機械では何も渡さなくてよい。

入っていない場合は絶対パスを渡す。twister には環境変数で渡す。

```shell
west build -b <board> apps/node -S <snippet> -- -DFCAN_CODEGEN=/abs/path/fcan_codegen
FCAN_CODEGEN=/abs/path/fcan_codegen west twister -T apps
```

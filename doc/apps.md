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
何を担うかは、ビルド時に重ねる snippet が決める。

## イメージが決まる軸

```shell
west build -b fibril_rc26_mainair_v01 apps/node -S rc26-mainair-usb -S rc26-air
```

| 軸 | 決めるもの | 置き場 |
| --- | --- | --- |
| 基板 | ペリフェラルの実体 | `boards/fibril/<board>/` |
| 機能 | ブロック型とその実装 | `lib/fibril_can_node/<type>/` |
| トランスポート | フレームがバスに届く経路 | `lib/fcan_transport/`（devicetree が選ぶ） |
| 焼く単位 | 配線の割り当て、ノード名、node_id | `snippets/<大会>/<名前>/` |
| 個体 | node_id | 実行時（当面は snippet の Kconfig） |

## snippet を 2 層に重ねる

`-S` は複数渡せて、並べた順に append される。
これを使って、焼く単位を 2 つの層に割ってある。

| 層 | 決めるもの | 例 |
| --- | --- | --- |
| トランスポート | 基板がバスに出る経路 | `rc26-mainair-usb` |
| デプロイ | ノード名、node_id、どの機能をどのピンに出すか | `rc26-air` |

RC26 MainAir には必ず USB の CAN が載り、電磁弁やエンコーダは搭載の有無が変わる。
経路を別の層に括り出しておくと、機能の増減がデプロイ snippet 1 つの中で閉じる。

機能側の overlay はボードが定義するラベルしか参照しない。
トランスポート snippet が作るラベル（`fcan_hub` など）には触れない。
この約束があるので、`-S` の順序を入れ替えても同じ blob が出る。

## snippet の中身

```text
snippets/rc26/mainair-usb/
├── snippet.yml                        name と、何を append するか
├── mainair-usb.conf                   USB スタックとログ
└── fibril_rc26_mainair_v01.overlay    gs_usb、hub、CAN、RNG

snippets/rc26/air/
├── snippet.yml
├── air.conf                           CONFIG_FIBRIL_NODE_SCHEMA と node_id
├── fibril_rc26_mainair_v01.overlay    機能ノードと配線の割り当て
└── schema/air.yaml                    node と limits
```

`snippet.yml` の `boards:` キーは正規表現にする。
比較の対象が `<board>/<qualifiers>` なので、ボード名そのままの完全一致キーは、SoC 名を持つボードに当たらない。

```yaml
boards:
  /fibril_rc26_mainair_v01.*/:
    append:
      EXTRA_DTC_OVERLAY_FILE: fibril_rc26_mainair_v01.overlay
```

## schema は 3 種類の断片から組む

codegen は複数のファイルを決定的にマージする。
`apps/node` はそれを使って、手で書く部分を最小にしている。

| 断片 | 出どころ | 中身 |
| --- | --- | --- |
| ノードヘッダ | `CONFIG_FIBRIL_NODE_SCHEMA` が指す YAML | `node:`、`protocol:`、`limits:` |
| ブロック型 | 各機能の `type.yaml` | `types:` |
| インスタンス | 各機能が devicetree から生成 | `instances:` |

ブロック型は実装済みのものを全部渡す。
インスタンス化されない型は blob を作る前に落とされるので、渡す一覧を誰も管理しなくてよい。

**このイメージが何を担うかは devicetree だけが決める。**
`fibril,fcan-*` のノードを置くと、そのノードから `instances:` の断片が生成され、機能のソースがコンパイルされる。
ノードを消せば両方消える。

手で書くのはノードヘッダだけになる。

```yaml
node: rc26_air{i}
protocol: 2

limits:
  max_frames: 16
  max_copy_entries: 128
```

ノード名の `{i}` は起動時の node_id で置換される。
schema はロボット 1 台の役割を述べるもので、基板 1 枚を述べるものではない。

## 機能を 1 つ足す

`lib/fibril_can_node/<type>/` に 4 つのファイルと、binding を 1 つ置く。
アプリケーションには触らない。

| ファイル | 内容 |
| --- | --- |
| `type.yaml` | ブロック型の定義。バスから見た姿 |
| `impl.c` | 生成されるハンドラの実装 |
| `Kconfig` | devicetree にノードがあるときだけ y になる真偽値と、`_MAX` |
| `CMakeLists.txt` | ソースの登録、`type.yaml` の登録、`fibril_can_node_instances()` |
| `dts/bindings/fibril_can_node/fibril,fcan-<type>.yaml` | ハードウェアの記述と `fcan-ns` |

`CMakeLists.txt` はこの形になる。

```cmake
zephyr_library_sources_ifdef(CONFIG_FIBRIL_CAN_NODE_SOLENOID solenoid.c)

if(CONFIG_FIBRIL_CAN_NODE_SOLENOID)
  set_property(GLOBAL APPEND PROPERTY fibril_can_node_type_schemas
               "${CMAKE_CURRENT_SOURCE_DIR}/type.yaml")

  fibril_can_node_instances(
    TYPE       Solenoid
    COMPATIBLE "fibril,fcan-solenoid"
    MAX        ${CONFIG_FIBRIL_CAN_NODE_SOLENOID_MAX})
endif()
```

`fibril_can_node_instances()` は devicetree から `fcan-ns` を読んで `instances:` の断片を吐く。
compatible を持つノードが 2 つ以上 okay なら、その場でビルドを止める。
実装は devicetree インスタンス 0 しか駆動しないので、2 つ目はハンドラの無いインスタンス群になるためである。

### `max_count` を Kconfig で持つ理由

`max_count` は静的確保の上限であって、個数ではない。
blob には現れず、バスと ROS グラフに出るのは実行時のインスタンス数だけである。
6 と 16 で生成物を比べても、`schema_blob.c` は完全に一致し、変わるのは状態配列の長さ 1 行だけだった。

余らせたコストは `sizeof(state)` × 余りバイトの `.bss` に収まる。
一方、CMake の devicetree API は phandle 配列を読めない（`dt_prop` が扱うのは string、int、boolean、array、uint8-array、string-array、path）。
配線の本数を CMake から数える手段がない以上、devicetree に本数を重複して書くより、Kconfig の上限で済ませるほうが行が減る。

`impl.c` の `BUILD_ASSERT` が、配線が上限を超えたときに落とす。

```c
static const struct gpio_dt_spec valves[] = {
  DT_INST_FOREACH_PROP_ELEM_SEP(0, gpios, GPIO_DT_SPEC_GET_BY_IDX, (, ))
};

BUILD_ASSERT(ARRAY_SIZE(valves) <= FCAN_SOLENOID_MAX_COUNT,
             "more gpios wired than CONFIG_FIBRIL_CAN_NODE_SOLENOID_MAX");
```

インスタンスとハードウェアの対応は devicetree が持つ。
phandle 配列の並びがそのままバス上のインスタンス番号になるので、並べ替えると全部の名前が変わる。

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

| overlay に書くもの | 選ばれるもの |
| --- | --- |
| `chosen { zephyr,canbus = &fdcanN; }` | `lib/fcan_transport/direct.c` |
| `compatible = "fibril,can-hub"` のノード | `lib/fcan_transport/hub.c` |
| `compatible = "gs_usb"` のノード | `lib/fcan_transport/gs_usb.c` を重ねる |

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

### 外部 port を USB に向ける

hub の外部 port は配線を持たない。ここに CANnectivity の gs_usb を被せると、
PC は SocketCAN からこのノードのバス全体を 1 本の canX として見る。
自分自身のトラフィックも同じインタフェースに乗る。

これも devicetree で決まる。`gs_usb` のノードを置くと `CONFIG_USBD_GS_USB` が
既定で立ち、それが `CONFIG_FCAN_TRANSPORT_GS_USB` を選ぶ。
conf 側で明示するのは `CONFIG_USB_DEVICE_STACK_NEXT=y` だけである。

```dts
gs_usb0: gs_usb0 {
    compatible = "gs_usb";
    label = "gs_usb";
};

fcan_hub: fcan_hub {
    compatible = "fibril,can-hub";
    status = "okay";
    peers = <&fdcan3>;
};
```

USB を有効にする順序に制約がある。
ホストが列挙できるようになるのは attach の後でなければならない。
`fcan_transport_attach()` が attach に続けて gs_usb を立ち上げるのはこのためで、
アプリケーションから順序を間違える余地を消してある。

gs_usb のノードが無ければ、この呼び出しは何もしない `static inline` になる。
呼び出し側に条件分岐は要らない。

VID/PID と文字列は CANnectivity の既定値に揃えてある。
ホスト側の udev ルールや gs_usb の pid フィルタがそのまま効く。
DFU と MSOSV2 記述子は持たないので、WinUSB の自動バインドや USB 経由の
ファーム更新が要る基板は CANnectivity 本体のアプリケーションを使う。

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
west build -b <board> apps/node -S <transport> -S <deployment> -- -DFCAN_CODEGEN=/abs/path/fcan_codegen
FCAN_CODEGEN=/abs/path/fcan_codegen west twister -T apps
```

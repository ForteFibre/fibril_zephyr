# fibril_zephyr

ForteFibre のロボット用基板で動く Zephyr ファームウェアを 1 か所にまとめた **workspace application** かつ **Zephyr module**。
自作ボードの定義、out-of-tree のドライバ、CAN FD 通信フレームワーク fibril_can を使ったサンプルを持つ。

## 何より先に読むもの

実装に手を入れる前に、最低でも以下に目を通す。

- 全体像と workspace のどこに何があるか: [doc/overview.md](doc/overview.md)
- 触るボードのピン配置とクロック: `boards/fibril/<board>/doc/index.rst`
- 触るドライバの配線要件と失敗の扱い: [doc/drivers/](doc/drivers)
- ドキュメント構成の決定と却下案: [doc/adr/](doc/adr)

## ビルドとテスト

**`west build` と `west flash` はサンドボックスの外で実行する。** ツールチェーンと USB デバイスにアクセスする必要がある。

```shell
west build -b <board> app                                   # ボード持ち込みの動作確認アプリ
west build -b <board> app -- -DEXTRA_CONF_FILE=debug.conf   # 診断用 Kconfig を重ねる
west twister -T tests --integration                          # ztest（native_sim で完結する）
west twister -T app --integration                            # アプリのビルド確認のみ
```

`samples/lib/fibril_can/` 以下のサンプルは `fcan_codegen` の場所を要求する。
`-DFCAN_CODEGEN=/abs/path/fcan_codegen` を渡す。

ドキュメントのビルドは `doc/` で行う。

```shell
cd doc && pip install -r requirements.txt && doxygen && make html
```

## workspace 上の位置

このリポジトリは workspace のルートではない。
`fibril_zephyr_ws/fibril_zephyr/` が本体で、`zephyr/` と `modules/` は 1 つ上にある。

サンプルが参照する fibril_can は **このリポジトリの中にはない**。
`modules/lib/fibril_can` にあり、`west.yml` が revision を固定している。
fibril_can 側の設計と不変条件はそのリポジトリの `CLAUDE.md` と `SPEC.md` が正本で、こちらで再実装しない。

## Zephyr module としての入口

`zephyr/module.yml` がビルドシステムから見た入口を宣言する。
ここを変えるとリポジトリ全体の見え方が変わるので、迂闊に触らない。

- `build.kconfig` → `Kconfig`（`drivers/Kconfig` と `lib/Kconfig` を rsource する）
- `build.cmake` → ルートの `CMakeLists.txt`（`include/` をインクルードパスに加え、`drivers/` と `lib/` を追加する）
- `settings.board_root` / `settings.dts_root` → リポジトリのルート
- `runners` → `scripts/example_runner.py`

`CMakeLists.txt` の `zephyr_syscall_include_directories(include)` は消さない。
`include/drivers/motor.h` と `include/drivers/encoder.h` が `__syscall` を使うため、これがないとシステムコールが生成されずリンクが通らない。

## 触るときに壊してはならない不変条件

見栄えではなく機能要件として効いている境界。

- **ドライバクラスの API は `include/drivers/<class>.h` に置き、実装は `drivers/<class>/` に置く。** デバイス固有の診断はクラスの API に載せず、`include/drivers/<class>/<driver>.h` に分ける。AMT21x の統計取得がその例で、トランザクションの失敗の分類は RS485 プロトコルの性質であってエンコーダ一般の性質ではない。
- **feedback の読み出しはバスを待たない。** エンコーダもモータも、読み出しはキャッシュ済みのスナップショットを返す。制御ループから呼ばれる前提なので、`get_feedback` の中でトランザクションを開始したり完了を待ったりしない。
- **AMT21x の受信は init で一度張り、通常運用中は止めない。** 2 Mbps ではコマンド送出から約 3 us で応答が始まるため、トランザクションごとに受信を張り直すと応答の先頭バイトを必ず落とす。同じ理由で `UART_RX_BUF_REQUEST` には必ず応答する。次のバッファを渡さないと受信が永久に停止する。
- **RS485 のドライバイネーブルは UART のハードウェアに任せる。** ソフトウェアでトグルする実装に変えない。折り返し区間のレイテンシに間に合わない。
- **RoboMaster は CAN バスを feedback から学習する。** バスが未知のあいだ、そのモータには電流指令を送らない。この順序を崩すと、複数バス構成で指令が誤ったバスに出る。
- **初期化時の `can_start()` の失敗を致命的にしない。** トランシーバに電源が来ていないバスはコントローラが初期化モードから出られず、これはデバッグプローブだけで給電しているときの通常の状態である。ここで失敗させると他のバスとモータ device まで使えなくなる。再試行は送信ワークから 500 ms に 1 回までに絞る。毎周期 `can_start()` を呼ぶとハードウェアのタイムアウトで送信周期を守れない。
- **1 台のデバイスの失敗が他のデバイスを止めない。** AMT21x のスキャンも RoboMaster の送信も、失敗したデバイスを飛ばして次に進む。
- **診断機能は Kconfig で切れる状態を保つ。** 製品ビルドでコストを負わないことが前提で、テストも有効時と無効時の両方をビルドしている。
- **ボード定義のピン割り当ては先行ファームウェアと一致させる。** CanMotor と RoboMaster Mini は CanMotorMbed の同名ターゲットから移してあり、既存ハードウェアが配線変更なしで動くことが要件である。ピンを動かすときは、なぜ動かせるのかを devicetree のコメントに残す。

## ドキュメント更新責務

コードを変えたら、対応する文書を**同じ PR で**更新する。

| 変更の種別 | 必須更新先 |
| --- | --- |
| ドライバの追加 | `doc/drivers/<driver>.md`、`doc/index.rst` の toctree、`doc/_doxygen/groups.dox`、公開ヘッダの `@defgroup` |
| ドライバの振る舞いの変更 | 該当する `doc/drivers/<driver>.md` |
| ボードの追加 | `boards/fibril/<board>/doc/index.rst`、`doc/boards/<board>.rst` のインクルードスタブ、`doc/index.rst` の toctree、README のボード表 |
| ボードの devicetree の変更 | 該当ボードの `doc/index.rst`（ピン表と既定の状態） |
| `west.yml` の revision | README と `doc/overview.md` のバージョン記述 |
| ディレクトリの責務、ビルド構成 | `doc/overview.md` |
| サンプルの追加 | サンプルの `README.md`、README のサンプル表 |
| 非自明な設計判断 | `doc/adr/` に新規 ADR と `doc/adr/index.md` の一覧 |

`.github/workflows/docs.yml` は Sphinx を `-W` で走らせる。
**toctree に載っていない文書があると CI が落ちる。**
`doc/` に Markdown を足すときは toctree に加えるか、`conf.py` の `exclude_patterns` に入れる。

ボード文書は Sphinx の srcdir の外にあるため、`doc/boards/<board>.rst` のスタブを忘れると Pages に出ない。
ボード文書の中から他の文書を参照するときは ``:doc:`/drivers/amt21` `` のように srcdir 起点の絶対パスを使う。
スタブ側の相対位置に依存する書き方をすると解決に失敗する。

Markdown 内の相対リンクは、拡張子のないパスを書くと myst が文書への相互参照と解釈して警告になる。
ディレクトリではなくファイルを指す。

## コミットとコメント

### コミット

Conventional Commits に日本語の subject を組み合わせる。
`git log` に出ている形に揃える。

```
feat(boards): rc26_mainair_v01 の board 定義を追加
fix(boards/rc26_mainair_v01): timer node に pinctrl-0 を持たせない
chore(west): fibril_can を main branch に repin
refactor(samples): hub_gs_usb_latency_probe main.c を整形
```

論理的に独立な変更は別コミットに切る。
コミットメッセージには **why** を書く（what は diff から読める）。

### コメント

- **devicetree のコメントには、その値を選んだ理由を書く。** クロックの分周やタイミングのプロパティは、値だけ見ても妥当性が判断できない。既存のボード定義はこれを守っており、ドキュメントの一次情報にもなっている。
- **ドキュメントに書いた背景をコードで繰り返さない。** コードに残すのは、その行を読んでいる人にしか意味のない局所的な why（呼び出し順の制約、暗黙の不変条件、非自明な副作用）に絞る。
- **変更履歴をコメントに書かない。** 履歴は git log と ADR が正本である。

## コードスタイル

`.clang-format` を置いていないため、整形は強制されていない。
スコープごとに違う形が使われているので、触るファイルの周囲に合わせる。

**`drivers/` と `include/` の C コード**（下記の上流由来のファイルを除く）

- インデントは半角スペース 2 つ
- `struct` の定義と関数定義は、次の行に `{` を置く
- `enum` の定義と制御構文は、同じ行に `{` を置く
- ポインタは `const struct device * dev` のように `*` の両側を空ける

**`samples/` の C コード**

Zephyr 上流のスタイルに従う。
インデントはタブ、ポインタは `struct can_frame *frame` と詰める。

**devicetree と binding**

- `boards/` の `.dts` はタブ
- `app/` の overlay は半角スペース 4 つ
- `dts/bindings/` の YAML は半角スペース 2 つ（YAML はタブを許さない）

上流から取り込んだ `drivers/blink/`、`drivers/sensor/example_sensor/`、`include/app/` はタブのままである。
これらを触るときもそのファイルの形に従い、無関係な整形を混ぜない。

## 上流由来の残骸

このリポジトリは Zephyr の `example-application` を出発点にしている。
次のものは上流の雛形がそのまま残っているだけで、実機では使っていない。
整理は別の PR で行う方針なので、ドキュメントやドライバの変更に混ぜない。

`drivers/blink/`、`drivers/sensor/example_sensor/`、`lib/custom/`、`scripts/example_west_command.py`、`scripts/example_runner.py`、`app/sample.yaml` の `name: example-application`、`.github/workflows/build.yml` の `path: example-application`、`boards/fibril/robomaster_v2/` の書きかけ。

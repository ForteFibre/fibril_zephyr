# fibril_zephyr

ForteFibre のロボット用基板で動く Zephyr ファームウェアを 1 か所にまとめた **workspace application** かつ **Zephyr module**。

## 何より先に読むもの

- 全体像と workspace のどこに何があるか: [doc/overview.md](doc/overview.md)
- 触るボードのピン配置とクロック: `boards/fibril/<board>/doc/index.rst`
- 触るドライバの配線要件と失敗の扱い: [doc/drivers/](doc/drivers)
- 構成の決定と却下案: [doc/adr/](doc/adr)

## workspace 上の位置

このリポジトリは workspace のルートではない。
west の workspace ディレクトリ（名前は `west init` の引数で決まる）の直下に `fibril_zephyr/` として置かれ、`zephyr/` と `modules/` は 1 つ上の階層にある兄弟である。

サンプルが参照する fibril_can は **このリポジトリの中にはない**。
`modules/lib/fibril_can` にあり、`west.yml` が revision を固定している。
fibril_can 側の設計と不変条件はそのリポジトリの `CLAUDE.md` と `SPEC.md` が正本で、こちらで再実装しない。

## ディレクトリ構成

| ディレクトリ | 内容 |
| --- | --- |
| `app/` | ボード持ち込みの動作確認用アプリケーション |
| `boards/fibril/` | 自作ボードの定義。ボードごとの文書は各 `doc/index.rst` |
| `drivers/` | out-of-tree ドライバの実装 |
| `dts/bindings/` | 上記ドライバの devicetree binding |
| `include/` | 公開ヘッダ。ドライバクラスの API はここが正本 |
| `lib/` | out-of-tree ライブラリ |
| `samples/` | ドライバ単体および fibril_can と組み合わせたサンプル |
| `tests/` | Twister から走る ztest |
| `scripts/` | west の拡張コマンドと runner |
| `doc/` | ガイド、ADR、Doxygen の設定 |

ドライバクラスの API は `include/drivers/<class>.h`、実装は `drivers/<class>/` に置く。
デバイス固有の API はクラスのヘッダに載せず、`include/drivers/<class>/<driver>.h` に分ける。

### Zephyr module としての入口

`zephyr/module.yml` がビルドシステムから見た入口を宣言する。
ここを変えるとリポジトリ全体の見え方が変わるので、迂闊に触らない。

- `build.kconfig` → `Kconfig`（`drivers/Kconfig` と `lib/Kconfig` を rsource する）
- `build.cmake` → ルートの `CMakeLists.txt`（`include/` をインクルードパスに加え、`drivers/` と `lib/` を追加する）
- `settings.board_root` / `settings.dts_root` → リポジトリのルート
- `runners` → `scripts/example_runner.py`

`CMakeLists.txt` の `zephyr_syscall_include_directories(include)` は消さない。
公開ヘッダが `__syscall` を使うため、これがないとシステムコールが生成されずリンクが通らない。

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

## ドキュメント更新責務

コードを変えたら、対応する文書を**同じ PR で**更新する。

| 変更の種別 | 必須更新先 |
| --- | --- |
| ドライバの追加 | `doc/drivers/<driver>.md`、`doc/index.rst` の toctree、`doc/_doxygen/groups.dox`、公開ヘッダの `@defgroup` |
| ドライバの振る舞いの変更 | 該当する `doc/drivers/<driver>.md` |
| ボードの追加 | `boards/fibril/<board>/doc/index.rst`、`doc/boards/<board>.rst` のインクルードスタブ、`doc/index.rst` の toctree、README のボード表 |
| ボードの devicetree の変更 | 該当ボードの `doc/index.rst`（ピン表と既定の状態） |
| `west.yml` の revision | README と `doc/overview.md` のバージョン記述 |
| ディレクトリの責務、ビルド構成 | `doc/overview.md` と CLAUDE.md のディレクトリ構成表（両方あるので片方だけ直さない） |
| サンプルの追加 | サンプルの `README.md`、README のサンプル表 |
| 非自明な設計判断 | `doc/adr/` に新規 ADR と `doc/adr/index.md` の一覧 |

### 文書を書くときの制約

`.github/workflows/docs.yml` は Sphinx を `-W` で走らせる。
**toctree に載っていない文書があると CI が落ちる。**
`doc/` に Markdown を足すときは toctree に加えるか、`conf.py` の `exclude_patterns` に入れる。

ボード文書は Sphinx の srcdir の外にあるため、`doc/boards/<board>.rst` のスタブを忘れると公開されない。
ボード文書の中から他の文書を参照するときは ``:doc:`/drivers/<driver>` `` のように srcdir 起点の絶対パスを使う。
スタブ側の相対位置に依存する書き方をすると解決に失敗する。

Markdown 内の相対リンクは、拡張子のないパスを書くと myst が文書への相互参照と解釈して警告になる。
ディレクトリではなくファイルを指す。

## コメント

- **devicetree のコメントには、その値を選んだ理由を書く。** クロックの分周やタイミングのプロパティは、値だけ見ても妥当性が判断できない。既存のボード定義はこれを守っており、ドキュメントの一次情報にもなっている。
- **ドキュメントに書いた背景をコードで繰り返さない。** コードに残すのは、その行を読んでいる人にしか意味のない局所的な why（呼び出し順の制約、暗黙の不変条件、非自明な副作用）に絞る。
- **変更履歴をコメントに書かない。** 履歴は git log と ADR が正本である。
- **テストは名前で意図を語る。** 名前で言えないなら名前を延ばす。直上に同じ内容のコメントを重ねない。

## コミット

Conventional Commits に日本語の subject を組み合わせる。
`git log` に出ている形に揃える。

```
feat(boards): rc26_mainair_v01 の board 定義を追加
fix(boards/rc26_mainair_v01): timer node に pinctrl-0 を持たせない
chore(west): fibril_can を main branch に repin
refactor(samples): hub_gs_usb_latency_probe main.c を整形
```

論理的に独立な変更（別レイヤ、別責務、テスト追加、コメント整理）は別コミットに切る。
1 つの PR に複数コミットを載せてよい。
コミットメッセージには **why** を書く（what は diff から読める）。

`main` に直接コミットしない。

## コードスタイル

`.clang-format` を置いていないため、整形は強制されていない。
スコープごとに違う形が使われているので、触るファイルの周囲に合わせる。

**`drivers/` と `include/` の C コード**

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

既存のファイルがこれと違う形なら、そのファイルの形に従う。
無関係な整形を混ぜない。

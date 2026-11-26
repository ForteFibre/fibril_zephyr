---
status: Accepted
date: 2026-09-08
---

# 0001. ドキュメントを doc/ に一本化し、ボード文書は Zephyr 慣習に従う

## Status

Accepted

## Context

このリポジトリは Zephyr の `example-application` を出発点にしている。
その名残として、README は上流の英語ボイラープレートに AMT21x ドライバの解説 100 行が挿入されただけの状態にあり、`doc/` の Sphinx 設定は `project = 'Example Application'` のまま GitHub Pages に公開されていた。
ドライバの解説が README にしかないため、RoboMaster モータドライバのように README に書かれなかったものは無文書のまま残っていた。

一方、同じ組織の `fibril_can` は番号付きの `docs/`、`docs/adr/`、日本語の `CLAUDE.md` という体裁を確立している。
読み方を揃える価値がある。

`doc/` には維持されている資産もある。
`doc/_doxygen/groups.dox` はエンコーダと AMT21x の Doxygen group を実際に整備してあり、`.github/workflows/docs.yml` が Doxygen と Sphinx を Pages に公開している。

開発環境の構築手順は `docs/install-md` ブランチに日本語 350 行の `INSTALL.md` として存在するが、取り込みは別途とした。

## Decision

1. **ドキュメントは日本語で書く。** README から外した AMT21x の解説も、英語のまま移さずに翻訳する。
2. **`doc/` に一本化する。** 新しく `docs/` を作らず、既存の Sphinx ツリーに `myst-parser` を入れて Markdown を解釈させる。ガイド本文は Markdown で書く。
3. **ボードごとの文書は `boards/fibril/<board>/doc/index.rst` に置く。** Zephyr のボードドキュメントの慣習に合わせ、reStructuredText で書く。Sphinx の srcdir は `doc/` なのでこれらは srcdir の外に出るが、`doc/boards/<board>.rst` に `.. include::` だけを持つスタブを置いて toctree に載せる。
4. **ガイド本文のファイル名に番号を振らない。** 順序は toctree が決める。

## Consequences

ガイドは Markdown、ボード文書は reStructuredText という二本立てになる。
`myst-parser` を入れても reStructuredText はそのまま解釈されるので、同じツリーに共存できる。

ボードを追加するときは、`boards/fibril/<board>/doc/index.rst` と `doc/boards/<board>.rst` のスタブ、そして README のボード表の 3 か所を触る。
スタブを忘れるとボード文書が Pages に出ない。

`docs.yml` は Sphinx を `-W` で走らせるため、toctree に載っていない文書があると CI が落ちる。
`doc/` に Markdown を足すときは toctree に加えるか、`conf.py` の `exclude_patterns` に入れる。
`_doxygen/` は Doxygen が自前で描画するページなので `exclude_patterns` に入れてある。

`INSTALL.md` を取り込む時点で、ファイル名に番号を振らない決定のおかげで並びを崩さずに追加できる。

## 却下した選択肢

**`docs/` を新設して Markdown 専用のツリーにする。**
`fibril_can` の体裁にはこちらが近く、GitHub 上でそのまま読める利点もある。
しかし `doc/` と `docs/` が並ぶ状態は、どちらに書くかを毎回迷わせる。

**`doc/` と `docs.yml` を削除して Markdown だけにする。**
最小構成にはなるが、整備済みの `groups.dox` と Doxygen による API リファレンスを捨てることになる。

**ボード文書を `doc/boards/*.md` として Markdown で書く。**
Sphinx の srcdir 内で完結するので仕掛けが要らない。
しかしボード文書が変わる契機は devicetree の変更であり、`.dts` と同じディレクトリに置いたほうが更新を忘れにくい。

**ボード文書を `conf.py` のフックで `doc/boards/` にコピーする。**
srcdir 外の文書を toctree に載せる一般的な回避策として検討したが、不要だった。
docutils の `include` は srcdir 外のファイルを警告なしに読み、Sphinx は依存関係も追跡する。
`-W` 付きのビルドで警告が出ないことを確認した。

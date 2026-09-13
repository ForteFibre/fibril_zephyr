# 設計判断の記録（ADR）

コードを読むだけでは辿れない**選択の理由**と**却下した選択肢**を残す場所。
ガイド本文が「いまどうなっているか」を書くのに対し、ADR は「なぜそうしたか」「ほかの案がなぜ採られなかったか」を書く。

## いつ書くか

- ドライバクラスやボード定義の約束事を増やす、変える、やめる
- ビルド構成やディレクトリの責務を変える
- 外部依存を増やす（新しい Zephyr module、ホスト側ツール）
- 互換性を壊す（公開ヘッダ、devicetree binding、ビルド手順）
- 性能や決定性のために非自明な実装を選ぶ

命名の整理、内部リファクタ、テスト追加は ADR にしない。
コミットメッセージとガイドの更新で足りる。

## 書き方

ファイル名は `NNNN-kebab-title.md`（4 桁ゼロ埋め、空き番号は使わない）。
下の一覧と、このページ末尾の toctree に 1 行足す。

採用後の本文は書き換えない。
判断が覆ったときは新しい ADR を起こし、古い方のステータスを `Superseded by NNNN` に変える。

## ステータス

- **Proposed**: 提案。合意形成中か、実装前。
- **Accepted**: 受理済み。実装と一致している。
- **Deprecated**: もう守らなくてよい。理由を一文添える。
- **Superseded by NNNN**: 別の ADR に置き換わった。

## 一覧

| 番号 | 表題 | ステータス |
| --- | --- | --- |
| [0001](0001-doc-structure.md) | ドキュメントを doc/ に一本化し、ボード文書は Zephyr 慣習に従う | Accepted |
| [0002](0002-encoder-feedback-layering.md) | encoder feedback を積算位置と速度で表し、積算の責務をドライバに置く | Accepted |
| [0003](0003-multi-app-structure.md) | 複数用途のファームウェアを 1 つのアプリケーションと機能ライブラリで構成する | Proposed |

```{toctree}
:maxdepth: 1
:hidden:

0001-doc-structure
0002-encoder-feedback-layering
0003-multi-app-structure
```

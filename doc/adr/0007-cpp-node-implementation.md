---
status: Proposed
date: 2026-09-21
---

# 0007. fibril_can スレーブを C++17 で書き、codegen の C++ ラッパを使う

## Status

Proposed

## Context

`apps/node` と `lib/fibril_can_node/` は C で書かれている。
これから載せたい制御のコード（`fibril_common` の制御器、軌道生成、シリアライザ）は C++17 のライブラリで、ROS 2 側と同じものを使うことがその目的である。
機能の実装が C のままだと、ノードの中に言語の境界が 1 本走り、境界を越えるためのラッパを機能ごとに書くことになる。

`fcan_codegen` は同じスキーマから C ラッパと C++17 ラッパのどちらも出せる。
C++ ラッパは薄いプロキシ層で、ランタイム本体（C99）は共通である。

決めるのは 2 つある。
**言語を移すか**と、**移すならラッパも C++ のものに替えるか**である。

後者は独立した選択肢になる。
生成される C ヘッダは `extern "C"` を持っているので、C ラッパのまま C++ で書くことはできる。

## Decision

`apps/node` と `lib/fibril_can_node/` を C++17 で書き、codegen には `--lang cxx`（CMake では `LANGUAGE CXX`）を渡す。
`drivers/`、`lib/fcan_transport/`、`tests/`、`samples/` は C のままにする。

生成物が C++ になることで、これまで手で守っていた対が型と寿命の側に移る。

- S2M Topic は RAII の publisher になり、`begin` と `commit` の対を書かなくなる
- M2S Topic は `std::optional` になり、戻り値の判定を忘れると値が取り出せない
- サービスハンドラは λ をインスタンスごとに登録する形になり、インスタンス番号は捕捉した値になる

3 つ目の代償として、**ハンドラの実装漏れがリンクエラーではなくなる**。
C ラッパは生成された宣言に対する実体が要るのでリンクが落ちたが、C++ では未登録のスロットが BAD_INDEX で応答される。
これを受けて、ハンドラの登録は各機能の `start` に集約する約束にする。
`start` は `register_all` の後・`fcan_transport_attach` の前という、ノードがまだバスに開いていない唯一の窓であり、登録と初期状態の publish を同じ場所に置ける。

`instance_counts` の埋め方は変えない。
`fibril_fcan_func` を辿って各機能が自分の枠を埋める形のままで、添字だけ `FCAN_ARRAY_<TYPE>` から `fcan_gen::<type>::block_array_index` に移る。

## Consequences

### C++ 標準ライブラリはフルのものが要る

生成ヘッダが `<functional>` と `<optional>` を include するため、`MINIMAL_LIBCPP` では足りない。
`CONFIG_REQUIRES_FULL_LIBCPP=y`（picolibc + libstdc++）を `apps/node/prj.conf` に置く。

RC26 MainAir V01（STM32G474）での実測は次のとおりで、心配したほどの増分にはならなかった。

| イメージ | FLASH | RAM |
| --- | --- | --- |
| `rc26-air`（C） | 86916 B | 48816 B |
| `rc26-air`（C++） | 87744 B | 49008 B |
| `rc26-robstride`（C） | 97936 B | 49904 B |
| `rc26-robstride`（C++） | 99924 B | 50416 B |

### ハンドラが掴むのはインスタンス番号だけにする

`std::function` はキャプチャが小さいうちはオブジェクトの中に収まるが、超えるとヒープを使う。
起動時に 1 回とはいえ、MCU でヒープに落ちる条件を機能ごとに考えたくない。
**ハンドラが捕捉してよいのはインスタンス番号までとし、状態はファイルスコープの配列に置く**。
現に、この規律で組んだ 2 つのイメージのどちらにも `operator new` はリンクされていない。

### `_MAX` の Kconfig は `.bss` を少し多く使うようになる

ハンドラのスロットは `std::array<..., max_count>` で静的に確保される。
`max_count` を余らせるコストに、サービス 1 つあたり `sizeof(std::function)`（32 bit ARM で 16 B）× 余り個数が加わる。
[doc/apps.md](../apps.md) の「`max_count` を Kconfig で持つ理由」はこの分を含めて読む。

### 静的コンストラクタが 1 つ増える

生成された `.cpp` のハンドラ置き場は `std::function` の配列なので、動的初期化が要る。
Zephyr は `CONFIG_STATIC_INIT_GNU` の `z_static_init_gnu()` を `INIT_LEVEL_APPLICATION` の前に走らせるので、`main` より先に構築は終わっている。
機能の側で静的初期化順に依存するオブジェクトを増やさない限り、この 1 つで済む。

### カットオーバーはイメージ単位になる

`fcan_invoke_codegen` は 1 つの言語しか出さず、C ラッパと C++ ラッパを同じターゲットにリンクする想定も無い。
機能を 1 つずつ移すことはできないので、実装済みのブロック型は同時に移す。

### fibril_can に前提が 1 つ増える

Zephyr 側のヘルパ `fcan_invoke_codegen` は `--lang c` 決め打ちだったので、`LANGUAGE` を受け取れるようにする変更を fibril_can に入れ、`west.yml` の revision をそこまで進めた。

## Alternatives considered

### C ラッパのまま C++ で書く

生成 C ヘッダは `extern "C"` を持つので、上流に手を入れずに今日できる。
却下した理由は、得られるものが言語だけだからである。
`begin`/`commit` の対も、`*_read` の戻り値の判定も、インスタンス番号の範囲検査も C のときと同じ形で残り、`robstride.c` の見た目はほとんど変わらない。
上流の変更 1 本で消せるものを、機能が増えるたびに書き続ける取引になる。

### `make_instance_counts` でカウントをコンパイル時に決める

C++ ラッパが薦める形だが、`make_instance_counts` は**全ブロック型を呼び出し側が名指しする** `static_assert` を持つ。
アプリケーションがブロック型の名前を 1 つも持たないことは [ADR 0003](0003-multi-app-structure.md) の中心なので、これを使うとアプリケーションが機能の一覧を知ることになる。
`max_instance_counts.data()` を渡す近道もあるが、これは Kconfig の上限を ANNOUNCE してしまい、配線していないインスタンスがバスに現れる。

### iterable section をやめて仮想基底と静的レジストリにする

C++ にするなら登録も C++ らしくできるが、`FIBRIL_FCAN_FUNC_DEFINE` は「アプリケーションがブロック型を名指ししない」をリンク時に解いていて、静的初期化順の問題も持たない。
仮想基底に替えると、機能ごとに動的初期化と vtable が増えるうえ、登録順が翻訳単位の順序に依存する。
移植と設計変更を同じ変更に混ぜない方針もあり、据え置いた。

---
status: Proposed
date: 2026-09-13
---

# 0005. motor_feedback.position を 64 bit の積算カウントにする

## Status

Proposed

## Context

ADR 0002 は `encoder_feedback.position` を `int64_t` の積算位置と決め、`motor_feedback.position` については型が揃わないことを宿題として残した。
残した理由は、当時 motor クラスの利用者が `drivers/motor/robomaster.c` だけで、統合の必要が立っていなかったためである。

`int32_t` のままでは実際に溢れる。
8192 counts/rev のロータで約 26 万回転、9000 rpm なら連続稼働 30 分弱で到達する。
溢れたとき、読み手には積算器が壊れたのか本当にそこまで回ったのかが区別できない。

RobStride のドライバを足すにあたって、この宿題が先に来た。
RobStride の積算位置も同じ性質を持つので、幅を決めないまま ADR 0004 の feedback の形を決められない。

## Decision

1. **`motor_feedback.position` を `int64_t` にする。** `encoder_feedback.position` と型が揃う。

2. **意味は「ドライバが折り返しを解いた積算カウント」とする。** これは `drivers/motor/robomaster.c` が既にしていることの明文化であり、新しい約束ではない。

3. **単位はドライバ固有とし、換算係数は各ドライバの文書に書く。** 物理量への変換は制御層に置く。ADR 0002 の決定 11 と同じ扱いである。

4. **換算係数のために devicetree のプロパティを増やさない。** RoboMaster の 8192 counts/rev は `model` が決まれば定数であり、RobStride の 8π rad あたり 65535 カウントは全機種で共通である。どちらもドライバが知っている定数なので、devicetree に書かせる理由がない。

5. **`orientation` は `int32_t` のまま残す。** 単回転の生値であって有界なので、幅が要らない。

6. **`enum motor_feedback_type` は変えない。** `MOTOR_FEEDBACK_POSITION` の意味は変わらない。

## Consequences

変更は 2 行である。
`include/drivers/motor.h` のフィールド宣言と、`tests/drivers/motor/robomaster/src/main.c` の `expected_position` の型である。
`drivers/motor/robomaster.c` は手を入れなくてよい。
積算は `int32_t` の差分を加算する形なので、加算先が広がるだけで通る。
初回の代入も `uint16_t` からの代入なので変わらない。

生成される `zephyr/syscalls/motor.h` は変わらない。
`motor_get_feedback()` の引数は `void *` なので、構造体の中身は署名に現れない。

`struct motor_feedback` は 4 byte 大きくなる。
アライメントを考えると実際にはもう少し増えるが、モータ 1 台あたりの話であり、`drivers/motor/robomaster.c` が上限としている 8 台でも数十 byte に収まる。

`position` を出力している箇所は今のところない。
サンプルとアプリが表示しているのは encoder 側の feedback である。
`int64_t` を `printk` で出すときに `%lld` が要る点は、最初の読み手が出てきたときに効く。

ADR 0004 の RobStride ドライバは、`MOTOR_FEEDBACK_POSITION` を立てられるようになる。
どのカウントを積算するかは ADR 0004 側の決定に属する。

## 却下した選択肢

**`int32_t` のまま残し、溢れの扱いを制御層に任せる。**
公開ヘッダを触らずに済む。
しかし折り返しを検出するには、積算器が何回巻き戻ったかを知る必要がある。
それはドライバの内側にしかない情報で、`position_epoch` に相当するものを motor クラスに足すほうが、幅を広げるより大がかりになる。

**`position` をクラス共通の SI 固定小数（µrad など）にする。**
消費者がドライバを知らずに物理量として読めるようになる。
しかし ADR 0002 の決定 11 は、counts から物理量への変換を制御層に置くと決めている。
motor クラスだけ別の規則にすると、encoder と motor の feedback を並べて扱う制御層が 2 つの流儀を覚えることになる。
`drivers/motor/robomaster.c` の報告値も変わるので、それを読む側の調整が必要になる。

**`orientation` も 64 bit にする。**
2 つのフィールドの型が揃う。
しかし `orientation` は単回転の生値であり、定義上有界である。
積算しないものに積算用の幅を与える理由がない。

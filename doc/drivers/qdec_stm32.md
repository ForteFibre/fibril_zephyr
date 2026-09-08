# 直交エンコーダ（`fibril,stm32-qdec`）

STM32 の汎用タイマをエンコーダモードで使い、直交出力のインクリメンタルエンコーダを読むドライバ。
タイマがエッジを数えるところまではハードウェアが行い、ドライバはカウンタを定期的に読んで積算位置と速度に変換する。

共通のエンコーダインタフェースは [include/drivers/encoder.h](../../include/drivers/encoder.h) にある。
このドライバ固有の診断はないので、専用のヘッダは持たない。

## デバイスの構成

ノードはタイマノードの子として置き、pinctrl は子が持つ。
タイマノード自身に `pinctrl-0` を書かない。

実機で通ったデバイスツリーの記述は [samples/drivers/qdec/boards/fibril_rc26_mainair_v01.overlay](../../samples/drivers/qdec/boards/fibril_rc26_mainair_v01.overlay) にある。

### ノード名の制約

**子ノードの名前に `qdec` を使わない。**
SoC の dtsi が各タイマの下に上流の `st,stm32-qdec` 用の無効ノードを同じ名前で持っている。
同じ名前を書くとそこにマージされ、`compatible` は上書きできても `st,input-filter-level` などのプロパティを引き継いでしまい、binding に宣言がないというエラーになる。

**1 つのターゲットに複数付けるときは、名前も別にする。**
このノードは `reg` を持たないため device 名はノード名そのものになる。
同じ名前のノードが 2 つあると、名前で区別できない device が 2 つできる。

## 上流の `st,stm32-qdec` を使わない理由

上流のドライバはタイマの auto-reload を counts-per-revolution の倍数に設定する。
2 の冪ではない値になるうえ、その値を API で公開しない。
`SENSOR_CHAN_ENCODER_COUNT` が返す生カウンタがどこでラップするのかを制御層が知りようがなく、積算位置を組み立てられない。

こちらは auto-reload をカウンタの全幅に置く（16 bit なら `0xFFFF`、32 bit なら `0xFFFFFFFF`）。
ラップ境界が 2 の冪に固定されるので、剰余差分だけでラップを解ける。
32 bit カウンタを持つのは G4 と F4 の TIM2・TIM5 で、判定は `IS_TIM_32B_COUNTER_INSTANCE` が行う。

判断の背景は [ADR 0002](../adr/0002-encoder-feedback-layering.md) にある。

## タイマの設定

**親の `st,prescaler` は 0 でなければならない。**
プリスケーラはエンコーダモードでも分周するので、0 以外だと counts を落とす。
`BUILD_ASSERT` でビルド時に落ちる。

`encoder-mode` は SMCR.SMS に書く値で、既定は x4（両入力の全エッジを数える）である。
データシートの counts-per-revolution はたいてい x4 を前提にしている。
x1 モード（`0x10006`、`0x10007`）は SMS の bit 3 を必要とし、これを持たない STM32 では `BUILD_ASSERT` がビルドを止める。

`input-filter-level` はデジタルフィルタの強さである。
配線が長い、あるいはシールドされていないために、動いていないのに counts が増えるような場合に上げる。

## ポーリング

ドライバは読み出し要求のあるなしに関わらず、`poll-interval-us` ごとにカウンタを読む。
要求時にだけ読む実装では、カウンタが 1 周する間に 1 回も読まれなければ積算が復元できない。
読み手が複数いる場合や制御ループが詰まった場合に、これは実際に起きる。

サンプリングは `k_timer` の満了ハンドラの中、つまり割り込み文脈で行う。
レジスタ 1 本の読み出しと整数演算しかないので、ワークキューに投げても得るものがなく、スケジューリング遅延が `sample_interval_us` に乗るぶんだけ悪くなる。

**`poll-interval-us` は解決できる速度の上限を決める。**
1 サンプルあたりの回転が半周を超えると、ラップがどちら向きだったのか区別がつかない。
16 bit カウンタなら 1 サンプルあたり 32768 counts が限界で、`poll-interval-us` が 1000 なら 32768 counts/ms になる。
2048 counts/rev のエンコーダで毎秒 16000 回転に相当するので、通常の機構では問題にならない。

`poll-interval-us` がカーネルの 1 tick より小さいと切り上げられる。
STM32 の既定は 10 kHz なので 100 us が下限である。

## フィードバック

`encoder_get_feedback()` はポーリングが最後に書いたスナップショットを返す。

| フィールド | 内容 |
| --- | --- |
| `position` | 積算位置（counts）。ラップ解決済み、`encoder_set_position()` のオフセット適用後 |
| `velocity` | 速度（counts/s） |
| `sample_interval_us` | 上の 2 つを求めた実測サンプル間隔 |
| `position_epoch` | 積算器を作り直した回数。`encoder_reset()` でのみ増える |
| `online` | 最初のサンプリング以降は常に true |
| `stale` | 常に false |
| `error_count` | 常に 0 |

`valid_mask` に立つのは `ENCODER_FEEDBACK_POSITION` と、2 サンプル目以降は `ENCODER_FEEDBACK_VELOCITY` である。
インデックスパルスがなければ単回転絶対位置は定まらないので、`SINGLE_TURN` と `TURNS` は立たない。

`online`、`stale`、`error_count` が固定値なのは、ハードウェアカウンタの読み出しに失敗する経路がないためである。
断線してもカウンタは動かなくなるだけで、ドライバからは静止しているのと区別がつかない。
**この 3 つでエンコーダの断線は検出できない。**

### 積算位置の起点

インクリメンタルエンコーダには絶対的な基準がないので、起動時の軸の位置が 0 になる。
`position_epoch` が増えるのは `encoder_reset()` を呼んだときだけである。

### 速度

単区間の差分なので、量子化ノイズが低速で信号を上回る。
どの程度平滑化するかは応用ごとに違うため、フィルタはドライバに持たせていない（ADR 0002 項 6）。

### 物理量への換算

`counts-per-revolution` は devicetree に置いてあるが、**ドライバはこれを使わない**。
counts から物理量への換算は制御層の仕事で、減速比や車輪径はデバイスではなくアプリの属性だからである（ADR 0002 項 11）。
アプリは `DT_PROP(node, counts_per_revolution)` で読む。

## API のうち対応しないもの

| 関数 | 戻り値 |
| --- | --- |
| `encoder_get_resolution()` | `-ENOTSUP`。単回転絶対位置を持たないので報告する分解能がない |
| `encoder_set_zero()` | `-ENOTSUP`。デバイス側に保存できるゼロ点がない。現在位置を 0 にしたいなら `encoder_set_position()` を使う |

`encoder_set_position()` が受け付ける大きさには上限（2^62）があり、超えると `-EINVAL` を返す。
オフセット自身と、以降の積算値との和が `int64_t` に収まる必要があるためで、amt21 と共通の制限である。

`encoder_reset()` はカウンタを 0 に戻し、積算器を作り直して `position_epoch` を進める。

## 注意点

- **回転の向きが逆なら overlay に `invert-direction` を足す。** 入力の極性反転では直らない。x4 モードで両方の入力を反転しても向きは変わらず、片方だけ反転すると位相関係もずれる。ドライバは符号をソフトウェアで反転する
- 1 回転させたときの `position` の変化が `counts-per-revolution` と合わない場合は、`encoder-mode` が想定と違う（x4 のつもりが x2 になっている等）ことを疑う
- 32 bit タイマ（TIM2、TIM5）を使えばラップ間隔は 65536 倍になるが、`poll-interval-us` を緩めてよい理由にはならない。速度の実測区間もこの間隔で決まる

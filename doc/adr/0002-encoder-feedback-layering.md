---
status: Proposed
date: 2026-09-08
---

# 0002. encoder feedback を積算位置と速度で表し、積算の責務をドライバに置く

## Status

Proposed

## Context

CanMotorMbed の制御ファームウェアをこのリポジトリに移植する見込みが立っている。
移植で守るべき本質は、Mbed 側の制御ループがエンコーダの種類を知らずに動くという性質である。
Mbed 側は quadrature、AMT21x（RS485）、RoboMaster（CAN）の 3 種を単一の抽象クラスの下に置き、制御ループは抽象クラスへの参照 1 本だけを持つ。

その抽象クラスが制御ループに渡している値は、符号付き積算位置（ラップ解決済み、多回転）、直近の更新区間での位置差分、物理量へのスケール係数、ソフトウェアオフセット、健全性の真偽値である。

現在の `include/drivers/encoder.h` は絶対値エンコーダの形をしている。
`position` は単回転の `uint32_t`（0 から 2^resolution - 1）、`turns` は `int32_t`、ほかに `angle_mdeg` がある。
`online`、`stale`、`timestamp_ms`、`error_count` は Mbed 側の健全性フラグより情報量が多い。
一方、積算位置、速度、ソフトウェアオフセットに相当するものはない。

積算をドライバが持つ前例はすでにある。
`drivers/motor/robomaster.c` は 8192 counts のラップを解いて `motor_feedback.position` に積算し、生のロータ角は `orientation` に置いている。

積算を制御層に置く案は、`timestamp_ms` の分解能が阻む。
`drivers/encoder/amt21.c` は `k_uptime_get()` でミリ秒精度のタイムスタンプを打つが、AMT21 バスの既定ポーリング間隔は 1000 us である。
制御層が位置差分から速度を求めようとすると、除数がタイムスタンプ 1 カウントぶんしかない。

Zephyr 上流の `zephyr/drivers/sensor/st/qdec_stm32/qdec_stm32.c` を quadrature の実装として流用する道も検討した。
このドライバは auto-reload を counts-per-revolution の倍数（2 の冪ではない値）に設定しておきながらその値を公開せず、速度もラップ解決も出さない。
制御層でラップを戻すのに必要な情報が API から取り出せない。

## Decision

1. **`encoder_feedback.position` を符号付き積算位置にする。** 型は `int64_t`、単位は counts、ラップ解決とソフトウェアオフセットの適用をドライバが済ませた値とする。
2. **単回転絶対位置は `single_turn` に改名する。** `uint32_t`、0 から 2^resolution - 1。既存の `turns` と対になって、デバイスが報告した生値であることを名前で示す。
3. **`single_turn` と `turns` にはオフセットを適用しない。** オフセットが効くのは `position` だけとする。単回転値をずらすには剰余演算が必要で、しかも制御には使わない。
4. **速度は `counts/s` で `int32_t` の `velocity` に載せる。** ドライバが算出する。
5. **速度の算出に公称ポーリング間隔を使わない。** `drivers/encoder/amt21.c` の poll thread はオーバーランした scan を捨てるため、scan が飛べば区間は 2 倍になる。エンコーダごとに commit 時刻を記録し、実測区間で割る。実測区間は `sample_interval_us` として feedback に載せ、制御層が自分でフィルタ窓を決められるようにする。
6. **速度のフィルタは制御層に置く。** 単区間差分の量子化ノイズは低速で信号を上回る。14 bit で区間 1000 us なら 1 count の誤差が 1000 counts/s になり、出力軸 1 rpm 相当の 273 counts/s を超える。どの程度平滑化するかは応用ごとに違うので、ドライバに方針を持たせない。
7. **`angle_mdeg` を削る。** `single_turn` と `resolution` から求まる。使っているのは shell とサンプルの表示だけである。
8. **ソフトウェアオフセットはドライバが持つ。** API は `encoder_set_position()` の 1 本とし、現在の積算位置を引数の値にする。Mbed 側のオフセット指定はこの 1 本で表せる。
9. **積算器の不連続を `position_epoch` で通知する。** ドライバが積算器を作り直すたびに値を進める。制御層は値が変わったら積分と微分の状態を捨てる。
10. **`position` の起点は、絶対値エンコーダなら初回に読めた絶対位置、インクリメンタルなら 0 とする。** 電源投入時の位置が意味を持つことが絶対値エンコーダの存在理由なので、これを 0 に潰さない。
11. **counts から物理量への変換は制御層に置く。** デバイス固有の分解能と counts-per-revolution は devicetree に、減速比と車輪径はアプリの設定に分ける。Mbed 側はこの両方をスケール係数 1 つに畳んでいた。

適用後の形はこうなる。

```c
enum encoder_feedback_type {
  ENCODER_FEEDBACK_POSITION    = 1,
  ENCODER_FEEDBACK_VELOCITY    = 1 << 1,
  ENCODER_FEEDBACK_SINGLE_TURN = 1 << 2,
  ENCODER_FEEDBACK_TURNS       = 1 << 3,
};

struct encoder_feedback
{
  uint32_t valid_mask;

  /* ドライバの連続位置推定 */
  int64_t position;
  int32_t velocity;
  uint32_t sample_interval_us;
  uint32_t position_epoch;

  /* デバイスが報告した生値 */
  uint32_t single_turn;
  int32_t turns;

  bool online;
  bool stale;
  int64_t timestamp_ms;
  uint32_t error_count;
};
```

`valid_mask` の立ち方は、AMT21 多回転品がすべて、AMT21 単回転品が `TURNS` 以外、quadrature が `POSITION` と `VELOCITY` だけになる。
quadrature がこの 2 つに限られるのは、インデックスパルスなしに単回転絶対位置が定まらないためである。

## Consequences

改名が触るのは 5 ファイル、約 25 行である。
`drivers/encoder/amt21.c` が 5 行、`drivers/encoder/amt21_shell.c` が 2 行、`samples/drivers/amt21/src/main.c` が 2 行、`app/src/main.c` が 1 行、`tests/drivers/encoder/amt21/src/main.c` が 15 行。
テストの 15 行はすべて単回転値のアサーションなので、`fb.position` を `fb.single_turn` に置き換えるだけの機械的な変更になる。
`drivers/motor/robomaster.c` の `feedback.position` は `struct motor_feedback` 側なので影響を受けない。
テストは enum 値をリテラルで書いていないので、ビットを振り直しても壊れない。
`doc/drivers/amt21.md` がフィールド名に触れているのは `error_count` の 1 か所だけである。

構造体のフィールドを増やすだけなら生成される `zephyr/syscalls/encoder.h` に触らずに済む。
`encoder_set_position()` は関数なのでここに含まれず、生成物が変わる。

Mbed からの移植では速度の単位が変わる。
Mbed 側の位置差分は制御 tick あたりの counts であり、失速検出の速度閾値と PID ゲインはその量に対して調整されている。
`counts/s` に正規化すると、これらの定数は制御周期ぶんスケールし直しになる。

`motor_feedback.position` は `int32_t` のままなので、encoder と型が揃わない。
8192 counts/rev のロータでは約 26 万回転で溢れ、9000 rpm なら連続稼働 30 分弱で到達する。
encoder と motor の feedback を統合する時点で対応する。

quadrature ドライバは `drivers/encoder/` に自前で書くことになる。
このドライバは読み出し要求のない期間も積算を進める必要があるため、`cui,amt21.yaml` の `poll-interval-us` と同じ形で自分でポーリングする。
要求時にだけラップを解く実装は、読み手が複数いる場合や制御ループが詰まった場合に、16 bit カウンタが 1 周する間に復元不能になる。

RoboMaster のロータ角を encoder として見せる shim device は、この決定の範囲外とする。
Mbed 側でモータ内蔵エンコーダが同一の抽象クラスに乗っていたのと同じ役割を果たすが、encoder クラスの形が固まってから別に決める。

counts から物理量への変換を制御層に置くと決めたが、Mbed 側でその層を提供している fibril_common は `west.yml` に入っていない。
どう取り込むかは別の決定として残る。

## 却下した選択肢

**`position` を単回転のまま残し、積算値を `count` として足す。**
既存の意味を変えないので改名の手間がない。
しかし積算位置は制御ループが常に使う値であり、単回転絶対位置は絶対値エンコーダでしか意味を持たない。
主たる値に副次的な名前を与えることになる。

**積算を制御層に置く。**
ドライバが状態を持たずに済み、読み手ごとに独立した積算器を持てる。
しかし AMT21 では単回転値と turns カウンタの二重のラップを制御層が再実装することになり、しかも `drivers/encoder/amt21.c` が持っている「どちらの読み出しが成功したか」を知らないまま行う必要がある。
`drivers/motor/robomaster.c` の既存の振る舞いとも食い違う。

**速度を制御 tick あたりの counts のままにする。**
Mbed 側の閾値とゲインをそのまま運べる。
しかしドライバの標本化周期と制御周期が一致する保証はなく、AMT21 バスは scan を捨てることがある。
制御周期を変えると同じ物理速度が別の数値になる API になる。

**ソフトウェアオフセットを制御層に持たせる。**
ドライバが状態を持たずに済む。
しかし積算器が作り直される契機はドライバの内側にしか見えない。
`encoder_reset()` 後の復帰、`encoder_set_zero()` によるデバイス再起動、offline からの復帰、AMT21 多回転品の turns カウンタが電源断でゼロに戻る場合がそれに当たる。
制御層のオフセットはこれらで黙って無効になる。
また、ハードウェアのゼロ点を持つデバイスに位置を設定するとき、EEPROM に書くのか RAM に留めるのかはドライバしか判断できない。
`dts/bindings/encoder/cui,amt21-encoder.yaml` によればゼロ点を保存できるのは単回転品だけである。

**上流の `qdec_stm32` を sensor API 越しに使う。**
実装を書かずに済む。
しかし auto-reload の値を公開しないため、`SENSOR_CHAN_ENCODER_COUNT` が返す生カウンタのラップ境界が制御層から分からない。
速度も積算も自前で用意することになり、得られるのはタイマの初期化コードだけである。

**`angle_mdeg` を残す。**
表示に便利で、既存のサンプルと shell がそのまま動く。
しかし `single_turn` と `resolution` から求まる冗長なフィールドであり、`position` が積算値になったあとは「どの回転の角度か」が曖昧になる。

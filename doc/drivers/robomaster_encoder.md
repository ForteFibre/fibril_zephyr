# RoboMaster のロータ角（`dji,robomaster-encoder`）

DJI RoboMaster C610 と C620 が feedback フレームで返すロータ角を、encoder クラスの device として見せるドライバ。
制御ループが、軸に付けたエンコーダの代わりにモータ自身のロータで閉ループを組めるようにする。
CanMotorMbed の `ROBOMASTER_ENCODER`（`RoboMasterEncoder`）にあたる。

共通のエンコーダインタフェースは [include/drivers/encoder.h](../../include/drivers/encoder.h) にある。
このドライバ固有の診断はないので、専用のヘッダは持たない。
判断の理由と却下した案は [ADR 0011](../adr/0011-robomaster-rotor-encoder.md) にある。

## デバイスの構成

ノードは `dji,robomaster-motor` のノードを `motor` で指す。
置き場所は問わない（RoboMaster のバスの子ではない）。

```devicetree
rm_enc1: robomaster-encoder-1 {
        compatible = "dji,robomaster-encoder";
        motor = <&rm_motor1>;
};
```

1 台のモータを読めるエンコーダは 1 つだけである。
2 つ目は初期化で `-EBUSY` になる。

このノードは `reg` を持たないので、device 名はノード名そのものになる。
複数置くときは名前を分ける。

`invert-direction` を付けると、位置と速度の符号が逆になる。
`single_turn` は ESC が返した値のまま変わらない。

`CONFIG_ENCODER_ROBOMASTER` は、このノードがあり、RoboMaster のモータドライバが有効なときに既定で立つ。

## 値の意味

| フィールド | 中身 |
| --- | --- |
| `position` | ロータ角の差分を積算した値。8192 counts/rev（減速前のロータ 1 回転） |
| `velocity` | 直前のフレームとの差分を、実際に測った間隔で割った値（counts/s） |
| `sample_interval_us` | その間隔。ESC は 1 ms ごとに返すので、ふつうは約 1000 us |
| `single_turn` | ESC が返したロータ角（0〜8191） |
| `position_epoch` | 積算をやり直すたびに進む |

`get_resolution` は 13（bit）を返す。

**counts は減速前のロータの counts である。**
M3508（C620）なら出力軸 1 回転は約 19 × 8192 counts になる。
物理量への換算は制御層が持つ（MdMotor なら `encoder/gain` に減速比を含める）。

速度に ESC が返す rpm を使わないのは、CanMotorMbed がロータ角の差分を使っていたためである。
1 ms のフレームでは 1 count の差が 1000 counts/s に当たり、低速では量子化が粗い。
MdMotor の速度のローパス（`velocity/filter_coe`）が、Mbed と同じくこれを均す。

## 積算の始まりとやり直し

最初のフレームのロータ角が、積算の始まりになる（絶対値エンコーダと同じ扱い）。
このとき `position_epoch` が進む。
最初のフレームだけでは間隔が無いので、`ENCODER_FEEDBACK_VELOCITY` は立たない。

モータが `feedback-timeout-ms`（`dji,robomaster` のプロパティ）より長く黙った後の最初のフレームでも、積算をやり直す。
途絶えている間にロータが何回転したかは分からないので、差分を足すと位置が嘘になる。
MdMotor は `position_epoch` が変わると出力を止め、マスターが指令し直すのを待つ。

**これより短い取りこぼしでは積算をやり直さない。**
差分は ±4096 counts の範囲で解くので、フレームの間にロータが半回転以上回ると、向きを取り違える。
9000 rpm のロータは 1 ms で約 1229 counts 回るので、3 フレーム続けて落とすと起きうる。
CanMotorMbed も同じ前提だった。

## API の振る舞い

| 関数 | 振る舞い |
| --- | --- |
| `encoder_get_feedback()` | 一度もフレームが無ければ `-ENODATA`。モータの feedback が古ければ `-EAGAIN` で `stale` が立つ。鮮度の判定はモータドライバに任せる |
| `encoder_set_position()` | 現在の積算位置を指定の値にする。ESC には何も書かない |
| `encoder_reset()` | 次のフレームから積算をやり直す。それまでは `-ENODATA` になる |
| `encoder_set_zero()` | `-ENOTSUP`。ESC にゼロ点を保存する場所は無い |

## しくみ

ドライバはモータの `motor_get_feedback()` を読みに行くのではなく、RoboMaster のモータドライバの受信経路から、フレームごとにロータ角と受信時刻を受け取る（`robomaster_motor_set_rotor_callback()`、[robomaster](robomaster.md) の「ロータ角の通知」）。
速度の割り算に使う間隔と、途絶えたことは、受信のたびに見ないと分からないためである。
積算は他のエンコーダドライバと同じ `encoder_accum` が行う。

## テスト

- [tests/drivers/encoder/robomaster](../../tests/drivers/encoder/robomaster/src/main.c) — 最初のフレームから始まること、折り返しと測った間隔での速度、`invert-direction`、途絶えた後のやり直し、`encoder_set_position()` と `encoder_reset()`、受け手が 1 つに限られること

```shell
west twister -T tests/drivers/encoder/robomaster --integration
```

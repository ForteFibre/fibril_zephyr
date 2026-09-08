# 直交エンコーダの読み出し

STM32 のタイマでデコードした直交エンコーダを 1 秒ごとに読み、積算位置と速度を回転数に直して表示する。
配線と `counts-per-revolution` が合っているかは、軸を手で回して表示が追随するかで確かめられる。

ドライバの説明は [doc/drivers/qdec_stm32.md](../../../doc/drivers/qdec_stm32.md) にある。

## 対応ボード

| ボード | 配線 |
| --- | --- |
| `fibril_rc26_mainair_v01` | J11（Encoder3）が TIM3_CH1/CH2（PB4/PB5）、J12（Encoder4）が TIM4_CH1/CH2（PB6/PB7） |

ほかのボードで動かすには `boards/<board>.overlay` を足す。
タイマノードを有効にして `st,prescaler = <0>` を書き、pinctrl を持つ子ノードを付ける。
**子ノードの名前に `qdec` を使わない。** SoC の dtsi が各タイマの下に同名の無効ノードを持っており、同じ名前を書くとそこにマージされて上流の `st,*` プロパティを引き継いでしまう。

複数付けるときは名前も別にする。
このノードは `reg` を持たないので device 名がノード名そのものになり、同じ名前だと 2 つの device を名前で区別できない。

## ビルドと実行

```shell
west build -b fibril_rc26_mainair_v01 samples/drivers/qdec
west flash
```

## 出力

```
[00:00:00.001,000] <inf> qdec_read: encoder3: 2048 counts per revolution
[00:00:00.001,000] <inf> qdec_read: encoder4: 2048 counts per revolution
[00:00:01.002,000] <inf> qdec_read: encoder3: pos=1024 (0.500 rev) vel=512 counts/s (0.250 rev/s) interval=1000us epoch=0
[00:00:01.002,000] <inf> qdec_read: encoder4: pos=0 (0.000 rev) vel=0 counts/s (0.000 rev/s) interval=1000us epoch=0
```

最初のサンプリングが終わるまでは `no sample yet` が出る。
表示は 1 秒ごとなので、`poll-interval-us` を 1 秒以上にしていれば複数回出る。

`vel=n/a` は速度がまだ求まっていないという意味である。
積算器を作り直した直後は区間が測れないので、1 回ぶんこう出る。

`epoch` は積算器を作り直した回数である。
回している最中にこれが増えるなら、`encoder_reset()` を呼んだ覚えがない限り異常である。

## 確認すること

- 軸を一方向に回すと `pos` が単調に増える（または減る）。逆に回すと符号が反転する
- 1 回転させたときの `pos` の変化が `counts-per-revolution` と一致する。合わなければ overlay の値か `encoder-mode` が違う
- 増える向きが期待と逆なら、overlay の子ノードに `invert-direction` を足す
- カウンタが 1 周する 65536 counts をまたいでも `pos` が飛ばない

## 注意

overlay の `counts-per-revolution` は**プレースホルダである**。
このボードには何が挿さるか決まっていないので、実際に使うエンコーダのデータシートの値に直すこと。
ドライバはこの値を使わないので、間違っていても `pos` と `vel` の counts は正しい。
狂うのは回転数への換算だけである。

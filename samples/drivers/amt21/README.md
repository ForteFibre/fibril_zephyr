# AMT21x エンコーダの読み出し

半二重 RS485 バス上の AMT21x アブソリュートエンコーダを読み、1 秒ごとに状態をログに出すサンプル。

devicetree で `status = "okay"` になっている `cui,amt21-encoder` の子ノードを `DT_FOREACH_STATUS_OKAY` で列挙するので、ノードを増やせばそのまま対象が増える。
1 台も有効になっていない場合は `BUILD_ASSERT` でビルドが止まる。

出力にはエンコーダの位置、回転数、角度に加えて、`online` と `stale`、エラー件数が並ぶ。
`CONFIG_ENCODER_AMT21_STATS` を有効にしてあるので、原因別のカウンタ（タイムアウト、チェックサム、デシンク、エコー、バス、再送）も同じ行に続く。

## 対応ボード

`boards/fibril_robomaster_miniv1.overlay` が実機で通った配線設定である。
USART3 を 2 Mbps で開き、DMA を張り、`amt21` バスノードと 14 ビットのエンコーダ 1 台を宣言する。

```shell
west build -b fibril_robomaster_miniv1 samples/drivers/amt21
west flash
```

他のボードに載せるときは、同じ内容の overlay を `boards/<ボード名>.overlay` として置く。
プロパティの意味と選び方は [doc/drivers/amt21.md](../../../doc/drivers/amt21.md) にある。

## クロックを疑うとき

`debug/hsi_fallback.overlay` は HSE 水晶を無視して HSI16 から SoC を回す切り分け用の overlay である。
PLL の分周は SYSCLK 160 MHz を保つように選んであるので、周辺クロックは変わらない。
コンソールの USART のカーネルクロックも HSI16 に固定するため、ボーレートの分周比が PLL の実際の落ち着き先に依存しなくなる。

```shell
west build -b fibril_robomaster_miniv1 samples/drivers/amt21 -- \
  -DEXTRA_DTC_OVERLAY_FILE=debug/hsi_fallback.overlay
```

この overlay で起動するのに board 既定では起動しない場合、あるいは文字化けがこれで直る場合は、24 MHz の HSE 回路（水晶、負荷容量、レイアウト）を疑う。

## 実行中の操作

`CONFIG_ENCODER_AMT21_SHELL` を有効にしてあるので、シェルから直接エンコーダを叩ける。

```
amt21 list
amt21 read <dev>
amt21 stats <dev>
amt21 errlog <dev>
amt21 bus-stats <bus>
```

時間が経つにつれて応答が落ちるようになる場合は `amt21 errlog` を見る。
実際に届いたバイト列が残るので、ノイズなのかフレームの衝突なのかを切り分けられる。

# fibril_can のルータを Zephyr の CAN デバイスとして使う

fibril_can の CAN ルータを `compatible = "fibril,can-router"` の Zephyr CAN コントローラとして露出させ、`can_send()` から駆動するサンプル。

```text
本番の構成:

PC --USB--> gs_usb --> fibril,can-router (uplink)
                            ├── downlink 0
                            └── downlink 1
```

同じ device に CANnectivity の `gs_usb` ノードをぶら下げると、ルータの uplink が USB になる。
PC が gs_usb 経由で送ったフレームはこの device に対する `can_send()` として届き、ルータがマスター方向へ転送するフレームは `can_add_rx_filter()` のコールバックに配送される。

ホストをつながずに動かした場合は `main()` がマスターの役を演じる。
ブロードキャストの DISCOVER は学習なしで全 downlink に展開されるので、電源投入直後から `fwd_up_to_down` のカウンタが動く。
これにより `can_send()` から ingress msgq、ルータスレッド、`fcan_router_on_rx`、downlink の `can_send()` までの経路がつながっていることを確認できる。

1 秒ごとに `fcan_router_get_diag()` のカウンタ（未知の標準 ID、未知のノード、送信失敗、ingress の取りこぼし）を出力する。

## 実行

`native_sim` 用の overlay は downlink に `zephyr,can-loopback` を 2 つ使うので、追加のハードウェアなしで動く。

```shell
west build -b native_sim samples/drivers/can_router
./build/zephyr/zephyr.exe
```

実機で動かすときは、`boards/<ボード名>.overlay` に `fibril,can-router` ノードと downlink となる CAN コントローラを書く。

ルータの設計と、なぜ Zephyr の CAN device として露出させているのかは、fibril_can 側の `docs/09-router.md` と `docs/adr/0009-router-as-zephyr-can-device.md` にある。

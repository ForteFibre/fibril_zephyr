# DJI RoboMaster モータ（`dji,robomaster`）

DJI の RoboMaster モータコントローラ C610 と C620 を CAN 経由で駆動するドライバ。
モータ 1 台が 1 つの device として現れ、電流指令と feedback スナップショットを扱う。

共通のモータインタフェースは [include/drivers/motor.h](../../include/drivers/motor.h) にある。
このドライバに固有の公開 API はない。

## デバイスの構成

devicetree のノードは 2 段になる。

**トランスポートノード**（`dji,robomaster`）が CAN コントローラを占有し、送信タイマーと受信フィルタを持つ。
**モータノード**（`dji,robomaster-motor`）はその子で、モータ 1 台に対応する。

```devicetree
robomaster_controller: robomaster-controller {
        compatible = "dji,robomaster";
        #address-cells = <1>;
        #size-cells = <0>;
        cans = <&fdcan2 &fdcan3>;
        feedback-timeout-ms = <100>;

        motor_front_left: motor@1 {
                compatible = "dji,robomaster-motor";
                reg = <1>;
                model = "c620";
                max-current = <10000>;
        };
};
```

トランスポートの `cans` には CAN コントローラを 1 個以上並べる。
上限は 4 個で、超えるとビルドが `BUILD_ASSERT` で止まる。
モータ ID は 1 つのトランスポート内で重複してはならない。

モータノードの `reg` がモータ ID で、範囲は 1 から 8 である。
`max-current` を省略すると 10000 になる。

`model` と `gear-ratio` はどちらも config に保持されるだけで、ドライバは読まない。
C610 と C620 は同じ扱いになる。
上位の制御側がメタデータとして参照するためのプロパティである。

## Kconfig

`CONFIG_MOTOR` でモータドライバクラス全体を有効にする。
`CONFIG_MOTOR_DJI_ROBOMASTER` は `CONFIG_CAN` と `dji,robomaster` ノードの存在に依存し、条件が揃えば既定で有効になる。
初期化優先度は `CONFIG_MOTOR_INIT_PRIORITY`（既定 85、`POST_KERNEL`）で決まる。

## 送信

トランスポートは 1 ms 周期のタイマーでワークをキューに積み、そのワークが指令フレームを送る。

モータ ID はグループ 2 つに分かれ、それぞれ 1 つの CAN フレームに 4 台ぶんの電流指令が入る。

| モータ ID | CAN ID | フレーム内の位置 |
| --- | --- | --- |
| 1 から 4 | `0x200` | `(ID - 1)` 番目の 16 ビット |
| 5 から 8 | `0x1FF` | `(ID - 5)` 番目の 16 ビット |

各スロットは符号付き 16 ビットのビッグエンディアンである。
そのグループに属するモータが 1 台もそのバスにいなければ、フレームは送らない。

`can_send()` は `K_NO_WAIT` で呼び、戻り値を見ない。
バスが混んでいて送れなかったフレームは、次のタイマー刻みで送り直される。

## CAN バスの自動検出

`cans` に複数のコントローラを並べたとき、どのモータがどのバスにいるかは devicetree に書かない。
ドライバが feedback フレームの到着したバスから学習する。

このため、**モータが一度も feedback を返していないあいだ、そのモータには電流指令が送られない**。
バスが未知の状態では、送信ワークがそのモータをどのフレームにも入れないためである。
`motor_set_output()` で与えた値は保持され、バスが判明した時点から送信に乗る。

一度バスが確定した後は、同じモータ ID の feedback が別のバスから来ても無視する。

## 受信

受信フィルタは ID `0x200`、マスク `0x7F0` で登録する。
`0x201` から `0x208` のフレームをモータ ID 1 から 8 に対応させ、それ以外は捨てる。
7 バイト未満のフレームも捨てる。

| バイト | 内容 |
| --- | --- |
| 0 から 1 | 機械角（0 から 8191、ビッグエンディアン） |
| 2 から 3 | 速度（符号付き 16 ビット） |
| 4 から 5 | 電流（符号付き 16 ビット） |
| 6 | 温度 |

値はモータコントローラの生の単位のままスナップショットに入る。
ドライバはスケーリングをしない。
`max-current` も同じ生の単位で解釈される。

`struct motor_feedback` の `valid_mask` には、電流、速度、位置、機械角、温度のすべてが立つ。

### 位置の積算

`orientation` は受信した機械角そのままで、1 回転で 0 から 8191 を巡る。
`position` はその差分を積算した値で、8192 の折り返しを跨ぐときに補正する（差分が 4096 を超えたら 8192 を引き、-4096 を下回ったら足す）。

最初の feedback フレームでは、`position` の初期値として機械角の生の値をそのまま入れる。
0 から始まるわけではないので、起動後の変位を見たい場合は最初の読み値を基準として引く。

## API の振る舞い

`motor_enable()` と `motor_disable()` はフラグを立てるだけで、CAN には何も送らない。
無効なモータのスロットには、フレームを組むときに明示的に 0 を書き込む。
指令を止めるのではなく、0 電流を送り続ける動作になる。

`motor_set_output()` が受け付けるのは `MOTOR_OUTPUT_MODE_CURRENT` と、その別名である `MOTOR_OUTPUT_MODE_TORQUE` だけである。
速度モードと電圧モードには `-ENOTSUP` を返す。
値は `±max-current` に飽和させてから保持する。

`motor_get_feedback()` は戻り値に関わらずスナップショットを書き込む。

| 戻り値 | 意味 |
| --- | --- |
| `0` | 有効かつ新鮮な feedback |
| `-ENODATA` | 一度も feedback を受け取っていない |
| `-EAGAIN` | `feedback-timeout-ms` より古い。`stale` が立つ |
| `-EINVAL` | 引数が NULL |

`-EAGAIN` のときも直前の値は読めるので、1 回の取りこぼしで制御ループを止める必要はない。

## トランシーバに電源が入っていない場合

初期化時の `can_start()` の失敗は致命的として扱わない。

トランシーバに電源が来ていないバスは RX がドミナントに張り付き、コントローラが初期化モードから出られない。
これはデバッグプローブだけで基板に給電しているときの通常の状態なので、ここで初期化を失敗させると他のバスとモータ device まで使えなくなる。

代わりに警告をログに出して先へ進み、送信ワークから起動を再試行する。
再試行は 500 ms に 1 回までに絞ってある。
初期化モードで固まったコントローラに対する `can_start()` はハードウェアのタイムアウトを待つため、毎周期呼ぶと 1 ms の送信周期を守れない。

モータ電源が入った時点でバスは自然に立ち上がる。

## テスト

振る舞いは `native_sim` 上の ztest で固定してある。

- [tests/drivers/motor/robomaster](../../tests/drivers/motor/robomaster/src/main.c) — 指令のグループ振り分け、無効化でスロットが 0 になること、未対応モードの拒否、`-ENODATA` と stale、feedback のデコードと折り返し、モータ ID とバスによる振り分け、バス自動検出と保持していた指令の反映
- [tests/drivers/motor/robomaster_start_retry](../../tests/drivers/motor/robomaster_start_retry/src/main.c) — 起動できないバスがあってもモータ device を失わず、後から復帰すること

```shell
west twister -T tests/drivers/motor --integration
```

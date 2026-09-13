# RobStride アクチュエータ（`robstride,bus`）

RobStride のアクチュエータを CAN 経由で駆動するドライバ。
アクチュエータ 1 台が 1 つの device として現れる。

指令はこのドライバに固有の公開 API [include/drivers/motor/robstride.h](../../include/drivers/motor/robstride.h) にある。
共通のモータインタフェース [include/drivers/motor.h](../../include/drivers/motor.h) からは、有効化と無効化と feedback スナップショットだけが使える。
この分け方の理由は [ADR 0004](../adr/0004-robstride-control-api-layering.md) にある。

## バスの要件

RobStride のプロトコルは 29 bit 拡張 ID の classic CAN で、データ長は 8 byte、ビットレートは 1 Mbps である。
**CAN FD とは同じコントローラを共有できない。**
fibril_can と同じ基板に載せる場合は、モータ用と fibril_can 用に別の FDCAN コントローラを割り当てる。

## デバイスの構成

devicetree のノードは 2 段になる。

**バスノード**（`robstride,bus`）が CAN コントローラを占有し、送信タイマーと受信フィルタを持つ。
**モータノード**（`robstride,motor`）はその子で、アクチュエータ 1 台に対応する。

```devicetree
robstride0: robstride {
        compatible = "robstride,bus";
        #address-cells = <1>;
        #size-cells = <0>;
        cans = <&fdcan2>;
        master-can-id = <0xfd>;
        feedback-timeout-ms = <100>;

        joint_knee: motor@7f {
                compatible = "robstride,motor";
                reg = <0x7f>;
                model = "rs00";
                max-current-ma = <8000>;
        };
};
```

`cans` には CAN コントローラを 1 個以上並べる。
上限は 4 個で、超えるとビルドが `BUILD_ASSERT` で止まる。

`master-can-id` はホスト側の識別子で、すべてのフレームの ID 下位 8 bit に載る。
受信フィルタがこの値を鍵にするため、**同じバス上のどのモータの `reg` とも重ならない値にする**。
重なっていると初期化が `-EINVAL` で失敗する。

モータノードの `reg` がモータの CAN ID で、範囲は 1 から 127 である。

`model` は速度、トルク、電流の範囲を決める。
指令の符号化と feedback の復号の両方がこの範囲を使うので、**値を間違えると失敗せずに全部の物理量がずれる**。
実機で確かめた RS00 と RS05 だけを受け付ける。

| `model` | 速度 | トルク | 電流 |
| --- | --- | --- | --- |
| `rs00` | ±33 rad/s | ±14 Nm | ±15.5 A |
| `rs05` | ±50 rad/s | ±5.5 Nm | ±11 A |

`max-current-ma`、`max-velocity-mrad-s`、`max-torque-mnm` はモータに書き込む上限で、省略すると機種の上限になる。

回転方向と減速比のプロパティは意図的に持たせていない。
どちらも制御層が counts を物理量に直すときのスケールに畳まれるもので、スケールをドライバと制御層で分け持つと積算位置が追いにくくなる。

## Kconfig

`CONFIG_MOTOR` でモータドライバクラス全体を有効にする。
`CONFIG_MOTOR_ROBSTRIDE` は `CONFIG_CAN` と `robstride,bus` ノードの存在に依存し、条件が揃えば既定で有効になる。

| Kconfig | 既定 | 内容 |
| --- | --- | --- |
| `CONFIG_MOTOR_ROBSTRIDE_MAX_MOTORS` | 8 | 1 つのバスに登録できるモータ数 |
| `CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS` | 5 | 指令の送信間隔。feedback の周期でもある |
| `CONFIG_MOTOR_ROBSTRIDE_PARAM_TIMEOUT_MS` | 50 | `robstride_get_parameter()` の待ち時間 |

ドライバは `float` を使う。
ゲインと上限のパラメータが IEEE754 を payload にそのまま載せるためで、STM32G4 では `CONFIG_FPU` を有効にしておく。

## 送信

バスは `CONFIG_MOTOR_ROBSTRIDE_TX_INTERVAL_MS` 周期のタイマーでワークをキューに積み、そのワークが指令フレームを送る。
RobStride には一斉送信のフレームがないので、ワークは登録済みのモータを順に回して 1 台ずつ送る。
モータ 1 台ごとに 1 フレーム必要なので、**バスの負荷は台数に比例する**。

`robstride_set_*()` は指令を保存するだけで、CAN の送信を待たない。
制御ループから呼んでよい。

### ハンドシェイク

有効化したモータには、次の段を 1 周期に 1 段ずつ進める。

1. 停止フレームと `run_mode` の書き込み（モータは停止中しか `run_mode` を受け付けない）
2. 電流、速度、トルクの上限
3. ループゲイン（`robstride_set_gains()` が呼ばれた場合だけ）
4. 有効化フレーム
5. 以降は毎周期、現在のモードの指令フレーム

既定ではゲインを書かない。
書かないかぎりモータは自分の不揮発メモリに持つゲインで動くので、ゼロで上書きして動かなくなることがない。

この段は次のときに最初からやり直す。

- `motor_enable()` を呼んだとき
- モードが変わったとき
- 故障フレームを受け取ったとき
- feedback の run state が running から外れたとき

いずれもモータ側が設定を落としている可能性がある場面で、こちらのキャッシュを信用しない。

### 無効なモータ

無効なモータには指令を送らない。
RobStride の feedback はコマンドへの返信として返るので、**無効なあいだは feedback が更新されない**。
存在確認だけは 500 ms 間隔で ID 取得フレームを送って行う。

`motor_disable()` は停止フレームを次の送信周期を待たずにその場で送る。

## 受信

受信フィルタは拡張 ID 1 本で、ID の下位 8 bit が `master-can-id` に一致するフレームを拾う。
モータからホストへ返るフレームはどの種別もこの位置にホストの識別子を置くので、feedback、故障通知、パラメータ応答、ID 応答がまとめて届く。

どの CAN コントローラにモータがぶら下がっているかは、最初の応答で分かる。
それまでは指令を全コントローラに送る。

### 位置の積算

線路上の位置は ±4π で折り返す 16 bit のコードである。
ドライバはこの折り返しを解いて積算し、`motor_feedback.position` に入れる。

**単位は線路上のコードそのもので、8π rad あたり 65535 カウントである。**
機種によらず同じスケールなので、物理量へは次の式で直せる。

```
rad = counts * (8 * PI / 65535) - 4 * PI
```

積算の起点は最初に受け取った絶対位置なので、電源投入時の位置が保たれる。

## API の振る舞い

`motor_set_output()` は**どのモードでも `-ENOTSUP` を返す**。
RobStride の指令は機種で範囲が変わる SI 量で、`enum motor_output_mode` に載せられる機種非依存の単位がない。
指令は `robstride.h` を使う。

`motor_get_feedback()` が `valid_mask` に立てるのは `MOTOR_FEEDBACK_POSITION` と `MOTOR_FEEDBACK_TEMPERATURE` だけである。
速度、トルク、単回転位置は `robstride_get_feedback()` から SI で読む。
温度は `motor_feedback` では ℃ の整数に丸まる（線路上は 0.1 ℃ 刻み）。

どちらの feedback も、モータが一度も応答していなければ `-ENODATA`、`feedback-timeout-ms` を超えていれば `stale` を立てて `-EAGAIN` を返す。

`robstride_set_position()` などの指令はモードの選択を兼ねる。
モードが変わると上のハンドシェイクをやり直すので、切り替えには数周期かかる。

`robstride_get_parameter()` はモータの応答を待ってブロックする。
制御ループや割り込みから呼んではならない。

## テスト

```shell
west twister -T tests/drivers/motor/robstride --integration
```

`zephyr,fake-can` を使い、送信フレームを捕まえて内容を検証し、受信コールバックに任意のフレームを注入する。

`can_fake` は ztest rule で各テストの前に FFF の fake を reset する。
`custom_fake` を before hook で入れ直さないと、completion callback を渡さない `can_send()` が CAN API の用意した semaphore を `K_FOREVER` で待ち続けて止まる。

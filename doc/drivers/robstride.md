# RobStride アクチュエータ（`robstride,bus`）

RobStride のアクチュエータを CAN 経由で駆動するドライバ。
アクチュエータ 1 台が 1 つの device として現れる。

指令はこのドライバに固有の公開 API [include/drivers/motor/robstride.h](../../include/drivers/motor/robstride.h) にある。
共通のモータインタフェース [include/drivers/motor.h](../../include/drivers/motor.h) からは、有効化と無効化と feedback スナップショットだけが使える。
この分け方の理由は [ADR 0004](../adr/0004-robstride-control-api-layering.md) にある。

このドライバを fibril_can のバスに出すには `RobstrideMotor` ブロック型を使う（[doc/apps.md](../apps.md)）。
ギア比、回転方向、指令途絶時のサーボオフはそちらの責務で、このドライバは持たない。

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
範囲は 0 から 255 で、外れるとビルドが `BUILD_ASSERT` で止まる。
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
現在有効な値は `robstride_get_limits()` で読み戻せる。
機種が公開されていないので、**無次元の指令を電流に直すときの全尺はこの値以外に取りようがない**（[doc/apps.md](../apps.md) の `RobstrideMotor` がそうしている）。
機種の上限を超える値を書いても機種の上限まで下げられる（`robstride_set_limits()` と同じ扱い）。
devicetree の整数は符号なしで C に届くので、負の値は巨大な正の数になる。
そのまま機種の上限に丸めると気づけないため、`INT32_MAX` を超える値はビルドが `BUILD_ASSERT` で止まる。

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
| `CONFIG_MOTOR_ROBSTRIDE_VERIFY` | y | ハンドシェイクで書いた値を読み戻して確かめる |
| `CONFIG_MOTOR_ROBSTRIDE_VERIFY_TIMEOUT_MS` | 20 | 1 回の読み戻しの応答を待つ時間 |
| `CONFIG_MOTOR_ROBSTRIDE_VERIFY_RETRIES` | 3 | 同じ組を書く回数。超えたら設定をやり直す |
| `CONFIG_MOTOR_ROBSTRIDE_RUNNING_TIMEOUT_MS` | 100 | 有効化してから Running の報告を待つ時間 |
| `CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS` | 200 | 設定に失敗してから、やり直すまでの間 |

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
判断の理由は [ADR 0019](../adr/0019-robstride-mode-params-readback.md) にある。

1. 停止フレームと `run_mode` の書き込み（モータは停止中しか `run_mode` を受け付けない）
2. 次の周期に、書いた値の読み戻し
3. 上限とモードごとのパラメータを、最大 5 つずつ書く。それぞれ次の周期に読み戻す
4. 目標値の書き込み（運転制御では飛ばす）
5. 有効化フレーム
6. Running の報告を待つ
7. 以降は毎周期、現在のモードの指令フレーム

**値は、モータが読み戻しで同じ値を返すまで書けたことにしない。**
classic CAN の ACK はバス上のどのノードでも返せるので、送信が成功しても、宛先のモータが受け取ったとは限らないからである。
応答の値は、許容誤差（1e-3。`spd_ki`、`cur_kp`、`cur_ki` は 1e-4。`run_mode` は完全一致）の中で一致すればよい。
`CONFIG_MOTOR_ROBSTRIDE_VERIFY_TIMEOUT_MS` のうちに一致しなければ、その組を書き直す。
`CONFIG_MOTOR_ROBSTRIDE_VERIFY_RETRIES` 回続けて確かめられなければ、設定に失敗したことにする。

書き込みと読み出しは同じ周期に積まない。
コントローラは ID の小さいフレームを先に送るので、同じ周期に積むと GET_PARAM（0x11）が SET_PARAM（0x12）を追い越し、古い値を読む。
目標値と有効化（0x03）を別の周期にするのも同じ理由である。

一致しない応答が来ても、すぐには書き直さない。
応答にはインデックスしか無く、前の読み出しへの遅れた応答と見分けられないからで、期限まで正しい応答を待つ。

有効化のあと、**Running で error_code が 0 の feedback が届くまで、指令を送らない。**
`CONFIG_MOTOR_ROBSTRIDE_RUNNING_TIMEOUT_MS` のうちに届かなければ、設定に失敗したことにする。
キャリブレーション中のモータは待ち続ける。

設定に失敗したら、`CONFIG_MOTOR_ROBSTRIDE_CONFIGURE_BACKOFF_MS` おいてから 1 からやり直す。
失敗のたびに `robstride_feedback` の `configure_failures` が 1 増える（起動からの通算で、戻らない）。
警告のログは、失敗が始まったときに 1 回だけ出す。電源の入っていないモータが 0.3 秒ごとに警告を出し続けないようにするためである。
`motor_enable()` を呼ぶと、この待ちを解いてすぐ始める。

全部済んだモータは、`robstride_feedback` の `configured` が真になる。

`CONFIG_MOTOR_ROBSTRIDE_VERIFY=n` にすると読み戻しを省き、送信が成功した時点で書けたことにする。
Running を待つのは、この設定によらない。

#### 書くパラメータ

| モード | `robstride_mode_params` から書くもの |
| --- | --- |
| すべて | （上限 `limit_cur`、`limit_spd`、`limit_torque` は値を渡さなくても書く） |
| `ROBSTRIDE_MODE_POSITION`（PP） | `vel_max` ← `max_velocity`、`acc_set` ← `acceleration`、`loc_kp`、`spd_kp`、`spd_ki`、`cur_kp`、`cur_ki` |
| `ROBSTRIDE_MODE_POSITION_CSP` | `limit_spd` ← `max_velocity`、`loc_kp`、`spd_kp`、`spd_ki`、`cur_kp`、`cur_ki` |
| `ROBSTRIDE_MODE_VELOCITY` | `acc_rad` ← `acceleration`、`spd_kp`、`spd_ki`、`cur_kp`、`cur_ki` |
| `ROBSTRIDE_MODE_CURRENT`、`ROBSTRIDE_MODE_OPERATION` | `cur_kp`、`cur_ki` |

CSP の `limit_spd` は `max_velocity` と上限（`robstride_set_limits()` か devicetree）の小さいほうになる。
上限を広げることはない。

`robstride_set_mode_params()` で値を渡していないモードでは、上限のほかは何も書かない。
モータは自分の不揮発メモリに持つ値で動くので、ゼロで上書きして動かなくなることがない。

#### やり直すとき

上の段は次のときに 1 からやり直す。

- `motor_enable()` を呼んだとき
- モードが変わったとき
- 故障フレームを受け取ったとき
- 停止していた CAN コントローラが起動し、そのモータがまだどのバスにいるか分かっていないとき
- 設定に失敗したとき

いずれもモータ側が設定を落としているか、こちらの書き込みが届いていない可能性がある場面で、こちらのキャッシュを信用しない。

feedback の run state が running から外れたときは、有効化の前（目標値の書き込み）からやり直す。
パラメータは書き直さない。

動いている途中で、いまのモードの値を `robstride_set_mode_params()` で、または上限を `robstride_set_limits()` で変えたときは、変わった組だけを書いて読み戻す。
止めず、有効化もやり直さない。
書いて確かめるあいだの 2〜4 周期だけ、指令が止まる。

#### 送信の失敗

**段は送信が成功したときにだけ進む。**
コントローラが受け取らなかったフレームの段は、その場で取り消して次の周期に組み立て直す。
受け取られたのに線路へ出なかった場合は完了コールバックが報告し、次の周期の先頭で同じように取り消す。
指令フレームは毎周期送り直すので取り消すものがなく、存在確認は自分の 500 ms 周期で戻ってくる。
読み出しのフレームが失われたときは、読み戻しの期限が来て書き直すので、取り消すものがない。

完了は数周期あとに届くことがあり、そのときには別の段が進んでいる。
そこで**どの段のフレームだったかを送信時に確定させ**、完了コールバックはその段に対して失敗を記録する。
パラメータの書き込みは、さらにどのパラメータだったかまで確定させる。
「いま進めている段」を取り消す作りにすると、届かなかった段が済んだことにされたまま次へ進む。

読み戻しで確かめ終えたパラメータについての失敗の報告は、無視する。
モータが値を持っていることは読み戻しが示しており、失敗したのはそれより前の書き込みだからである。
これを無視しないと、`run_mode` の遅れた失敗のために、動いているモータを止めて書き直すことになる。

### 送信は完了を待たない

`can_send()` に完了コールバックを渡さないと、**フレームが線路に出るまで `K_FOREVER` で待つ**（`zephyr/drivers/can/can_common.c` の `z_impl_can_send()`）。
`K_NO_WAIT` が縛るのは送信メールボックスの空き待ちだけである。
誰も ACK を返さないバスではこれが送信ワークを止め、システムワークキュー全体を巻き込む。
`motor_disable()` やパラメータ API のように直接呼ぶ経路では、呼び出し元のスレッドが止まる。

そのためドライバは**常に完了コールバックを渡す**。
送信 API の返り値はキューに入ったかどうかだけを表し、線路に出たかどうかはコールバックが後から報告する。
同じ理由で fibril_can の hub も NULL を禁じている。

### CAN コントローラが起動できないとき

CAN トランシーバの給電がマイコンと別系統だと、モータ電源が入るまでトランシーバは RX をドミナントに張り続け、コントローラは初期化モードから出られない。
デバッグプローブだけで基板を動かしている間はこれが通常の状態なので、`can_start()` の失敗は致命的にしない。
バスノードの初期化は続け、送信ワークから 500 ms 間隔で `can_start()` をやり直す。

起動していないコントローラには送信しない。
そのためモータが 1 本のバスにしかいない構成では、電源が入るまでハンドシェイクの段が 1 つも進まず、入った時点で最初から流れる。

コントローラが複数あると、まだ応答のないモータのフレームは全コントローラに送られる。
1 本でも起動していれば送信は成功と数えられるため、**あとから起動したコントローラの先にいたモータはハンドシェイクを取りこぼしている**。
そこでコントローラが起動したら、どのバスにいるか分かっていないモータの段をやり直す。
応答のあったモータは所属バスが分かっており、そのバスは起動済みなので対象にしない。

### 無効なモータ

無効なモータには指令を送らない。
RobStride の feedback はコマンドへの返信として返るので、**無効なあいだは feedback が更新されない**。
存在確認だけは 500 ms 間隔で ID 取得フレームを送って行う。

`motor_disable()` は停止フレームを次の送信周期を待たずにその場で送る。
**コントローラが受け取らなければそのエラーを返す。**
指令は止まるが、停止フレームが届くまでモータは最後の指令を保持し続けるので、呼び出し側が「止めたつもり」にならないようにする。
返り値がエラーでも送信ワークが毎周期この停止フレームを送り直すので、バスが戻れば止まる。

受け取られたあとに線路へ出なかった場合は、返り値では分からない（上の「送信は完了を待たない」）。
この場合は完了コールバックが失敗を記録し、送信ワークが停止フレームを送り直す。

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
`robstride_set_zero()` が通ったあとも同じで、次の feedback が新しい原点から積算をやり直す。

## API の振る舞い

`motor_set_output()` は**どのモードでも `-ENOTSUP` を返す**。
RobStride の指令は機種で範囲が変わる SI 量で、`enum motor_output_mode` に載せられる機種非依存の単位がない。
指令は `robstride.h` を使う。

`motor_get_feedback()` が `valid_mask` に立てるのは `MOTOR_FEEDBACK_POSITION` と `MOTOR_FEEDBACK_TEMPERATURE` だけである。
速度、トルク、単回転位置は `robstride_get_feedback()` から SI で読む。
温度は `motor_feedback` では ℃ の整数に丸まる（線路上は 0.1 ℃ 刻み）。

**`valid_mask` が立つのは feedback フレームを 1 度でも受け取ってからである。**
存在確認にしか答えていないモータは `online` だが測定値を 1 つも持たないので、`valid_mask` は 0 のままにする。

`online` と測定値の鮮度は別に扱う。
`online` はモータからのどの種別のフレームでも立つが、**`timestamp_ms` と `stale` は feedback フレームの時刻だけで決まる**。
存在確認やパラメータ応答が返っても、古い位置と速度が新しくなったことにはならない。

どちらの feedback も返り値は 3 つに分かれる。

| 状態 | 返り値 |
| --- | --- |
| どの種別のフレームも返してこない | `-ENODATA` |
| 応答はあるが feedback フレームを 1 度も受け取っていない | `stale` を立てて `-EAGAIN` |
| feedback はあるが `feedback-timeout-ms` より古い | `stale` を立てて `-EAGAIN` |

2 行目には、存在確認にしか答えていないモータと、`robstride_set_zero()` で積算をやり直した直後のモータが入る。
どちらも直前のフレームは新しいが、読める測定値を持たない。
無効なモータは指令を受けないので feedback が止まり、受け取り済みだったものもやがて 3 行目に落ちる。

`robstride_set_position()` などの指令はモードの選択を兼ねる。
モードが変わると上のハンドシェイクをやり直すので、切り替えには数周期かかる。

`robstride_get_parameter()` はモータの応答を待ってブロックする。
制御ループや割り込みから呼んではならない。
読み出しの枠はモータごとに 1 つで、呼び出しから戻るまで塞がる。
応答が届いた時点ではなく**戻るまで**なので、その間の別の読み出しは `-EBUSY` になる。
先に枠を空けると、まだ結果を読んでいない呼び出し元の答えを次の読み出しが上書きできてしまう。

**タイムアウトした読み出しの応答は、同じインデックスを次に読んだときの答えになりうる。**
RobStride の応答はインデックスしか持たず、どの要求への返信かを示すものが無いためで、ドライバ側では区別できない。
タイムアウトのあとに同じインデックスを読み直すと、1 つ前の要求に対する値が返ることがある。
設定値の読み出しでは同じ値なので実害は出にくいが、変化する量を読むときは意識する。
1 回読み捨てる手もあるが、応答が本当に失われていた場合に次の読み出しが自分の応答を捨てて余計にタイムアウトするので採っていない。

ハンドシェイクの読み戻しと同じインデックスを読んでいるときは、1 つの応答が両方の答えになる。
どちらもモータの現在値を知りたいだけなので、取り合わない。

`robstride_clear_faults()` はモータ自身の故障ラッチを解く。
ハンドシェイクのどの段もこれを解かないので、トリップしたモータを回すにはこれを呼ぶ。
**解除のフレームは停止フレームなので、出力も同時に落ちる。**
回りながら故障を確認する手段ではない。
原因が残っていれば次に回そうとした時点でまたトリップする。

`robstride_set_parameter()` は整数 payload のインデックスに限り、値が `uint32_t` で表せることを確かめる。
負値、`uint32_t` を超える値、NaN は `-EINVAL` で、フレームは送らない。
線路上に載せようがない値を変換すると未定義動作になる。

## テスト

```shell
west twister -T tests/drivers/motor/robstride --integration
```

`zephyr,fake-can` を使い、送信フレームを捕まえて内容を検証し、受信コールバックに任意のフレームを注入する。

`can_fake` は ztest rule で各テストの前に FFF の fake を reset する。
`custom_fake` を before hook で入れ直さないと、completion callback を渡さない `can_send()` が CAN API の用意した semaphore を `K_FOREVER` で待ち続けて止まる。

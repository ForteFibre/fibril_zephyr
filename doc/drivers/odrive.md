# ODrive モータコントローラ（`odrive,bus`）

ODrive を CANSimple で駆動するドライバ。
軸 1 つが 1 つの device として現れる。

対象は **ODrive Pro / S1 / Micro のファームウェア 0.6 系**である。
ODrive 3.x はメッセージ集合が違うので対応しない。

指令はこのドライバに固有の公開 API [include/drivers/motor/odrive.h](../../include/drivers/motor/odrive.h) にある。
共通のモータインタフェース [include/drivers/motor.h](../../include/drivers/motor.h) からは、有効化と無効化と feedback スナップショットだけが使える。
この分け方の理由は [ADR 0006](../adr/0006-odrive-control-api-layering.md) にある。

## バスの要件

CANSimple は **11 bit 標準 ID の classic CAN** で、データ長は 8 byte である。
**CAN FD とは同じコントローラを共有できない。**
fibril_can と同じ基板に載せる場合は、ODrive 用と fibril_can 用に別の FDCAN コントローラを割り当てる。

`fibril_robomaster_miniv4`（FDCAN1 / 2 / 3）と `fibril_rc26_mainair_v01`（FDCAN2 / 3）はこの条件を満たす。
`fibril_robomaster_miniv1` と `fibril_robomaster_miniv3` は FDCAN1 のみ、
`fibril_canmotor_tourobo2023` は MCP2517FD が 1 本なので、fibril_can とは併用できない。

## ODrive 側に必要な設定

**このドライバは ODrive の設定を書きにいかない。**
ODrive は設定を自分の不揮発メモリに持ち、odrivetool で調整して `save_configuration()` で保存する運用が前提である。
有効化のたびにファームウェアが上書きすると、odrivetool で見える値とどちらが効いているのか追えなくなる。

そのぶん、**次の設定が正しくないとドライバは黙って期待どおりに動かない。**

```python
# 軸の CAN ノード ID。63 は未設定を表す予約値なので、0 から 62 を割り当てる。
odrv0.axis0.config.can.node_id = 5

# 周期メッセージの間隔。既定で有効なのは heartbeat と encoder estimates だけで、
# 残りは 0（無効）である。有効にしない測定値は valid_mask が立たない。
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv0.axis0.config.can.encoder_msg_rate_ms = 10
odrv0.axis0.config.can.iq_msg_rate_ms = 20          # odrive_feedback の iq_*
odrv0.axis0.config.can.temperature_msg_rate_ms = 200 # 温度（クラスの温度もこれ）
odrv0.axis0.config.can.torques_msg_rate_ms = 20      # torque_target / torque_estimate
odrv0.axis0.config.can.bus_voltage_msg_rate_ms = 200 # bus_voltage / bus_current
odrv0.axis0.config.can.error_msg_rate_ms = 200       # disarm_reason

odrv0.can.config.baud_rate = 500000
odrv0.save_configuration()
```

次の 3 つは**既定のままにする**。
ドライバがこれらの既定値を前提に符号化・復号している。

| 設定 | 既定 | 変えると |
| --- | --- | --- |
| `input_vel_scale` | 0.001 | `odrive_set_position()` の速度前置補償がずれる |
| `input_torque_scale` | 0.001 | 同じくトルク前置補償がずれる |
| `circular_setpoints` | 無効 | 位置が折り返し、`motor_feedback.position` が連続でなくなる |

## デバイスの構成

devicetree のノードは 2 段になる。

**バスノード**（`odrive,bus`）が CAN コントローラを占有し、送信タイマーと受信フィルタを持つ。
**軸ノード**（`odrive,axis`）はその子で、ODrive 1 台に対応する。

```devicetree
odrive0: odrive {
        compatible = "odrive,bus";
        #address-cells = <1>;
        #size-cells = <0>;
        can = <&fdcan2>;
        heartbeat-timeout-ms = <300>;
        estimate-timeout-ms = <100>;

        wheel_left: axis@5 {
                compatible = "odrive,axis";
                reg = <5>;
                has-motor-thermistor;
        };
};
```

`can` は **1 個**の CAN コントローラである。
ODrive は `node_id` で宛先が決まりホスト側の識別子を持たないので、コントローラをまたいで軸を探す必要がない。
コントローラを 2 本使うなら `odrive,bus` を 2 つ書く。

`reg` が軸の CAN ノード ID で、範囲は 0 から 62 である。
63 は未設定の ODrive が名乗る予約値なので、ビルド時に `BUILD_ASSERT` で弾く。

`has-motor-thermistor` は、そのモータにサーミスタが**配線されていて ODrive 側で設定済み**のときだけ書く。
設定されていない ODrive はモータ温度に 0 を返し、0 ℃ は「冷えている」と読めてしまうので、
このプロパティがない軸では `MOTOR_FEEDBACK_TEMPERATURE` を立てない。

上限とゲインのプロパティは意図的に持たせていない。
ODrive 側の設定が正本であり、実行時に変えたいときだけ `odrive_set_limits()` と `odrive_set_gains()` を使う。

## Kconfig

`CONFIG_MOTOR` でモータドライバクラス全体を有効にする。
`CONFIG_MOTOR_ODRIVE` は `CONFIG_CAN` と `odrive,bus` ノードの存在に依存し、条件が揃えば既定で有効になる。

| Kconfig | 既定 | 内容 |
| --- | --- | --- |
| `CONFIG_MOTOR_ODRIVE_MAX_AXES` | 4 | バス 1 本に登録できる軸の数。軸ごとに受信フィルタを 1 本張るので、コントローラのフィルタ本数にも縛られる |
| `CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS` | 10 | 指令の送信間隔 |
| `CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS` | 200 | closed loop の確認が来ないときに要求し直す間隔 |
| `CONFIG_MOTOR_ODRIVE_WORKQ_STACK_SIZE` | 1024 | コールバックを走らせるスレッドのスタック |
| `CONFIG_MOTOR_ODRIVE_WORKQ_PRIORITY` | 5 | 同スレッドの優先度 |

## 受信

受信フィルタは軸ごとに 1 本で、`id = node_id << 5`、`mask = 0x7E0` である。
ID の上位 6 bit が `node_id`、下位 5 bit が `cmd_id` なので、これ 1 本でその軸のすべてのメッセージが取れる。

**feedback は ODrive の放送を受けるだけで、こちらから取りにいかない。**
RobStride のように指令への返信として返るのではないので、無効な軸でも測定値は更新され続ける。
RTR は使わない。

### 測定値の有無

`odrive_feedback.valid_mask` が、どの測定値が現在有効かを示す。
各ビットは対応する周期メッセージが `estimate-timeout-ms` 以内に届いていれば立つ。
**多くの周期メッセージは ODrive の既定で無効なので、ビットが立たない原因はたいてい通信ではなく設定漏れである。**

### 位置のスケール

`Pos_Estimate` は rev 単位の float32 で、**すでに多回転の連続値**である。
ドライバは折り返しを解かず、倍率を掛けるだけである。

**`motor_feedback.position` の単位は 1 rev = 65536 カウントである。**

```
rev = counts / 65536
```

float32 の刻み幅は値が 2^k のとき 2^(k-23) なので、1/65536 rev の刻みは 128 rev で float32 の刻みに追いつく。
**128 rev まではカウントと `Pos_Estimate` が 1 対 1 に対応し、そこから 2 倍ごとにカウントが 2 段ずつ飛ぶ。**

## 送信

バスは `CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS` 周期のタイマーでワークを積み、そのワークが指令フレームを送る。
`odrive_set_*()` は指令を保存するだけで CAN の送信を待たない。
制御ループから呼んでよい。

タイマーは**有効な軸が 1 つもなくても回り続ける**。
heartbeat のタイムアウト判定も同じタイマーが行うので、有効化する前に軸が生きているかを確かめられる。

### 有効化の手順

`motor_enable()` を呼ぶと、1 周期に 1 段ずつ進む。

1. `Clear_Errors`
2. `Set_Controller_Mode`（現在の制御モードと input mode）
3. `Set_Axis_State(CLOSED_LOOP_CONTROL)`
4. heartbeat の `axis_state` が 8 になるまで、`CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS` ごとに 2 と 3 をやり直す
5. 確認できたら、以降は毎周期、現在のモードの指令フレーム

**4 の再試行では `Clear_Errors` を送らない。**
過電流で落ちた軸をループの中で自動的に復帰させると、原因が取り除かれないまま再投入を繰り返すことになる。

### 故障からの復帰

**有効化中に軸が closed loop から外れたら、ドライバは再投入しない。**
ODrive が自分で disarm するのは故障したときであり、復帰には `Clear_Errors` が要る。
ドライバは指令の送信を止め、自身の状態も無効に落とし、状態コールバックで知らせる。

**heartbeat が `heartbeat-timeout-ms` を超えて途絶えたときも同じ扱いになる。**
heartbeat が来ないということは closed loop にいる確認が取れないということであり、
そのまま有効にしておくと、通信が戻った瞬間にドライバが黙って軸を再投入することになる。
通信断からの復帰も上位が `motor_enable()` で明示的に行う。

復帰させるかどうかは上位が決める。
復帰は `motor_enable()` の 1 回で足りる（上の手順を最初からやり直すので、`odrive_clear_errors()` を先に呼ぶ必要はない）。

```c
static void on_state(const struct device *dev, const struct odrive_feedback *fb, void *user_data)
{
        if (!fb->enabled && (fb->active_errors & RECOVERABLE_MASK)) {
                motor_enable(dev);
        }
}

odrive_set_state_callback(wheel_left, on_state, NULL);
```

### TRAP_TRAJ の制約

`ODRIVE_INPUT_MODE_TRAP_TRAJ` は、**同じ値の `Set_Input_Pos` を受け取るたびに軌道を再計画する**。
毎周期再送すると軌道が振り出しに戻り続け、`Trajectory_Done_Flag` が立たない。
そのためこのモードだけは、目標が変わったときにしか送らない。

**その結果、このモードでは ODrive の watchdog に餌が届かない。**
watchdog を戻すのは `Set_Input_*` と closed loop への遷移だけで、ほかに手段がない。
`ODRIVE_INPUT_MODE_TRAP_TRAJ` と `axis.config.enable_watchdog` は併用できない。

## コールバック

軸ごとに 2 本登録できる。

| 登録 | 呼ばれるとき |
| --- | --- |
| `odrive_set_state_callback()` | heartbeat の `axis_state` / `procedure_result` / `active_errors` が変わったとき、online / offline が変わったとき |
| `odrive_set_feedback_callback()` | `Get_Encoder_Estimates` が届いたとき（既定 100 Hz） |

2 本に分かれているのは、再投入の判断だけが欲しい上位が 100 Hz のコールバックを受けずに済むようにするためである。

**コールバックはバスごとの専用ワークキューから呼ばれる。**
CAN の受信コールバックは割り込みコンテキストで呼ばれるが、そこからは呼ばない。
したがってコールバックの中から `motor_enable()` などこのドライバの API を呼んでよい。
指令の送信はシステムワークキューで走るので、コールバックが長引いても watchdog の餌は遅れない。

**呼び出しは合流することがある。**
ワークが処理される前に次のフレームが届くと、呼び出しは 1 回にまとまり、渡されるのは常に最新のスナップショットである。
イベントの列は保存しない。
ODrive の `active_errors` は `Clear_Errors` まで残るので、途中の heartbeat を 1 つ落としても原因は読める。

## API の振る舞い

`motor_set_output()` は**どのモードでも `-ENOTSUP` を返す**。
ODrive の指令範囲は ODrive 自身の設定で決まり、`enum motor_output_mode` に載せられる機種非依存の単位がない。
指令は `odrive.h` を使う。

`motor_get_feedback()` が `valid_mask` に立てるのは `MOTOR_FEEDBACK_POSITION` と、
`has-motor-thermistor` がある軸での `MOTOR_FEEDBACK_TEMPERATURE` だけである。
速度、電流、トルクは `odrive_get_feedback()` から rev/s、A、Nm で読む。

どちらの feedback も、heartbeat が一度も届いていなければ `-ENODATA`、
encoder estimates が `estimate-timeout-ms` を超えていれば `stale` を立てて `-EAGAIN` を返す。

`odrive_set_position()` などの指令はモードの選択を兼ねる。
モードが変わると `Set_Controller_Mode` を送り直すので、切り替えには 1 周期余計にかかる。

`odrive_request_axis_state()` は、ドライバが軸を握っている間は `-EBUSY` を返す。
キャリブレーションを走らせるには先に `motor_disable()` を呼ぶ。
`motor_disable()` は `Set_Axis_State(IDLE)` をその場で送るので、
ODrive は IDLE の直後に要求した状態を受け取ることになる。
キャリブレーションは IDLE から始める手順なので、これで問題ない。

`odrive_estop()` は `Estop` フレームを送り、軸を `ESTOP_REQUESTED` として disarm させる。
これはエラーとして残るので、`motor_disable()` とは復帰の手間が違う。
次の `motor_enable()` が手順 1 で消す。

## テスト

```shell
west twister -T tests/drivers/motor/odrive --integration
```

`zephyr,fake-can` を使い、送信フレームを捕まえて内容を検証し、受信コールバックに任意のフレームを注入する。

`can_fake` は ztest rule で各テストの前に FFF の fake を reset する。
`custom_fake` を before hook で入れ直さないと、completion callback を渡さない `can_send()` が CAN API の用意した semaphore を `K_FOREVER` で待ち続けて止まる。

コールバックは専用ワークキューで走るので、フレームを注入してから結果を検証するまでに `k_sleep()` を挟む必要がある。
また、armed の状態を保つテストは heartbeat を注入し続ける必要がある。
実機の ODrive が heartbeat を止めないのと同じで、止めるとドライバは軸を落とす。

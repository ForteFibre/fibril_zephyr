---
status: Proposed
date: 2026-09-13
---

# 0006. ODrive の制御 API を push 型 feedback と機種固有ヘッダで載せる

## Status

Proposed

## Context

ODrive のモータコントローラをこのリポジトリのドライバとして載せる見込みが立っている。
ROS 2 側には `ros_odrive` という参照実装があり、CANSimple プロトコルの解釈はそちらで実機に対して確かめられている。
ここで決めるのは、その解釈を Zephyr のドライバクラスにどう載せるかである。

対象は ODrive Pro、S1、Micro のファームウェア 0.6 系である。
ODrive 3.x は CANSimple のメッセージ集合が異なり、参照実装も非対応なので対象外とする。

CANSimple のバスは classic CAN、**11 bit の標準 ID**、データ長 8 byte である。
ID は `node_id << 5 | cmd_id` で、`node_id` が上位 6 bit、`cmd_id` が下位 5 bit を占める。
`node_id` の範囲は 0 から 0x3F で、**0x3F は未設定を表す予約値**である。
未設定の ODrive は 0x3F のまま起動し、周期メッセージを送らない。
既定のビットレートは autobaud（`can.config.baud_rate = 0`）で、実運用では ODrive 側で明示的に設定する。

payload は IEEE754 の単精度浮動小数点数をそのまま載せる。
位置は rev、速度は rev/s、トルクは Nm、電流は A である。
RobStride のような 16 bit への量子化はない。

**feedback の届き方が RobStride と根本的に違う。**
RobStride の feedback は指令フレームへの返信として返るが、ODrive は**自分の周期で放送する**。
周期は ODrive 側の設定 `axis.config.can.*_msg_rate_ms` が決める。
既定で有効なのは heartbeat（100 ms）と encoder estimates（10 ms）の 2 つだけで、
iq、temperature、torques、bus voltage/current、error、powers は**既定で無効**である。
ホスト側は放送を待つだけでよく、値を取りにいくための送信を必要としない。

明示的に値を要求する手段として RTR フレームも使える。
ODrive は同じ ID に対応する payload を載せた data frame で応答する。

状態遷移はホストが明示的に要求する。
`Set_Axis_State`（0x007）に `AXIS_STATE_CLOSED_LOOP_CONTROL`（8）を書くと出力が有効になり、`AXIS_STATE_IDLE`（1）で無効になる。
遷移は非同期で、結果は heartbeat（0x001）の `Axis_State` と `Procedure_Result` に現れる。
エラーを抱えたままでは closed loop に入れないので、`Clear_Errors`（0x018）で先に落とす必要がある。
`Estop`（0x002）は payload なしで、軸を `ESTOP_REQUESTED` として disarm させる。
これはエラーとして残るので、復帰には `Clear_Errors` が要る。

指令は制御モードごとに別のフレームである。
`Set_Controller_Mode`（0x00B）が `control_mode` と `input_mode` を 1 フレームで運び、
`Set_Input_Pos`（0x00C）、`Set_Input_Vel`（0x00D）、`Set_Input_Torque`（0x00E）が目標値を運ぶ。
`axis.config.enable_watchdog` が有効なら、この 3 つのいずれかを周期的に送らないと軸が停止する。

Zephyr の `can_add_rx_filter()` に渡すコールバックは**割り込みコンテキストで呼ばれる**と文書化されている。
`drivers/motor/robstride.c` の受信経路もそれを前提に、spinlock だけを使って受信処理をその場で完結させている。
このリポジトリの公開ヘッダには、いまのところアプリへのコールバックを渡す API がない。

現在の `include/drivers/motor.h` はこの形に合わない。
`motor_set_output()` が受け取るのは `enum motor_output_mode` と `int16_t` の 1 値だけで、
`Set_Input_Pos` が同時に運ぶ位置・速度前置補償・トルク前置補償の 3 値を表せない。
位置は rev 単位の連続値で、回転数に上限がないので `int16_t` に載せる自然なスケールがない。

feedback の側は ADR 0004 が RobStride について論じたのと同じ問題を持つ。
`struct motor_feedback` の数値フィールドには単位の契約がなく、
唯一の実装である `drivers/motor/robomaster.c` が入れているのは生値である。
ODrive が報告するのは SI の物理量である。

位置については RobStride と事情が違う。
`Get_Encoder_Estimates`（0x009）の `Pos_Estimate` は**すでに多回転の連続値**で、折り返しを解く必要がない。
ただし float32 なので、回転数が増えるほど分解能が落ちる。
ODrive 側で `circular_setpoints` を有効にすると `Pos_Estimate` が折り返すようになり、この前提が崩れる。

温度は `Get_Temperature`（0x015）が FET とモータの 2 つを報告する。
モータ側はサーミスタを設定していなければ 0 を返す。
`drivers/motor/robomaster.c` が `motor_feedback.temperature` に入れているのは ESC が報告するモータ温度であり、
`drivers/motor/robstride.c` が入れているのもモータ側の温度である。

参照実装には 1 点、生成済みメッセージ定義と食い違う箇所がある。
`Set_Input_Pos` の `Vel_FF` と `Torque_FF` は DBC 由来の定義では 0.001 倍率の `int16` だが、
参照実装は `int8` で書き込んでいる。

## Decision

1. **対象は ODrive Pro / S1 / Micro のファームウェア 0.6 系に限る。** ODrive 3.x は `compatible` に載せない。`node_id` の予約値 0x3F は `reg` として受け付けず、binding の範囲を 0 から 62 とする。

2. **`motor_driver_api` は実装する。** ADR 0004 の決定 1 と同じ理由で、`enable`、`disable`、`get_feedback` の 3 つは上位がモータの機種を知らずに呼べる。

3. **`motor_set_output()` はどのモードでも `-ENOTSUP` を返す。** 電圧モードはプロトコルに存在するが参照実装が未対応で、実機で確かめられていない。速度と電流については、`enum motor_output_mode` に載せられる単位の契約がクラス側にない。

4. **`motor_feedback` に立てる `valid_mask` は `MOTOR_FEEDBACK_POSITION` と、決定 16 の条件を満たすときの `MOTOR_FEEDBACK_TEMPERATURE` だけとする。** `velocity`、`current`、`orientation` は立てない。ODrive は `Get_Iq` で電流を A で報告しているので、ADR 0004 が RobStride について述べた「報告している量が違う」という理由はここでは成り立たない。立てない理由は、`drivers/motor/robomaster.c` が同じフィールドに生値を入れていて、クラスに単位の契約がないことだけである。

5. **`motor_feedback.position` は 1 rev = 65536 カウントに固定する。** ADR 0005 の決定 4 のとおり devicetree には持たせず、`doc/drivers/odrive.md` に書く。float32 の刻み幅は値が 2^k のとき 2^(k-23) なので、1/65536 rev の刻みは 128 rev で float32 の刻みに追いつく。**128 rev まではカウントと `Pos_Estimate` が 1 対 1 に対応し、そこから 2 倍ごとにカウントが 2 段ずつ飛ぶようになる。**

6. **位置はドライバが積算しない。** ODrive がすでに多回転の連続値を報告しているので、ドライバがするのは倍率の適用だけである。`drivers/motor/robomaster.c` と `drivers/motor/robstride.c` が折り返しを解いているのと、ここは違う。ODrive 側で `circular_setpoints` を有効にすると連続性が失われるので、`doc/drivers/odrive.md` で禁止する。

7. **feedback は放送を受けるだけとし、RTR は使わない。** ドライバが必要とする値はすべて周期メッセージで届く。ドライバは ODrive 側の `*_msg_rate_ms` 設定に依存することになるので、`doc/drivers/odrive.md` に必要な設定を列挙する。

8. **周期メッセージが届いているかは `odrive_feedback.valid_mask` で示す。** 既定で無効な周期メッセージが多く、設定漏れは「値が 0 のまま」という形で静かに現れる。フィールドごとに「一度も届いていない」と「届いたが古い」を区別できるようにする。

9. **`online` は heartbeat、`stale` は encoder estimates で判定する。** heartbeat は既定で有効な唯一のメッセージなので死活監視の鍵に適する。位置と速度の鮮度は encoder estimates の到着時刻で測る。binding は `heartbeat-timeout-ms`（既定 300）と `estimate-timeout-ms`（既定 100）を別々に持つ。

10. **送信タイマーが送るのは指令フレームだけとする。** ODrive の watchdog は `Set_Input_*` の到着で再始動するので、有効な軸には `CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS` ごとに現在のモードの指令フレームを 1 通送る。feedback の取得には送信が要らないので、無効な軸には何も送らない。`odrive_set_*()` は指令を保存するだけで CAN の送信を待たず、制御ループから呼んでよい。

11. **`motor_enable()` は `Clear_Errors` を 1 度だけ送る。** 手順は `Clear_Errors` → `Set_Controller_Mode` → `Set_Axis_State(CLOSED_LOOP_CONTROL)` で、heartbeat の `Axis_State` が 8 になるまで `Set_Controller_Mode` 以降を `CONFIG_MOTOR_ODRIVE_STATE_RETRY_MS` ごとに繰り返す。**再試行では `Clear_Errors` を送らない。** 過電流で落ちた軸をループの中で自動的に復帰させると、原因が取り除かれないまま再投入を繰り返すことになる。明示的に消したい場合のために `odrive_clear_errors()` を置く。

12. **有効化中に軸が closed loop から外れたら、ドライバは再投入しない。** ODrive が自分で disarm するのは故障したときであり、復帰には `Clear_Errors` が要る。決定 11 と同じ理由で自動では消さない。`odrive_feedback` に `axis_state`、`procedure_result`、`active_errors`、`disarm_reason` を出し、再投入するかどうかは上位が決める。
上位が disarm を取りこぼさずに気づけるよう、決定 22 の状態コールバックを対で用意する。
**このときドライバ自身の状態は無効に落とし、決定 10 の指令送信を止める。**
ODrive が受け付けない指令を送り続けても意味がなく、watchdog の餌だけが残るのは紛らわしい。
`motor_enable()` はいつ呼ばれても決定 11 の手順を最初からやり直すので、
上位の再投入は `motor_enable()` の 1 回で足りる。
`odrive_clear_errors()` は有効化を伴わずにエラーだけ消したいときのためにある。
`odrive_estop()` のあとも同じで、`Estop` が残す `ESTOP_REQUESTED` は次の `motor_enable()` が消す。
`drivers/motor/robstride.c` が run state の離脱でハンドシェイクをやり直すのとは、ここで分かれる。

13. **`motor_disable()` は `Set_Axis_State(IDLE)` を次の送信周期を待たずにその場で送る。** `odrive_estop()` は別の関数として置き、`Estop`（0x002）を送る。`Estop` は `ESTOP_REQUESTED` をエラーとして残すので、`motor_disable()` とは復帰の手間が違う。同じ関数にまとめない。

14. **指令とパラメータの読み書きは `include/drivers/motor/odrive.h` に置く。単位は線路上のものをそのまま使い、rev、rev/s、Nm、A とする。** ADR 0004 の決定 5 が `float` と SI を選んだのと同じ趣旨だが、ODrive の線路が rev であり、ゲインと上限の単位も rev 系である。rad に直すとヘッダの数値が odrivetool で設定した値と一致しなくなる。

15. **`input_mode` は実行時の API で設定し、devicetree には置かない。** ADR 0004 の決定 11 と同じ判断である。既定は `INPUT_MODE_PASSTHROUGH` とし、`Set_Controller_Mode` を送るときに現在の `control_mode` と一緒に載せる。`INPUT_MODE_TRAP_TRAJ` のために `odrive_set_traj_limits()` を置く。
**決定 10 との相互作用は未確認である。** `INPUT_MODE_TRAP_TRAJ` で同じ `Set_Input_Pos` を再送したときに ODrive が軌道を再計画するなら、
`CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS` ごとにプロファイルが振り出しに戻り、`Trajectory_Done_Flag` が立たなくなる。
`INPUT_MODE_PASSTHROUGH` と `INPUT_MODE_VEL_RAMP` にはこの懸念がない。
実機で確かめ、再計画されるようなら `INPUT_MODE_TRAP_TRAJ` に限って値が変わったときだけ送る形に決定 10 を直す。
その場合そのモードでは watchdog の餌が途切れるので、`enable_watchdog` との併用も併せて確かめる。

16. **`motor_feedback.temperature` にはモータ温度を入れ、`odrive,axis` に `has-motor-thermistor` を持たせる。** `drivers/motor/robomaster.c` と `drivers/motor/robstride.c` が同じフィールドに入れているのはモータ側の温度なので、FET 温度を入れると同じフィールドが実装ごとに別の場所を指す。サーミスタが未設定の ODrive はモータ温度に 0 を返し、0 ℃ は「冷えている」と読めてしまうので、**`has-motor-thermistor` がないノードでは `MOTOR_FEEDBACK_TEMPERATURE` を立てない。** サーミスタの有無は配線が決まれば変わらないので、ADR 0004 の決定 11 が devicetree に置いてよいとした条件に当てはまる。FET 温度は `odrive_feedback` からのみ読める。

17. **上限値をドライバから書かない。** ODrive の `vel_limit` と `current_limit` は odrivetool で設定して `save_configuration()` で保存するものであり、有効化のたびに上書きすると設定が黙って変わる。devicetree に上限のプロパティを持たせず、実行時に変えたい場合だけ `odrive_set_limits()` を使う。

18. **devicetree は transport の親ノードと軸の子ノードの二段にし、親が持つ CAN コントローラは 1 個とする。** RobStride の binding が `cans` を phandle の配列にしているのは、ホスト ID が 1 つでモータがどのコントローラにいるか分からないからである。ODrive は `node_id` で宛先が決まりホスト ID を持たないので、コントローラをまたぐ必要がない。コントローラが 2 本あるなら `odrive,bus` を 2 つ書く。

19. **受信フィルタは軸ごとに 1 本張る。** `id = node_id << 5`、`mask = 0x7E0` で、その軸のすべての `cmd_id` が 1 本で取れる。`CONFIG_CAN_MAX_FILTER` とコントローラのフィルタ本数が軸数の上限になるので、`CONFIG_MOTOR_ODRIVE_MAX_AXES` の既定は控えめにする。

20. **compatible の vendor prefix は `odrive` とする。** `dts/bindings/vendor-prefixes.txt` に 1 行足す。親が `odrive,bus`、子が `odrive,axis` になる。`reg` が `axis.config.can.node_id` に対応することが名前から分かるので、`robstride,motor` に揃えるより誤解が少ない。

21. **`Set_Input_Pos` の前置補償は生成済みメッセージ定義に従い、0.001 倍率の `int16` で符号化する。** 参照実装は `int8` で書いているが、DBC 由来の定義と食い違っている。実機で確かめた時点で定義側が正しければこのままとし、違っていればコメントを添えて参照実装に合わせる。

22. **軸ごとに 2 本のコールバックを実行時に登録できるようにする。** 状態コールバックは heartbeat が報告する `axis_state`、`procedure_result`、`active_errors` のいずれかが変わったときと、`heartbeat-timeout-ms` による online / offline の遷移で呼ぶ。feedback コールバックは `Get_Encoder_Estimates` が届いたときに呼ぶ。2 本に分けるのは、**再投入の判断だけが欲しい上位が 100 Hz の feedback を受け取らずに済むようにする**ためである。iq、temperature、torques、bus、error はスナップショットを更新するだけで呼び出さない。ODrive 側の周期設定によって発火頻度が変わるコールバックを増やしても、上位が使える保証がない。

23. **コールバックは割り込みコンテキストではなく、バスごとの専用ワークキューから呼ぶ。** 受信経路は spinlock の下でスナップショットを更新して保留ビットを立て、ワークを submit するだけにする。ワークハンドラがロックの外でコールバックを呼ぶ。決定 12 の再投入は `motor_enable()` を呼ぶことであり、**これをコールバックの中から直接呼べる**ことがこの決定の目的である。**送信タイマーは有効な軸が 1 つもなくても回し続ける。** タイムアウトの判定とワークの submit もこのタイマーが行うので、止めると offline の遷移が通知されない。`drivers/motor/robstride.c` は stale と online を読み出しの時点で計算しているが、コールバックで知らせる以上、誰も読まなくても判定が進む必要がある。有効化する前に軸が生きているかを確かめる、という使い方がこれで成り立つ。
ワークキューを専用に持つのは、上位のコールバックが長引いても決定 10 の指令送信（watchdog の餌）を遅らせないためである。指令送信のワークは `drivers/motor/robstride.c` と同じくシステムワークキューに置く。スタックサイズと優先度は Kconfig で変えられるようにする。

24. **コールバックは合流してよいものとする。** ワークが処理される前に次のフレームが届いた場合、呼び出しは 1 回にまとまり、上位が見るのは常に最新のスナップショットである。イベントの列は保存しない。再投入の判断に要るのは現在の状態であって遷移の履歴ではなく、ODrive の `active_errors` は `Clear_Errors` まで残るので、途中の heartbeat を 1 つ落としても消える情報がない。

## Consequences

ドライバは `float` を使う。
payload が IEEE754 をそのまま載せるので、内部を固定小数で書いても最後に変換が要る。
ADR 0004 の Consequences と同じく、`CONFIG_FPU` を有効にした Cortex-M4F と `native_sim` のどちらでも問題にならない。

**ドライバの振る舞いが ODrive 側の設定に依存する。**
`iq_msg_rate_ms`、`temperature_msg_rate_ms`、`torques_msg_rate_ms`、`bus_voltage_msg_rate_ms`、`error_msg_rate_ms` は既定で無効なので、
これらを有効にしない限り対応する `odrive_feedback` のフィールドは `valid_mask` が立たない。
`doc/drivers/odrive.md` に odrivetool のコマンドを含めて列挙する。

放送を受けるだけなので、軸を増やしてもホストの送信は増えない。
バス上のトラフィックは ODrive 側の周期設定で決まる。
RobStride のように「台数に比例してホストの送信が増える」形にはならない。

有効な軸に対しては、指令が変わらなくても `CONFIG_MOTOR_ODRIVE_TX_INTERVAL_MS` ごとにフレームが出る。
`enable_watchdog` を使わない構成ではこの送信は冗長だが、指令の鮮度が一定に保たれる利点を採る。

故障からの復帰が手動になる。
決定 11 と 12 により、ドライバは一度落ちた軸を自分で戻さない。
上位は状態コールバックで disarm を受け取り、`motor_enable()` を自分で呼ぶ。
決定 23 によりコールバックはスレッドコンテキストで走るので、その場で呼べる。

バスごとにスレッドが 1 本増える。
コールバックを 1 本も登録しない構成でもワークキューは立ち上がるので、
RAM の見積もりには `CONFIG_MOTOR_ODRIVE_WORKQ_STACK_SIZE` × バスノード数が乗る。

fibril_can と同じ CAN コントローラを共有できない。
ODrive は 11 bit 標準 ID の classic CAN、fibril_can は CAN FD である。
`fibril_robomaster_miniv4`（FDCAN1 / 2 / 3）と `fibril_rc26_mainair_v01`（FDCAN2 / 3）はコントローラを 2 つ以上持つのでこの条件を満たす。
`fibril_robomaster_miniv1` と `fibril_robomaster_miniv3` は FDCAN1 のみで、`fibril_canmotor_tourobo2023` は MCP2517FD が 1 本なので、fibril_can との併用はできない。

汎用 motor クラス越しに ODrive を他のモータと差し替えられるのは、
`enable`、`disable`、死活監視、`motor_feedback.position` までである。
`position` のスケールがドライバごとに違う点は ADR 0002 の決定 11 と ADR 0004 の Consequences が述べたのと同じである。

ROS 側への露出はこの決定の範囲外とする。
`lib/fibril_can_node/` にブロック型を足す話は、ドライバの形が固まってから別に決める。

## 却下した選択肢

**RTR で値を取りにいく。**
周期メッセージを ODrive 側で有効にしなくても値が読める。
しかし応答を待つ処理が要るので、`odrive_get_feedback()` がブロックするか、
要求と応答を照合する状態機械を持つことになる。
周期メッセージはそれを何もせずに与える。
軸数と読みたい量の積だけ要求フレームが増える点も、決定 10 が避けた送信量の増加をそのまま招く。
Zephyr 側でも RTR の受信は `CONFIG_CAN_ACCEPT_RTR` で明示的に有効にしないと落とされるなど、既定から外れた経路になる。

**単位を rad と rad/s にして `robstride.h` と揃える。**
リポジトリ内でモータの API の単位が 1 つになる。
しかし ODrive の線路とゲインと上限はすべて rev 系で、odrivetool で読み書きする値も rev 系である。
ヘッダだけ rad にすると、`odrive_set_limits()` に渡した値と ODrive 側で見える値が一致しなくなる。
ADR 0004 の決定 5 が線路の量をそのまま出すことを選んだ趣旨に沿うのは rev の側である。

**`motor_feedback.current` に `Get_Iq` の `Iq_Measured` を A で入れる。**
ODrive は実際に電流を A で報告しているので、値としては正しい。
しかし `drivers/motor/robomaster.c` が同じフィールドに入れているのは指令スケールの生値であり、
同じ `MOTOR_FEEDBACK_CURRENT` が実装ごとに別の単位を指すことになる。
ADR 0004 の決定 2 が feedback について避けたのと同じ状態である。

**位置を 1 rev = 1000000 カウント（マイクロ rev）にする。**
10 進で読みやすく、`int64_t` の範囲にも余裕がある。
しかし刻みが細かいぶん、float32 の刻みに追いつかれるのが早い。
1/1000000 rev は約 8 rev で float32 の刻みと並び、そこから先はカウントが飛び始める。
65536 なら同じことが起きるのは 128 rev で、使える範囲が 16 倍広い。
どちらの倍率でも、追い越した先ではカウントが飛ぶという性質は変わらない。

**軸が closed loop から外れたら自動で `Clear_Errors` して再投入する。**
通信の瞬断や一時的な過電圧から自動で戻れる。
しかし過電流や過熱で落ちた軸も同じ経路で戻すことになり、原因が残ったまま再投入を繰り返す。
どの故障なら戻してよいかはドライバではなく機体を知っている層の判断である。

**`Estop` を `motor_disable()` の実装にする。**
非常停止がクラス越しに呼べるようになる。
しかし `Estop` は `ESTOP_REQUESTED` をエラーとして残すので、`motor_disable()` のあと `motor_enable()` で戻れなくなる。
クラスの `disable` と `enable` は対になっているべきで、片方だけ手順が増えるのは分かりにくい。

**上限（`vel_limit`、`current_limit`）を devicetree に置き、有効化のたびに書き込む。**
機体ごとの上限をボード定義に集められ、`drivers/motor/robstride.c` と形が揃う。
しかし ODrive は設定を自分の不揮発メモリに持ち、odrivetool で調整して保存する運用が前提である。
有効化のたびに上書きすると、odrivetool で見える値とファームウェアが書いた値のどちらが効いているのかを追う手間が増える。
RobStride と違い、ODrive 側の設定は実機ごとに作り込まれている。

**`odrive,bus` に複数の CAN コントローラを並べる。**
`dts/bindings/motor/robstride,bus.yaml` と形が揃う。
しかし ODrive はホスト ID を持たず `node_id` で宛先が決まるので、
どのコントローラに軸がいるかを探る必要がない。
決定 19 の軸ごとフィルタと組み合わせると、コントローラ数 × 軸数のフィルタを張るか、
コントローラを特定するまで全コントローラに送るかのどちらかになる。
バスノードを 2 つ書けば同じ構成が表せるので、複雑さに見合わない。

**受信フィルタを `mask = 0` の 1 本にしてソフトウェアで振り分ける。**
フィルタの本数が軸数に依存しなくなる。
しかしバス上の他のホストが出す指令フレームや、devicetree に書いていない `node_id` のフレームまで
すべて割り込みコンテキストに上がってくる。
ハードウェアが落とせるものをソフトウェアで落とす理由がない。

**`motor_feedback.temperature` に FET 温度を入れる。**
FET 温度は基板上のセンサなので必ず値を持ち、`has-motor-thermistor` のようなプロパティが要らない。
しかし `drivers/motor/robomaster.c` と `drivers/motor/robstride.c` が同じフィールドに入れているのはモータ側の温度である。
過熱の判定に使うフィールドが、実装によってモータを指したりインバータを指したりするのは、
ADR 0004 の決定 2 が feedback について避けたのと同じ状態である。

**コールバックを CAN の受信コールバックからその場で呼ぶ。**
スレッドが増えず、遅延も最小になる。
しかし Zephyr の受信コールバックは割り込みコンテキストで呼ばれるので、
上位は待てず、ログも出せず、決定 12 の再投入をその場で行えない。
再投入の判断を上位に任せると決めた以上、上位が普通に書けるコンテキストで呼ばないと意味がない。

**イベントをリングバッファに積み、上位が取りこぼさないようにする。**
状態遷移の履歴が残る。
しかし再投入の判断に要るのは現在の状態であり、
バッファが溢れたときにどう振る舞うかという問題を新たに抱える。
ODrive の `active_errors` は `Clear_Errors` まで残るので、履歴がなくても原因は読める。

**コールバックを 1 本にして、イベント種別を引数で渡す。**
登録の API が 1 つで済む。
しかし feedback は既定で 100 Hz、状態変化は稀である。
1 本にすると、再投入の判断だけが欲しい上位も 100 Hz で呼ばれ、
毎回イベント種別を見て捨てることになる。

**サーミスタの有無を devicetree ではなく `Motor_Temperature != 0` で判定する。**
プロパティが 1 つ減る。
しかし本当に 0 ℃ 付近まで冷えた環境で `valid_mask` が落ちることになり、
設定漏れと低温を区別できない。
配線で決まる事実は devicetree に書くほうが、実装の挙動が読んで分かる。

---
status: Proposed
date: 2026-11-14
---

# 0024. 汎用 motor クラスの指令と feedback を float の SI 単位にする

## Status

Proposed

## Context

`include/drivers/motor.h` は、指令と feedback の値の単位を決めていない（#52）。

`motor_set_output()` の説明は「mode が示す単位の値」とだけ書き、mode ごとの単位はどこにもない。
`struct motor_feedback` の `current`、`velocity`、`temperature` も単位を持たない。
`position` だけは ADR 0005 がドライバ固有の積算カウントと決めている。

ドライバの実装は次のように分かれている。

| | `set_output` | `current` / `velocity` | `position` | `temperature` |
|---|---|---|---|---|
| RoboMaster | CURRENT のみ。コントローラの生のコード | 生のコード | 8192 カウント = ロータ 1 回転 | ℃（コントローラの報告のまま） |
| RobStride | `-ENOTSUP`（ADR 0004 の決定 4） | 立てない（ADR 0004 の決定 2） | 65535 カウント = 8π rad | ℃ |
| ODrive | `-ENOTSUP`（ADR 0022 の決定 3） | 立てない（ADR 0022 の決定 4） | 65536 カウント = 1 rev | ℃ |

RobStride と ODrive が指令を断り、`current` と `velocity` を立てないのは、どちらも「クラスに単位の契約がない」ことが理由である。
指令は `robstride.h`、`odrive.h` の `float` の API で受けている。
クラス越しに機種を問わず扱えるのは、`enable`、`disable`、死活、温度、倍率の違う `position` までになっている。

クラス越しにモータを使う側は 2 つで、どちらも RoboMaster の生のコードを前提にしている。

- `lib/fibril_can_node/md_motor/md_motor.cpp` は、duty に `duty-full-scale`（既定 10000）を掛けた生のコードを `MOTOR_OUTPUT_MODE_CURRENT` で送る。ROS に出す `feedback.current` は生の値 / 1000 で、これは ADR 0013 の決定 4 で以前のファームウェアに合わせて選んだ値である。
- `lib/fibril_can_node/adc_port/adc_port.cpp` は、モータのポートの値を生の値 / 10000 にする（ADR 0013 の Context）。

VESC のドライバを足す予定がある。
VESC は電流、速度、duty の指令をそのまま受け付けるので、クラスに単位があれば `motor_set_output()` を実装できる。
単位がないままだと、RoboMaster の生のコードを真似て単位の決まりを 1 つ増やすか、RobStride と ODrive のように断るかの 2 択になる。

型の制約もある。
`int16_t` の 1 値では、数百 A までの電流と、0.001 rad/s 程度の分解能を要る速度を、1 つの固定倍率で載せにくい。
Zephyr の `__syscall` は `float` の引数を扱えない（上流にも `float` を受ける syscall はない）。
このリポジトリは `CONFIG_USERSPACE` を使っていない。

## Decision

### 1. 指令は `float` の SI 単位で渡し、motor クラスを syscall にしない

```c
int motor_set_output(const struct device * dev, enum motor_output_mode mode, float value);
```

`motor_enable()`、`motor_disable()`、`motor_get_feedback()` も含め、クラスの 4 つの関数から `__syscall` を外し、`static inline` の関数にする。
`#include <zephyr/syscalls/motor.h>` を消す。

指令を作る制御層も、指令を受ける機種固有の API も `float` で計算している。
間に整数を挟むと、両側で変換が増えるだけになる。

### 2. mode と単位

| mode | 単位 | ビット |
|---|---|---|
| `MOTOR_OUTPUT_MODE_CURRENT` | A | `1` |
| `MOTOR_OUTPUT_MODE_VELOCITY` | rad/s | `1 << 1` |
| `MOTOR_OUTPUT_MODE_VOLTAGE` | V | `1 << 2` |
| `MOTOR_OUTPUT_MODE_TORQUE` | Nm | `1 << 3` |
| `MOTOR_OUTPUT_MODE_DUTY` | -1..1（無次元） | `1 << 4` |

- **`TORQUE` を `CURRENT` の別名にしない。** 単位を決めると、A と Nm は別の量になる。ODrive はトルクを、RoboMaster のコントローラは電流を受け付ける。トルク定数を知らないドライバに、片方からもう片方を作らせない。
- **`VOLTAGE` は残す。** モータにかかる電圧を V で指令する。電源電圧が下がっても同じ電圧がかかる点で、`DUTY` と別の意味を持つ。電圧を直接受け付けないコントローラでも、電源電圧を測っていれば duty に直して実装できる（VESC は STATUS_5 の入力電圧で割る）。電源電圧が読めないときは `-EAGAIN` を返す。
- **`DUTY` を足す。** VESC がそのまま受け付ける指令で、md_motor も中では duty で計算している。
- **`POSITION` は足さない。** VESC の位置指令は 1 回転の中の角度、ODrive は多回転の連続値で、零点と軌道の扱いも機種で違う。共通の意味を決められるまで、機種固有のヘッダに残す。
- 速度と位置は、**ドライバが測っている軸**のものとする。RoboMaster ならロータである。減速比は ADR 0002 の決定 11 のとおり制御層に置く。
- 対応しない mode には `-ENOTSUP` を、有限でない値には `-EINVAL` を返す。

### 3. feedback の単位

```c
struct motor_feedback
{
  uint32_t valid_mask;
  float current;      /* A */
  float velocity;     /* rad/s */
  float torque;       /* Nm */
  float temperature;  /* ℃、モータ側 */
  int64_t position;   /* ドライバ固有の積算カウント（ADR 0005） */
  int32_t orientation;
  bool online;
  bool stale;
  int64_t timestamp_ms;
};
```

- `MOTOR_FEEDBACK_TORQUE` を足す。RobStride と ODrive は電流ではなくトルクを報告する（ADR 0004 の決定 2）。
- `temperature` はモータ側の温度とする。インバータの温度は入れない（ADR 0022 の決定 16 と同じ）。
- `position` と `orientation` は変えない。ADR 0005 の理由（`float` で積算すると精度が落ちる）は今も当てはまる。

### 4. 1 回転あたりのカウント数と零点と、対応する mode をクラスから読めるようにする

```c
struct motor_info
{
  uint32_t output_modes;    /* enum motor_output_mode のビットの和 */
  float counts_per_rev;     /* position の 1 回転あたりのカウント */
  float zero_counts;        /* 機器の零点（0 rad）に当たる position */
};

int motor_get_info(const struct device * dev, struct motor_info * info);
```

`counts_per_rev` は RoboMaster で 8192、RobStride で 65535 / 4、ODrive で 65536 になる。
ADR 0005 の決定 3 は換算を各ドライバの文書に書くとした。
ここで足すのは、使う側が機種を知らずに換算できる手段であり、カウントの意味は変えない。

`zero_counts` は、RoboMaster で 0、RobStride で 65535 / 2、ODrive で 0 になる。
軸の角度は `(position - zero_counts) * 2π / counts_per_rev` rad である。
RobStride はカウント 0 が -4π rad なので、0 rad がカウント 32767.5 になり、整数にならない。そのため `float` にする。
零点がどこにあり、電源を入れ直しても保たれるかは、ドライバの文書に書く。RoboMaster の 0 は、電源を入れたときのロータの 1 回転の中の機械角 0 で、出力軸に対しては保たれない。
`zero_counts` は #55 で足した（2026-11-16、ユーザーが選んだ）。いまクラスの `position` を読む md_motor と odometry は差分か自前の零点しか使わないが、機種を知らない側が `counts_per_rev` だけを読んで 0 カウントを 0 rad と取り違えないようにする。

### 5. RoboMaster のドライバをこの単位に移す

- 指令と feedback の電流は `model` ごとの係数で A と生のコードを換算する。今は読まれていない `model` を、ドライバが使うようになる。
- 速度はロータの rpm を rad/s に直す。
- `max-current`（生の単位）を `max-current-ma` にする。RobStride の `max-current-ma` と名前と単位を揃える。
- 温度を報告しないコントローラでは `MOTOR_FEEDBACK_TEMPERATURE` を立てない。

### 6. 使う側を A に移す

- **md_motor の指令**：duty に `duty-full-scale-ma` を掛けた電流を `MOTOR_OUTPUT_MODE_CURRENT` で送る。あわせて ADR 0010 の決定 5（duty 1.0 は生の電流値 10000）を改訂する。`duty-full-scale-ma` はモータごとの配列とし、既定値を持たせない。今の `duty-full-scale` は 1 つの生の値を全モータに使っていて、C610 と C620 が混ざると A では違う値になる。調整済みのゲインの意味を保つには、モータごとに今の生の値に当たる A を書く必要がある。
- **md_motor の `feedback.current`**：A で出す。**あわせて ADR 0013 の決定 4（生の値 / 1000）を改訂する。** `current/offset` と、ROS 側の電流のしきい値は A で設定し直す。
- **adc_port のモータのポート**：A で出す。あわせて ADR 0013 の決定 1 のうち、モータを「報告する電流 / 10000」に正規化する部分を改訂する。キャリブレーションのしきい値は A で設定し直す。

### 7. RobStride と ODrive は後の PR で移す

この ADR を実装する PR では、`motor.h`、RoboMaster、md_motor、adc_port を同時に移す。クラスの契約と、それに頼る側が別々に変わると壊れるからである。
RobStride と ODrive は、新しい `struct motor_feedback` に合わせる最小の変更（`temperature` を `float` にする）だけを同じ PR に含める。
`motor_set_output()` の実装と、`torque`、`velocity` の feedback は後の PR で足す（#55）。
そのとき ADR 0004 の決定 2 と 4、ADR 0022 の決定 3 と 4 を改訂する。
ODrive の `odrive.h` は rev 単位のまま残し、クラスの入口でだけ rad に直す（ADR 0022 の決定 14 は変えない）。

#55 で移した結果は次のとおりである。

| ドライバ | `output_modes` | feedback の `valid_mask`（位置のほか） |
|---|---|---|
| RobStride | `CURRENT`、`VELOCITY` | 速度、トルク、温度 |
| ODrive | `TORQUE`、`VELOCITY` | 速度、トルク、電流、温度（`has-motor-thermistor` があるとき） |

RobStride の `TORQUE` は、運動制御モードで kp と kd を 0 にすれば出せるが、実機で確かめていないので載せない（ADR 0004 の決定 4）。
ODrive の `CURRENT` は、CAN に電流を指令するメッセージが無いので載せない（ADR 0022 の決定 3）。

## Consequences

- クラスの公開ヘッダと devicetree binding（`dji,robomaster-motor` の `max-current`、`fibril,fcan-md-motor` の `duty-full-scale`）の互換を壊す。既存の snippet と overlay は同じ PR で書き換える。
- md_motor と adc_port が ROS に出す電流の値の意味が変わる。何が変わり何が変わらないかは、下の「ROS から見える値」にまとめた。
- `motor_set_output()` を syscall にしないので、将来 `CONFIG_USERSPACE` を使うときは、値を構造体のポインタで渡す syscall を別に足す必要がある。
- `struct motor_feedback` が `float` を含むので、FPU のない MCU では feedback を読むたびにソフトウェアの浮動小数点演算が入る。いまのボードはすべて Cortex-M4F で、ADR 0004 と 0022 がすでに `float` を受け入れている。
- RobStride と ODrive は、決定 7 の PR（#55）で、ほかのドライバと同じくクラスの単位で指令と feedback を受け渡すようになった。位置の指令と、運動制御や `input_mode` のような機種に固有の指令は、機種専用の API に残る。
- 新しいドライバ（VESC など）は、汎用クラスに指令を実装でき、md_motor のような機種を知らない使う側から使えるようになる。
- **RoboMaster の換算係数は仮の値で置く。** 実装では次の値を使い、DJI の資料か実機で確かめて、違えば係数だけを直す。
  - C620：指令と feedback とも ±16384 = ±20 A。
  - C610：指令と feedback とも ±10000 = ±10 A。feedback の倍率は確かめていない。
  - C610 は温度を報告しないとみなし、`MOTOR_FEEDBACK_TEMPERATURE` を立てない。

### ROS から見える値

汎用クラスの値は、そのまま ROS には出ない。
ROS に届くのは、汎用クラスを使うブロック型（md_motor と adc_port）が fibril_can で出す値である。
ODrive と RobStride のブロック型は機種専用の API を使っているので、この ADR では ROS 側の値が変わらない。

| ROS から見える値 | 今 | この ADR の後 | ROS 側ですること |
|---|---|---|---|
| MdMotor の `feedback.current`（`fibril_control_msgs/msg/Feedback`） | 生の値 / 1000 | A | しきい値や記録を A で読み直す。想定している係数では、C620 で今の値の 20 / 16.384 ≈ 1.22 倍になる。仮の係数では、C610 は今と同じ値になる（±10000 = ±10 A なので、生の値 / 1000 がそのまま A）|
| MdMotor の `current/offset` | `feedback.current` と同じ単位（生の値 / 1000） | A | 設定していれば A で書き直す |
| MdMotor の `calibrate_current_baseline` が測るゼロ点 | `feedback.current` と同じ単位 | A | なし。ゼロ点はノードの中だけで使い、応答は成否だけである |
| AdcPort の `threshold`（モータのポート） | 生の値 / 10000 | A | **既定値 0.5 の意味が変わる。** 今は生の値 5000（C620 で約 6.1 A）だが、移した後は 0.5 A になる。モータのポートでは明示して設定する |
| AdcPort の `threshold`（ADC のポート） | フルスケールに対する 0..1 | 変わらない | なし |
| MdMotor の `target`、`trajectory`、`feedback.output`、`velocity`、`position` | duty と、`encoder/gain` × カウント | 変わらない | なし |
| MdMotor の速度と位置の PID のゲイン、`friction/*`、`accel_ff_gain`、`duty/slew_rate` | duty で表した値 | 変わらない | なし。ただし `duty-full-scale-ma` が今の生の値に当たる A になっている場合に限る（決定 6） |

AdcPort の `threshold` は、1 つのパラメータでありながら、ポートの種類によって単位が違う（ADC は 0..1、モータは A）。
これはこの ADR の前からそうで、この ADR では変えない。

## 却下した選択肢

### `int32_t` のミリ／マイクロ単位にする

Zephyr の regulator（µV）や sensor と同じ書き方で、syscall にもでき、devicetree の `max-current-ma` とも揃う。
しかし量ごとに接頭辞を選ぶ必要がある。
たとえば duty をミリ単位にすると 0.1 % 刻みになり、md_motor が今送っている 1/10000 の分解能より粗くなる。
指令を作る側も受ける側も `float` なので、両側で変換が増える。

### `int16_t` のまま、mode ごとに固定の倍率を決める

型を変えずに済み、syscall も残せる。
しかし電流を 10 mA 刻みにすると ±327 A で頭打ちになり、速度を rpm にすると 1 rpm（約 0.1 rad/s）より細かく指令できない。
範囲と分解能のどちらかを、機種によっては満たせない。

### `float` のまま、構造体のポインタで syscall を残す

userspace を使うときに困らない。
しかし今は userspace を使っておらず、呼び出しのたびに構造体を組む手間だけが増える。
使うことになったときに足せば足りる。

### `TORQUE` を `CURRENT` の別名のまま残す

既存の enum の値が変わらない。
しかし単位を決めると A と Nm は別の量になり、別名のままでは、トルクしか受け付けない ODrive と電流しか受け付けない RoboMaster のどちらかが、意味の違う値を受け取る。

### `VOLTAGE` を外す

実装しているドライバが今は 1 つもない。
しかし電圧の指令は、電源電圧が変わってもモータへの電圧を保てる点で `DUTY` と意味が違い、VESC は入力電圧を測っているので実装できる。

### `POSITION` を足す

int16 の制約がなくなったので、型としては載せられる。
しかし VESC は 1 回転の中の角度、ODrive は多回転の連続値で、零点の決め方も違う。
共通の意味を決めないまま mode だけ足すと、同じ値が機種ごとに別の位置を指す。

### 零点を `motor_info` に持たせず、ドライバの文書に書くだけにする

`motor_info` が増えない。
いまクラスの `position` を読む md_motor と odometry は、差分か自前のキャリブレーションしか使わない。
しかし `counts_per_rev` だけをクラスから読めると、機種を知らない側は 0 カウントを 0 rad と読みやすく、RobStride では 4π rad ずれる。
換算に要る 2 つの値のうち 1 つだけをクラスに置くことになる。

### RobStride の積算を 32768 ずらし、どのドライバでもカウント 0 を零点にする

`motor_info` が増えず、全ドライバで「カウント 0 が機器の零点」と言える。
しかし 0 rad はカウント 32767.5 にあるので、半カウント（約 0.0002 rad）ずれる。
ADR 0004 の決定 3（積算するカウントは線路上のコードそのもの）も崩れる。

### `position` を全機種で 65536 カウント = 1 回転に揃える

使う側が換算を知らなくて済む。
しかし RobStride の線路は 8π rad あたり 65535 段で、整数倍の換算にならない。
ADR 0005 の決定 3 の理由（線路の分解能をそのまま保つ）が崩れるので、`counts_per_rev` を読ませるほうを選んだ。

### md_motor と adc_port が出す電流を、今の値のまま保つ

ROS 側の設定を移さずに済む。
しかし生の値 / 1000 や / 10000 を作るには、使う側が機種ごとの生のコードの倍率を知る必要があり、単位を決めた意味がなくなる。
devicetree に互換のための係数を残す案も、同じ理由で採らない。

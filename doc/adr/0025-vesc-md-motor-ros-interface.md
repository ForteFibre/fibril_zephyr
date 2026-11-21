# 0025. VESCはMdMotorのROSインタフェースだけを公開する

Status: Proposed

## Context

VESCは`drivers/motor/vesc.c`で汎用motorクラスを実装し、その上の`MdMotor`が位置・速度PIDと
ROS向けの指令・feedbackを担う。初期実装では、これとは別にVESCの状態、telemetry、packet
freshness、CAN ID bitmapをVESC専用ROS messageとして公開しようとした。

この形では、電流、出力、接続状態などを`MdMotor`の`Feedback`とVESC専用topicの二箇所へ
出す。RoboMasterからVESCへ出力段を替えたときにROS consumerも変更する必要が生じ、
`MdMotor`がmotor driverの差を隠す層であることと矛盾する。また、VESC専用messageを
通信基盤の`fibril_can_msgs`へ置くと、共通プロトコルが接続デバイスを知ることになる。

## Decision

VESCを出力段に使う構成が公開するROS制御インタフェースは、`MdMotor`の既存契約だけにする。

- `motorN/target`: `fibril_control_msgs/msg/Target`
- `motorN/feedback`: `fibril_control_msgs/msg/Feedback`
- reset、sensor、triggerなどのservice: `MdMotor`の既存service
- PID、encoder、timeout: `MdMotor`の既存parameter

`VescMotor`と`VescBus`のblock型は、CAN ID、電流上限、feedback timeout、再接続数、指令周期の
parameterだけを同じschemaへ加える。VESC専用topicは持たない。

VESC protocolのpacket age、bitmap、counter、fault詳細は、ファームウェア内部のdiagnostics APIと
明示的に有効化する診断ログへ残す。これらを将来ROSへ出す場合は、具体的なconsumerと、複数の
motor driverで共通にできる契約を先に定義する。VESC専用型を`fibril_can`へ追加しない。

## Consequences

- RoboMasterとVESCで、ROS側のpublisher、subscriber、message型、topic名を変えずに済む。
- VESCの電流と健全性は`Feedback.current`と`Feedback.is_ready`で読む。
- VESC固有の詳細診断を常時ROS graphやrosbagから読むことはできない。bring-upでは診断buildの
  ログを使う。
- bridgeが捨てるだけのVESC topicがwire、frame table、copy poolを消費しなくなる。

## Rejected alternatives

### VESC専用ROS messageを`fibril_can_msgs`に置く

通信基盤が特定デバイスを知り、リポジトリの責務を越えるため採らない。

### `fibril_vesc_msgs`を新設する

同じ値を`MdMotor`と二重に公開し、consumerをVESCへ結合するため、制御面には採らない。
VESC固有診断のconsumerが具体化した場合にだけ再検討する。

### VESC専用topicをROS mappingなしでwireに残す

master側のconsumerがなく、帯域とRAMだけを使うため採らない。

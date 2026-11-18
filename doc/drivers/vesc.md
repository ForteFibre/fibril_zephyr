# VESC CAN driver

The VESC driver owns one classic CAN controller and exposes eight fixed logical
motor slots. A slot starts unassigned and cannot transmit until the
`vesc_node.motorN.vesc_can_id` and `max_current_a` parameters have been
accepted. IDs are never discovered into slots automatically.

`MOTOR_OUTPUT_MODE_CURRENT` is amperes. The driver clamps every frame to
`absolute-max-current-ma`; `MdMotor` scales duty by the runtime
`max_current_a`. CAN packet IDs and physical conversions live in
`vesc_protocol.c`, separately from connection and safety state.

Any valid STATUS packet contributes to connection state. A slot becomes online
after `reconnect_frames` consecutive packets and becomes stale after
`feedback_timeout_ms`. Timeout, a nonzero queried fault, unassignment, or ID
change sends 0 A and disables the slot. Reconnection alone does not re-enable
it: `MdMotor` must receive a new target and call `motor_enable()`.

STATUS_4 position uses 18000 counts/revolution according to
`ForteFibre/ros_vesc@716a9dc`. The encoder shim resolves each delta by the
shortest path, leaves exactly half a turn unchanged, retains the last position
through an outage, and uses the actual receive interval for velocity.

The selected controller is the vendor-described **VESC CAN FOC ESC** for a 6S
(22--26 V) supply, with a 60 A-class MOSFET stage and CAN configurable to
1 Mbit/s. Its firmware reference is `vedderb/bldc` at
`cb79df8960f77fcdf368f3b921d24d5a3ed86345`. That source confirms the 29-bit
ID layout, ID 255 as broadcast, SET_CURRENT ID 1 with signed A x1000,
STATUS/4/5 layouts, the selective fault reply, and a configurable command
timeout. This deployment reserves ID 254 as the MiniV4 fault-reply destination
and never emits broadcast commands. Runtime motor IDs are therefore 0--253;
254 and 255 may be observed in diagnostics but are never assigned or controlled.

STATUS_4 PID position is `mc_interface_get_pid_pos_now()` in degrees x50 and
is normalized to one revolution. With a configured encoder, the firmware
derives it from the encoder angle and applies encoder inversion, motor
direction, and PID offset. It is the motor-side sensor angle; any gearbox/output
shaft conversion remains outside the VESC protocol layer.

> **Hardware TODO:** verify on the selected controller that its flashed image
> matches the referenced firmware and measure STATUS periods, jitter, loss,
> bus load, 0 A behavior, and regenerative behavior. Verify the position sign
> after the actual encoder calibration and motor installation. Also verify the
> safe absolute current for the actual
> motor and power system: the controller manual recommends 60 A absolute for
> M3508 and 30 A for M2006. The constants are deliberately confined to the
> protocol/units layer so this does not alter `MdMotor` or diagnostics schemas.

On RoboMaster Mini V4, `miniv4-md-vesc8` assigns MOTCAN1 (`fdcan3`) at 1 Mbit/s
and creates the eight VESC slots, encoder shims, `MdMotor` instances, and VESC
diagnostics.

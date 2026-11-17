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

> **Hardware TODO:** select the VESC hardware and firmware, then verify CAN ID
> reservations, SET_CURRENT scaling/sign, STATUS packet layouts and periods,
> STATUS_4 mechanical meaning/range/direction, fault replies, command timeout,
> and safe absolute current. The constants are deliberately confined to the
> protocol/units layer so this does not alter `MdMotor` or diagnostics schemas.

On RoboMaster Mini V4, `miniv4-md-vesc8` assigns MOTCAN1 (`fdcan3`) at 1 Mbit/s
and creates the eight VESC slots, encoder shims, `MdMotor` instances, and VESC
diagnostics.

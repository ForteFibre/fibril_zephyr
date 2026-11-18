# VESC CAN driver

The VESC driver owns one classic CAN controller and exposes eight fixed logical
motor slots. A slot starts unassigned and cannot transmit until the
`motorN.vesc_can_id` and `motorN.max_current_a` parameters have been
accepted. IDs are never discovered into slots automatically.

The VescMotor and its MdMotor intentionally share the canonical `motorN`
fibril_can namespace. Existing parameters such as `motorN.invert`,
`motorN.encoder.gain`, and `motorN.velocity.kp` therefore keep the same names
as RoboMaster deployments. VESC-only slot parameters are added beside them;
bus-wide defaults are under `vesc_bus`.

For a firmware node with CAN node ID 3, the ROS parameter file is shaped like
this:

```yaml
fibril_can_bridge:
  ros__parameters:
    can_iface: can0

    # Existing MdMotor parameters, reusable across motor drivers.
    md_controller3.motor0.invert: false
    md_controller3.motor0.encoder.gain: "2 * _pi / (18000 * 19)"
    md_controller3.motor0.velocity.kp: 0.08
    md_controller3.motor0.velocity.ki: 0.0004
    md_controller3.motor0.position.kp: 8.0
    md_controller3.motor0.use_cascade_position: true

    # VESC-only slot parameters.
    md_controller3.motor0.vesc_can_id: 37
    md_controller3.motor0.max_current_a: 12.207

    # VESC bus defaults; a motorN value of zero inherits these.
    md_controller3.vesc_bus.feedback_timeout_ms: 100
    md_controller3.vesc_bus.reconnect_frames: 3
    md_controller3.vesc_bus.command_period_us: 5000
```

`MOTOR_OUTPUT_MODE_CURRENT` is amperes. The driver clamps every frame to
`absolute-max-current-ma`; `MdMotor` scales duty by the runtime
`max_current_a`. CAN packet IDs and physical conversions live in
`vesc_protocol.c`, separately from connection and safety state.

Any valid STATUS packet contributes to connection state. A slot becomes online
after `reconnect_frames` packets without a `feedback_timeout_ms` gap and becomes
stale after that timeout. Each STATUS kind also has its own age and freshness;
in particular, other traffic cannot keep a missing STATUS_4 position valid.
Timeout, a nonzero queried fault, unassignment, or ID change sends 0 A and
disables the slot. Reconnection alone does not re-enable it: `MdMotor` must
receive a new target and call `motor_enable()`.

Velocity, position, and trajectory targets stop as soon as STATUS_4 becomes
stale and require a new target after position feedback returns. An open-loop
DUTY target maps to current and remains allowed without encoder feedback. A CAN
DUTY target does not make `is_ready` true while the encoder is unavailable,
because the published position and velocity are not valid. A CAN ID change
invalidates the old motor's accumulated encoder position; the new motor needs
two STATUS_4 samples before closed-loop control has velocity again.

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

`max_current_a` is both the DUTY 1.0 scale and a slot-local final current clamp;
the devicetree `absolute-max-current-ma` remains an independent broader guard.
Diagnostics report ages and fresh masks for STATUS, STATUS_4, STATUS_5, and the
selective fault reply. CAN transmit-completion errors are counted separately
from queueing errors and are also included in the aggregate transmit error
count.

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

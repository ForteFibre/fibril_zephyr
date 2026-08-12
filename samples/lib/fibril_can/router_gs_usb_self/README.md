# router_gs_usb_self

A single Fibril RoboMaster Mini V1 acts as both a fibril_can slave (the
router's *self* node) and a USB-to-CAN gateway (via CANnectivity's `gs_usb`
device class). A host on the PC side sees one CAN interface that combines the
self node's traffic with anything else attached to the `fdcan1` downlink.

```
PC (SocketCAN or fibril_can_web) --USB FS--> zephyr_udc0 (&usb)
                                                 ├─ gs_usb subsys
                                                 ↓
                                          fcan_router (uplink)
                                            ├── self node (schema=example_node)
                                            └── downlink: fdcan1
```

The self node reuses the codegen output and application logic from
`samples/lib/fibril_can/example_node` verbatim, so the app-side surface is
identical to that sample. The differences are all in `main.c` (three-step
router+self init, no HAL wrapper around a physical CAN device, no manual
`fcan_poll` on the app thread) and in `src/usbd_setup.c` (minimal USBD next
boilerplate).

## Building

The build needs `fcan_codegen` on the host so it can produce the schema
wrappers before Zephyr compiles the app. See
`fibril_can/zephyr/cmake/fcan_invoke_codegen.cmake` for how the CLI is
resolved.

```
FCAN_CODEGEN=/abs/path/fcan_codegen \
  west build -b fibril_robomaster_miniv1 \
    fibril_zephyr/samples/lib/fibril_can/router_gs_usb_self
```

Twister works the same way:

```
FCAN_CODEGEN=/abs/path/fcan_codegen \
  twister -T fibril_zephyr/samples/lib/fibril_can/router_gs_usb_self
```

## Bringing it up on Linux

1. `west flash` the board and plug USB-C into the host.
2. `dmesg | grep gs_usb` should show a new device (VID/PID match CANnectivity
   defaults, `0x1209:0xca01`).
3. `ip link` lists a fresh `canX`. Bring it up:
   ```
   sudo ip link set up canX type can
   ```
4. Optional -- run `candump canX` to see the raw HEARTBEATs the self node
   emits every second.
5. Start the fibril_can bridge so ROS entities appear:
   ```
   ros2 launch fibril_can_example example_slave.launch.py
   ```
   The bridge auto-discovers the self node; `/example_node16/motor0/...` and
   `/example_node16/imu/...` show up under `ros2 topic list`.
6. If a second board (with `example_node` or any other fibril_can slave) is
   wired onto the fdcan1 downlink, its topics land on the same `canX` in
   parallel.

`fibril_can_web`'s master UI works too -- connect via WebUSB from
`pnpm dev:master`, the self node appears in the Overview tab.

## Watching what the router sees

The board prints one line per second to USART2 (PD5/PD6, 115200 8N1):

```
router_gs_usb_self: alive: state=RUNNING up->dn[0]=42 dn->up[0]=17 \
    drop_send=0 ingress_drops=0
```

- `up->dn[0]` -- frames the router pushed onto fdcan1 (usually commands from
  the master).
- `dn->up[0]` -- frames fdcan1 slaves published to the master.
- `drop_send=` non-zero means the downlink TX mailbox was full when the
  router tried to forward. Bump `CONFIG_CAN_FCAN_ROUTER_TX_TIMEOUT_MS` or
  investigate a stuck slave.
- `ingress_drops=` non-zero means the driver's ingress queue overflowed.
  Bump `CONFIG_CAN_FCAN_ROUTER_MSGQ_DEPTH`.

## Troubleshooting

- **`gs_usb` doesn't enumerate.** Watch USART2 for the sample's own init
  log (`fcan_init OK`, `fcan_router_attach_self OK`, ..., `usbd_enable OK`).
  Missing `usbd_enable OK` points at a USB stack issue; toggle
  `CONFIG_UDC_DRIVER_LOG_LEVEL_DBG=y` and
  `CONFIG_USBD_GS_USB_LOG_LEVEL_DBG=y` in `prj.conf` for verbose output.
- **Provisioning never completes.** The self node's `attach_self` must run
  before the host opens the gs_usb channel. `main.c` sequences that, but if
  you refactor it, keep step 3 (`fcan_router_attach_self`) strictly before
  step 7 (`usbd_setup_enable`). See `fibril_can/docs/09-router.md`.
- **VID/PID collision with a real CANnectivity dongle.** The sample uses
  CANnectivity's default 0x1209:0xca01 on purpose so existing udev rules
  work. Override with `CONFIG_CANNECTIVITY_USB_PID=<hex>` in an extra
  Kconfig fragment if the collision matters.

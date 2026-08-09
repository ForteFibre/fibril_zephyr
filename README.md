# Zephyr Example Application

<a href="https://github.com/zephyrproject-rtos/example-application/actions/workflows/build.yml?query=branch%3Amain">
  <img src="https://github.com/zephyrproject-rtos/example-application/actions/workflows/build.yml/badge.svg?event=push">
</a>
<a href="https://github.com/zephyrproject-rtos/example-application/actions/workflows/docs.yml?query=branch%3Amain">
  <img src="https://github.com/zephyrproject-rtos/example-application/actions/workflows/docs.yml/badge.svg?event=push">
</a>
<a href="https://zephyrproject-rtos.github.io/example-application">
  <img alt="Documentation" src="https://img.shields.io/badge/documentation-3D578C?logo=sphinx&logoColor=white">
</a>
<a href="https://zephyrproject-rtos.github.io/example-application/doxygen">
  <img alt="API Documentation" src="https://img.shields.io/badge/API-documentation-3D578C?logo=c&logoColor=white">
</a>

This repository contains a Zephyr example application. The main purpose of this
repository is to serve as a reference on how to structure Zephyr-based
applications. Some of the features demonstrated in this example are:

- Basic [Zephyr application][app_dev] skeleton
- [Zephyr workspace applications][workspace_app]
- [Zephyr modules][modules]
- [West T2 topology][west_t2]
- [Custom boards][board_porting]
- Custom [devicetree bindings][bindings]
- Out-of-tree [drivers][drivers]
- Out-of-tree libraries
- Example CI configuration (using GitHub Actions)
- Custom [west extension][west_ext]
- Custom [Zephyr runner][runner_ext]
- Doxygen and Sphinx documentation boilerplate

This repository is versioned together with the [Zephyr main tree][zephyr]. This
means that every time that Zephyr is tagged, this repository is tagged as well
with the same version number, and the [manifest](west.yml) entry for `zephyr`
will point to the corresponding Zephyr tag. For example, the `example-application`
v2.6.0 will point to Zephyr v2.6.0. Note that the `main` branch always
points to the development branch of Zephyr, also `main`.

[app_dev]: https://docs.zephyrproject.org/latest/develop/application/index.html
[workspace_app]: https://docs.zephyrproject.org/latest/develop/application/index.html#zephyr-workspace-app
[modules]: https://docs.zephyrproject.org/latest/develop/modules.html
[west_t2]: https://docs.zephyrproject.org/latest/develop/west/workspaces.html#west-t2
[board_porting]: https://docs.zephyrproject.org/latest/guides/porting/board_porting.html
[bindings]: https://docs.zephyrproject.org/latest/guides/dts/bindings.html
[drivers]: https://docs.zephyrproject.org/latest/reference/drivers/index.html
[zephyr]: https://github.com/zephyrproject-rtos/zephyr
[west_ext]: https://docs.zephyrproject.org/latest/develop/west/extensions.html
[runner_ext]: https://docs.zephyrproject.org/latest/develop/modules.html#external-runners

## Drivers

### AMT21x absolute encoder (`cui,amt21`)

Same Sky (formerly CUI Devices) AMT21 series absolute encoders on a half-duplex
RS485 bus. Both the 12-bit and 14-bit resolutions are supported, as are the
single-turn and multi-turn variants and both the 2 Mbps and the adjustable data
rate options. The generic interface lives in `include/drivers/encoder.h` and the
driver-specific diagnostics in `include/drivers/encoder/amt21.h`.

One bus node owns the UART and polls every encoder child node in turn, so each
encoder appears as its own device. Readings are taken from a cached snapshot and
never block on the bus, which makes them usable from a control loop.

#### Wiring requirements

The UART must be able to drive the transceiver in hardware, for example through
the `de-enable` property of an STM32 UART. Toggling driver enable from software
adds latency around the turnaround window and is not viable at 2 Mbps.

Keep `de-deassert-time` small. It is measured in sixteenths of a bit time, and a
large value holds the transceiver enabled past the end of the outgoing stop bit,
straight into the window where the encoder starts replying. That collision
corrupts both frames.

On boards whose transceiver has its receiver permanently enabled, the
transmitted command byte comes back on the receive line. Declare `tx-echo` on the
bus node so the driver discards it; without the property the driver works it out
from the first transaction.

#### Why responses get dropped, and what the driver does about it

At 2 Mbps the encoder starts replying about 3 us after the command byte. That is
far too soon to arm a receiver in software, so the driver enables reception once
at init and never disables it during normal operation. Arming the receiver per
transaction is the single most likely way to lose the first byte of every
response. For the same reason the driver always answers `UART_RX_BUF_REQUEST`:
failing to hand over the next buffer stops reception permanently.

The encoder also needs a gap between consecutive commands, which
`inter-command-delay-us` provides. Sending commands back to back without it is
the other common cause of dropped responses.

Beyond that, a failed transaction is retried up to `max-retries` times, and an
encoder is only declared offline after `offline-threshold` consecutive failures.
Until then the last good reading is kept and reported as stale, so a single
dropped response does not disturb a control loop. Reception is resynchronised
after a truncated or overlong response, and restarted if the UART reports it
stopped. A scan that overruns `poll-interval-us` causes the next scan to be
skipped rather than queued, and a failing encoder never holds up the others.

#### Diagnostics

Three levels, so a production build carries none of the cost:

| Level | Configuration | Cost per encoder |
| --- | --- | --- |
| Aggregate count in `struct encoder_feedback.error_count` | always available | 4 bytes |
| Per-cause counters through `amt21_get_stats()` and `amt21_bus_get_stats()` | `CONFIG_ENCODER_AMT21_STATS` | ~60 bytes, plus 24 per bus |
| Recent failures with the raw bytes received, through `amt21_get_error_log()` | `CONFIG_ENCODER_AMT21_ERROR_LOG_SIZE` | 16 bytes per entry |

The counters are 32-bit and wrap around, so compare successive readings rather
than treating them as totals. Statistics are exposed as driver-specific
functions rather than through the encoder class, because the ways a transaction
can fail are a property of the RS485 protocol and not of encoders in general.
Counters are split between per-encoder and per-bus scopes: a UART overrun
disturbs every transaction on the bus, so attributing it to one encoder would be
misleading.

`CONFIG_ENCODER_AMT21_SHELL` adds an `amt21` command for use on hardware:

```
amt21 list
amt21 read <dev>
amt21 stats <dev>
amt21 errlog <dev>
amt21 bus-stats <bus>
amt21 zero <dev>
amt21 reset <dev>
```

`amt21 errlog` is usually the fastest way to work out why responses are being
dropped, because it keeps the bytes that actually arrived. `debug.conf` enables
the statistics and the error log.

#### Notes

- Polling faster than the internal update rate of the encoder gains nothing:
  that rate is 100 us for 14-bit devices and 25 us for 12-bit devices.
- Node addresses must be multiples of four. The low two bits carry the command,
  which is why up to 64 encoders can share one bus.
- Only single-turn devices can store a zero point; `encoder_set_zero()` returns
  `-ENOTSUP` on a multi-turn device. Both that command and `encoder_reset()`
  make the encoder restart, so it stops answering for about 200 ms.
- The turns counter is not retained across a power cycle.

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the ``example-application`` and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://github.com/zephyrproject-rtos/example-application --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

### Building and running

To build the application, run the following command:

```shell
cd example-application
west build -b $BOARD app
```

where `$BOARD` is the target board.

Note that Zephyr sample boards may be used if an appropriate overlay is
provided (see `app/boards`).

A sample debug configuration is also provided. To apply it, run the following
command:

```shell
west build -b $BOARD app -- -DEXTRA_CONF_FILE=debug.conf
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```

### Testing

To execute Twister integration tests, run the following command:

```shell
west twister -T tests --integration
```

### Documentation

A minimal documentation setup is provided for Doxygen and Sphinx. To build the
documentation first change to the ``doc`` folder:

```shell
cd doc
```

Before continuing, check if you have Doxygen installed. It is recommended to
use the same Doxygen version used in [CI](.github/workflows/docs.yml). To
install Sphinx, make sure you have a Python installation in place and run:

```shell
pip install -r requirements.txt
```

API documentation (Doxygen) can be built using the following command:

```shell
doxygen
```

The output will be stored in the ``_build_doxygen`` folder. Similarly, the
Sphinx documentation (HTML) can be built using the following command:

```shell
make html
```

The output will be stored in the ``_build_sphinx`` folder. You may check for
other output formats other than HTML by running ``make help``.

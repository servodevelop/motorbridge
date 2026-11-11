# FashionStar API and Mode Reference

> 中文：[FASHIONSTAR_API.zh-CN.md](FASHIONSTAR_API.zh-CN.md)

Practical reference for FashionStar CAN and UART/RS-485 control in motorbridge.

## 1) Basics

- Vendor: `fashionstar`. Model `fashionstar` selects CAN; model `uart` selects native UART/RS-485. The following CAN settings apply to the default model.
- Classic CAN, standard 11-bit arbitration ID **0x321** for both directions. `motor_id` is the ServoID in the payload, **0..254**; 255 is not accepted as an individual motor.
- `feedback_id` is fixed at `0x321` (801), independent of ServoID.
- Transports: `auto`, `socketcan`, `mcu-serial`. The MCU firmware must implement motorbridge's existing serial protocol. The official UC-06 web tool uses a different serial wrapper.
- Match the bus bitrate to the servo's configured value (documented default: 500 kbit/s). Linux uses a preconfigured `can0`; PCAN channels can specify `can0@500000`.

## 2) Supported `motor_cli` Modes

| Mode | Behavior |
|---|---|
| `scan` | Read model register `0x01`; default range 0..254 |
| `ping` | Read the model code, without moving |
| `status`, `read` | Query position, status, temperature and current; estimate velocity from consecutive position samples |
| `enable`, `stop` | Hold the current position (`0x7B=0x11`) |
| `disable` | Release output (`0x7B=0x10`) |
| `zero`, `set-zero` | Release output, wait 50 ms, then set the current position as zero |
| `pos`, `pos-vel` | Move to an absolute target with a positive speed limit (native mode 2) |
| `read-param`, `write-param` | Use existing raw integer register interfaces |

Continuous velocity (`VEL`), MIT/quantitative torque and `FORCE_POS` are **currently unsupported** by this adapter; these calls return errors. Torque enable/disable is an output switch, not a quantitative torque command.

The current protocol has no native continuous-velocity command. For extended rotation, use `POS_VEL` with a distant absolute target within ±1024 turns and a positive speed limit. The servo stops when it reaches that target.

## 3) Arguments and Units

| Argument | Default | Meaning |
|---|---|---|
| `--motor-id` | 1 | ServoID, 0..254 |
| `--feedback-id` | `0x321` | Fixed CAN arbitration ID |
| `--pos` | 0 | Absolute position in rad, within ±1024 turns |
| `--vlim` | 1 | Positive speed limit in rad/s; rounded to 1..65535 units of 0.1°/s |
| `--timeout-ms` | 500 | Reply timeout; protected-register unlock delays are additional |
| `--param-id` | `0x14` (read), `0` (write) | Register address |
| `--param-type` | `u32` | `u16` or `u32`, always raw integer bits |
| `--param-value` | required for writing | Unsigned raw value, decimal or `0x` hex |
| `--loop`, `--dt-ms` | 1, 20 | Command count and interval; repeating POS_VEL still has an angle target |

## 4) Common Command Examples

```bash
motor_cli --vendor fashionstar --channel can0 --mode scan --start-id 0 --end-id 16
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode status
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode pos-vel --pos 1.5707963 --vlim 1
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode stop
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode read-param --param-id 0x81 --param-type u32
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --set-motor-id 2
# Power-cycle the servo, then connect with the new ServoID:
motor_cli --vendor fashionstar --channel can0 --motor-id 2 --mode ping
```

## 5) Registers, State and Configuration

The adapter sends position mode `0x52=2`, signed target `0x53/0x54`, speed limit `0x58`, then execute `0x59=1`. Target position uses signed 32-bit units of 0.1°. Configured power and acceleration are preserved.

The existing u32 register interface combines low/high registers for addresses `0x03`, `0x15`, `0x50`, `0x53`; other supported addresses carry 16-bit raw values. Signed values use the two's-complement representation of their register width. Address permissions and documented value ranges are checked before transmission. PID registers `0xC8..0xD5` are automatically unlocked, accessed and relocked; unlock settling takes 400 ms before waiting for a reply.

Unified state contains position, status, estimated velocity (`vel`, rad/s), current (`torq`, A) and servo temperature (`t_mos`, °C). Velocity is the average between consecutive complete position samples; the first sample, or the first sample after a failed query or an origin/turn-count reset, has no velocity estimate. Current register `0x11` uses mA in the official CAN SDK and web tool; dividing by 1000 gives A. As with MyActuator, `torq` carries current, not torque in N·m. Temperature uses the official CAN tool's default NTC conversion (B=3435, T0=298.15 K, ADC full scale 4096); ADC values outside 1..4095 are invalid. The single servo temperature follows the existing single-temperature vendor mapping to `t_mos`; `t_rotor` remains unavailable. Unavailable values are `NaN` in the C ABI/Python/C++ and `null` in WS JSON. No state is reported until a complete position/status/temperature/current query succeeds. `stop` holds position; `disable` releases output.

**Configuration changes require a power cycle.** A successful write means that the request was sent. ID register `0x81` does not retarget the live handle: power-cycle, recreate the controller/motor with the new ID, then verify. Bitrate changes likewise require a power cycle and a matching host bitrate. Runtime target/execute/stop commands do not require a power cycle. There is no separate defined store command in this adapter.

Python SDK/CLI examples: [FASHIONSTAR_API.md](../bindings/python/FASHIONSTAR_API.md). WS uses existing `set_target`, `pos_vel`, `state_once`, `get_register_u32`, `write_register_u32`, `scan`, `set_id`, `verify`; `set_id` returns `verified:false` and `requires_power_cycle:true`. The default `param_stream` reads voltage (`0x10`), current (`0x11`), power (`0x12`), temperature (`0x13`), status (`0x14`), multi-turn position (`0x15/0x16`), turn count (`0x17`) and single-turn angle (`0x18`) as raw register values.

## 6) Native UART/RS-485

Use `--model uart --transport uart --serial-port /dev/ttyUSB0 --serial-baud 115200` with `--vendor fashionstar`. Select the baud rate configured on the servo; the UART factory default is 115200, while CLI serial options retain their existing defaults. The link uses 8-N-1 without flow control and an adapter that handles half-duplex direction.

`motor_id` remains ServoID 0..254. Keep `feedback_id=0x321` as an interface placeholder; UART has no CAN arbitration ID or separate feedback address. Existing scan, ID update, control, state and parameter-stream operations apply.

Parameter APIs retain the existing FashionStar IDs, translated inside the driver: model `0x01`, firmware `0x02`, serial `0x03/0x04`, state `0x10..0x18`, configuration `0x80..0x89`, `0x8B`, `0x8C`, `0x8E..0x93`. These are motorbridge parameter IDs, not native UART addresses. For example, `0x81` maps to UART ID parameter 34. The baud index at `0x82` uses UART values 1..8 (9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000). Unsupported CAN-only parameters return errors. Configuration requires a power cycle.

Position control uses UART command `0x0F` with a speed limit, no acceleration/deceleration ramp and power 0 (configured limit). Stop/enable hold position; disable releases output; zero sets the current position as the origin. Feedback provides position, status, current in A and temperature in °C; velocity is estimated from consecutive samples. VEL, MIT, quantitative torque and FORCE_POS remain unsupported.

```bash
motor_cli --vendor fashionstar --model uart --transport uart --serial-port /dev/ttyUSB0 --serial-baud 115200 --motor-id 0 --mode status
```

WS uses the same operations and parameter IDs. Set `model:"uart"`, `transport:"uart"`, `serial_port` and `serial_baud:115200`; keep `feedback_id:801` or omit it.

## 7) Protocol References

- [Official CAN protocol](https://fashionstar.com.hk/wiki/protocols/servo/can-bus/) ([中文](https://fashionstar.com.cn/wiki/protocols/servo/can-bus/))
- [Official SDKs](https://fashionstar.com.cn/wiki/sdk/servo/)
- [Official CAN web configuration tool](https://fashionstar.com.cn/wiki/software/servo/canbus-web-config-tool/)
- [Official UART/RS-485 protocol](https://fashionstar.com.cn/wiki/protocols/servo/uart-rs485/)
- [Official UART web controller](https://fashionstar.com.cn/wiki/software/servo/data/web-controller/)

# FashionStar Python API

> 中文：[FASHIONSTAR_API.zh-CN.md](FASHIONSTAR_API.zh-CN.md)

Use the existing `Controller`/`Motor` API for FashionStar CAN and UART/RS-485 servos. Protocol coverage, units, PCAN channel setup and Rust CLI commands are in the [FashionStar reference](../../motor_cli/FASHIONSTAR_API.md).

## SDK

```python
from motorbridge import Controller

ctrl = Controller("can0")
try:
    motor = ctrl.add_fashionstar_motor(0, 0x321, "fashionstar")
    try:
        motor.send_pos_vel(1.5707963, 1.0)  # rad, rad/s
        motor.request_feedback()
        print(motor.get_state_snapshot())
    finally:
        motor.close()
finally:
    try:
        ctrl.close_bus()
    finally:
        ctrl.close()
```

This cleanup closes the connection. The existing `with Controller(...)` also calls `shutdown()` on exit, releasing servo output. Position queried after `request_feedback()` may still be in transit; it does not confirm arrival.

- `add_fashionstar_motor(motor_id, feedback_id, model)`: ServoID 0..254; `fashionstar` selects CAN and `uart` selects UART/RS-485.
- Use existing `enable()` (stop and hold), `disable()`, `set_zero_position()`, `send_pos_vel(pos, vlim)`, `get_register_u32(rid, timeout_ms)`, `write_register_u32(rid, value)`.
- `ensure_mode(Mode.POS_VEL)` is available. VEL, MIT/quantitative torque and FORCE_POS are currently unsupported; operations without a protocol mapping return errors.
- `vel` is estimated from consecutive position samples; `torq` carries current in A (`0x11` in mA divided by 1000), following MyActuator's mapping; `t_mos` is servo temperature in °C using the official CAN tool's default conversion. The first velocity sample, invalid temperature readings and rotor temperature are `NaN`. `torq` does not represent torque in N·m.
- CAN and UART ID (`0x81`) and baud (`0x82`) changes take effect immediately and persist across power cycles. For CAN, wait 120 ms, then reconnect with the new ID and matching host bitrate to verify. Other CAN configuration follows the power-cycle procedure. The live handle retains its old settings.

## Python CLI

```bash
motorbridge scan --vendor fashionstar --channel can0 --start-id 0 --end-id 16
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode ping
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode pos-vel --pos 1.5707963 --vlim 1
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode read-param --param-id 0x81 --param-type u32
motorbridge id-set --vendor fashionstar --channel can0 --motor-id 0 --new-motor-id 2
```

Python CLI modes: `enable`, `disable`, `pos-vel`, `zero`, `set-zero`, `ping`, `read-param`, `write-param`. Use existing `scan` and `id-set` subcommands; `run --set-motor-id` reuses the ID update flow. CLI cleanup closes the connection without an additional disable command. CAN transports: `auto|socketcan|mcu-serial|uc06`; UC-06 usage is shown below.

## Native UART/RS-485

Use `Controller.from_uart()` with `model="uart"` and the existing `feedback_id=0x321` placeholder; the factory defaults to 115200 baud. Motor methods, units and parameter IDs remain the same: ID configuration uses `0x81`, model query uses `0x01`. The driver translates to native UART addresses; see [UART parameters](../../motor_cli/FASHIONSTAR_API.md#6-native-uartrs-485).

Use the SDK example above with these controller and motor constructors; keep the same cleanup:

```python
ctrl = Controller.from_uart("/dev/ttyUSB0", 115200)
motor = ctrl.add_fashionstar_motor(0, 0x321, "uart")
```

```bash
motorbridge scan --vendor fashionstar --model uart --transport uart --serial-port /dev/ttyUSB0 --serial-baud 115200 --start-id 0 --end-id 16
motorbridge run --vendor fashionstar --model uart --transport uart --serial-port /dev/ttyUSB0 --serial-baud 115200 --motor-id 0 --mode ping
```

## UC-06 USB-to-CAN

Use `Controller.from_uc06(serial_port, can_bitrate)` with model `fashionstar`. The CAN bitrate defaults to 1000000 bit/s; the USB serial rate is fixed at 1000000 baud. Choose the bitrate configured on the servo. Existing USB CAN controllers and native UART remain separate entrypoints.

```python
ctrl = Controller.from_uc06("/dev/ttyUSB0", 1000000)
motor = ctrl.add_fashionstar_motor(0, 0x321, "fashionstar")
```

Keep the SDK cleanup shown above. All existing FashionStar CAN motor methods apply.

```bash
motorbridge scan --vendor fashionstar --transport uc06 --serial-port /dev/ttyUSB0 --can-bitrate 1000000 --start-id 0 --end-id 10
```

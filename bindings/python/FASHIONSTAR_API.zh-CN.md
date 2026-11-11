# FashionStar Python API

> English: [FASHIONSTAR_API.md](FASHIONSTAR_API.md)

通过现有 `Controller`/`Motor` 接口接入 FashionStar CAN 与 UART/RS-485 舵机。协议范围、单位、PCAN 通道配置和 Rust CLI 命令见 [FashionStar 参考](../../motor_cli/FASHIONSTAR_API.zh-CN.md)。

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

上述退出方式只关闭连接。现有 `with Controller(...)` 在退出时还会执行 `shutdown()`，使舵机卸力。`request_feedback()` 后读到的位置可能仍在运动途中，不表示已到达目标。

- `add_fashionstar_motor(motor_id, feedback_id, model)`：ServoID 范围 0..254；`fashionstar` 选择 CAN，`uart` 选择 UART/RS-485。
- 使用已有 `enable()`（停止并保持）、`disable()`、`set_zero_position()`、`send_pos_vel(pos, vlim)`、`get_register_u32(rid, timeout_ms)`、`write_register_u32(rid, value)`。
- `ensure_mode(Mode.POS_VEL)` 可用。VEL、MIT/定量力矩与 FORCE_POS 暂不支持；其他尚无协议映射的操作返回错误。
- `vel` 由相邻位置采样估算；`torq` 沿用 MyActuator 的映射，表示电流 A（`0x11` 的 mA 值除以 1000）；`t_mos` 为按官方 CAN 工具默认公式换算的舵机温度（℃）。首次速度采样、无效温度与转子温度为 `NaN`。`torq` 不表示 N·m 力矩。
- CAN 配置写入需要重新上电生效；UART 的 ID 和波特率修改立即生效并在断电后保留。写 `0x81` 或 `0x82` 后，关闭旧连接，按新配置创建连接并读取验证；CAN 舵机需先重新上电，当前句柄不会自动切换 ID。

## Python CLI

```bash
motorbridge scan --vendor fashionstar --channel can0 --start-id 0 --end-id 16
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode ping
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode pos-vel --pos 1.5707963 --vlim 1
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode read-param --param-id 0x81 --param-type u32
motorbridge id-set --vendor fashionstar --channel can0 --motor-id 0 --new-motor-id 2
```

Python CLI 模式：`enable`、`disable`、`pos-vel`、`zero`、`set-zero`、`ping`、`read-param`、`write-param`；扫描和改 ID 使用已有 `scan`、`id-set` 子命令。`run --set-motor-id` 同样复用改 ID 流程。CLI 退出只关闭连接，不额外卸力。链路为 `auto|socketcan|mcu-serial`；UC-06 串口协议不能直接混用。

## 原生 UART/RS-485

使用 `Controller.from_uart()`、`model="uart"` 和现有的 `feedback_id=0x321` 占位值；工厂方法默认波特率 115200。电机方法、单位和参数编号保持一致：ID 配置使用 `0x81`，型号查询使用 `0x01`。驱动内部转换为 UART 原生地址，详见 [UART 参数说明](../../motor_cli/FASHIONSTAR_API.zh-CN.md#6原生-uartrs-485)。

沿用上方 SDK 示例的关闭流程，将连接和电机的创建替换为：

```python
ctrl = Controller.from_uart("/dev/ttyUSB0", 115200)
motor = ctrl.add_fashionstar_motor(0, 0x321, "uart")
```

```bash
motorbridge scan --vendor fashionstar --model uart --transport uart --serial-port /dev/ttyUSB0 --serial-baud 115200 --start-id 0 --end-id 16
motorbridge run --vendor fashionstar --model uart --transport uart --serial-port /dev/ttyUSB0 --serial-baud 115200 --motor-id 0 --mode ping
```

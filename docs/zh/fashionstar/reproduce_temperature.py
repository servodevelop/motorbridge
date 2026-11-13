"""Reproduce FashionStar CAN temperature anomalies through motorbridge / UC-06."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
os.environ.setdefault('MOTORBRIDGE_LIB', str(ROOT / 'target/release/libmotor_abi.so'))
sys.path.insert(0, str(ROOT / 'bindings/python/src'))
from motorbridge import Controller


def temperature(raw):
    if not 0 < raw < 4096:
        return None
    return 1 / (1 / 298.15 + math.log(raw / (4096 - raw)) / 3435) - 273.15


def angle(motor):
    value = motor.get_register_u32(0x15, 500)
    return (value - (1 << 32) if value & (1 << 31) else value) / 10


def reproduce(port, motor_id, moves):
    controller = Controller.from_uc06(port, 1_000_000)
    motor = None
    saved = {}
    result = dict(reproduced=False, motor_id=motor_id, moves=0, reads=0)
    started = time.monotonic()
    try:
        motor = controller.add_fashionstar_motor(motor_id, 0, 'fashionstar')
        saved = {r: motor.get_register_u32(r, 500) for r in (0x52, 0x53, 0x56, 0x57, 0x58)}
        for r in (0x56, 0x57):
            motor.write_register_u32(r, 500)
            assert motor.get_register_u32(r, 500) == 500
        previous = None
        for n in range(moves):
            target = 360.1 if angle(motor) < 180 else 0.1
            motor.send_pos_vel(math.radians(target), math.radians(90))
            deadline = time.monotonic() + 5.4
            result['moves'] = n + 1
            while time.monotonic() < deadline:
                raw = motor.get_register_u32(0x13, 500)
                value = temperature(raw)
                result['reads'] += 1
                if value is None or value >= 65:
                    result.update(reproduced=True, register='0x13', raw=raw,
                                  temperature_c=value, previous_raw=previous,
                                  position_deg=angle(motor), target_deg=target)
                    result['rechecks'] = []
                    for _ in range(3):
                        time.sleep(0.05)
                        value = motor.get_register_u32(0x13, 500)
                        result['rechecks'].append(dict(raw=value, temperature_c=temperature(value)))
                    return result
                previous = raw
            if abs(angle(motor) - target) > 1.5:
                raise RuntimeError('Position command did not reach its target')
        return result
    finally:
        try:
            if motor is not None:
                motor.disable()
                time.sleep(0.05)
                for register, value in saved.items():
                    motor.write_register_u32(register, value)
                assert all(motor.get_register_u32(r, 500) == v for r, v in saved.items())
        finally:
            if motor is not None:
                motor.close()
            controller.close_bus()
            controller.close()
            result['elapsed_s'] = round(time.monotonic() - started, 3)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--motor-id', type=int, default=1)
    parser.add_argument('--moves', type=int, default=30)
    args = parser.parse_args()
    if args.moves < 1:
        parser.error('--moves must be positive')
    result = reproduce(args.port, args.motor_id, args.moves)
    print(json.dumps(result, ensure_ascii=False), flush=True)
    sys.exit(0 if result['reproduced'] else 1)

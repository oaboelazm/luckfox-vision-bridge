from __future__ import annotations

import argparse
import time

import serial


def parse_args():
    p = argparse.ArgumentParser(description="UART loopback tester")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--count", type=int, default=1000)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ser = serial.Serial(args.port, args.baud, timeout=0.05)

    ok = 0
    t0 = time.time()
    for i in range(args.count):
        payload = f"PING:{i}\n".encode("ascii")
        ser.write(payload)
        got = ser.readline()
        if got == payload:
            ok += 1

    dt = time.time() - t0
    print(f"ok={ok}/{args.count} rate={args.count / max(dt, 1e-6):.1f} msg/s")
    ser.close()
    return 0 if ok == args.count else 1


if __name__ == "__main__":
    raise SystemExit(main())

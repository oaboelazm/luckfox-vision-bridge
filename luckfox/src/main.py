from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import signal
import time

from bridge.uart_bridge import bridge_worker
from vision.apriltag_worker import apriltag_worker
from vision.plant_rknn_worker import plant_worker


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Luckfox vision bridge runtime")
    parser.add_argument("--config", required=True, help="Path to JSON config")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)

    ctx = mp.get_context("spawn")
    manager = ctx.Manager()
    shared = manager.dict()
    shared["apriltag"] = None
    shared["plant"] = None
    shared["status"] = None

    stop_flag = ctx.Event()

    workers = [
        ctx.Process(target=apriltag_worker, args=(config, shared, stop_flag), name="vision_apriltag"),
        ctx.Process(target=bridge_worker, args=(config, shared, stop_flag), name="comm_bridge_uart"),
    ]

    if config.get("plant", {}).get("enabled", False):
        workers.append(ctx.Process(target=plant_worker, args=(config, shared, stop_flag), name="vision_plant_rknn"))

    def _shutdown(_sig, _frm):
        stop_flag.set()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    for p in workers:
        p.start()

    try:
        while any(p.is_alive() for p in workers):
            time.sleep(0.2)
    finally:
        stop_flag.set()
        for p in workers:
            p.join(timeout=2.0)
            if p.is_alive():
                p.terminate()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

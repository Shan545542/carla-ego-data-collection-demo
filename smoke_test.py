import argparse
import os
import re
from pathlib import Path

import carla


def default_windows_host() -> str:
    if "CARLA_HOST" in os.environ:
        return os.environ["CARLA_HOST"]

    resolv_conf = Path("/etc/resolv.conf")
    if resolv_conf.exists():
        match = re.search(r"^nameserver\s+(\S+)", resolv_conf.read_text(), re.M)
        if match:
            return match.group(1)
    return "127.0.0.1"


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal CARLA connection test.")
    parser.add_argument("--host", default=default_windows_host())
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--timeout", default=10.0, type=float)
    args = parser.parse_args()

    print(f"Connecting to CARLA at {args.host}:{args.port} ...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world = client.get_world()
    print("Connection OK")
    print(f"Map: {world.get_map().name}")
    print(f"Actors: {len(world.get_actors())}")


if __name__ == "__main__":
    main()

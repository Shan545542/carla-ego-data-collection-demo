import argparse
import os
import random
import re
import time
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
    parser = argparse.ArgumentParser(description="Spawn one ego vehicle and enable autopilot.")
    parser.add_argument("--host", default=default_windows_host())
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--duration", default=30, type=int)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter("vehicle.*")
    vehicle_bp = random.choice(vehicle_blueprints)

    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No vehicle spawn points found in this map.")

    vehicle = None
    try:
        for spawn_point in random.sample(spawn_points, len(spawn_points)):
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
            if vehicle is not None:
                break

        if vehicle is None:
            raise RuntimeError("Could not spawn vehicle. Try restarting the CARLA world.")

        vehicle.set_autopilot(True)
        print(f"Spawned {vehicle.type_id} at actor id {vehicle.id}")
        print(f"Autopilot enabled for {args.duration} seconds.")

        spectator = world.get_spectator()
        for _ in range(args.duration):
            transform = vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location + carla.Location(z=35),
                    carla.Rotation(pitch=-90),
                )
            )
            time.sleep(1)

    finally:
        if vehicle is not None:
            print("Destroying vehicle.")
            vehicle.destroy()


if __name__ == "__main__":
    main()

import argparse
import csv
import json
import math
import os
import queue
import random
import re
import time
from datetime import datetime
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


def make_run_dir(output_root: Path) -> Path:
    run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir = output_root / run_id
    (run_dir / "rgb").mkdir(parents=True, exist_ok=False)
    return run_dir


def choose_vehicle_blueprint(world: carla.World, pattern: str) -> carla.ActorBlueprint:
    blueprints = world.get_blueprint_library().filter(pattern)
    if not blueprints:
        raise RuntimeError(f"No vehicle blueprint matched pattern: {pattern}")

    blueprint = random.choice(blueprints)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", "ego")
    if blueprint.has_attribute("color"):
        color = random.choice(blueprint.get_attribute("color").recommended_values)
        blueprint.set_attribute("color", color)
    return blueprint


def spawn_vehicle(world: carla.World, blueprint: carla.ActorBlueprint, spawn_index: int) -> carla.Vehicle:
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No vehicle spawn points found in this map.")

    if spawn_index >= 0:
        ordered_points = [spawn_points[spawn_index % len(spawn_points)]]
    else:
        ordered_points = random.sample(spawn_points, len(spawn_points))

    for spawn_point in ordered_points:
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            return vehicle

    raise RuntimeError("Could not spawn ego vehicle. Try another spawn point or restart the world.")


def attach_rgb_camera(
    world: carla.World,
    vehicle: carla.Vehicle,
    width: int,
    height: int,
    fov: float,
) -> tuple[carla.Sensor, queue.Queue]:
    camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(width))
    camera_bp.set_attribute("image_size_y", str(height))
    camera_bp.set_attribute("fov", str(fov))
    camera_bp.set_attribute("sensor_tick", "0.0")

    camera_transform = carla.Transform(
        carla.Location(x=1.6, z=1.7),
        carla.Rotation(pitch=-5.0),
    )
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    image_queue: queue.Queue = queue.Queue()
    camera.listen(image_queue.put)
    return camera, image_queue


def get_sensor_data(sensor_queue: queue.Queue, frame: int, timeout: float = 2.0) -> carla.Image:
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = max(0.01, deadline - time.time())
        data = sensor_queue.get(timeout=remaining)
        if data.frame >= frame:
            return data
    raise TimeoutError(f"Timed out waiting for camera frame {frame}.")


def follow_vehicle_with_spectator(world: carla.World, vehicle: carla.Vehicle) -> None:
    transform = vehicle.get_transform()
    forward = transform.get_forward_vector()
    location = transform.location - forward * 8.0 + carla.Location(z=4.0)
    rotation = carla.Rotation(pitch=-15.0, yaw=transform.rotation.yaw)
    world.get_spectator().set_transform(carla.Transform(location, rotation))


def speed_mps(vehicle: carla.Vehicle) -> float:
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


def write_config(run_dir: Path, args: argparse.Namespace, map_name: str) -> None:
    config = vars(args).copy()
    config["map"] = map_name
    config["created_at"] = datetime.now().isoformat(timespec="seconds")
    with (run_dir / "config.json").open("w", encoding="utf-8") as file:
        json.dump(config, file, indent=2)


def plot_trajectory(csv_path: Path, output_path: Path) -> None:
    import matplotlib.pyplot as plt

    xs: list[float] = []
    ys: list[float] = []
    speeds: list[float] = []
    with csv_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
            speeds.append(float(row["speed_mps"]))

    if not xs:
        return

    fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
    scatter = ax.scatter(xs, ys, c=speeds, s=10, cmap="viridis")
    ax.plot(xs, ys, linewidth=1.0, alpha=0.45)
    ax.scatter(xs[0], ys[0], marker="o", s=70, label="start")
    ax.scatter(xs[-1], ys[-1], marker="x", s=80, label="end")
    ax.set_title("Ego Vehicle Trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.colorbar(scatter, ax=ax, label="speed (m/s)")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect front RGB images and ego vehicle states in CARLA 0.9.15."
    )
    parser.add_argument("--host", default=default_windows_host())
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--tm-port", default=8000, type=int)
    parser.add_argument("--timeout", default=20.0, type=float)
    parser.add_argument("--duration", default=30.0, type=float, help="Run duration in seconds.")
    parser.add_argument("--fps", default=10.0, type=float, help="Simulation FPS in synchronous mode.")
    parser.add_argument("--town", default="", help="Optional map name, for example Town10HD_Opt.")
    parser.add_argument("--vehicle-filter", default="vehicle.*")
    parser.add_argument("--spawn-index", default=-1, type=int, help="-1 means random spawn point.")
    parser.add_argument("--width", default=1280, type=int)
    parser.add_argument("--height", default=720, type=int)
    parser.add_argument("--fov", default=90.0, type=float)
    parser.add_argument("--output-root", default="outputs")
    parser.add_argument("--seed", default=42, type=int)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    random.seed(args.seed)

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    world = client.get_world()
    if args.town and not world.get_map().name.endswith(args.town):
        print(f"Loading map {args.town} ...")
        world = client.load_world(args.town)

    output_root = Path(args.output_root)
    run_dir = make_run_dir(output_root)
    write_config(run_dir, args, world.get_map().name)

    original_settings = world.get_settings()
    traffic_manager = client.get_trafficmanager(args.tm_port)
    actors: list[carla.Actor] = []

    try:
        fixed_delta_seconds = 1.0 / args.fps
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = fixed_delta_seconds
        if hasattr(settings, "substepping"):
            settings.substepping = True
            settings.max_substep_delta_time = max(0.01, fixed_delta_seconds / 10.0)
            settings.max_substeps = 10
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(args.seed)

        vehicle_bp = choose_vehicle_blueprint(world, args.vehicle_filter)
        vehicle = spawn_vehicle(world, vehicle_bp, args.spawn_index)
        actors.append(vehicle)
        vehicle.set_autopilot(True, traffic_manager.get_port())

        camera, image_queue = attach_rgb_camera(world, vehicle, args.width, args.height, args.fov)
        actors.append(camera)

        csv_path = run_dir / "vehicle_state.csv"
        total_frames = max(1, int(args.duration * args.fps))

        print(f"Output: {run_dir}")
        print(f"Map: {world.get_map().name}")
        print(f"Ego vehicle: {vehicle.type_id} (actor id {vehicle.id})")
        print(f"Collecting {total_frames} frames at {args.fps:g} FPS ...")

        with csv_path.open("w", encoding="utf-8", newline="") as file:
            fieldnames = [
                "step",
                "frame",
                "timestamp",
                "image_path",
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
                "vx",
                "vy",
                "vz",
                "speed_mps",
                "throttle",
                "steer",
                "brake",
                "hand_brake",
                "reverse",
                "gear",
            ]
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()

            for step in range(total_frames):
                frame = world.tick()
                image = get_sensor_data(image_queue, frame)
                follow_vehicle_with_spectator(world, vehicle)

                transform = vehicle.get_transform()
                velocity = vehicle.get_velocity()
                control = vehicle.get_control()
                image_path = run_dir / "rgb" / f"{step:06d}.png"
                image.save_to_disk(str(image_path))

                writer.writerow(
                    {
                        "step": step,
                        "frame": frame,
                        "timestamp": image.timestamp,
                        "image_path": str(image_path.relative_to(run_dir)),
                        "x": transform.location.x,
                        "y": transform.location.y,
                        "z": transform.location.z,
                        "roll": transform.rotation.roll,
                        "pitch": transform.rotation.pitch,
                        "yaw": transform.rotation.yaw,
                        "vx": velocity.x,
                        "vy": velocity.y,
                        "vz": velocity.z,
                        "speed_mps": speed_mps(vehicle),
                        "throttle": control.throttle,
                        "steer": control.steer,
                        "brake": control.brake,
                        "hand_brake": control.hand_brake,
                        "reverse": control.reverse,
                        "gear": control.gear,
                    }
                )

                if (step + 1) % max(1, int(args.fps)) == 0 or step == total_frames - 1:
                    print(f"Saved {step + 1}/{total_frames} frames")

        plot_trajectory(csv_path, run_dir / "trajectory.png")
        print("Done.")
        print(f"RGB images: {run_dir / 'rgb'}")
        print(f"Vehicle states: {csv_path}")
        print(f"Trajectory plot: {run_dir / 'trajectory.png'}")

    finally:
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        for actor in reversed(actors):
            if actor.is_alive:
                actor.destroy()


if __name__ == "__main__":
    main()

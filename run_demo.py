import argparse
import contextlib
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


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def default_windows_host() -> str:
    if "CARLA_HOST" in os.environ:
        return os.environ["CARLA_HOST"]

    resolv_conf = Path("/etc/resolv.conf")
    if resolv_conf.exists():
        match = re.search(r"^nameserver\s+(\S+)", resolv_conf.read_text(), re.M)
        if match:
            return match.group(1)
    return "127.0.0.1"


def make_run_dir(output_root: Path, include_semantic: bool, include_lidar: bool) -> Path:
    run_id = datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir = output_root / run_id
    (run_dir / "rgb").mkdir(parents=True, exist_ok=False)
    if include_semantic:
        (run_dir / "semantic").mkdir(parents=True, exist_ok=False)
    if include_lidar:
        (run_dir / "lidar").mkdir(parents=True, exist_ok=False)
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


def attach_semantic_camera(
    world: carla.World,
    vehicle: carla.Vehicle,
    width: int,
    height: int,
    fov: float,
) -> tuple[carla.Sensor, queue.Queue]:
    camera_bp = world.get_blueprint_library().find("sensor.camera.semantic_segmentation")
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


def attach_lidar(
    world: carla.World,
    vehicle: carla.Vehicle,
    fps: float,
    channels: int,
    points_per_second: int,
    lidar_range: float,
) -> tuple[carla.Sensor, queue.Queue]:
    lidar_bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("channels", str(channels))
    lidar_bp.set_attribute("points_per_second", str(points_per_second))
    lidar_bp.set_attribute("range", str(lidar_range))
    lidar_bp.set_attribute("rotation_frequency", str(fps))
    lidar_bp.set_attribute("upper_fov", "10.0")
    lidar_bp.set_attribute("lower_fov", "-30.0")
    lidar_bp.set_attribute("sensor_tick", "0.0")

    lidar_transform = carla.Transform(
        carla.Location(x=0.0, z=2.4),
        carla.Rotation(),
    )
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

    lidar_queue: queue.Queue = queue.Queue()
    lidar.listen(lidar_queue.put)
    return lidar, lidar_queue


def get_sensor_data(sensor_queue: queue.Queue, frame: int, timeout: float = 2.0):
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


def build_route(
    world_map: carla.Map,
    start_location: carla.Location,
    route_length: float,
    spacing: float,
) -> list[carla.Waypoint]:
    start_waypoint = world_map.get_waypoint(
        start_location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    route = [start_waypoint]
    current = start_waypoint
    distance = 0.0

    while distance < route_length:
        next_waypoints = current.next(spacing)
        if not next_waypoints:
            break
        current = next_waypoints[0]
        route.append(current)
        distance += spacing

    if len(route) < 2:
        raise RuntimeError("Could not build a route from the current spawn point.")
    return route


def write_route_csv(route: list[carla.Waypoint], output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=["index", "x", "y", "z", "yaw"])
        writer.writeheader()
        for index, waypoint in enumerate(route):
            transform = waypoint.transform
            writer.writerow(
                {
                    "index": index,
                    "x": transform.location.x,
                    "y": transform.location.y,
                    "z": transform.location.z,
                    "yaw": transform.rotation.yaw,
                }
            )


def nearest_route_index(location: carla.Location, route: list[carla.Waypoint]) -> int:
    best_index = 0
    best_distance_sq = float("inf")
    for index, waypoint in enumerate(route):
        route_location = waypoint.transform.location
        dx = location.x - route_location.x
        dy = location.y - route_location.y
        distance_sq = dx * dx + dy * dy
        if distance_sq < best_distance_sq:
            best_index = index
            best_distance_sq = distance_sq
    return best_index


def find_lookahead_index(
    transform: carla.Transform,
    route: list[carla.Waypoint],
    nearest_index: int,
    lookahead_distance: float,
) -> int:
    vehicle_location = transform.location
    yaw = math.radians(transform.rotation.yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    for index in range(nearest_index, len(route)):
        target_location = route[index].transform.location
        dx = target_location.x - vehicle_location.x
        dy = target_location.y - vehicle_location.y
        local_x = cos_yaw * dx + sin_yaw * dy
        distance = math.sqrt(dx * dx + dy * dy)
        if local_x > 0.0 and distance >= lookahead_distance:
            return index
    return len(route) - 1


def signed_cross_track_error(location: carla.Location, route: list[carla.Waypoint], index: int) -> float:
    segment_start = route[min(index, len(route) - 2)].transform.location
    segment_end = route[min(index + 1, len(route) - 1)].transform.location

    vx = segment_end.x - segment_start.x
    vy = segment_end.y - segment_start.y
    wx = location.x - segment_start.x
    wy = location.y - segment_start.y
    segment_norm = math.sqrt(vx * vx + vy * vy)
    if segment_norm < 1e-6:
        return 0.0
    return (vx * wy - vy * wx) / segment_norm


def pure_pursuit_steer(
    transform: carla.Transform,
    target_location: carla.Location,
    lookahead_distance: float,
    wheelbase: float,
    max_steer_angle_deg: float,
) -> tuple[float, float]:
    yaw = math.radians(transform.rotation.yaw)
    dx = target_location.x - transform.location.x
    dy = target_location.y - transform.location.y
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy

    lookahead = max(lookahead_distance, 1e-3)
    curvature = 2.0 * local_y / (lookahead * lookahead)
    steering_angle_rad = math.atan(wheelbase * curvature)
    max_steer_rad = math.radians(max_steer_angle_deg)
    steering_angle_rad = clamp(steering_angle_rad, -max_steer_rad, max_steer_rad)
    steer_cmd = clamp(steering_angle_rad / max_steer_rad, -1.0, 1.0)
    return steer_cmd, math.degrees(steering_angle_rad)


class SpeedPID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        max_throttle: float,
        max_brake: float,
        integral_limit: float,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = 0.0

    def step(self, target_speed: float, current_speed: float, dt: float) -> tuple[float, float, float]:
        error = target_speed - current_speed
        self.integral = clamp(
            self.integral + error * dt,
            -self.integral_limit,
            self.integral_limit,
        )
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error

        command = self.kp * error + self.ki * self.integral + self.kd * derivative
        if command >= 0.0:
            throttle = clamp(command, 0.0, self.max_throttle)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-command, 0.0, self.max_brake)
        return throttle, brake, error


def compute_route_progress(nearest_index: int, route: list[carla.Waypoint]) -> float:
    if len(route) <= 1:
        return 0.0
    return 100.0 * nearest_index / (len(route) - 1)


def plot_route_tracking(
    route: list[carla.Waypoint],
    tracking_csv_path: Path,
    output_path: Path,
) -> None:
    import matplotlib.pyplot as plt

    route_x = [waypoint.transform.location.x for waypoint in route]
    route_y = [waypoint.transform.location.y for waypoint in route]
    vehicle_x: list[float] = []
    vehicle_y: list[float] = []
    cross_track_errors: list[float] = []

    with tracking_csv_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            vehicle_x.append(float(row["x"]))
            vehicle_y.append(float(row["y"]))
            cross_track_errors.append(abs(float(row["cross_track_error_m"])))

    if not vehicle_x:
        return

    fig, ax = plt.subplots(figsize=(8, 6), dpi=150)
    ax.plot(route_x, route_y, "--", linewidth=1.5, label="reference route")
    scatter = ax.scatter(vehicle_x, vehicle_y, c=cross_track_errors, s=10, cmap="magma", label="ego path")
    ax.scatter(vehicle_x[0], vehicle_y[0], marker="o", s=70, label="start")
    ax.scatter(vehicle_x[-1], vehicle_y[-1], marker="x", s=80, label="end")
    ax.set_title("Route Tracking")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.colorbar(scatter, ax=ax, label="abs cross-track error (m)")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


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
    parser.add_argument("--semantic", action="store_true", help="Save semantic segmentation frames.")
    parser.add_argument("--lidar", action="store_true", help="Save LiDAR point clouds as PLY files.")
    parser.add_argument("--lidar-channels", default=32, type=int)
    parser.add_argument("--lidar-points-per-second", default=56000, type=int)
    parser.add_argument("--lidar-range", default=50.0, type=float)
    parser.add_argument("--control-mode", default="autopilot", choices=["autopilot", "pure_pursuit"])
    parser.add_argument("--route-length", default=120.0, type=float, help="Reference route length in meters.")
    parser.add_argument("--route-spacing", default=2.0, type=float, help="Route waypoint spacing in meters.")
    parser.add_argument("--lookahead-distance", default=8.0, type=float, help="Pure Pursuit lookahead distance.")
    parser.add_argument("--wheelbase", default=2.8, type=float, help="Bicycle-model wheelbase in meters.")
    parser.add_argument("--target-speed", default=8.0, type=float, help="Target speed in m/s.")
    parser.add_argument("--max-speed", default=12.0, type=float, help="Safety cap for target speed in m/s.")
    parser.add_argument("--max-steer-angle", default=35.0, type=float, help="Physical steering limit in degrees.")
    parser.add_argument("--max-throttle", default=0.6, type=float)
    parser.add_argument("--max-brake", default=0.8, type=float)
    parser.add_argument("--speed-kp", default=0.35, type=float)
    parser.add_argument("--speed-ki", default=0.05, type=float)
    parser.add_argument("--speed-kd", default=0.02, type=float)
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
    run_dir = make_run_dir(output_root, args.semantic, args.lidar)
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
        route = None
        target_speed = min(args.target_speed, args.max_speed)
        speed_controller = SpeedPID(
            args.speed_kp,
            args.speed_ki,
            args.speed_kd,
            args.max_throttle,
            args.max_brake,
            integral_limit=10.0,
        )

        if args.control_mode == "autopilot":
            vehicle.set_autopilot(True, traffic_manager.get_port())
        else:
            vehicle.set_autopilot(False)
            world.tick()
            route = build_route(
                world.get_map(),
                vehicle.get_transform().location,
                args.route_length,
                args.route_spacing,
            )
            write_route_csv(route, run_dir / "route_waypoints.csv")

        camera, image_queue = attach_rgb_camera(world, vehicle, args.width, args.height, args.fov)
        actors.append(camera)
        semantic_queue = None
        if args.semantic:
            semantic_camera, semantic_queue = attach_semantic_camera(
                world,
                vehicle,
                args.width,
                args.height,
                args.fov,
            )
            actors.append(semantic_camera)
        lidar_queue = None
        if args.lidar:
            lidar, lidar_queue = attach_lidar(
                world,
                vehicle,
                args.fps,
                args.lidar_channels,
                args.lidar_points_per_second,
                args.lidar_range,
            )
            actors.append(lidar)

        csv_path = run_dir / "vehicle_state.csv"
        tracking_csv_path = run_dir / "tracking_metrics.csv" if route is not None else None
        total_frames = max(1, int(args.duration * args.fps))

        print(f"Output: {run_dir}")
        print(f"Map: {world.get_map().name}")
        print(f"Ego vehicle: {vehicle.type_id} (actor id {vehicle.id})")
        print(f"Control mode: {args.control_mode}")
        if route is not None:
            print(f"Route waypoints: {len(route)}")
            print(f"Target speed: {target_speed:g} m/s (cap: {args.max_speed:g} m/s)")
        print(f"Collecting {total_frames} frames at {args.fps:g} FPS ...")

        with contextlib.ExitStack() as stack:
            file = stack.enter_context(csv_path.open("w", encoding="utf-8", newline=""))
            fieldnames = [
                "step",
                "frame",
                "timestamp",
                "rgb_path",
                "semantic_path",
                "lidar_path",
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

            tracking_writer = None
            if tracking_csv_path is not None:
                tracking_file = stack.enter_context(tracking_csv_path.open("w", encoding="utf-8", newline=""))
                tracking_writer = csv.DictWriter(
                    tracking_file,
                    fieldnames=[
                        "step",
                        "frame",
                        "timestamp",
                        "x",
                        "y",
                        "nearest_index",
                        "target_index",
                        "route_progress_pct",
                        "cross_track_error_m",
                        "target_speed_mps",
                        "current_speed_mps",
                        "speed_error_mps",
                        "steering_angle_deg",
                        "steer_cmd",
                        "throttle_cmd",
                        "brake_cmd",
                    ],
                )
                tracking_writer.writeheader()

            for step in range(total_frames):
                control_target_index = -1
                steering_angle_deg = 0.0
                if route is not None:
                    pre_tick_transform = vehicle.get_transform()
                    current_speed = speed_mps(vehicle)
                    nearest_index = nearest_route_index(pre_tick_transform.location, route)
                    control_target_index = find_lookahead_index(
                        pre_tick_transform,
                        route,
                        nearest_index,
                        args.lookahead_distance,
                    )
                    target_location = route[control_target_index].transform.location
                    steer_cmd, steering_angle_deg = pure_pursuit_steer(
                        pre_tick_transform,
                        target_location,
                        args.lookahead_distance,
                        args.wheelbase,
                        args.max_steer_angle,
                    )
                    throttle_cmd, brake_cmd, _speed_error = speed_controller.step(
                        target_speed,
                        current_speed,
                        fixed_delta_seconds,
                    )
                    if nearest_index >= len(route) - 2:
                        throttle_cmd = 0.0
                        brake_cmd = args.max_brake
                    vehicle.apply_control(
                        carla.VehicleControl(
                            throttle=throttle_cmd,
                            steer=steer_cmd,
                            brake=brake_cmd,
                            hand_brake=False,
                            reverse=False,
                        )
                    )

                frame = world.tick()
                image = get_sensor_data(image_queue, frame)
                semantic_image = None
                if semantic_queue is not None:
                    semantic_image = get_sensor_data(semantic_queue, frame)
                lidar_data = None
                if lidar_queue is not None:
                    lidar_data = get_sensor_data(lidar_queue, frame)
                follow_vehicle_with_spectator(world, vehicle)

                transform = vehicle.get_transform()
                velocity = vehicle.get_velocity()
                control = vehicle.get_control()
                current_speed = speed_mps(vehicle)
                rgb_path = run_dir / "rgb" / f"{step:06d}.png"
                semantic_path = ""
                lidar_path = ""
                image.save_to_disk(str(rgb_path))
                if semantic_image is not None:
                    semantic_file = run_dir / "semantic" / f"{step:06d}.png"
                    semantic_image.save_to_disk(
                        str(semantic_file),
                        carla.ColorConverter.CityScapesPalette,
                    )
                    semantic_path = str(semantic_file.relative_to(run_dir))
                if lidar_data is not None:
                    lidar_file = run_dir / "lidar" / f"{step:06d}.ply"
                    lidar_data.save_to_disk(str(lidar_file))
                    lidar_path = str(lidar_file.relative_to(run_dir))

                writer.writerow(
                    {
                        "step": step,
                        "frame": frame,
                        "timestamp": image.timestamp,
                        "rgb_path": str(rgb_path.relative_to(run_dir)),
                        "semantic_path": semantic_path,
                        "lidar_path": lidar_path,
                        "x": transform.location.x,
                        "y": transform.location.y,
                        "z": transform.location.z,
                        "roll": transform.rotation.roll,
                        "pitch": transform.rotation.pitch,
                        "yaw": transform.rotation.yaw,
                        "vx": velocity.x,
                        "vy": velocity.y,
                        "vz": velocity.z,
                        "speed_mps": current_speed,
                        "throttle": control.throttle,
                        "steer": control.steer,
                        "brake": control.brake,
                        "hand_brake": control.hand_brake,
                        "reverse": control.reverse,
                        "gear": control.gear,
                    }
                )

                if route is not None and tracking_writer is not None:
                    nearest_index = nearest_route_index(transform.location, route)
                    target_index = find_lookahead_index(
                        transform,
                        route,
                        nearest_index,
                        args.lookahead_distance,
                    )
                    cross_track_error = signed_cross_track_error(transform.location, route, nearest_index)
                    tracking_writer.writerow(
                        {
                            "step": step,
                            "frame": frame,
                            "timestamp": image.timestamp,
                            "x": transform.location.x,
                            "y": transform.location.y,
                            "nearest_index": nearest_index,
                            "target_index": target_index,
                            "route_progress_pct": compute_route_progress(nearest_index, route),
                            "cross_track_error_m": cross_track_error,
                            "target_speed_mps": target_speed,
                            "current_speed_mps": current_speed,
                            "speed_error_mps": target_speed - current_speed,
                            "steering_angle_deg": steering_angle_deg,
                            "steer_cmd": control.steer,
                            "throttle_cmd": control.throttle,
                            "brake_cmd": control.brake,
                        }
                    )

                if (step + 1) % max(1, int(args.fps)) == 0 or step == total_frames - 1:
                    print(f"Saved {step + 1}/{total_frames} frames")

        plot_trajectory(csv_path, run_dir / "trajectory.png")
        if route is not None and tracking_csv_path is not None:
            plot_route_tracking(route, tracking_csv_path, run_dir / "route_tracking.png")
        print("Done.")
        print(f"RGB images: {run_dir / 'rgb'}")
        print(f"Vehicle states: {csv_path}")
        print(f"Trajectory plot: {run_dir / 'trajectory.png'}")
        if tracking_csv_path is not None:
            print(f"Tracking metrics: {tracking_csv_path}")
            print(f"Route tracking plot: {run_dir / 'route_tracking.png'}")

    finally:
        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        for actor in reversed(actors):
            if actor.is_alive:
                actor.destroy()


if __name__ == "__main__":
    main()

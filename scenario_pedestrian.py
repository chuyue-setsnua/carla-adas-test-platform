#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CARLA Pedestrian Crossing Scenario Test
========================================
Tests AEB (Automatic Emergency Braking) response to a pedestrian crossing
the road in front of the ego vehicle.

Controls:
    SPACE - Start the scenario
    R     - Reset the scenario
    Q     - Quit
"""

import carla
import pygame
import numpy as np
import ctypes
import time
import os
import sys
import math
import msvcrt
from can_simulator import CANDatalogger
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
W, H = 1024, 512
CRUISING_SPEED = 6.0          # m/s (~29 km/h) target cruising speed
CRUISE_WAIT = 5.0             # seconds to reach cruising speed before crossing
PEDESTRIAN_SPEED = 1.4        # m/s walking speed
LOOK_AHEAD = 8.0              # metres for lane-keeping waypoint lookup
AEB_TTC_THRESHOLD = 3.0       # seconds – start braking below this TTC
AEB_MAX_DECEL = 6.0           # m/s^2 maximum braking deceleration
ACC_GAIN = 0.8                # proportional gain for ACC throttle
SCENARIO_TIMEOUT = 30.0       # seconds
REPORT_DIR = r"E:\CARLA\test_report"
REPORT_PATH = os.path.join(REPORT_DIR, "pedestrian_report.png")
WALKER_LATERAL_OFFSET = 6.0   # metres to the side of the road centre
WALKER_SPAWN_AHEAD = 40.0     # metres ahead of ego for walker spawn


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------
def force_focus(screen):
    """Force the pygame window to the foreground on Windows."""
    try:
        hwnd = pygame.display.get_wm_info()["window"]
        ctypes.windll.user32.SetForegroundWindow(hwnd)
    except Exception:
        pass


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def yaw_to_vec(yaw_deg):
    """Convert CARLA yaw (degrees) to a unit forward vector (x, y)."""
    rad = math.radians(yaw_deg)
    return math.cos(rad), math.sin(rad)


def perpendicular_right(yaw_deg):
    """Return unit vector pointing to the right of the given yaw."""
    rad = math.radians(yaw_deg)
    return math.sin(rad), -math.cos(rad)


# ---------------------------------------------------------------------------
# Lane-keeping controller
# ---------------------------------------------------------------------------
def keep_lane(vehicle, world_map, target_speed):
    """
    Compute (throttle, steer) to follow the lane at *target_speed*.
    Uses waypoint-based pure-pursuit style steering.
    """
    loc = vehicle.get_location()
    vel = vehicle.get_velocity()
    speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    # Current yaw
    ego_yaw = vehicle.get_transform().rotation.yaw

    # Look-ahead waypoint on the lane
    wp = world_map.get_waypoint(loc)
    next_wps = wp.next(LOOK_AHEAD)
    if not next_wps:
        return 0.0, 0.0, speed
    target_wp = next_wps[0]
    target_loc = target_wp.transform.location

    # Yaw error
    target_yaw = math.degrees(math.atan2(target_loc.y - loc.y,
                                          target_loc.x - loc.x))
    yaw_err = target_yaw - ego_yaw
    # Normalise to [-180, 180]
    while yaw_err > 180.0:
        yaw_err -= 360.0
    while yaw_err < -180.0:
        yaw_err += 360.0

    # Cross-track error (signed lateral distance)
    dx = target_loc.x - loc.x
    dy = target_loc.y - loc.y
    right_x, right_y = perpendicular_right(ego_yaw)
    cross_err = dx * right_x + dy * right_y

    # Steering: yaw correction + cross-track correction
    steer = clamp(yaw_err / 45.0 + cross_err * 0.3, -1.0, 1.0)

    # Throttle: simple proportional speed control
    throttle = clamp(ACC_GAIN * (target_speed - speed), 0.0, 1.0)

    return throttle, steer, speed


# ---------------------------------------------------------------------------
# AEB controller (pedestrian-aware)
# ---------------------------------------------------------------------------
def aeb_control(vehicle, pedestrian, current_speed, dt):
    """
    Override throttle/brake when a pedestrian is detected close and crossing.
    Returns (throttle, brake, ttc).
    """
    if pedestrian is None or not pedestrian.is_alive:
        return None, None, float("inf")

    ego_loc = vehicle.get_location()
    ped_loc = pedestrian.get_location()

    dx = ped_loc.x - ego_loc.x
    dy = ped_loc.y - ego_loc.y
    dist = math.sqrt(dx * dx + dy * dy)

    # Relative longitudinal speed (approach rate)
    ego_vel = vehicle.get_velocity()
    ped_vel = pedestrian.get_velocity()
    ego_yaw = vehicle.get_transform().rotation.yaw
    fx, fy = yaw_to_vec(ego_yaw)

    # Longitudinal components along ego heading
    ego_lon = ego_vel.x * fx + ego_vel.y * fy
    ped_lon = ped_vel.x * fx + ped_vel.y * fy
    approach_speed = ego_lon - ped_lon  # positive => closing in

    if approach_speed <= 0.05:
        return None, None, float("inf")  # not approaching

    ttc = dist / approach_speed

    if ttc < AEB_TTC_THRESHOLD and dist < 50.0:
        # AEB active: no throttle, apply brake proportional to urgency
        # More aggressive: higher minimum brake + emergency full brake at close range
        if dist < 8.0:
            brake = 1.0  # emergency full brake
        else:
            brake = clamp(1.8 - (ttc / AEB_TTC_THRESHOLD), 0.6, 1.0)
        return 0.0, brake, ttc

    return None, None, ttc


# ---------------------------------------------------------------------------
# Sensor callbacks
# ---------------------------------------------------------------------------
class SensorData:
    """Shared container for sensor readings."""
    def __init__(self):
        self.collision_occurred = False
        self.collision_actor_type = ""
        self.collision_time = None
        self.camera_image = None


def on_collision(event, sensor_data, scenario_start_time):
    """Collision sensor callback – only flag walker collisions."""
    actor = event.other_actor
    actor_type = actor.type_id if actor else ""
    if "walker" in actor_type:
        sensor_data.collision_occurred = True
        sensor_data.collision_actor_type = actor_type
        sensor_data.collision_time = time.time() - scenario_start_time


def on_camera(image, sensor_data):
    sensor_data.camera_image = image


# ---------------------------------------------------------------------------
# Report generation
# ---------------------------------------------------------------------------
def generate_report(log, verdict):
    """Generate matplotlib report and save to REPORT_PATH."""
    os.makedirs(REPORT_DIR, exist_ok=True)

    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.3)

    t = np.array(log["time"])

    # --- Speed chart ---
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, log["ego_speed"], "b-", linewidth=1.5, label="Ego Speed (m/s)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Speed (m/s)")
    ax1.set_title("Ego Vehicle Speed")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # --- Distance chart ---
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(t, log["ped_distance"], "r-", linewidth=1.5, label="Pedestrian Dist (m)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Distance (m)")
    ax2.set_title("Distance to Pedestrian")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    # --- TTC chart ---
    ax3 = fig.add_subplot(gs[1, 1])
    ttc_arr = np.array(log["ttc"])
    ttc_clipped = np.clip(ttc_arr, 0, 20)
    ax3.plot(t, ttc_clipped, "g-", linewidth=1.5, label="TTC (s)")
    ax3.axhline(y=AEB_TTC_THRESHOLD, color="orange", linestyle="--",
                linewidth=1.2, label=f"AEB Threshold ({AEB_TTC_THRESHOLD}s)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("TTC (s)")
    ax3.set_title("Time-to-Collision (TTC)")
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3)

    # --- Summary / verdict ---
    ax4 = fig.add_subplot(gs[2, :])
    ax4.axis("off")
    color = "green" if verdict == "PASS" else "red"
    summary_text = (
        f"Verdict:  {verdict}\n"
        f"Collision:  {'Yes' if log['collision'] else 'No'}\n"
        f"Min TTC:  {min(log['ttc']):.2f} s\n"
        f"Min Distance:  {min(log['ped_distance']):.2f} m\n"
        f"Max Speed:  {max(log['ego_speed']):.2f} m/s\n"
        f"Scenario Duration:  {t[-1]:.1f} s" if len(t) > 0 else "No data"
    )
    ax4.text(0.5, 0.5, summary_text, transform=ax4.transAxes,
             fontsize=14, verticalalignment="center", horizontalalignment="center",
             fontfamily="monospace",
             bbox=dict(boxstyle="round,pad=0.8", facecolor=color, alpha=0.15,
                       edgecolor=color, linewidth=2))
    ax4.set_title("Scenario Summary", fontsize=13, fontweight="bold")

    fig.suptitle("CARLA Pedestrian Crossing – AEB Test Report", fontsize=15,
                 fontweight="bold", y=0.98)

    fig.savefig(REPORT_PATH, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[REPORT] Saved to {REPORT_PATH}")


# ---------------------------------------------------------------------------
# Scenario class
# ---------------------------------------------------------------------------
class PedestrianScenario:
    def __init__(self, client, world, world_map):
        self.client = client
        self.world = world
        self.world_map = world_map
        self.ego = None
        self.walker = None
        self.camera_sensor = None
        self.collision_sensor = None
        self.sensor_data = SensorData()
        self.initial_ego_transform = None
        self.initial_walker_transform = None
        self.scenario_start_time = None
        self.crossing_triggered = False
        self.crossing_direction = None

        # Logging
        self.log = {
            "time": [],
            "ego_speed": [],
            "ped_distance": [],
            "ttc": [],
            "collision": False,
        }

        # CAN bus tracker
        self.prev_long_speed = 0.0
        self.prev_steer_deg = 0.0
        self.can_logger = None  # initialised after sensors
        self.aeb_active = False  # persistent AEB once triggered

    # ------------------------------------------------------------------
    # Spawning
    # ------------------------------------------------------------------
    def spawn_ego(self):
        """Spawn ego vehicle at a CARLA spawn point (Town04 preferred)."""
        spawn_points = self.world_map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available on this map.")

        # Try to pick a spawn point on a straight road section
        chosen = spawn_points[0]
        for sp in spawn_points:
            wp = self.world_map.get_waypoint(sp.location)
            if wp and wp.is_junction is False:
                chosen = sp
                break

        bp_lib = self.world.get_blueprint_library()
        vehicle_bps = bp_lib.filter("vehicle.tesla.model3")
        if not vehicle_bps:
            vehicle_bps = bp_lib.filter("vehicle.*")
        bp = vehicle_bps[0]
        bp.set_attribute("role_name", "hero")

        self.ego = self.world.try_spawn_actor(bp, chosen)
        if self.ego is None:
            # Retry with a few other spawn points
            for sp in spawn_points[:10]:
                self.ego = self.world.try_spawn_actor(bp, sp)
                if self.ego is not None:
                    break
        if self.ego is None:
            raise RuntimeError("Failed to spawn ego vehicle.")

        # Sync physics — get_transform() returns (0,0,0) without ticks
        for _ in range(10):
            self.world.tick()
            time.sleep(0.01)

        self.initial_ego_transform = self.ego.get_transform()
        print(f"[SPAWN] Ego at ({chosen.location.x:.1f}, {chosen.location.y:.1f})")

    def spawn_walker(self):
        """Spawn a pedestrian on the side of the road ahead of the ego."""
        # Ensure ego transform is synced before reading
        for _ in range(5):
            self.world.tick()
            time.sleep(0.01)

        ego_tf = self.ego.get_transform()
        ego_yaw = ego_tf.rotation.yaw
        fx, fy = yaw_to_vec(ego_yaw)

        # Position ahead of ego
        ahead_x = ego_tf.location.x + fx * WALKER_SPAWN_AHEAD
        ahead_y = ego_tf.location.y + fy * WALKER_SPAWN_AHEAD

        # Lateral offset to the right side of the road
        rx, ry = perpendicular_right(ego_yaw)
        spawn_x = ahead_x + rx * WALKER_LATERAL_OFFSET
        spawn_y = ahead_y + ry * WALKER_LATERAL_OFFSET

        # Project to the nearest drivable waypoint for a valid Z
        snap_wp = self.world_map.get_waypoint(
            carla.Location(x=spawn_x, y=spawn_y, z=ego_tf.location.z),
            project_to_road=False
        )
        if snap_wp is None:
            # Fallback: use the ahead point and offset less
            snap_wp = self.world_map.get_waypoint(
                carla.Location(x=ahead_x, y=ahead_y, z=ego_tf.location.z)
            )
            spawn_x = snap_wp.transform.location.x + rx * (WALKER_LATERAL_OFFSET * 0.6)
            spawn_y = snap_wp.transform.location.y + ry * (WALKER_LATERAL_OFFSET * 0.6)

        spawn_z = snap_wp.transform.location.z + 0.2  # slightly above ground

        walker_loc = carla.Location(x=spawn_x, y=spawn_y, z=spawn_z)

        bp_lib = self.world.get_blueprint_library()
        walker_bps = bp_lib.filter("walker.pedestrian.*")
        if not walker_bps:
            raise RuntimeError("No walker blueprints found.")
        walker_bp = walker_bps[0]

        walker_tf = carla.Transform(walker_loc)
        self.walker = self.world.try_spawn_actor(walker_bp, walker_tf)
        if self.walker is None:
            # Retry a few times with small offsets
            for offset in [0.5, -0.5, 1.0, -1.0]:
                walker_tf.location.x += offset
                walker_tf.location.y += offset
                self.walker = self.world.try_spawn_actor(walker_bp, walker_tf)
                if self.walker is not None:
                    break
        if self.walker is None:
            raise RuntimeError("Failed to spawn walker.")

        # Sync physics for walker
        for _ in range(10):
            self.world.tick()
            time.sleep(0.01)

        self.initial_walker_transform = self.walker.get_transform()

        # Compute crossing direction: perpendicular to road, towards the lane
        # (from right side of road towards left → negate the right vector)
        self.crossing_direction = (-rx, -ry)

        print(f"[SPAWN] Walker at ({spawn_x:.1f}, {spawn_y:.1f}), "
              f"crossing dir=({-rx:.2f}, {-ry:.2f})")

    # ------------------------------------------------------------------
    # Sensors
    # ------------------------------------------------------------------
    def attach_sensors(self):
        """Attach camera and collision sensors to the ego vehicle."""
        bp_lib = self.world.get_blueprint_library()

        # RGB Camera
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(W))
        cam_bp.set_attribute("image_size_y", str(H))
        cam_bp.set_attribute("fov", "100")
        cam_tf = carla.Transform(carla.Location(x=1.6, z=1.7))
        self.camera_sensor = self.world.spawn_actor(cam_bp, cam_tf, attach_to=self.ego)
        self.camera_sensor.listen(lambda img: on_camera(img, self.sensor_data))

        # Collision sensor
        col_bp = bp_lib.find("sensor.other.collision")
        col_tf = carla.Transform()
        self.collision_sensor = self.world.spawn_actor(col_bp, col_tf, attach_to=self.ego)
        self.collision_sensor.listen(
            lambda evt: on_collision(evt, self.sensor_data, self.scenario_start_time)
        )
        print("[SENSORS] Camera + Collision attached.")

        # --- CAN bus logger ---
        self.can_logger = CANDatalogger("E:/CARLA/test_report/pedestrian_can_log.asc")
        print("[CAN] Logger -> test_report/pedestrian_can_log.asc")

    # ------------------------------------------------------------------
    # Walker control
    # ------------------------------------------------------------------
    def start_walker_crossing(self):
        """Command the pedestrian to walk across the road."""
        if self.walker is None or not self.walker.is_alive:
            return
        cx, cy = self.crossing_direction
        ctrl = carla.WalkerControl()
        ctrl.speed = PEDESTRIAN_SPEED
        ctrl.direction = carla.Vector3D(x=cx, y=cy, z=0.0)
        self.walker.apply_control(ctrl)
        self.crossing_triggered = True
        print("[WALKER] Crossing started.")

    def stop_walker(self):
        """Stop the pedestrian."""
        if self.walker and self.walker.is_alive:
            ctrl = carla.WalkerControl()
            ctrl.speed = 0.0
            ctrl.direction = carla.Vector3D(x=0, y=0, z=0)
            self.walker.apply_control(ctrl)

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------
    def reset(self):
        """Reset ego and walker to their initial positions."""
        print("[RESET] Resetting scenario...")

        # Stop walker
        self.stop_walker()

        # Destroy sensors
        if self.camera_sensor and self.camera_sensor.is_alive:
            self.camera_sensor.stop()
            self.camera_sensor.destroy()
            self.camera_sensor = None
        if self.collision_sensor and self.collision_sensor.is_alive:
            self.collision_sensor.stop()
            self.collision_sensor.destroy()
            self.collision_sensor = None

        # Reset ego
        if self.ego and self.ego.is_alive:
            self.ego.set_velocity(carla.Vector3D(0, 0, 0))
            self.ego.set_angular_velocity(carla.Vector3D(0, 0, 0))
            self.ego.apply_control(carla.VehicleControl(throttle=0, brake=1.0, steer=0))
            time.sleep(0.3)
            self.ego.set_transform(self.initial_ego_transform)
            self.ego.set_velocity(carla.Vector3D(0, 0, 0))
            self.ego.set_angular_velocity(carla.Vector3D(0, 0, 0))

        # Reset walker
        if self.walker and self.walker.is_alive:
            self.walker.set_transform(self.initial_walker_transform)
            self.stop_walker()

        # Reset state
        self.sensor_data = SensorData()
        self.crossing_triggered = False
        self.scenario_start_time = None
        self.prev_long_speed = 0.0
        self.prev_steer_deg = 0.0
        self.aeb_active = False
        self.log = {
            "time": [],
            "ego_speed": [],
            "ped_distance": [],
            "ttc": [],
            "collision": False,
        }

        # Re-attach sensors
        self.attach_sensors()
        print("[RESET] Done. Press SPACE to start.")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def cleanup(self):
        """Destroy all actors and sensors."""
        self.stop_walker()

        # Close CAN logger
        if self.can_logger is not None:
            frames = self.can_logger.get_message_count()
            self.can_logger.close()
            print(f"[CAN] {frames} frames logged.")

        actors = [self.camera_sensor, self.collision_sensor, self.walker, self.ego]
        for actor in actors:
            if actor is not None and actor.is_alive:
                try:
                    actor.destroy()
                except Exception:
                    pass
        print("[CLEANUP] All actors destroyed.")


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------
def init_pygame():
    """Initialise pygame display window."""
    pygame.init()
    screen = pygame.display.set_mode((W, H), pygame.SWSURFACE)
    pygame.display.set_caption("CARLA Pedestrian Crossing – AEB Test")
    force_focus(screen)
    return screen


def render_frame(screen, sensor_data, scenario):
    """Render camera view + HUD overlay."""
    # Draw camera image
    if sensor_data.camera_image is not None:
        img = sensor_data.camera_image
        array = np.frombuffer(img.raw_data, dtype=np.uint8)
        array = array.reshape((img.height, img.width, 4))
        # BGRA → RGB
        surface = pygame.surfarray.make_surface(array[:, :, :3][:, :, ::-1])
        surface = pygame.transform.flip(surface, False, True)
        screen.blit(surface, (0, 0))
    else:
        screen.fill((30, 30, 30))

    # HUD overlay
    font = pygame.font.SysFont("consolas", 16)
    hud_color = (255, 255, 255)
    bg_color = (0, 0, 0, 180)

    if scenario.ego is not None:
        vel = scenario.ego.get_velocity()
        speed_kmh = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2) * 3.6
        lines = [f"Speed: {speed_kmh:.1f} km/h"]
    else:
        lines = ["Ego: N/A"]

    if scenario.log["ped_distance"]:
        lines.append(f"Ped Dist: {scenario.log['ped_distance'][-1]:.1f} m")
    if scenario.log["ttc"]:
        ttc_val = scenario.log["ttc"][-1]
        ttc_str = f"{ttc_val:.1f}" if ttc_val < 100 else "INF"
        lines.append(f"TTC: {ttc_str} s")

    if scenario.crossing_triggered:
        lines.append("WALKER: CROSSING")
    if scenario.sensor_data.collision_occurred:
        lines.append("*** COLLISION ***")

    if scenario.scenario_start_time is None:
        lines.append("Press SPACE to start")

    y = 5
    for line in lines:
        surf = font.render(line, True, hud_color)
        bg = pygame.Surface((surf.get_width() + 8, surf.get_height() + 4), pygame.SRCALPHA)
        bg.fill((0, 0, 0, 160))
        screen.blit(bg, (3, y - 2))
        screen.blit(surf, (7, y))
        y += 22

    pygame.display.flip()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    # --- Connect to CARLA ---
    print("[INIT] Connecting to CARLA server (localhost:2000)...")
    try:
        client = carla.Client("127.0.0.1", 2000)
        client.set_timeout(10.0)
        world = client.get_world()
    except Exception as e:
        print(f"[ERROR] Cannot connect to CARLA: {e}")
        sys.exit(1)

    # --- Load map ---
    current_map = world.get_map().name
    print(f"[MAP] Current map: {current_map}")
    if "Town04" not in current_map:
        try:
            print("[MAP] Attempting to load Town04...")
            world = client.load_world("Town04")
            time.sleep(2.0)
            print("[MAP] Town04 loaded.")
        except Exception:
            print("[MAP] Town04 unavailable, trying Town01...")
            try:
                world = client.load_world("Town01")
                time.sleep(2.0)
                print("[MAP] Town01 loaded.")
            except Exception as e2:
                print(f"[MAP] Fallback failed: {e2}. Using current map.")
    world_map = world.get_map()

    # Set synchronous mode with fixed dt
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # --- Init pygame ---
    screen = init_pygame()

    # --- Create scenario ---
    scenario = PedestrianScenario(client, world, world_map)
    scenario.spawn_ego()
    scenario.spawn_walker()
    scenario.attach_sensors()

    print("=" * 60)
    print("  CARLA Pedestrian Crossing – AEB Test")
    print("  SPACE = Start  |  R = Reset  |  Q = Quit")
    print("=" * 60)

    # --- Main loop ---
    running = True
    scenario_active = False
    verdict = None

    while running:
        # Keyboard input (msvcrt for Windows terminal)
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b"q" or key == b"Q":
                running = False
                continue
            elif key == b" ":
                if not scenario_active and scenario.scenario_start_time is None:
                    scenario_active = True
                    scenario.scenario_start_time = time.time()
                    print("[START] Scenario started!")
                elif not scenario_active and scenario.scenario_start_time is not None:
                    # Already finished, ignore
                    pass
            elif key == b"r" or key == b"R":
                scenario_active = False
                scenario.reset()
                verdict = None
                continue

        # Also handle pygame events (window close, etc.)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        dt = 0.05  # fixed_delta_seconds

        if scenario_active and scenario.scenario_start_time is not None:
            elapsed = time.time() - scenario.scenario_start_time

            # ----- Determine ego target speed -----
            if elapsed < CRUISE_WAIT:
                target_speed = CRUISING_SPEED
            else:
                target_speed = CRUISING_SPEED  # maintain cruising

            # ----- Lane-keeping + ACC -----
            throttle, steer, speed = keep_lane(scenario.ego, world_map, target_speed)

            # ----- Trigger pedestrian crossing -----
            if not scenario.crossing_triggered and elapsed >= CRUISE_WAIT:
                scenario.start_walker_crossing()

            # ----- AEB check (with persistent braking) -----
            aeb_throttle, aeb_brake, ttc = aeb_control(
                scenario.ego, scenario.walker, speed, dt
            )

            # Once AEB triggers, keep braking until pedestrian clears
            if aeb_throttle is not None:
                scenario.aeb_active = True
            elif scenario.aeb_active:
                # Check if pedestrian has passed — release only when safe
                if scenario.walker and scenario.walker.is_alive:
                    ped_dist_now = scenario.ego.get_location().distance(
                        scenario.walker.get_location())
                    if ped_dist_now > 20.0 and speed < 0.5:
                        scenario.aeb_active = False  # pedestrian cleared
                else:
                    scenario.aeb_active = False  # walker gone (dead or despawned)

            # ----- Apply vehicle control -----
            ctrl = carla.VehicleControl()
            if scenario.aeb_active or aeb_throttle is not None:
                # AEB active (persistent)
                ctrl.throttle = 0.0
                if scenario.walker and scenario.walker.is_alive:
                    ped_dist_now = scenario.ego.get_location().distance(
                        scenario.walker.get_location())
                    if ped_dist_now < 6.0:
                        ctrl.brake = 1.0
                    elif ped_dist_now < 15.0:
                        ctrl.brake = 0.8
                    else:
                        ctrl.brake = aeb_brake if aeb_brake else 0.5
                else:
                    ctrl.brake = 0.5
                ctrl.steer = steer
            else:
                ctrl.throttle = throttle
                ctrl.brake = 0.0
                ctrl.steer = steer
            scenario.ego.apply_control(ctrl)

            # ----- CAN bus logging -----
            if scenario.can_logger is not None:
                ego_fwd = scenario.ego.get_transform().get_forward_vector()
                ego_vel = scenario.ego.get_velocity()
                ego_long_speed = ego_vel.x * ego_fwd.x + ego_vel.y * ego_fwd.y
                scenario.can_logger.log_from_vehicle(
                    scenario.ego,
                    radar_target_vehicle=None,  # no lead vehicle in pedestrian scenario
                    prev_velocity=scenario.prev_long_speed,
                    prev_steering=scenario.prev_steer_deg,
                    dt=dt
                )
                scenario.prev_long_speed = ego_long_speed
                scenario.prev_steer_deg = -ctrl.steer * 720.0

            # ----- Compute metrics -----
            if scenario.walker and scenario.walker.is_alive:
                ego_loc = scenario.ego.get_location()
                ped_loc = scenario.walker.get_location()
                ped_dist = ego_loc.distance(ped_loc)
            else:
                ped_dist = float("inf")
                ttc = float("inf")

            # ----- Log data -----
            scenario.log["time"].append(elapsed)
            scenario.log["ego_speed"].append(speed)
            scenario.log["ped_distance"].append(ped_dist)
            scenario.log["ttc"].append(ttc)

            # ----- End conditions -----
            end_scenario = False
            end_reason = ""

            # 1) Collision with walker
            if scenario.sensor_data.collision_occurred:
                scenario.log["collision"] = True
                end_scenario = True
                end_reason = "COLLISION with pedestrian"

            # 2) Ego stopped after pedestrian crossed
            if (scenario.crossing_triggered and speed < 0.3
                    and elapsed > CRUISE_WAIT + 2.0):
                # Check if pedestrian has crossed (moved significantly)
                if ped_dist > 10.0 or (scenario.walker and not scenario.walker.is_alive):
                    end_scenario = True
                    end_reason = "Ego stopped, pedestrian passed"

            # 3) Timeout
            if elapsed >= SCENARIO_TIMEOUT:
                end_scenario = True
                end_reason = "Timeout (30s)"

            if end_scenario:
                scenario_active = False
                # Stop vehicle
                scenario.ego.apply_control(
                    carla.VehicleControl(throttle=0, brake=1.0, steer=0)
                )
                scenario.stop_walker()

                # Determine verdict
                # PASS: no collision AND ego stopped or AEB activated
                # FAIL: collision occurred OR ego did not brake
                if scenario.log["collision"]:
                    verdict = "FAIL"
                else:
                    min_ttc = min(scenario.log["ttc"]) if scenario.log["ttc"] else float("inf")
                    if min_ttc < AEB_TTC_THRESHOLD:
                        verdict = "PASS"  # AEB engaged, no collision
                    elif speed < 1.0:
                        verdict = "PASS"  # ego stopped in time
                    else:
                        verdict = "FAIL"

                print(f"[END] {end_reason}")
                print(f"[VERDICT] {verdict}")

                # Generate report
                try:
                    generate_report(scenario.log, verdict)
                except Exception as e:
                    print(f"[REPORT] Failed to generate: {e}")

                print("Press R to reset, Q to quit.")

        # Render frame
        render_frame(screen, scenario.sensor_data, scenario)

        # Tick world
        world.tick()

        # Small sleep to avoid busy-waiting between ticks
        time.sleep(0.005)

    # --- Cleanup ---
    scenario.cleanup()
    settings.synchronous_mode = False
    world.apply_settings(settings)
    pygame.quit()
    print("[EXIT] Done.")


if __name__ == "__main__":
    main()

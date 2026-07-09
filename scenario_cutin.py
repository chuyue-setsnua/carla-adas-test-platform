"""
CARLA Scenario Test: Cut-In (Adjacent Lane Vehicle Cuts In)
============================================================
Scene: Ego vehicle follows lead vehicle. A vehicle from the adjacent
       lane suddenly changes lane (cuts in) in front of the ego vehicle.
Goal:  Test whether ego AEB can detect and respond to the cut-in.

Controls (terminal):
  SPACE = start scenario
  R     = reset / replay
  Q     = quit

Metrics recorded: distance to cut-in vehicle, speed, TTC, collision event
Output: E:\\CARLA\\test_report\\cutin_report.png + cutin_can_log.asc
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

# ===== Scenario Parameters =====
EGO_SPEED = 60        # km/h target speed
LEAD_SPEED = 60       # km/h
CUTIN_SPEED = 65      # km/h - cut-in vehicle slightly faster
INITIAL_GAP = 30      # meters - ego to lead
CUTIN_LATERAL_OFFSET = 3.5  # meters - adjacent lane offset
CUTIN_AFTER_SEC = 4.0 # seconds before cut-in starts
CUTIN_DURATION = 2.5  # seconds to complete lane change
SAFETY_DIST = 2.0     # meters
DT = 0.05

# ===== Display =====
W, H = 1024, 512


def force_focus():
    try:
        hwnd = pygame.display.get_wm_info()["window"]
        ctypes.windll.user32.SetForegroundWindow(hwnd)
    except Exception:
        pass


def keep_lane(vehicle, speed_ms, carla_map):
    """Lane-keeping steering for any vehicle."""
    try:
        vt = vehicle.get_transform()
        vloc = vt.location
        vyaw = math.radians(vt.rotation.yaw)
        wp = carla_map.get_waypoint(vloc, project_to_road=True)
        look_ahead = max(8.0, speed_ms * 1.5)
        next_wps = wp.next(look_ahead)
        if next_wps:
            target_loc = next_wps[0].transform.location
            target_yaw = math.radians(next_wps[0].transform.rotation.yaw)
            yaw_err = target_yaw - vyaw
            while yaw_err > math.pi:
                yaw_err -= 2 * math.pi
            while yaw_err < -math.pi:
                yaw_err += 2 * math.pi
            dx = target_loc.x - vloc.x
            dy = target_loc.y - vloc.y
            angle_to_target = math.atan2(dy, dx)
            cross_err = angle_to_target - vyaw
            while cross_err > math.pi:
                cross_err -= 2 * math.pi
            while cross_err < -math.pi:
                cross_err += 2 * math.pi
            steer = 2.0 * yaw_err + 3.0 * cross_err
            speed_gain = min(1.0, 3.0 / max(speed_ms, 1.0))
            return max(-1.0, min(1.0, steer * speed_gain))
    except Exception:
        pass
    return 0.0


def aeb_control(ego_v_ms, target_speed_ms, dist_to_lead, lead_is_braking):
    """ACC + AEB longitudinal controller."""
    ctrl = carla.VehicleControl()

    # AEB mode when lead is braking
    if lead_is_braking:
        ctrl.throttle = 0.0
        ttc = dist_to_lead / max(ego_v_ms, 0.1)
        if dist_to_lead < SAFETY_DIST + 3:
            ctrl.brake = 1.0
        elif ttc < 2.0:
            ctrl.brake = min(1.0, 1.5 - ttc * 0.5)
        elif ttc < 4.0:
            ctrl.brake = min(1.0, 0.8 - ttc * 0.15)
        else:
            ctrl.brake = 0.3
        return ctrl

    # Normal ACC mode
    desired_gap = max(SAFETY_DIST + 5, ego_v_ms * 2.0)
    if dist_to_lead < SAFETY_DIST + 2:
        ctrl.throttle = 0.0
        ctrl.brake = 1.0
    elif dist_to_lead < desired_gap:
        gap_ratio = dist_to_lead / desired_gap
        if gap_ratio < 0.5:
            ctrl.throttle = 0.0
            ctrl.brake = min(1.0, (1.0 - gap_ratio) * 1.5)
        else:
            speed_error = target_speed_ms - ego_v_ms
            ctrl.throttle = max(0, speed_error * 0.3 * gap_ratio)
            ctrl.brake = max(0, -speed_error * 0.3)
    else:
        speed_error = target_speed_ms - ego_v_ms
        if speed_error > 2:
            ctrl.throttle = 1.0
        elif speed_error > 0:
            ctrl.throttle = min(1.0, 0.5 + speed_error * 0.3)
        elif speed_error > -2:
            ctrl.throttle = 0.0
        else:
            ctrl.throttle = 0.0
            ctrl.brake = min(1.0, abs(speed_error) * 0.5)

    return ctrl


def main():
    print("=" * 50)
    print("  CARLA Scenario: Cut-In Test")
    print("=" * 50)
    print()
    print(f"  Parameters:")
    print(f"    Speed:       {EGO_SPEED} km/h")
    print(f"    Gap:         {INITIAL_GAP} m")
    print(f"    Cut-in lane: {CUTIN_LATERAL_OFFSET} m offset")
    print(f"    Cut-in at:   {CUTIN_AFTER_SEC} s")
    print()

    # ===== Connect =====
    print("[1/4] Connecting to CARLA...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(15.0)

    available_maps = client.get_available_maps()
    target_map = None
    # Prefer Town04 (multi-lane highway) for cut-in scenario
    for preferred in ["Town04", "Town03", "Town01"]:
        for m in available_maps:
            if preferred in m:
                target_map = m
                break
        if target_map:
            break

    if target_map:
        world = client.load_world(target_map)
        time.sleep(2)
        print(f"      Loaded map: {target_map}")
    else:
        world = client.get_world()
        print(f"      Using existing map: {world.get_map().name}")

    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()
    carla_map = world.get_map()

    # ===== Clear old actors =====
    print("[2/4] Setting up vehicles...")
    for a in world.get_actors().filter("vehicle.*"):
        try:
            a.destroy()
        except Exception:
            pass
    time.sleep(1.0)
    # Verify all cleared
    remaining = len(list(world.get_actors().filter("vehicle.*")))
    if remaining > 0:
        print(f"      WARNING: {remaining} vehicles still present after cleanup!")
        for a in world.get_actors().filter("vehicle.*"):
            try:
                a.destroy()
            except Exception:
                pass
        time.sleep(0.5)
    print(f"      Vehicles in world: {len(list(world.get_actors().filter('vehicle.*')))}")

    # ===== Spawn vehicles =====
    ego_bp = bp_lib.filter("vehicle.*")[0]
    lead_bp = bp_lib.filter("vehicle.*")[1]
    cutin_bp = bp_lib.filter("vehicle.*")[2]
    print(f"      ego_bp:   {ego_bp.id}")
    print(f"      lead_bp:  {lead_bp.id}")
    print(f"      cutin_bp: {cutin_bp.id}")
    print(f"      Spawn points: {len(spawn_points)}, first at {spawn_points[0].location}")
    # Ensure world is ready
    world.tick()

    # Quick test: spawn ego_bp at first spawn point to verify
    test_v = None
    try:
        test_v = world.spawn_actor(ego_bp, spawn_points[0])
        for _ in range(10):
            world.tick()
            time.sleep(0.01)
        test_loc = test_v.get_transform().location
        print(f"      Test spawn ego: expected={spawn_points[0].location} actual={test_loc} drift={test_loc.distance(spawn_points[0].location):.1f}m")
        test_v.destroy()
    except RuntimeError as e:
        print(f"      Test spawn ego FAILED: {e}")

    try:
        test_v = world.spawn_actor(lead_bp, spawn_points[0])
        for _ in range(10):
            world.tick()
            time.sleep(0.01)
        test_loc = test_v.get_transform().location
        print(f"      Test spawn lead: expected={spawn_points[0].location} actual={test_loc} drift={test_loc.distance(spawn_points[0].location):.1f}m")
        test_v.destroy()
    except RuntimeError as e:
        print(f"      Test spawn lead FAILED: {e}")

    ego_vehicle = None
    lead_vehicle = None
    cutin_vehicle = None

    fail_count = {"ego": 0, "lead": 0, "cutin": 0}
    debug_count = 0

    for sp_idx, sp in enumerate(spawn_points):
        # --- Step 1: Spawn EGO at spawn point ---
        try:
            ev = world.spawn_actor(ego_bp, sp)
        except RuntimeError:
            fail_count["ego"] += 1
            continue

        # Wait for physics to sync transform (get_transform returns 0,0,0 otherwise)
        for _ in range(10):
            world.tick()
            time.sleep(0.01)

        ev_tf = ev.get_transform()
        ev_loc = ev_tf.location
        ev_yaw = ev_tf.rotation.yaw
        ev_yaw_rad = math.radians(ev_yaw)
        ev_wp = carla_map.get_waypoint(ev_loc, project_to_road=True)

        # --- Step 2: Geometric forward projection for LEAD ---
        lv = None
        offsets = [INITIAL_GAP, INITIAL_GAP + 2, INITIAL_GAP - 2,
                   INITIAL_GAP + 5, INITIAL_GAP - 5,
                   INITIAL_GAP + 8, INITIAL_GAP - 8]
        for offset_m in offsets:
            lx = sp.location.x + offset_m * math.cos(ev_yaw_rad)
            ly = sp.location.y + offset_m * math.sin(ev_yaw_rad)
            lt = carla.Transform(
                carla.Location(x=lx, y=ly, z=sp.location.z),
                ev_tf.rotation)
            try:
                lv = world.spawn_actor(lead_bp, lt)
            except RuntimeError:
                continue

            # Wait for physics to sync transform
            for _ in range(10):
                world.tick()
                time.sleep(0.01)

            # Validate: actual position not silently relocated
            lv_loc = lv.get_transform().location
            actual_gap = ev_loc.distance(lv_loc)
            if actual_gap < 5 or actual_gap > INITIAL_GAP * 2.5:
                lv.destroy()
                lv = None
                continue

            # Validate same lane
            lv_wp = carla_map.get_waypoint(lv_loc, project_to_road=True)
            if lv_wp.road_id != ev_wp.road_id or lv_wp.lane_id != ev_wp.lane_id:
                lv.destroy()
                lv = None
                continue

            break  # Found valid lead

        if lv is None:
            if debug_count < 5:
                print(f"  [skip] sp#{sp_idx} @ ({sp.location.x:.0f},{sp.location.y:.0f}) yaw={ev_yaw:.0f}: no valid lead (7 offsets tried)")
                # Test one offset to see what happens
                tx = sp.location.x + INITIAL_GAP * math.cos(ev_yaw_rad)
                ty = sp.location.y + INITIAL_GAP * math.sin(ev_yaw_rad)
                test_tf = carla.Transform(carla.Location(x=tx, y=ty, z=sp.location.z), ev_tf.rotation)
                try:
                    tv = world.spawn_actor(lead_bp, test_tf)
                    tv_loc = tv.get_transform().location
                    gap = ev_loc.distance(tv_loc)
                    tw = carla_map.get_waypoint(tv_loc, project_to_road=True)
                    print(f"  [debug]   lead_actual=({tv_loc.x:.0f},{tv_loc.y:.0f}) gap={gap:.0f}m road={tw.road_id if tw else 'None'} lane={tw.lane_id if tw else 'None'} ego_road={ev_wp.road_id} ego_lane={ev_wp.lane_id}")
                    tv.destroy()
                except RuntimeError as e:
                    print(f"  [debug]   lead spawn RuntimeError: {e}")
                debug_count += 1
            ev.destroy()
            fail_count["lead"] += 1
            continue

        lv_tf = lv.get_transform()

        # --- Step 3: Cut-in from spawn points (adjacent lane) ---
        cv = None
        for sp2 in spawn_points:
            d_ego = ev_loc.distance(sp2.location)
            d_lead = lv_tf.location.distance(sp2.location)
            if d_ego < 8 or d_lead < 8:
                continue
            sp2_wp = carla_map.get_waypoint(sp2.location, project_to_road=True)
            if sp2_wp is None:
                continue
            if sp2_wp.road_id != ev_wp.road_id:
                continue
            if sp2_wp.lane_id == ev_wp.lane_id:
                continue
            # Direction check (use waypoint yaw, not vehicle yaw)
            y2 = sp2_wp.transform.rotation.yaw
            yd = abs(y2 - ev_wp.transform.rotation.yaw) % 360
            if yd > 180:
                yd = 360 - yd
            if yd > 90:
                continue
            try:
                cv = world.spawn_actor(cutin_bp, sp2)
            except RuntimeError:
                continue

            # Wait for physics to sync transform
            for _ in range(10):
                world.tick()
                time.sleep(0.01)

            # Validate cutin not silently relocated
            cv_loc = cv.get_transform().location
            cv_drift = cv_loc.distance(sp2.location)
            if cv_drift > 10.0 or cv_loc.length() < 5.0:
                cv.destroy()
                continue

            break

        if cv is None:
            fail_count["cutin"] += 1
            ev.destroy()
            lv.destroy()
            continue

        # SUCCESS!
        ego_vehicle = ev
        lead_vehicle = lv
        cutin_vehicle = cv
        ego_yaw_saved = ev_yaw
        print(f"      Spawn #{sp_idx}: all 3 vehicles placed")
        break

    if ego_vehicle is None or lead_vehicle is None or cutin_vehicle is None:
        print("[!] Could not spawn all 3 vehicles")
        print(f"      Failures: ego={fail_count['ego']} lead={fail_count['lead']} cutin={fail_count['cutin']}")
        print(f"      Total spawn points tried: {len(spawn_points)}")
        for v in [ego_vehicle, lead_vehicle, cutin_vehicle]:
            if v is not None:
                try:
                    v.destroy()
                except Exception:
                    pass
        return

    ego_init_tf = ego_vehicle.get_transform()
    lead_init_tf = lead_vehicle.get_transform()
    cutin_init_tf = cutin_vehicle.get_transform()

    print(f"      Ego:    ({ego_init_tf.location.x:.1f}, {ego_init_tf.location.y:.1f})")
    print(f"      Lead:   ({lead_init_tf.location.x:.1f}, {lead_init_tf.location.y:.1f})")
    print(f"      Cut-in: ({cutin_init_tf.location.x:.1f}, {cutin_init_tf.location.y:.1f})")
    print(f"      Gap(lead):  {ego_init_tf.location.distance(lead_init_tf.location):.1f}m")
    print(f"      Gap(cutin): {ego_init_tf.location.distance(cutin_init_tf.location):.1f}m")

    # ===== Sensors =====
    print("[3/4] Attaching sensors...")

    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(W))
    cam_bp.set_attribute("image_size_y", str(H))
    cam_bp.set_attribute("fov", "90")
    camera = world.spawn_actor(
        cam_bp, carla.Transform(carla.Location(x=1.6, z=1.7)),
        attach_to=ego_vehicle)

    cam_buf = [None]
    def cam_cb(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
        cam_buf[0] = arr[:, :, :3][:, :, ::-1]
    camera.listen(cam_cb)

    col_bp = bp_lib.find("sensor.other.collision")
    col_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego_vehicle)
    collision_flag = [False]
    lead_collision = [False]
    non_lead_collision = [False]
    collision_time = [None]
    def col_cb(event):
        collision_flag[0] = True
        collision_time[0] = time.time()
        if event.other_actor.id == lead_vehicle.id or event.other_actor.id == cutin_vehicle.id:
            lead_collision[0] = True
            print(f"  [!] COLLISION with {event.other_actor.type_id}!")
        else:
            non_lead_collision[0] = True
            print(f"  [~] Off-road collision: {event.other_actor.type_id}")
    col_sensor.listen(col_cb)

    print("      [OK] Camera + Collision sensors")

    # --- CAN bus logger ---
    can_logger = CANDatalogger("E:/CARLA/test_report/cutin_can_log.asc")
    print(f"      [OK] CAN bus logger -> test_report/cutin_can_log.asc")

    # ===== Display =====
    pygame.init()
    screen = pygame.display.set_mode((W, H), pygame.SWSURFACE)
    pygame.display.set_caption("CARLA Cut-In Test | SPACE=start  R=reset  Q=quit")
    force_focus()
    font = pygame.font.SysFont("consolas", 16)
    font_big = pygame.font.SysFont("consolas", 28, bold=True)

    # ===== State =====
    scenario_running = False
    scenario_time = 0.0
    cutin_active = False

    metrics = {
        "time": [], "ego_speed": [], "lead_speed": [],
        "cutin_speed": [], "dist_to_lead": [], "dist_to_cutin": [], "ttc": [],
    }

    # CAN bus tracking for acceleration & steering rate
    prev_long_speed = 0.0
    prev_steer_deg = 0.0

    def calc_ttc(d, v_ego, v_target):
        rel_v = v_ego - v_target
        if rel_v > 0.1:
            return d / rel_v
        return 999.0

    def reset_scenario():
        nonlocal scenario_running, scenario_time, cutin_active, prev_long_speed, prev_steer_deg
        scenario_running = False
        scenario_time = 0.0
        cutin_active = False
        collision_flag[0] = False
        lead_collision[0] = False
        non_lead_collision[0] = False
        collision_time[0] = None
        for k in metrics:
            metrics[k].clear()
        prev_long_speed = 0.0
        prev_steer_deg = 0.0
        for v in [lead_vehicle, ego_vehicle, cutin_vehicle]:
            v.apply_control(carla.VehicleControl())
        lead_vehicle.set_transform(lead_init_tf)
        time.sleep(0.2)
        ego_vehicle.set_transform(ego_init_tf)
        time.sleep(0.2)
        cutin_vehicle.set_transform(cutin_init_tf)
        print("  Reset complete. Press SPACE to start.")

    def generate_report():
        report_dir = "E:/CARLA/test_report"
        os.makedirs(report_dir, exist_ok=True)
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            t = np.array(metrics["time"])
            ego_spd = np.array(metrics["ego_speed"]) * 3.6
            lead_spd = np.array(metrics["lead_speed"]) * 3.6
            cutin_spd = np.array(metrics["cutin_speed"]) * 3.6
            dist_lead = np.array(metrics["dist_to_lead"])
            dist_cutin = np.array(metrics["dist_to_cutin"])
            ttc = np.array(metrics["ttc"])

            fig, axes = plt.subplots(2, 2, figsize=(14, 8))
            fig.suptitle("CARLA Cut-In Test Report", fontsize=14, fontweight='bold')

            # Speed
            ax = axes[0, 0]
            ax.plot(t, ego_spd, 'b-', label='Ego', linewidth=2)
            ax.plot(t, lead_spd, 'r-', label='Lead', linewidth=2)
            ax.plot(t, cutin_spd, 'y-', label='Cut-in', linewidth=2)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Speed (km/h)")
            ax.set_title("Vehicle Speed")
            ax.legend()
            ax.grid(True)

            # Distance
            ax = axes[0, 1]
            ax.plot(t, dist_lead, 'g-', label='Ego-Lead', linewidth=2)
            ax.plot(t, dist_cutin, 'm-', label='Ego-CutIn', linewidth=2)
            ax.axhline(y=SAFETY_DIST, color='r', linestyle='--', label=f'Safety ({SAFETY_DIST}m)')
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Distance (m)")
            ax.set_title("Following Distance")
            ax.legend()
            ax.grid(True)

            # TTC
            ax = axes[1, 0]
            ttc_clipped = np.clip(ttc, 0, 15)
            ax.plot(t, ttc_clipped, 'm-', linewidth=2)
            ax.axhline(y=1.5, color='r', linestyle='--', label='Critical (1.5s)')
            ax.axhline(y=3.0, color='orange', linestyle='--', label='Warning (3.0s)')
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("TTC (s)")
            ax.set_title("Time-to-Collision")
            ax.legend()
            ax.grid(True)

            # Summary
            ax = axes[1, 1]
            ax.axis('off')
            min_dist_c = np.min(dist_cutin) if len(dist_cutin) > 0 else 0
            min_dist_l = np.min(dist_lead) if len(dist_lead) > 0 else 0
            min_dist = min(min_dist_c, min_dist_l)
            min_ttc = np.min(ttc_clipped) if len(ttc_clipped) > 0 else 0
            verdict = "COLLISION" if lead_collision[0] else ("PASS" if min_dist > SAFETY_DIST else "WARNING")
            vc = {'PASS': 'green', 'WARNING': 'orange', 'COLLISION': 'red'}.get(verdict, 'black')

            summary = [
                f"Test: Cut-In Scenario",
                f"",
                f"Speed:          {EGO_SPEED} km/h",
                f"Initial Gap:    {INITIAL_GAP} m",
                f"Cut-in offset:  {CUTIN_LATERAL_OFFSET} m",
                f"",
                f"Min Dist(lead): {min_dist_l:.2f} m",
                f"Min Dist(cut):  {min_dist_c:.2f} m",
                f"Min TTC:        {min_ttc:.2f} s",
                f"Collision:      {'YES' if lead_collision[0] else 'NO'}",
                f"",
                f"VERDICT:  {verdict}",
            ]
            for i, line in enumerate(summary):
                c = vc if 'VERDICT' in line else 'black'
                w = 'bold' if 'VERDICT' in line else 'normal'
                ax.text(0.1, 0.95 - i * 0.075, line, transform=ax.transAxes,
                        fontsize=11, fontweight=w, color=c, fontfamily='monospace')

            plt.tight_layout()
            path = os.path.join(report_dir, "cutin_report.png")
            plt.savefig(path, dpi=150)
            plt.close()
            print(f"\n  Report saved: {path}")
            print(f"  VERDICT: {verdict}")
            return verdict
        except ImportError:
            print("  [!] matplotlib not available")
            return "UNKNOWN"

    # ===== Main loop =====
    print("[4/4] Ready!")
    print("      SPACE = start  R = reset  Q = quit")
    print()

    clock = pygame.time.Clock()

    try:
        while True:
            clock.tick(int(1 / DT))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        return
                    elif event.key == pygame.K_SPACE and not scenario_running:
                        scenario_running = True
                        scenario_time = 0.0
                        cutin_active = False
                        collision_flag[0] = False
                        lead_collision[0] = False
                        non_lead_collision[0] = False
                        for k in metrics:
                            metrics[k].clear()
                        target_ms = EGO_SPEED / 3.6
                        lead_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                        ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                        cutin_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                        print("  Scenario STARTED!")
                    elif event.key == pygame.K_r:
                        reset_scenario()

            while msvcrt.kbhit():
                key = msvcrt.getch()
                ch = key.decode('ascii', errors='ignore').lower()
                if ch == 'q':
                    return
                elif ch == ' ' and not scenario_running:
                    scenario_running = True
                    scenario_time = 0.0
                    cutin_active = False
                    collision_flag[0] = False
                    lead_collision[0] = False
                    non_lead_collision[0] = False
                    for k in metrics:
                        metrics[k].clear()
                    target_ms = EGO_SPEED / 3.6
                    lead_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                    ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                    cutin_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                    print("  Scenario STARTED!")
                elif ch == 'r':
                    reset_scenario()

            if scenario_running and not lead_collision[0]:
                scenario_time += DT
                target_ms = EGO_SPEED / 3.6

                # --- Lead vehicle: accelerate then cruise ---
                lead_vel = lead_vehicle.get_velocity()
                lead_speed_ms = np.sqrt(lead_vel.x**2 + lead_vel.y**2 + lead_vel.z**2)
                if lead_speed_ms < target_ms:
                    lead_vehicle.apply_control(carla.VehicleControl(
                        throttle=1.0, steer=keep_lane(lead_vehicle, lead_speed_ms, carla_map)))
                else:
                    lead_vehicle.apply_control(carla.VehicleControl(
                        throttle=0.0, brake=0.1, steer=keep_lane(lead_vehicle, lead_speed_ms, carla_map)))

                # --- Cut-in vehicle behavior ---
                cutin_vel = cutin_vehicle.get_velocity()
                cutin_speed_ms = np.sqrt(cutin_vel.x**2 + cutin_vel.y**2 + cutin_vel.z**2)

                if scenario_time > CUTIN_AFTER_SEC and not cutin_active:
                    cutin_active = True
                    print(f"  [!] Cut-in vehicle CHANGING LANE at {cutin_speed_ms * 3.6:.0f} km/h!")

                lateral_offset = 999.0
                if cutin_active:
                    cutin_tf = cutin_vehicle.get_transform()
                    ego_tf = ego_vehicle.get_transform()
                    cutin_yaw = math.radians(cutin_tf.rotation.yaw)

                    dx = cutin_tf.location.x - ego_tf.location.x
                    dy = cutin_tf.location.y - ego_tf.location.y
                    lateral_offset = dx * (-math.sin(cutin_yaw)) + dy * math.cos(cutin_yaw)

                    if abs(lateral_offset) > 0.5:
                        steer_dir = -np.sign(lateral_offset) * 0.4
                    else:
                        steer_dir = keep_lane(cutin_vehicle, cutin_speed_ms, carla_map)

                    cutin_target = CUTIN_SPEED / 3.6
                    if cutin_speed_ms < cutin_target:
                        cutin_ctrl = carla.VehicleControl(throttle=1.0, steer=steer_dir)
                    else:
                        cutin_ctrl = carla.VehicleControl(throttle=0.0, brake=0.05, steer=steer_dir)
                    cutin_vehicle.apply_control(cutin_ctrl)
                else:
                    if cutin_speed_ms < target_ms:
                        cutin_vehicle.apply_control(carla.VehicleControl(
                            throttle=1.0, steer=keep_lane(cutin_vehicle, cutin_speed_ms, carla_map)))
                    else:
                        cutin_vehicle.apply_control(carla.VehicleControl(
                            throttle=0.0, brake=0.1, steer=keep_lane(cutin_vehicle, cutin_speed_ms, carla_map)))

                # --- Ego vehicle ACC + AEB ---
                ego_vel = ego_vehicle.get_velocity()
                ego_speed_ms = np.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)

                ego_loc = ego_vehicle.get_transform().location
                lead_loc = lead_vehicle.get_transform().location
                cutin_loc = cutin_vehicle.get_transform().location
                dist_lead = ego_loc.distance(lead_loc)
                dist_cutin = ego_loc.distance(cutin_loc)

                nearest_dist = min(dist_lead, dist_cutin)
                is_braking = cutin_active and dist_cutin < dist_lead

                ego_ctrl = aeb_control(ego_speed_ms, target_ms, nearest_dist, is_braking)
                ego_ctrl.steer = keep_lane(ego_vehicle, ego_speed_ms, carla_map)
                ego_vehicle.apply_control(ego_ctrl)

                # --- Record metrics ---
                ttc_lead = calc_ttc(dist_lead, ego_speed_ms, lead_speed_ms)
                ttc_cutin = calc_ttc(dist_cutin, ego_speed_ms, cutin_speed_ms)
                ttc_val = min(ttc_lead, ttc_cutin)

                metrics["time"].append(scenario_time)
                metrics["ego_speed"].append(ego_speed_ms)
                metrics["lead_speed"].append(lead_speed_ms)
                metrics["cutin_speed"].append(cutin_speed_ms)
                metrics["dist_to_lead"].append(dist_lead)
                metrics["dist_to_cutin"].append(dist_cutin)
                metrics["ttc"].append(ttc_val)

                # --- CAN bus logging (Vector .asc format) ---
                ego_fwd = ego_vehicle.get_transform().get_forward_vector()
                ego_long_speed_ms = ego_vel.x * ego_fwd.x + ego_vel.y * ego_fwd.y
                nearest = lead_vehicle if dist_lead <= dist_cutin else cutin_vehicle
                can_logger.log_from_vehicle(
                    ego_vehicle,
                    radar_target_vehicle=nearest,
                    prev_velocity=prev_long_speed,
                    prev_steering=prev_steer_deg,
                    dt=DT
                )
                prev_long_speed = ego_long_speed_ms
                prev_steer_deg = -ego_ctrl.steer * 720.0

                # End conditions
                if lead_collision[0]:
                    print("  Scenario COMPLETE - COLLISION!")
                    scenario_running = False
                    generate_report()
                elif cutin_active and abs(lateral_offset) < 0.5 and scenario_time > CUTIN_AFTER_SEC + CUTIN_DURATION + 5:
                    print("  Scenario COMPLETE!")
                    scenario_running = False
                    generate_report()
                elif scenario_time > 30:
                    print("  Scenario timeout.")
                    scenario_running = False
                    generate_report()

            # --- Render ---
            screen.fill((0, 0, 0))

            if cam_buf[0] is not None:
                cam_surface = pygame.surfarray.make_surface(cam_buf[0].swapaxes(0, 1))
                cam_scaled = pygame.transform.scale(cam_surface, (W, H // 2))
                screen.blit(cam_scaled, (0, 0))

            if scenario_running or len(metrics["time"]) > 0:
                ego_vel = ego_vehicle.get_velocity()
                ego_spd = 3.6 * np.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)
                ego_loc = ego_vehicle.get_transform().location
                d_lead = ego_loc.distance(lead_vehicle.get_transform().location)
                d_cutin = ego_loc.distance(cutin_vehicle.get_transform().location)

                hud_lines = [
                    f"Speed: {ego_spd:.0f} km/h",
                    f"D(lead):  {d_lead:.1f} m",
                    f"D(cutin): {d_cutin:.1f} m",
                    f"Time:  {scenario_time:.1f} s",
                ]
                for i, line in enumerate(hud_lines):
                    color = (255, 255, 255)
                    if "D(cutin)" in line and d_cutin < 10:
                        color = (255, 50, 50)
                    s = font.render(line, True, color)
                    screen.blit(s, (10, 10 + i * 22))

                if cutin_active:
                    s = font_big.render("CUT-IN!", True, (255, 200, 0))
                    screen.blit(s, (W - 200, 10))

                if lead_collision[0]:
                    s = font_big.render("COLLISION!", True, (255, 0, 0))
                    screen.blit(s, (W // 2 - 80, H // 4))
                elif non_lead_collision[0]:
                    s = font.render("Off-road collision (not counted)", True, (255, 200, 0))
                    screen.blit(s, (W // 2 - 160, H // 4))

            # Chart
            chart_y = H // 2
            chart_h = H // 2
            chart_surface = pygame.Surface((W, chart_h))
            chart_surface.fill((20, 20, 30))
            screen.blit(chart_surface, (0, chart_y))

            if len(metrics["time"]) > 2:
                t = np.array(metrics["time"])
                dl = np.array(metrics["dist_to_lead"])
                dc = np.array(metrics["dist_to_cutin"])
                es = np.array(metrics["ego_speed"]) * 3.6
                ls = np.array(metrics["lead_speed"]) * 3.6
                cs = np.array(metrics["cutin_speed"]) * 3.6
                t_max = max(t[-1], 1.0)

                for arr, color in [(dl, (0, 200, 0)), (dc, (200, 0, 200)),
                                   (es, (50, 100, 255)), (ls, (255, 50, 50)),
                                   (cs, (255, 255, 50))]:
                    for i in range(1, len(t)):
                        x1 = int((t[i-1] / t_max) * (W - 20)) + 10
                        y1 = chart_y + chart_h - int(min(arr[i-1], 80) / 80 * (chart_h - 30)) - 15
                        x2 = int((t[i] / t_max) * (W - 20)) + 10
                        y2 = chart_y + chart_h - int(min(arr[i], 80) / 80 * (chart_h - 30)) - 15
                        pygame.draw.line(screen, color, (x1, y1), (x2, y2), 2)

            labels = [
                ("Green=D(lead) Blue=EgoSpd", (255, 255, 255)),
                ("Magenta=D(cutin) Red=LeadSpd", (255, 255, 255)),
                ("Yellow=CutinSpd", (255, 255, 255)),
            ]
            for i, (txt, color) in enumerate(labels):
                s = font.render(txt, True, color)
                screen.blit(s, (W - 320, chart_y + 5 + i * 20))

            if not scenario_running and len(metrics["time"]) == 0:
                s = font_big.render("Press SPACE to start", True, (200, 200, 200))
                screen.blit(s, (W // 2 - 140, H // 4 + 50))

            pygame.display.flip()

    finally:
        if len(metrics["time"]) > 2 and scenario_running:
            generate_report()
        print("\nCleaning up...")
        camera.stop()
        col_sensor.stop()
        for actor in [camera, col_sensor, lead_vehicle, ego_vehicle, cutin_vehicle]:
            try:
                actor.destroy()
            except Exception:
                pass
        pygame.quit()
        frames_total = can_logger.get_message_count()
        can_logger.close()
        print(f"      CAN frames logged: {frames_total}")
        print("Done!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

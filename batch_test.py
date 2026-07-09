"""
CARLA AEB Batch Testing Script
================================
Runs the AEB (Automatic Emergency Braking) test scenario multiple times
with different parameter combinations and generates a summary report.

Parameter sweep:
  - Ego speed:      [40, 50, 60, 70, 80] km/h
  - Following gap:  [15, 20, 30, 40] m
  - Lead decel:     [4.0, 6.0, 8.0, 10.0] m/s^2

Total combinations: 5 * 4 * 4 = 80 tests

Usage:
    python batch_test.py

Output:
    E:/CARLA/test_report/batch_report.png
    E:/CARLA/test_report/batch_results.csv
"""

import carla
import numpy as np
import time
import os
import sys
import math
import itertools
from datetime import datetime

# ===================================================================
# Parameter Sweep Configuration
# ===================================================================
EGO_SPEEDS = [40, 50, 60, 70, 80]           # km/h
FOLLOWING_GAPS = [15, 20, 30, 40]           # meters
LEAD_DECELS = [4.0, 6.0, 8.0, 10.0]         # m/s^2

# ===================================================================
# Scenario Constants (matching scenario_aeb_test.py)
# ===================================================================
LEAD_SPEED_KMH = 60        # lead vehicle initial speed (km/h)
BRAKE_AFTER_SEC = 3.0      # seconds before lead vehicle brakes
SAFETY_DIST = 2.0           # meters - below this = collision risk
DT = 0.05                   # simulation timestep (seconds)
MAX_SCENARIO_TIME = 30.0    # max seconds per test
CARLA_HOST = "localhost"
CARLA_PORT = 2000
CARLA_TIMEOUT = 15.0
REPORT_DIR = "E:/CARLA/test_report"
REPORT_PATH = os.path.join(REPORT_DIR, "batch_report.png")

# ===================================================================
# ACC + AEB Controller (aligned with scenario_aeb_test.py)
# ===================================================================
def simple_acc(ego_v_ms, target_speed_ms, dist_to_lead, lead_is_braking):
    """
    Combined ACC (longitudinal) + AEB controller.
    When lead_is_braking=True: pure AEB mode, no throttle, TTC-based braking.
    Returns (throttle, brake) tuple.
    """
    throttle = 0.0
    brake = 0.0

    # ===== AEB mode: lead is braking, NO THROTTLE ALLOWED =====
    if lead_is_braking:
        throttle = 0.0
        # TTC-based braking: shorter TTC = harder brake
        rel_v = max(ego_v_ms, 0.1)
        ttc = dist_to_lead / rel_v
        if dist_to_lead < SAFETY_DIST + 3:
            brake = 1.0  # emergency full brake
        elif ttc < 2.0:
            brake = min(1.0, 1.5 - ttc * 0.5)  # aggressive
        elif ttc < 4.0:
            brake = min(1.0, 0.8 - ttc * 0.15)
        else:
            brake = 0.3  # light pre-braking
        return throttle, brake

    # ===== Normal ACC mode =====
    desired_gap = max(SAFETY_DIST + 5, ego_v_ms * 2.0)

    if dist_to_lead < SAFETY_DIST + 2:
        # Emergency brake!
        throttle = 0.0
        brake = 1.0
    elif dist_to_lead < desired_gap:
        # Too close - slow down proportionally
        gap_ratio = dist_to_lead / desired_gap  # 0..1
        if gap_ratio < 0.5:
            # Very close - hard brake
            throttle = 0.0
            brake = min(1.0, (1.0 - gap_ratio) * 1.5)
        else:
            # Moderately close - ease off throttle
            speed_error = target_speed_ms - ego_v_ms
            throttle = max(0, speed_error * 0.3 * gap_ratio)
            brake = max(0, -speed_error * 0.3)
    else:
        # Normal cruising - maintain target speed
        speed_error = target_speed_ms - ego_v_ms
        if speed_error > 2:
            throttle = 1.0
            brake = 0.0
        elif speed_error > 0:
            throttle = min(1.0, 0.5 + speed_error * 0.3)
            brake = 0.0
        elif speed_error > -2:
            throttle = 0.0
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(1.0, abs(speed_error) * 0.5)

    return throttle, brake


def calc_ttc(distance, v_ego, v_lead):
    """Calculate Time-To-Collision."""
    relative_v = v_ego - v_lead  # positive = approaching
    if relative_v > 0.1:
        return distance / relative_v
    return 999.0  # safe (not approaching)


# ===================================================================
# Lane Keeping Controller (aligned with scenario_aeb_test.py)
# ===================================================================
def keep_lane(world_map, vehicle_loc, vehicle_yaw_deg, speed_ms):
    """
    Pure lane-keeping: returns steer value [-1, 1] to follow road.
    Uses waypoint.next() for look-ahead along road network.
    """
    try:
        wp = world_map.get_waypoint(vehicle_loc, project_to_road=True)
        look_ahead = max(8.0, speed_ms * 1.5)
        next_wps = wp.next(look_ahead)
        if not next_wps:
            return 0.0
        target_loc = next_wps[0].transform.location
        target_yaw = math.radians(next_wps[0].transform.rotation.yaw)
        vyaw = math.radians(vehicle_yaw_deg)

        # Yaw error (heading difference)
        yaw_err = target_yaw - vyaw
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        # Cross-track error (lateral deviation from road center)
        dx = target_loc.x - vehicle_loc.x
        dy = target_loc.y - vehicle_loc.y
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
        return 0.0


# ===================================================================
# CARLA Connection
# ===================================================================
def connect_carla():
    """Connect to CARLA server and load a suitable map."""
    print("[INFO] Connecting to CARLA at %s:%d ..." % (CARLA_HOST, CARLA_PORT))
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(CARLA_TIMEOUT)

    # Try to load Town04 (preferred for straight roads)
    available_maps = client.get_available_maps()
    target_map = None
    for m in available_maps:
        if "Town04" in m:
            target_map = m
            break
    if target_map is None:
        for m in available_maps:
            if "Town01" in m:
                target_map = m
                break

    if target_map:
        world = client.load_world(target_map)
        time.sleep(2)
        print("[INFO] Loaded map: %s" % target_map)
    else:
        world = client.get_world()
        print("[INFO] Using existing map: %s" % world.get_map().name)

    return client, world


# ===================================================================
# Vehicle Spawning (ego-first approach, aligned with scenario_aeb_test.py)
# ===================================================================
def spawn_vehicles(world, ego_speed_kmh, following_gap):
    """
    Spawn ego and lead vehicles using the proven method:
    - Ego at a spawn point (validated)
    - Lead computed geometrically ahead of ego with multiple offset retries
    - Position validation to catch silent (0,0,0) relocation

    Returns (lead_vehicle, ego_vehicle) or raises RuntimeError.
    """
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    # --- Ego vehicle (blue) ---
    ego_bp = bp_lib.filter("vehicle.tesla.*")[0]
    if ego_bp.has_attribute("color"):
        ego_bp.set_attribute("color", "0,0,255")

    # --- Lead vehicle (red) ---
    lead_bp = bp_lib.filter("vehicle.*")[0]
    if lead_bp.has_attribute("color"):
        lead_bp.set_attribute("color", "255,0,0")

    ego_vehicle = None
    lead_vehicle = None

    for sp_idx, sp in enumerate(spawn_points):
        # --- Step 1: Spawn ego at spawn point ---
        try:
            ev = world.spawn_actor(ego_bp, sp)
        except RuntimeError:
            continue

        # CRITICAL: get_transform() returns (0,0,0) immediately after spawn.
        # We must use the spawn point for position calculations, not get_transform().
        # Let a few ticks pass so transforms settle.
        for _ in range(10):
            world.tick()
            time.sleep(0.01)

        ev_loc = ev.get_transform().location
        ev_yaw = ev.get_transform().rotation.yaw
        ev_yaw_rad = math.radians(ev_yaw)

        # --- Step 2: Compute lead position ahead of ego using spawn point coords ---
        lv = None
        offsets = [following_gap,
                   following_gap + 2, following_gap - 2,
                   following_gap + 5, following_gap - 5,
                   following_gap + 8, following_gap - 8]
        for offset_m in offsets:
            # Use spawn point (sp) location for geometry, NOT ev_loc
            lx = sp.location.x + offset_m * math.cos(ev_yaw_rad)
            ly = sp.location.y + offset_m * math.sin(ev_yaw_rad)
            lt = carla.Transform(
                carla.Location(x=lx, y=ly, z=ev_loc.z),
                carla.Rotation(yaw=ev_yaw)
            )
            try:
                lv = world.spawn_actor(lead_bp, lt)
            except RuntimeError:
                continue
            # Let ticks settle for lead too
            for _ in range(10):
                world.tick()
                time.sleep(0.01)
            lv_loc = lv.get_transform().location
            # Validate actual gap is reasonable
            actual_gap = ev_loc.distance(lv_loc)
            if actual_gap < 3 or actual_gap > following_gap * 3:
                try:
                    lv.destroy()
                except Exception:
                    pass
                lv = None
                continue
            break

        if lv is None:
            try:
                ev.destroy()
            except Exception:
                pass
            continue

        # SUCCESS
        ego_vehicle = ev
        lead_vehicle = lv
        break

    if ego_vehicle is None or lead_vehicle is None:
        raise RuntimeError("Could not spawn ego and lead vehicles after trying all spawn points")

    return lead_vehicle, ego_vehicle


# ===================================================================
# Sensor Setup
# ===================================================================
def attach_sensors(world, ego_vehicle, lead_vehicle):
    """
    Attach collision sensor to ego vehicle.
    Distance is computed via get_transform() instead of obs_sensor (unreliable).
    Returns (col_sensor, state_dict).
    """
    bp_lib = world.get_blueprint_library()

    # Collision sensor - filter to only count collisions with lead_vehicle
    col_bp = bp_lib.find("sensor.other.collision")
    col_sensor = world.spawn_actor(
        col_bp,
        carla.Transform(),
        attach_to=ego_vehicle
    )
    state = {
        "collision": False,
        "collision_time": None,
        "lead_id": lead_vehicle.id,
    }

    def col_cb(event):
        # Only register collision with lead vehicle
        if event.other_actor.id == state["lead_id"]:
            state["collision"] = True
            state["collision_time"] = time.time()
    col_sensor.listen(col_cb)

    return col_sensor, state


# ===================================================================
# Headless AEB Test Runner
# ===================================================================
def run_single_test(world, carla_map, ego_speed_kmh, following_gap, lead_decel):
    """
    Run one AEB test headlessly (no pygame, no display).

    Parameters
    ----------
    world : carla.World
    carla_map : carla.Map
    ego_speed_kmh : float - ego initial speed in km/h
    following_gap : float - initial gap in meters
    lead_decel : float - lead vehicle braking deceleration in m/s^2

    Returns
    -------
    dict with keys:
        verdict, min_distance, min_ttc, collision, duration,
        ego_speed_kmh, following_gap, lead_decel
    """
    lead_vehicle = None
    ego_vehicle = None
    col_sensor = None

    try:
        # Spawn vehicles
        lead_vehicle, ego_vehicle = spawn_vehicles(
            world, ego_speed_kmh, following_gap
        )

        # Attach sensors
        col_sensor, state = attach_sensors(
            world, ego_vehicle, lead_vehicle
        )

        # Allow physics to settle
        time.sleep(0.5)
        for _ in range(5):
            world.tick()

        # --- Accelerate both vehicles to target speed ---
        target_ms = ego_speed_kmh / 3.6
        lead_target_ms = LEAD_SPEED_KMH / 3.6

        # Warm-up: accelerate for up to 8 seconds or until both reach speed
        warmup_time = 0.0
        warmup_max = 8.0
        while warmup_time < warmup_max:
            lead_vel = lead_vehicle.get_velocity()
            lead_spd = math.sqrt(lead_vel.x**2 + lead_vel.y**2 + lead_vel.z**2)
            ego_vel = ego_vehicle.get_velocity()
            ego_spd = math.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)

            if lead_spd >= lead_target_ms * 0.9 and ego_spd >= target_ms * 0.9:
                break

            # Lead control
            lead_steer = keep_lane(carla_map, lead_vehicle.get_transform().location,
                                   lead_vehicle.get_transform().rotation.yaw, lead_spd)
            if lead_spd < lead_target_ms:
                lead_vehicle.apply_control(carla.VehicleControl(
                    throttle=1.0, steer=lead_steer))
            else:
                lead_vehicle.apply_control(carla.VehicleControl(
                    throttle=0.3, steer=lead_steer))

            # Ego control
            ego_steer = keep_lane(carla_map, ego_vehicle.get_transform().location,
                                  ego_vehicle.get_transform().rotation.yaw, ego_spd)
            if ego_spd < target_ms:
                ego_vehicle.apply_control(carla.VehicleControl(
                    throttle=1.0, steer=ego_steer))
            else:
                ego_vehicle.apply_control(carla.VehicleControl(
                    throttle=0.3, steer=ego_steer))

            world.tick()
            time.sleep(DT)
            warmup_time += DT

        # --- Main scenario loop ---
        scenario_time = 0.0
        lead_braking = False
        distances = []
        ttcs = []

        while scenario_time < MAX_SCENARIO_TIME:
            scenario_time += DT

            lead_vel = lead_vehicle.get_velocity()
            lead_speed_ms = math.sqrt(lead_vel.x**2 + lead_vel.y**2 + lead_vel.z**2)
            ego_vel = ego_vehicle.get_velocity()
            ego_speed_ms = math.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)

            # Lead vehicle behavior
            if scenario_time > BRAKE_AFTER_SEC and not lead_braking:
                lead_braking = True

            if lead_braking:
                lead_ctrl = carla.VehicleControl()
                lead_ctrl.throttle = 0.0
                lead_ctrl.brake = min(1.0, lead_decel / 10.0)
                lead_ctrl.steer = keep_lane(
                    carla_map, lead_vehicle.get_transform().location,
                    lead_vehicle.get_transform().rotation.yaw, lead_speed_ms)
                lead_vehicle.apply_control(lead_ctrl)
            else:
                lead_steer = keep_lane(
                    carla_map, lead_vehicle.get_transform().location,
                    lead_vehicle.get_transform().rotation.yaw, lead_speed_ms)
                if lead_speed_ms < lead_target_ms:
                    lead_vehicle.apply_control(carla.VehicleControl(
                        throttle=0.8, steer=lead_steer))
                else:
                    lead_vehicle.apply_control(carla.VehicleControl(
                        throttle=0.0, steer=lead_steer))

            # Ego ACC + AEB (distance via get_transform, not obs_sensor)
            ego_loc = ego_vehicle.get_transform().location
            lead_loc = lead_vehicle.get_transform().location
            dist = ego_loc.distance(lead_loc)

            ego_throttle, ego_brake = simple_acc(
                ego_speed_ms, target_ms, dist, lead_braking)
            ego_ctrl = carla.VehicleControl()
            ego_ctrl.throttle = ego_throttle
            ego_ctrl.brake = ego_brake
            ego_ctrl.steer = keep_lane(
                carla_map, ego_loc,
                ego_vehicle.get_transform().rotation.yaw, ego_speed_ms)
            ego_vehicle.apply_control(ego_ctrl)

            # Record metrics
            ttc_val = calc_ttc(dist, ego_speed_ms, lead_speed_ms)
            distances.append(dist)
            ttcs.append(ttc_val)

            # Check collision
            if state["collision"]:
                break

            # Check end conditions
            if lead_braking and lead_speed_ms < 0.5:
                if ego_speed_ms < 0.5 or scenario_time > BRAKE_AFTER_SEC + 15:
                    break

            world.tick()
            time.sleep(DT)

        # --- Compute results ---
        min_dist = min(distances) if distances else 999.0
        min_ttc = min(ttcs) if ttcs else 999.0

        if state["collision"]:
            verdict = "COLLISION"
        elif min_dist > SAFETY_DIST:
            verdict = "PASS"
        else:
            verdict = "WARNING"

        return {
            "verdict": verdict,
            "min_distance": min_dist,
            "min_ttc": min_ttc,
            "collision": state["collision"],
            "duration": scenario_time,
            "ego_speed_kmh": ego_speed_kmh,
            "following_gap": following_gap,
            "lead_decel": lead_decel,
        }

    except Exception as e:
        return {
            "verdict": "ERROR",
            "min_distance": -1.0,
            "min_ttc": -1.0,
            "collision": False,
            "duration": 0.0,
            "ego_speed_kmh": ego_speed_kmh,
            "following_gap": following_gap,
            "lead_decel": lead_decel,
            "error": str(e),
        }

    finally:
        # Cleanup all actors
        if col_sensor is not None:
            try:
                col_sensor.stop()
                col_sensor.destroy()
            except Exception:
                pass
        for actor in [lead_vehicle, ego_vehicle]:
            if actor is not None:
                try:
                    actor.apply_control(carla.VehicleControl())
                    actor.destroy()
                except Exception:
                    pass
        time.sleep(0.3)


# ===================================================================
# Server Health Check
# ===================================================================
def server_health_check(world):
    """
    Check if CARLA server is in a healthy state.
    Cleans up any leftover actors and verifies spawn functionality.
    """
    # Clean up any lingering vehicles
    vehicles = list(world.get_actors().filter("vehicle.*"))
    for v in vehicles:
        try:
            v.destroy()
        except Exception:
            pass
    time.sleep(0.5)

    # Verify remaining actor count
    remaining = list(world.get_actors().filter("vehicle.*"))
    if remaining:
        print("  [WARN] %d vehicles still in world after cleanup" % len(remaining))

    return True


# ===================================================================
# Batch Test Runner
# ===================================================================
def run_batch_tests():
    """Run all parameter combinations and collect results."""
    # Build test matrix
    combos = list(itertools.product(EGO_SPEEDS, FOLLOWING_GAPS, LEAD_DECELS))
    total = len(combos)

    print()
    print("=" * 65)
    print("  CARLA AEB Batch Testing")
    print("=" * 65)
    print()
    print("  Parameter sweep:")
    print("    Ego speeds:      %s km/h" % EGO_SPEEDS)
    print("    Following gaps:  %s m" % FOLLOWING_GAPS)
    print("    Lead decels:     %s m/s^2" % LEAD_DECELS)
    print()
    print("  Total tests: %d" % total)
    print("  Timeout per test: %.0f seconds" % MAX_SCENARIO_TIME)
    print("  Started at: %s" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print()

    # Connect to CARLA
    client, world = connect_carla()
    carla_map = world.get_map()

    # Initial cleanup
    server_health_check(world)

    results = []
    start_time = time.time()
    error_streak = 0  # consecutive errors - trigger server recovery

    for idx, (speed, gap, decel) in enumerate(combos, start=1):
        progress = "[%3d/%d]" % (idx, total)
        params_str = "Speed=%d Gap=%d Decel=%.1f" % (speed, gap, decel)
        sys.stdout.write("  %s %-32s ... " % (progress, params_str))
        sys.stdout.flush()

        # Server recovery: reload map after 3 consecutive errors
        if error_streak >= 3:
            print("RECOVERING")
            print("  [RECOVER] Server health issue detected, reloading map...")
            try:
                # Load a different map first, then switch back
                for m in client.get_available_maps():
                    if "Town03" in m:
                        client.load_world(m)
                        time.sleep(2)
                        break
                # Switch back to target map
                for m in client.get_available_maps():
                    if "Town04" in m:
                        client.load_world(m)
                        time.sleep(2)
                        break
                world = client.get_world()
                carla_map = world.get_map()
                server_health_check(world)
            except Exception as e:
                print("  [RECOVER] Map reload failed: %s" % e)
            error_streak = 0
            sys.stdout.write("  %s %-32s ... " % (progress, params_str))
            sys.stdout.flush()

        test_start = time.time()
        result = run_single_test(world, carla_map, speed, gap, decel)
        test_elapsed = time.time() - test_start

        results.append(result)

        # Track error streak
        if result["verdict"] == "ERROR":
            error_streak += 1
        else:
            error_streak = 0

        # Format output
        if result["verdict"] == "ERROR":
            print("ERROR (%s)" % result.get("error", "unknown"))
        else:
            detail = "min_dist=%.1fm, min_ttc=%.1fs, %.1fs" % (
                result["min_distance"],
                result["min_ttc"],
                test_elapsed,
            )
            print("%s (%s)" % (result["verdict"], detail))

        # Periodic cleanup between tests
        server_health_check(world)

    elapsed_total = time.time() - start_time

    # Print summary
    pass_count = sum(1 for r in results if r["verdict"] == "PASS")
    warn_count = sum(1 for r in results if r["verdict"] == "WARNING")
    col_count = sum(1 for r in results if r["verdict"] == "COLLISION")
    err_count = sum(1 for r in results if r["verdict"] == "ERROR")

    print()
    print("  ===== BATCH TEST SUMMARY =====")
    print("  Total tests:  %d" % total)
    if total > 0:
        print("  PASS:         %d (%.1f%%)" % (pass_count, 100.0 * pass_count / total))
        print("  WARNING:      %d (%.1f%%)" % (warn_count, 100.0 * warn_count / total))
        print("  COLLISION:    %d (%.1f%%)" % (col_count, 100.0 * col_count / total))
        if err_count > 0:
            print("  ERROR:        %d (%.1f%%)" % (err_count, 100.0 * err_count / total))
    print("  Total time:   %.1f seconds (%.1f min)" % (elapsed_total, elapsed_total / 60.0))
    print()

    return results


# ===================================================================
# Report Generation
# ===================================================================
def generate_batch_report(results):
    """
    Generate summary report with matplotlib:
      - Heatmap 1: Ego Speed vs Following Gap -> Pass Rate (%)
      - Heatmap 2: Ego Speed vs Lead Decel -> Pass Rate (%)
      - Bar chart: Overall pass/fail/warning counts
      - Table: All test results
    Saves to REPORT_PATH.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[ERROR] matplotlib not available. Cannot generate report.")
        return None

    os.makedirs(REPORT_DIR, exist_ok=True)

    # Filter out errors for heatmap calculations
    valid_results = [r for r in results if r["verdict"] != "ERROR"]

    if not valid_results:
        print("[WARN] No valid results to report.")
        return None

    # --- Compute heatmap data ---
    # Heatmap 1: Speed (cols) x Gap (rows) -> pass rate
    speed_gap_matrix = np.zeros((len(FOLLOWING_GAPS), len(EGO_SPEEDS)))
    speed_gap_counts = np.zeros_like(speed_gap_matrix)
    for r in valid_results:
        si = EGO_SPEEDS.index(r["ego_speed_kmh"])
        gi = FOLLOWING_GAPS.index(r["following_gap"])
        speed_gap_matrix[gi, si] += 1 if r["verdict"] == "PASS" else 0
        speed_gap_counts[gi, si] += 1
    # Avoid division by zero
    with np.errstate(divide="ignore", invalid="ignore"):
        speed_gap_rate = np.where(
            speed_gap_counts > 0,
            (speed_gap_matrix / speed_gap_counts) * 100.0,
            np.nan,
        )

    # Heatmap 2: Speed (cols) x Decel (rows) -> pass rate
    speed_decel_matrix = np.zeros((len(LEAD_DECELS), len(EGO_SPEEDS)))
    speed_decel_counts = np.zeros_like(speed_decel_matrix)
    for r in valid_results:
        si = EGO_SPEEDS.index(r["ego_speed_kmh"])
        di = LEAD_DECELS.index(r["lead_decel"])
        speed_decel_matrix[di, si] += 1 if r["verdict"] == "PASS" else 0
        speed_decel_counts[di, si] += 1
    with np.errstate(divide="ignore", invalid="ignore"):
        speed_decel_rate = np.where(
            speed_decel_counts > 0,
            (speed_decel_matrix / speed_decel_counts) * 100.0,
            np.nan,
        )

    # --- Counts for bar chart ---
    pass_count = sum(1 for r in valid_results if r["verdict"] == "PASS")
    warn_count = sum(1 for r in valid_results if r["verdict"] == "WARNING")
    col_count = sum(1 for r in valid_results if r["verdict"] == "COLLISION")
    err_count = sum(1 for r in results if r["verdict"] == "ERROR")

    # --- Create figure ---
    fig = plt.figure(figsize=(20, 22))
    fig.suptitle(
        "CARLA AEB Batch Test Report\n%s" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        fontsize=16, fontweight="bold", y=0.98,
    )

    # Use gridspec for layout: 2 heatmap rows, 1 bar chart, 1 table
    gs = fig.add_gridspec(4, 2, hspace=0.45, wspace=0.35,
                          top=0.93, bottom=0.03, left=0.08, right=0.95)

    # ---- Heatmap 1: Speed vs Gap ----
    ax1 = fig.add_subplot(gs[0, 0])
    cmap = plt.cm.RdYlGn  # red=bad, green=good
    im1 = ax1.imshow(
        speed_gap_rate,
        cmap=cmap, vmin=0, vmax=100, aspect="auto",
        interpolation="nearest",
    )
    ax1.set_xticks(range(len(EGO_SPEEDS)))
    ax1.set_xticklabels(["%d" % s for s in EGO_SPEEDS], fontsize=10)
    ax1.set_yticks(range(len(FOLLOWING_GAPS)))
    ax1.set_yticklabels(["%d" % g for g in FOLLOWING_GAPS], fontsize=10)
    ax1.set_xlabel("Ego Speed (km/h)")
    ax1.set_ylabel("Following Gap (m)")
    ax1.set_title("Pass Rate (%): Speed vs Gap", fontsize=12, fontweight="bold")
    # Annotate cells
    for i in range(len(FOLLOWING_GAPS)):
        for j in range(len(EGO_SPEEDS)):
            val = speed_gap_rate[i, j]
            if not np.isnan(val):
                color = "white" if val < 50 else "black"
                ax1.text(j, i, "%.0f%%" % val, ha="center", va="center",
                         fontsize=11, fontweight="bold", color=color)
            else:
                ax1.text(j, i, "N/A", ha="center", va="center",
                         fontsize=9, color="gray")
    plt.colorbar(im1, ax=ax1, shrink=0.8, label="Pass Rate (%)")

    # ---- Heatmap 2: Speed vs Decel ----
    ax2 = fig.add_subplot(gs[0, 1])
    im2 = ax2.imshow(
        speed_decel_rate,
        cmap=cmap, vmin=0, vmax=100, aspect="auto",
        interpolation="nearest",
    )
    ax2.set_xticks(range(len(EGO_SPEEDS)))
    ax2.set_xticklabels(["%d" % s for s in EGO_SPEEDS], fontsize=10)
    ax2.set_yticks(range(len(LEAD_DECELS)))
    ax2.set_yticklabels(["%.1f" % d for d in LEAD_DECELS], fontsize=10)
    ax2.set_xlabel("Ego Speed (km/h)")
    ax2.set_ylabel("Lead Decel (m/s^2)")
    ax2.set_title("Pass Rate (%): Speed vs Lead Decel", fontsize=12, fontweight="bold")
    for i in range(len(LEAD_DECELS)):
        for j in range(len(EGO_SPEEDS)):
            val = speed_decel_rate[i, j]
            if not np.isnan(val):
                color = "white" if val < 50 else "black"
                ax2.text(j, i, "%.0f%%" % val, ha="center", va="center",
                         fontsize=11, fontweight="bold", color=color)
            else:
                ax2.text(j, i, "N/A", ha="center", va="center",
                         fontsize=9, color="gray")
    plt.colorbar(im2, ax=ax2, shrink=0.8, label="Pass Rate (%)")

    # ---- Bar chart: Overall counts ----
    ax3 = fig.add_subplot(gs[1, 0])
    categories = ["PASS", "WARNING", "COLLISION"]
    counts = [pass_count, warn_count, col_count]
    colors = ["#2ecc71", "#f39c12", "#e74c3c"]
    if err_count > 0:
        categories.append("ERROR")
        counts.append(err_count)
        colors.append("#95a5a6")

    bars = ax3.bar(categories, counts, color=colors, edgecolor="black", linewidth=0.8)
    ax3.set_ylabel("Number of Tests")
    ax3.set_title("Overall Verdict Distribution", fontsize=12, fontweight="bold")
    ax3.set_ylim(0, max(counts) * 1.2 if max(counts) > 0 else 10)
    ax3.grid(axis="y", alpha=0.3)
    # Add count labels on bars
    for bar, count in zip(bars, counts):
        total_v = len(results)
        pct = 100.0 * count / total_v if total_v > 0 else 0
        ax3.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + max(counts) * 0.02,
            "%d\n(%.1f%%)" % (count, pct),
            ha="center", va="bottom", fontsize=11, fontweight="bold",
        )

    # ---- Summary stats panel ----
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.axis("off")
    # Compute some stats
    all_dists = [r["min_distance"] for r in valid_results if r["min_distance"] > 0]
    all_ttcs = [r["min_ttc"] for r in valid_results if r["min_ttc"] > 0 and r["min_ttc"] < 900]
    durations = [r["duration"] for r in valid_results if r["duration"] > 0]

    stats_lines = [
        ("Test Configuration", True, "#2c3e50"),
        ("", False, "black"),
        ("Speeds:   %s km/h" % EGO_SPEEDS, False, "#333"),
        ("Gaps:     %s m" % FOLLOWING_GAPS, False, "#333"),
        ("Decels:   %s m/s^2" % LEAD_DECELS, False, "#333"),
        ("", False, "black"),
        ("Results Summary", True, "#2c3e50"),
        ("", False, "black"),
        ("Total:       %d tests" % len(results), False, "#333"),
        ("Valid:       %d tests" % len(valid_results), False, "#333"),
        ("PASS:        %d (%.1f%%)" % (pass_count, 100.0 * pass_count / max(len(valid_results), 1)), False, "#2ecc71"),
        ("WARNING:     %d (%.1f%%)" % (warn_count, 100.0 * warn_count / max(len(valid_results), 1)), False, "#f39c12"),
        ("COLLISION:   %d (%.1f%%)" % (col_count, 100.0 * col_count / max(len(valid_results), 1)), False, "#e74c3c"),
        ("", False, "black"),
        ("Distance Stats", True, "#2c3e50"),
        ("", False, "black"),
    ]
    if all_dists:
        stats_lines.append(("Mean min dist:   %.2f m" % np.mean(all_dists), False, "#333"))
        stats_lines.append(("Min min dist:    %.2f m" % np.min(all_dists), False, "#333"))
        stats_lines.append(("Max min dist:    %.2f m" % np.max(all_dists), False, "#333"))
    if all_ttcs:
        stats_lines.append(("", False, "black"))
        stats_lines.append(("TTC Stats", True, "#2c3e50"))
        stats_lines.append(("", False, "black"))
        stats_lines.append(("Mean min TTC:    %.2f s" % np.mean(all_ttcs), False, "#333"))
        stats_lines.append(("Min min TTC:     %.2f s" % np.min(all_ttcs), False, "#333"))
    if durations:
        stats_lines.append(("", False, "black"))
        stats_lines.append(("Avg duration:    %.1f s" % np.mean(durations), False, "#333"))

    for i, (text, is_header, color) in enumerate(stats_lines):
        ax4.text(
            0.05, 0.97 - i * 0.038, text,
            transform=ax4.transAxes,
            fontsize=10.5 if not is_header else 12,
            fontweight="bold" if is_header else "normal",
            color=color, fontfamily="monospace",
            verticalalignment="top",
        )

    # ---- Detailed results table ----
    ax5 = fig.add_subplot(gs[2:, :])
    ax5.axis("off")
    ax5.set_title("Detailed Test Results", fontsize=13, fontweight="bold", pad=15)

    # Table headers
    col_headers = [
        "#", "Speed\n(km/h)", "Gap\n(m)", "Decel\n(m/s^2)",
        "Verdict", "Min Dist\n(m)", "Min TTC\n(s)", "Duration\n(s)"
    ]
    # Build table data (limit to keep readable)
    max_rows = min(len(results), 80)
    cell_data = []
    cell_colors = []
    for i in range(max_rows):
        r = results[i]
        verdict = r["verdict"]
        row = [
            str(i + 1),
            "%d" % r["ego_speed_kmh"],
            "%d" % r["following_gap"],
            "%.1f" % r["lead_decel"],
            verdict,
            "%.2f" % r["min_distance"] if r["min_distance"] >= 0 else "ERR",
            "%.2f" % r["min_ttc"] if r["min_ttc"] >= 0 else "ERR",
            "%.1f" % r["duration"],
        ]
        cell_data.append(row)
        # Color the verdict cell
        vcolor = {
            "PASS": "#d5f5e3",
            "WARNING": "#fdebd0",
            "COLLISION": "#fadbd8",
            "ERROR": "#d5d8dc",
        }.get(verdict, "#ffffff")
        cell_colors.append(["#ffffff"] * 4 + [vcolor] + ["#ffffff"] * 3)

    table = ax5.table(
        cellText=cell_data,
        colLabels=col_headers,
        cellColours=cell_colors,
        colColours=["#34495e"] * len(col_headers),
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8.5)
    table.scale(1.0, 1.3)
    # Style header
    for j in range(len(col_headers)):
        cell = table[0, j]
        cell.set_text_props(color="white", fontweight="bold", fontsize=9)

    # Save
    plt.savefig(REPORT_PATH, dpi=150, bbox_inches="tight")
    plt.close()
    print("[INFO] Report saved: %s" % REPORT_PATH)
    return REPORT_PATH


# ===================================================================
# CSV Export
# ===================================================================
def export_csv(results, csv_path=None):
    """Export results to CSV for further analysis."""
    if csv_path is None:
        csv_path = os.path.join(REPORT_DIR, "batch_results.csv")
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)

    headers = [
        "test_number", "ego_speed_kmh", "following_gap_m", "lead_decel_ms2",
        "verdict", "min_distance_m", "min_ttc_s", "duration_s", "collision"
    ]
    with open(csv_path, "w") as f:
        f.write(",".join(headers) + "\n")
        for i, r in enumerate(results, start=1):
            row = [
                str(i),
                str(r["ego_speed_kmh"]),
                str(r["following_gap"]),
                "%.1f" % r["lead_decel"],
                r["verdict"],
                "%.3f" % r["min_distance"],
                "%.3f" % r["min_ttc"],
                "%.2f" % r["duration"],
                "1" if r["collision"] else "0",
            ]
            f.write(",".join(row) + "\n")

    print("[INFO] CSV exported: %s" % csv_path)
    return csv_path


# ===================================================================
# Main Entry Point
# ===================================================================
def main():
    """Run the full batch test pipeline."""
    print()
    print("  Starting CARLA AEB Batch Test at %s" % datetime.now().strftime("%H:%M:%S"))
    print()

    # Phase 1: Run all tests
    results = run_batch_tests()

    # Phase 2: Generate report
    print("[INFO] Generating batch report...")
    report_path = generate_batch_report(results)

    # Phase 3: Export CSV
    csv_path = export_csv(results)

    # Final summary
    print()
    print("=" * 65)
    print("  Batch testing complete!")
    if report_path:
        print("  Report:  %s" % report_path)
    print("  CSV:     %s" % csv_path)
    print("=" * 65)
    print()

    return results


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Batch test cancelled by user.")
        sys.exit(130)
    except Exception as e:
        print("\n[FATAL] %s" % e)
        import traceback
        traceback.print_exc()
        sys.exit(1)

"""
CARLA Scenario Test: Lead Vehicle Emergency Braking (AEB Test)
==============================================================
Scene: Ego vehicle follows lead vehicle, lead vehicle brakes suddenly.
Goal:  Test whether ego vehicle's AEB can avoid collision.

Controls (pygame window):
  SPACE = start scenario
  R     = reset / replay
  Q/Esc = quit

Metrics recorded: distance, speed, TTC, collision event
Output:   E:\CARLA\test_report\ (charts + verdict)
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

# ===== Scenario Parameters (tune these!) =====
EGO_SPEED = 60        # km/h initial speed
LEAD_SPEED = 60       # km/h initial speed
INITIAL_GAP = 30      # meters following distance
LEAD_DECEL = 8.0      # m/s^2 lead vehicle braking deceleration
BRAKE_AFTER_SEC = 5.0 # seconds before lead vehicle brakes
SAFETY_DIST = 2.0     # meters - below this = collision risk
DT = 0.05             # simulation timestep

# ===== Display =====
W, H = 1024, 512

def force_focus():
    try:
        hwnd = pygame.display.get_wm_info()["window"]
        ctypes.windll.user32.SetForegroundWindow(hwnd)
    except:
        pass


def find_straight_road(world, bp_lib):
    """Find a good straight road section for the test."""
    spawn_points = world.get_map().get_spawn_points()
    # Pick two spawn points that are roughly in the same direction
    # Try to find points that are close together on the same road
    best_pair = None
    best_dist = float('inf')
    for i in range(len(spawn_points)):
        for j in range(i+1, min(i+20, len(spawn_points))):
            p1 = spawn_points[i].location
            p2 = spawn_points[j].location
            dist = p1.distance(p2)
            if 15 < dist < 50 and dist < best_dist:
                best_dist = dist
                best_pair = (i, j)
    if best_pair:
        return spawn_points[best_pair[0]], spawn_points[best_pair[1]]
    # Fallback: just use first two
    return spawn_points[0], spawn_points[1]


def main():
    print("=" * 50)
    print("  CARLA AEB Test: Lead Vehicle Emergency Braking")
    print("=" * 50)
    print()
    print(f"  Parameters:")
    print(f"    Initial speed:  {EGO_SPEED} km/h")
    print(f"    Following gap:  {INITIAL_GAP} m")
    print(f"    Lead decel:     {LEAD_DECEL} m/s^2")
    print(f"    Safety dist:    {SAFETY_DIST} m")
    print()

    # ===== Connect =====
    print("[1/4] Connecting to CARLA...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(15.0)

    # Load a map with wide straight roads (Town04 preferred for AEB tests)
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
        print(f"      Loaded map: {target_map}")
    else:
        world = client.get_world()
        print(f"      Using existing map: {world.get_map().name}")

    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    # ===== Setup vehicles =====
    print("[2/4] Setting up vehicles...")

    # Clear old actors first to avoid spawn conflicts
    for a in world.get_actors().filter("vehicle.*"):
        try:
            a.destroy()
        except Exception:
            pass
    time.sleep(0.5)

    carla_map = world.get_map()

    # Find a good spawn point with a long straight road ahead
    lead_bp = bp_lib.filter("vehicle.*")[0]
    if lead_bp.has_attribute("color"):
        lead_bp.set_attribute("color", "255,0,0")

    ego_bp = bp_lib.filter("vehicle.tesla.*")[0]
    if ego_bp is None:
        ego_bp = bp_lib.filter("vehicle.*")[1]
    if ego_bp.has_attribute("color"):
        ego_bp.set_attribute("color", "0,0,255")

    lead_vehicle = None
    ego_vehicle = None
    lead_wp = None

    # Simple proven approach:
    # 1. Spawn lead at a spawn point
    # 2. Get waypoint from lead's actual position
    # 3. Use previous(INITIAL_GAP) to go back along the road
    # 4. Spawn ego there (may be adjacent lane on multi-lane roads, which is fine)
    for sp_idx, sp in enumerate(spawn_points):
        # Strategy: spawn ego FIRST at the spawn point,
        # then spawn lead 30m AHEAD. This way ego always gets
        # a validated spawn point, lead goes forward.
        
        # --- Spawn ego vehicle at pre-validated spawn point ---
        try:
            ev = world.spawn_actor(ego_bp, sp)
        except RuntimeError:
            continue

        # --- Compute lead position ahead of ego ---
        ego_yaw_rad = math.radians(sp.rotation.yaw)
        lead_x = sp.location.x + INITIAL_GAP * math.cos(ego_yaw_rad)
        lead_y = sp.location.y + INITIAL_GAP * math.sin(ego_yaw_rad)
        lead_z = ev.get_transform().location.z  # actual road z from ego

        # --- Spawn lead vehicle ahead ---
        lead_tf = carla.Transform(
            carla.Location(x=lead_x, y=lead_y, z=lead_z),
            sp.rotation
        )
        try:
            lv = world.spawn_actor(lead_bp, lead_tf)
        except RuntimeError:
            # Try slightly different positions
            lv = None
            for adj in [2, -2, 5, -5]:
                alt_x = sp.location.x + (INITIAL_GAP + adj) * math.cos(ego_yaw_rad)
                alt_y = sp.location.y + (INITIAL_GAP + adj) * math.sin(ego_yaw_rad)
                try:
                    lv = world.spawn_actor(lead_bp, carla.Transform(
                        carla.Location(x=alt_x, y=alt_y, z=lead_z), sp.rotation))
                    break
                except RuntimeError:
                    continue
            if lv is None:
                try:
                    ev.destroy()
                except Exception:
                    pass
                continue

        # SUCCESS!
        lead_vehicle = lv
        ego_vehicle = ev
        
        # Save exact spawn transforms for reliable reset
        ego_spawn_tf = carla.Transform(sp.location, sp.rotation)  # from spawn point
        lead_spawn_tf = lead_tf  # the transform we computed and used
        
        # Get waypoints from actual positions for the lane-keeping controller
        lv_tf = lv.get_transform()
        lead_wp = carla_map.get_waypoint(lv_tf.location, project_to_road=True)

        ego_test_tf = ev.get_transform()
        gap = ego_test_tf.location.distance(lv_tf.location)
        yaw_diff = abs(ego_test_tf.rotation.yaw - lv_tf.rotation.yaw)
        if yaw_diff > 180:
            yaw_diff = 360 - yaw_diff
        print(f"      Spawn #{sp_idx}: gap={gap:.1f}m, yaw_diff={yaw_diff:.1f}deg (may be adjacent lane)")
        break

    if lead_vehicle is None or ego_vehicle is None:
        print("[!] Could not spawn both vehicles — all spawn points exhausted")
        return

    lead_tf = lead_vehicle.get_transform()
    ego_tf = ego_vehicle.get_transform()
    print(f"      Lead @ ({lead_tf.location.x:.1f}, {lead_tf.location.y:.1f})")
    print(f"      Ego  @ ({ego_tf.location.x:.1f}, {ego_tf.location.y:.1f})")
    print(f"      Gap  = {lead_tf.location.distance(ego_tf.location):.1f} m")

    # Save initial positions for scenario reset (use spawn transforms, not get_transform)
    ego_initial_tf = ego_spawn_tf
    lead_initial_tf = lead_spawn_tf

    # ===== Attach sensors =====
    print("[3/4] Attaching sensors...")

    # RGB Camera
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(W))
    cam_bp.set_attribute("image_size_y", str(H))
    cam_bp.set_attribute("fov", "90")
    camera = world.spawn_actor(
        cam_bp,
        carla.Transform(carla.Location(x=1.6, z=1.7)),
        attach_to=ego_vehicle
    )

    cam_buf = [None]
    def cam_cb(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
        cam_buf[0] = arr[:, :, :3][:, :, ::-1]
    camera.listen(cam_cb)

    # Collision sensor
    col_bp = bp_lib.find("sensor.other.collision")
    col_sensor = world.spawn_actor(
        col_bp,
        carla.Transform(),
        attach_to=ego_vehicle
    )
    collision_flag = [False]
    lead_collision = [False]
    non_lead_collision = [False]
    collision_time = [None]
    def col_cb(event):
        collision_flag[0] = True
        collision_time[0] = time.time()
        if event.other_actor.id == lead_vehicle.id:
            lead_collision[0] = True
            print(f"  [!] COLLISION WITH LEAD VEHICLE!")
        else:
            non_lead_collision[0] = True
            print(f"  [~] Off-road collision: {event.other_actor.type_id}")
    col_sensor.listen(col_cb)

    # Obstacle detector (measures distance to object ahead)
    obs_bp = bp_lib.find("sensor.other.obstacle")
    obs_bp.set_attribute("distance", "50")
    obs_bp.set_attribute("hit_radius", "1")
    obs_sensor = world.spawn_actor(
        obs_bp,
        carla.Transform(carla.Location(x=2.5, z=1.0)),
        attach_to=ego_vehicle
    )
    obs_dist = [999.0]
    def obs_cb(event):
        obs_dist[0] = event.distance
    obs_sensor.listen(obs_cb)

    print("      [OK] Camera + Collision + Obstacle sensors")

    # ===== Init display =====
    pygame.init()
    screen = pygame.display.set_mode((W, H), pygame.SWSURFACE)
    pygame.display.set_caption("CARLA AEB Test | SPACE=start  R=reset  Q=quit")
    force_focus()
    font = pygame.font.SysFont("consolas", 16)
    font_big = pygame.font.SysFont("consolas", 28, bold=True)

    # ===== Scenario state =====
    scenario_running = False
    scenario_time = 0.0
    lead_braking = False

    # Metrics storage
    metrics = {
        "time": [],
        "ego_speed": [],
        "lead_speed": [],
        "distance": [],
        "ttc": [],
    }

    def calc_ttc(d, v_ego, v_lead):
        """Calculate Time-To-Collision."""
        relative_v = v_ego - v_lead  # m/s, positive = approaching
        if relative_v > 0.1:
            return d / relative_v
        return 999.0  # safe (not approaching)

    def reset_scenario():
        nonlocal scenario_running, scenario_time, lead_braking
        scenario_running = False
        scenario_time = 0.0
        lead_braking = False
        collision_flag[0] = False
        lead_collision[0] = False
        non_lead_collision[0] = False
        collision_time[0] = None
        metrics["time"].clear()
        metrics["ego_speed"].clear()
        metrics["lead_speed"].clear()
        metrics["distance"].clear()
        metrics["ttc"].clear()

        # Reset vehicles to saved initial positions
        lead_vehicle.apply_control(carla.VehicleControl())
        ego_vehicle.apply_control(carla.VehicleControl())

        lead_vehicle.set_transform(lead_initial_tf)
        time.sleep(0.2)
        ego_vehicle.set_transform(ego_initial_tf)
        print("  Scenario reset. Press SPACE to start.")

    def get_actual_distance():
        """Compute actual distance between ego and lead vehicles."""
        ego_loc = ego_vehicle.get_transform().location
        lead_loc = lead_vehicle.get_transform().location
        return ego_loc.distance(lead_loc)

    def keep_lane(vehicle, speed_ms):
        """Pure lane-keeping: returns steer value [-1, 1] to follow road."""
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
                while yaw_err > math.pi: yaw_err -= 2*math.pi
                while yaw_err < -math.pi: yaw_err += 2*math.pi
                dx = target_loc.x - vloc.x
                dy = target_loc.y - vloc.y
                angle_to_target = math.atan2(dy, dx)
                cross_err = angle_to_target - vyaw
                while cross_err > math.pi: cross_err -= 2*math.pi
                while cross_err < -math.pi: cross_err += 2*math.pi
                steer = 2.0 * yaw_err + 3.0 * cross_err
                speed_gain = min(1.0, 3.0 / max(speed_ms, 1.0))
                return max(-1.0, min(1.0, steer * speed_gain))
        except Exception:
            pass
        return 0.0

    def simple_acc(ego_v_ms, target_speed_ms, dist_to_lead, lead_is_braking):
        """Combined ACC (longitudinal) + Lane-keeping (lateral) controller.
        When lead_is_braking=True: pure AEB mode, no throttle, aggressive brake."""
        ctrl = carla.VehicleControl()

        # Lateral control: keep lane
        ctrl.steer = keep_lane(ego_vehicle, ego_v_ms)

        # ===== AEB mode: lead is braking, NO THROTTLE ALLOWED =====
        if lead_is_braking:
            ctrl.throttle = 0.0
            # TTC-based braking: shorter TTC = harder brake
            rel_v = ego_v_ms - 0  # lead is decelerating, use ego speed as rough estimate
            ttc = dist_to_lead / max(rel_v, 0.1)
            if dist_to_lead < SAFETY_DIST + 3:
                ctrl.brake = 1.0  # emergency full brake
            elif ttc < 2.0:
                ctrl.brake = min(1.0, 1.5 - ttc * 0.5)  # aggressive
            elif ttc < 4.0:
                ctrl.brake = min(1.0, 0.8 - ttc * 0.15)
            else:
                ctrl.brake = 0.3  # light pre-braking
            return ctrl

        # ===== Normal ACC mode =====
        desired_gap = max(SAFETY_DIST + 5, ego_v_ms * 2.0)

        if dist_to_lead < SAFETY_DIST + 2:
            # Emergency brake!
            ctrl.throttle = 0.0
            ctrl.brake = 1.0
        elif dist_to_lead < desired_gap:
            # Too close - slow down proportionally
            gap_ratio = dist_to_lead / desired_gap  # 0..1
            if gap_ratio < 0.5:
                # Very close - hard brake
                ctrl.throttle = 0.0
                ctrl.brake = min(1.0, (1.0 - gap_ratio) * 1.5)
            else:
                # Moderately close - ease off throttle
                speed_error = target_speed_ms - ego_v_ms
                ctrl.throttle = max(0, speed_error * 0.3 * gap_ratio)
                ctrl.brake = max(0, -speed_error * 0.3)
        else:
            # Normal cruising - maintain target speed
            speed_error = target_speed_ms - ego_v_ms
            if speed_error > 2:
                # Well below target - full throttle to catch up
                ctrl.throttle = 1.0
                ctrl.brake = 0.0
            elif speed_error > 0:
                # Close to target - moderate throttle
                ctrl.throttle = min(1.0, 0.5 + speed_error * 0.3)
                ctrl.brake = 0.0
            elif speed_error > -2:
                # Slightly over - coast
                ctrl.throttle = 0.0
                ctrl.brake = 0.0
            else:
                # Significantly over speed - hard brake
                ctrl.throttle = 0.0
                ctrl.brake = min(1.0, abs(speed_error) * 0.5)

        return ctrl

    def generate_report():
        """Generate test report with charts."""
        report_dir = "E:/CARLA/test_report"
        os.makedirs(report_dir, exist_ok=True)

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            t = np.array(metrics["time"])
            ego_spd = np.array(metrics["ego_speed"]) * 3.6  # m/s -> km/h
            lead_spd = np.array(metrics["lead_speed"]) * 3.6
            dist = np.array(metrics["distance"])
            ttc = np.array(metrics["ttc"])

            fig, axes = plt.subplots(2, 2, figsize=(14, 8))
            fig.suptitle("CARLA AEB Test Report - Lead Vehicle Emergency Braking", fontsize=14, fontweight='bold')

            # Speed plot
            ax = axes[0, 0]
            ax.plot(t, ego_spd, 'b-', label='Ego Vehicle', linewidth=2)
            ax.plot(t, lead_spd, 'r-', label='Lead Vehicle', linewidth=2)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Speed (km/h)")
            ax.set_title("Vehicle Speed")
            ax.legend()
            ax.grid(True)

            # Distance plot
            ax = axes[0, 1]
            ax.plot(t, dist, 'g-', linewidth=2)
            ax.axhline(y=SAFETY_DIST, color='r', linestyle='--', label=f'Safety threshold ({SAFETY_DIST}m)')
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Distance (m)")
            ax.set_title("Following Distance")
            ax.legend()
            ax.grid(True)

            # TTC plot
            ax = axes[1, 0]
            ttc_clipped = np.clip(ttc, 0, 15)
            ax.plot(t, ttc_clipped, 'm-', linewidth=2)
            ax.axhline(y=1.5, color='r', linestyle='--', label='Critical TTC (1.5s)')
            ax.axhline(y=3.0, color='orange', linestyle='--', label='Warning TTC (3.0s)')
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("TTC (s)")
            ax.set_title("Time-to-Collision")
            ax.legend()
            ax.grid(True)

            # Summary
            ax = axes[1, 1]
            ax.axis('off')
            min_dist = np.min(dist) if len(dist) > 0 else 0
            min_ttc = np.min(ttc_clipped) if len(ttc_clipped) > 0 else 0
            verdict = "COLLISION" if lead_collision[0] else ("PASS" if min_dist > SAFETY_DIST else "WARNING")
            verdict_color = {'PASS': 'green', 'WARNING': 'orange', 'COLLISION': 'red'}.get(verdict, 'black')

            summary = [
                f"Test: Lead Vehicle Emergency Braking",
                f"",
                f"Initial Speed:    {EGO_SPEED} km/h",
                f"Following Gap:    {INITIAL_GAP} m",
                f"Lead Decel:       {LEAD_DECEL} m/s^2",
                f"",
                f"Min Distance:     {min_dist:.2f} m",
                f"Min TTC:          {min_ttc:.2f} s",
                f"Max Ego Decel:    N/A",
                f"Collision:        {'YES' if lead_collision[0] else 'NO'}",
                f"",
                f"VERDICT:  {verdict}",
            ]
            for i, line in enumerate(summary):
                color = verdict_color if 'VERDICT' in line else 'black'
                weight = 'bold' if 'VERDICT' in line else 'normal'
                ax.text(0.1, 0.95 - i * 0.075, line, transform=ax.transAxes,
                        fontsize=11, fontweight=weight, color=color,
                        fontfamily='monospace')

            plt.tight_layout()
            report_path = os.path.join(report_dir, "aeb_test_report.png")
            plt.savefig(report_path, dpi=150)
            plt.close()
            print(f"\n  Report saved: {report_path}")
            print(f"  VERDICT: {verdict}")
            print(f"  Min distance: {min_dist:.2f} m")
            print(f"  Min TTC: {min_ttc:.2f} s")
            return verdict

        except ImportError:
            print("  [!] matplotlib not available, skipping chart generation")
            return "UNKNOWN"

    # ===== Main loop =====
    print("[4/4] Ready!")
    print("      SPACE = start scenario  R = reset  Q = quit")
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
                        lead_braking = False
                        collision_flag[0] = False
                        lead_collision[0] = False
                        non_lead_collision[0] = False
                        metrics["time"].clear()
                        metrics["ego_speed"].clear()
                        metrics["lead_speed"].clear()
                        metrics["distance"].clear()
                        metrics["ttc"].clear()

                        # Set initial speeds
                        target_ms = EGO_SPEED / 3.6
                        lead_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                        ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                        print("  Scenario STARTED!")

                    elif event.key == pygame.K_r:
                        reset_scenario()

            # Terminal keyboard fallback (works even if pygame loses focus)
            while msvcrt.kbhit():
                key = msvcrt.getch()
                ch = key.decode('ascii', errors='ignore').lower()
                if ch == 'q':
                    return
                elif ch == ' ' and not scenario_running:
                    scenario_running = True
                    scenario_time = 0.0
                    lead_braking = False
                    collision_flag[0] = False
                    lead_collision[0] = False
                    non_lead_collision[0] = False
                    metrics["time"].clear()
                    metrics["ego_speed"].clear()
                    metrics["lead_speed"].clear()
                    metrics["distance"].clear()
                    metrics["ttc"].clear()
                    target_ms = EGO_SPEED / 3.6
                    lead_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                    ego_vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                    print("  Scenario STARTED!")
                elif ch == 'r':
                    reset_scenario()

            if scenario_running and not lead_collision[0]:
                scenario_time += DT
                target_ms = EGO_SPEED / 3.6

                # --- Lead vehicle behavior ---
                lead_vel = lead_vehicle.get_velocity()
                lead_speed_ms = np.sqrt(lead_vel.x**2 + lead_vel.y**2 + lead_vel.z**2)

                if scenario_time > BRAKE_AFTER_SEC and not lead_braking:
                    lead_braking = True
                    print(f"  [!] Lead vehicle BRAKING at {lead_speed_ms * 3.6:.0f} km/h!")

                if lead_braking:
                    # Apply strong braking + lane keeping
                    lead_ctrl = carla.VehicleControl()
                    lead_ctrl.throttle = 0.0
                    lead_ctrl.brake = min(1.0, LEAD_DECEL / 10.0)
                    lead_ctrl.steer = keep_lane(lead_vehicle, lead_speed_ms)
                    lead_vehicle.apply_control(lead_ctrl)
                else:
                    # Maintain speed + lane keeping
                    if lead_speed_ms < target_ms:
                        lead_vehicle.apply_control(carla.VehicleControl(
                            throttle=1.0, steer=keep_lane(lead_vehicle, lead_speed_ms)))
                    else:
                        lead_vehicle.apply_control(carla.VehicleControl(
                            throttle=0.0, brake=0.1, steer=keep_lane(lead_vehicle, lead_speed_ms)))

                # --- Ego vehicle ACC + AEB ---
                ego_vel = ego_vehicle.get_velocity()
                ego_speed_ms = np.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)
                dist = get_actual_distance()

                ego_ctrl = simple_acc(ego_speed_ms, target_ms, dist, lead_braking)
                ego_vehicle.apply_control(ego_ctrl)

                # --- Record metrics ---
                ttc_val = calc_ttc(dist, ego_speed_ms, lead_speed_ms)
                metrics["time"].append(scenario_time)
                metrics["ego_speed"].append(ego_speed_ms)
                metrics["lead_speed"].append(lead_speed_ms)
                metrics["distance"].append(dist)
                metrics["ttc"].append(ttc_val)

                # Check end conditions
                if lead_collision[0]:
                    print("  Scenario COMPLETE - COLLISION with lead vehicle!")
                    scenario_running = False
                    generate_report()
                elif lead_braking and lead_speed_ms < 0.5:
                    if ego_speed_ms < 0.5 or scenario_time > BRAKE_AFTER_SEC + 15:
                        print("  Scenario COMPLETE!")
                        scenario_running = False
                        generate_report()
                elif scenario_time > 30:
                    print("  Scenario timeout - ending.")
                    scenario_running = False
                    generate_report()

            # --- Render ---
            screen.fill((0, 0, 0))

            # Camera view (top half)
            if cam_buf[0] is not None:
                cam_surface = pygame.surfarray.make_surface(cam_buf[0].swapaxes(0, 1))
                cam_scaled = pygame.transform.scale(cam_surface, (W, H // 2))
                screen.blit(cam_scaled, (0, 0))

            # HUD overlay on camera
            if scenario_running or len(metrics["time"]) > 0:
                ego_vel = ego_vehicle.get_velocity()
                ego_spd = 3.6 * np.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)
                lead_vel = lead_vehicle.get_velocity()
                lead_spd = 3.6 * np.sqrt(lead_vel.x**2 + lead_vel.y**2 + lead_vel.z**2)
                dist = get_actual_distance()
                ttc_val = calc_ttc(dist, ego_spd/3.6, lead_spd/3.6)

                hud_lines = [
                    f"Speed: {ego_spd:.0f} km/h",
                    f"Dist:  {dist:.1f} m",
                    f"TTC:   {ttc_val:.1f} s",
                    f"Time:  {scenario_time:.1f} s",
                ]

                for i, line in enumerate(hud_lines):
                    color = (255, 255, 255)
                    if "Dist" in line and dist < SAFETY_DIST + 3:
                        color = (255, 50, 50)
                    if "TTC" in line and ttc_val < 2.0:
                        color = (255, 50, 50)
                    s = font.render(line, True, color)
                    screen.blit(s, (10, 10 + i * 22))

                if lead_braking:
                    s = font_big.render("LEAD BRAKING!", True, (255, 0, 0))
                    screen.blit(s, (W - 280, 10))

                if lead_collision[0]:
                    s = font_big.render("COLLISION!", True, (255, 0, 0))
                    screen.blit(s, (W // 2 - 80, H // 4))
                elif non_lead_collision[0]:
                    s = font.render("Off-road collision (not counted)", True, (255, 200, 0))
                    screen.blit(s, (W // 2 - 160, H // 4))

            # Bottom half: real-time chart
            chart_y = H // 2
            chart_h = H // 2
            chart_surface = pygame.Surface((W, chart_h))
            chart_surface.fill((20, 20, 30))
            screen.blit(chart_surface, (0, chart_y))

            if len(metrics["time"]) > 2:
                t = np.array(metrics["time"])
                dist_arr = np.array(metrics["distance"])
                ego_arr = np.array(metrics["ego_speed"]) * 3.6
                lead_arr = np.array(metrics["lead_speed"]) * 3.6

                t_max = max(t[-1], 1.0)
                # Draw distance curve
                for i in range(1, len(t)):
                    x1 = int((t[i-1] / t_max) * (W - 20)) + 10
                    y1 = chart_y + chart_h - int(min(dist_arr[i-1], 60) / 60 * (chart_h - 30)) - 15
                    x2 = int((t[i] / t_max) * (W - 20)) + 10
                    y2 = chart_y + chart_h - int(min(dist_arr[i], 60) / 60 * (chart_h - 30)) - 15
                    pygame.draw.line(screen, (0, 200, 0), (x1, y1), (x2, y2), 2)

                # Draw ego speed curve
                for i in range(1, len(t)):
                    x1 = int((t[i-1] / t_max) * (W - 20)) + 10
                    y1 = chart_y + chart_h - int(ego_arr[i-1] / 80 * (chart_h - 30)) - 15
                    x2 = int((t[i] / t_max) * (W - 20)) + 10
                    y2 = chart_y + chart_h - int(ego_arr[i] / 80 * (chart_h - 30)) - 15
                    pygame.draw.line(screen, (50, 100, 255), (x1, y1), (x2, y2), 2)

                # Draw lead speed curve
                for i in range(1, len(t)):
                    x1 = int((t[i-1] / t_max) * (W - 20)) + 10
                    y1 = chart_y + chart_h - int(lead_arr[i-1] / 80 * (chart_h - 30)) - 15
                    x2 = int((t[i] / t_max) * (W - 20)) + 10
                    y2 = chart_y + chart_h - int(lead_arr[i] / 80 * (chart_h - 30)) - 15
                    pygame.draw.line(screen, (255, 50, 50), (x1, y1), (x2, y2), 2)

                # Safety line
                safe_y = chart_y + chart_h - int(SAFETY_DIST / 60 * (chart_h - 30)) - 15
                pygame.draw.line(screen, (255, 255, 0), (10, safe_y), (W - 10, safe_y), 1)

            # Legend
            labels = [
                ("Green = Distance(m)", (0, 200, 0)),
                ("Blue = Ego Speed(km/h)", (50, 100, 255)),
                ("Red = Lead Speed(km/h)", (255, 50, 50)),
            ]
            for i, (txt, color) in enumerate(labels):
                s = font.render(txt, True, color)
                screen.blit(s, (W - 280, chart_y + 5 + i * 20))

            if not scenario_running and len(metrics["time"]) == 0:
                s = font_big.render("Press SPACE to start", True, (200, 200, 200))
                screen.blit(s, (W // 2 - 140, H // 4 + 50))

            pygame.display.flip()

    finally:
        # Generate report if we have data but it wasn't generated yet
        if len(metrics["time"]) > 2 and scenario_running:
            generate_report()
        print("\nCleaning up...")
        camera.stop()
        col_sensor.stop()
        obs_sensor.stop()
        for actor in [camera, col_sensor, obs_sensor, lead_vehicle, ego_vehicle]:
            try:
                actor.destroy()
            except Exception:
                pass
        pygame.quit()
        print("Done!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

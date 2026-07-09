"""
CARLA Multi-Sensor Demo
Sensors: RGB Camera + Semantic Segmentation Camera + LiDAR
Features: NPC traffic, data recording, multi-panel display
Controls (in pygame window): WASD/Arrows = drive, X = reverse, V = toggle view,
                              C = record data, T = spawn traffic, Q/Esc = quit
"""
import carla
import pygame
import numpy as np
import ctypes
import time
import os
import sys

CARLA_HOST = "localhost"
CARLA_PORT = 2000
W = 640  # panel width
H = 360  # panel height

# Semantic segmentation color map (CARLA class id -> RGB)
SEM_COLORS = {
    0:  (0, 0, 0),        # None
    1:  (70, 70, 70),      # Building
    2:  (100, 40, 40),     # Fence
    3:  (55, 90, 80),      # Other
    4:  (220, 20, 60),     # Pedestrian
    5:  (153, 153, 153),   # Pole
    6:  (157, 234, 50),    # RoadLine
    7:  (128, 64, 128),    # Road
    8:  (244, 35, 232),    # Sidewalk
    9:  (107, 142, 35),    # Vegetation
    10: (0, 0, 142),       # Vehicle
    11: (102, 102, 156),   # Wall
    12: (220, 220, 0),     # TrafficLight
    13: (70, 130, 180),    # TrafficSign
    14: (81, 0, 81),       # Sky
    15: (150, 100, 100),   # Terrain
}

def sem_to_color(sem_image):
    """Convert semantic segmentation raw data to colored RGB image."""
    array = np.frombuffer(sem_image.raw_data, dtype=np.uint8)
    array = array.reshape((sem_image.height, sem_image.width, 4))
    class_ids = array[:, :, 2]
    rgb = np.zeros((sem_image.height, sem_image.width, 3), dtype=np.uint8)
    for cid, color in SEM_COLORS.items():
        mask = class_ids == cid
        rgb[mask] = color
    return rgb


def force_window_focus():
    """Force the pygame window to foreground on Windows."""
    try:
        user32 = ctypes.windll.user32
        hwnd = pygame.display.get_wm_info()["window"]
        user32.SetForegroundWindow(hwnd)
    except Exception:
        pass


def main():
    # ===== 1. Connect =====
    print("[1/6] Connecting to CARLA...")
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    print(f"      Map: {world.get_map().name}")

    # ===== 2. Spawn ego vehicle =====
    print("[2/6] Spawning ego vehicle...")
    vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()
    # Try multiple spawn points in case some are occupied
    vehicle = None
    for sp in spawn_points[:20]:
        try:
            vehicle = world.spawn_actor(vehicle_bp, sp)
            break
        except RuntimeError:
            continue
    if vehicle is None:
        # Last resort: clear all old actors and retry
        print("      All spawn points occupied, clearing old actors...")
        for actor in world.get_actors().filter("vehicle.*"):
            if not actor.attributes.get("role_name") == "hero":
                actor.destroy()
        time.sleep(0.5)
        vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    print(f"      Tesla Model 3 spawned")

    # ===== 3. Attach sensors =====
    print("[3/6] Attaching sensors...")

    # --- RGB Camera ---
    rgb_bp = bp_lib.find("sensor.camera.rgb")
    rgb_bp.set_attribute("image_size_x", str(W))
    rgb_bp.set_attribute("image_size_y", str(H))
    rgb_bp.set_attribute("fov", "100")
    rgb_cam = world.spawn_actor(
        rgb_bp,
        carla.Transform(carla.Location(x=1.6, z=1.7)),
        attach_to=vehicle
    )

    rgb_buf = [None]
    def rgb_cb(img):
        arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape((img.height, img.width, 4))
        rgb_buf[0] = arr[:, :, :3][:, :, ::-1]
    rgb_cam.listen(rgb_cb)
    print("      [OK] RGB Camera")

    # --- Semantic Segmentation Camera ---
    sem_bp = bp_lib.find("sensor.camera.semantic_segmentation")
    sem_bp.set_attribute("image_size_x", str(W))
    sem_bp.set_attribute("image_size_y", str(H))
    sem_bp.set_attribute("fov", "100")
    sem_cam = world.spawn_actor(
        sem_bp,
        carla.Transform(carla.Location(x=1.6, z=1.7)),
        attach_to=vehicle
    )

    sem_buf = [None]
    def sem_cb(img):
        sem_buf[0] = sem_to_color(img)
    sem_cam.listen(sem_cb)
    print("      [OK] Semantic Segmentation Camera")

    # --- LiDAR ---
    lidar_bp = bp_lib.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute("channels", "32")
    lidar_bp.set_attribute("range", "50")
    lidar_bp.set_attribute("points_per_second", "100000")
    lidar_bp.set_attribute("rotation_frequency", "20")
    lidar_cam = world.spawn_actor(
        lidar_bp,
        carla.Transform(carla.Location(x=0.0, z=2.5)),
        attach_to=vehicle
    )

    lidar_buf = [None]
    def lidar_cb(data):
        points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
        lidar_buf[0] = points
    lidar_cam.listen(lidar_cb)
    print("      [OK] LiDAR (32ch, 50m range)")

    # ===== 4. Init display (2x2 grid) =====
    print("[4/6] Initializing multi-panel display...")
    pygame.init()
    screen_w = W * 2
    screen_h = H * 2
    screen = pygame.display.set_mode((screen_w, screen_h), pygame.SWSURFACE)
    pygame.display.set_caption("CARLA Multi-Sensor | WASD=drive  X=reverse  V=view  C=record  T=traffic  Q=quit")

    # Force window focus so keyboard works in the pygame window
    force_window_focus()

    font = pygame.font.SysFont("consolas", 14)

    def draw_label(text, x, y, color=(255, 255, 255)):
        surface = font.render(text, True, color)
        screen.blit(surface, (x + 5, y + 5))

    def draw_lidar_bev(points):
        """Render LiDAR point cloud as bird's eye view."""
        bev = np.zeros((H, W, 3), dtype=np.uint8)
        if points is None:
            return bev
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        mask = (np.abs(x) < 50) & (np.abs(y) < 50)
        x, y, z = x[mask], y[mask], z[mask]
        if len(x) == 0:
            return bev
        px = ((-y / 50.0 + 1.0) * 0.5 * W).astype(int)
        py = ((-x / 50.0 + 1.0) * 0.5 * H).astype(int)
        px = np.clip(px, 0, W - 1)
        py = np.clip(py, 0, H - 1)
        h_norm = np.clip((z + 2.0) / 6.0, 0, 1)
        r = (h_norm * 255).astype(np.uint8)
        g = ((1 - np.abs(h_norm - 0.5) * 2) * 255).astype(np.uint8)
        b = ((1 - h_norm) * 255).astype(np.uint8)
        bev[py, px] = np.stack([r, g, b], axis=1)
        cx, cy = W // 2, H // 2
        bev[cy-2:cy+3, cx] = [255, 255, 255]
        bev[cy, cx-2:cx+3] = [255, 255, 255]
        return bev

    # ===== 5. State =====
    record_dir = "E:/CARLA/recorded_data"
    recording = False
    frame_id = 0
    traffic_spawned = False
    actors_to_clean = []
    view_mode = 0  # 0=all, 1=RGB, 2=SEM, 3=LiDAR

    # ===== 6. Main loop =====
    print("[5/6] Ready! Click the pygame window and drive!")
    print("      WASD/Arrows=drive  X=reverse  V=view  C=record  T=traffic  Q=quit")
    print()

    control = carla.VehicleControl()
    clock = pygame.time.Clock()

    try:
        while True:
            clock.tick(30)

            # --- Process pygame events ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                        return
                    elif event.key == pygame.K_x:
                        control.reverse = not control.reverse
                        print(f"  reverse={'ON' if control.reverse else 'OFF'}")
                    elif event.key == pygame.K_v:
                        view_mode = (view_mode + 1) % 4
                        modes = ["all panels", "RGB only", "Semantic only", "LiDAR BEV"]
                        print(f"  view: {modes[view_mode]}")
                    elif event.key == pygame.K_c:
                        recording = not recording
                        if recording:
                            os.makedirs(record_dir, exist_ok=True)
                            frame_id = 0
                            print(f"  RECORDING ON -> {record_dir}")
                        else:
                            print(f"  RECORDING OFF ({frame_id} frames saved)")
                    elif event.key == pygame.K_t:
                        if not traffic_spawned:
                            print("  Spawning traffic...")
                            tm = client.get_trafficmanager(8000)
                            tm.set_global_distance_to_leading_vehicle(2.5)
                            for sp in spawn_points[1:15]:
                                vbp = np.random.choice(bp_lib.filter("vehicle.*"))
                                npc = world.try_spawn_actor(vbp, sp)
                                if npc:
                                    npc.set_autopilot(True, 8000)
                                    actors_to_clean.append(npc)
                            walker_bp = bp_lib.filter("walker.pedestrian.*")
                            for _ in range(10):
                                wb = np.random.choice(walker_bp)
                                loc = world.get_random_location_from_navigation()
                                if loc:
                                    walker = world.try_spawn_actor(wb, carla.Transform(loc))
                                    if walker:
                                        actors_to_clean.append(walker)
                            print(f"  Spawned {len(actors_to_clean)} actors")
                            traffic_spawned = True
                        else:
                            print("  Traffic already spawned")
                    elif event.key == pygame.K_SPACE:
                        control.hand_brake = not control.hand_brake

                elif event.type == pygame.KEYUP:
                    # Reset steer when releasing turn keys
                    if event.key in (pygame.K_a, pygame.K_LEFT, pygame.K_d, pygame.K_RIGHT):
                        control.steer = 0.0

            # --- Read held keys ---
            keys = pygame.key.get_pressed()

            # Throttle / Brake
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                control.throttle = min(1.0, control.throttle + 0.05)
                control.brake = 0.0
            elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
                control.throttle = 0.0
                control.brake = min(1.0, control.brake + 0.1)
            else:
                control.throttle = max(0.0, control.throttle - 0.02)
                control.brake = max(0.0, control.brake - 0.05)

            # Steering (smooth while held)
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                control.steer = max(-1.0, control.steer - 0.1)
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                control.steer = min(1.0, control.steer + 0.1)

            vehicle.apply_control(control)

            # --- Recording ---
            if recording and rgb_buf[0] is not None and sem_buf[0] is not None:
                fdir = os.path.join(record_dir, f"{frame_id:06d}")
                os.makedirs(fdir, exist_ok=True)
                from PIL import Image
                Image.fromarray(rgb_buf[0]).save(os.path.join(fdir, "rgb.png"))
                Image.fromarray(sem_buf[0]).save(os.path.join(fdir, "semantic.png"))
                if lidar_buf[0] is not None:
                    np.save(os.path.join(fdir, "lidar.npy"), lidar_buf[0])
                t = vehicle.get_transform()
                with open(os.path.join(fdir, "pose.txt"), "w") as f:
                    f.write(f"x={t.location.x:.4f} y={t.location.y:.4f} z={t.location.z:.4f} "
                            f"yaw={t.rotation.yaw:.4f} pitch={t.rotation.pitch:.4f} roll={t.rotation.roll:.4f}")
                frame_id += 1
                if frame_id % 30 == 0:
                    print(f"  recorded {frame_id} frames")

            # --- Render ---
            screen.fill((0, 0, 0))
            blank = np.zeros((H, W, 3), dtype=np.uint8)

            if view_mode == 0:  # All panels
                rgb_img = rgb_buf[0] if rgb_buf[0] is not None else blank
                sem_img = sem_buf[0] if sem_buf[0] is not None else blank
                lid_img = draw_lidar_bev(lidar_buf[0])

                for img, pos, label in [
                    (rgb_img, (0, 0), "RGB Camera"),
                    (sem_img, (W, 0), "Semantic Segmentation"),
                    (lid_img, (0, H), "LiDAR Bird's Eye View"),
                ]:
                    surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))
                    screen.blit(surface, pos)
                    draw_label(label, pos[0], pos[1])

                # Info panel (bottom-right)
                info_x, info_y = W, H
                info_surface = pygame.Surface((W, H))
                info_surface.fill((20, 20, 30))
                screen.blit(info_surface, (info_x, info_y))

                vel = vehicle.get_velocity()
                spd = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                loc = vehicle.get_transform().location
                lines = [
                    f"Speed: {spd:.1f} km/h",
                    f"Pos: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})",
                    f"Reverse: {'ON' if control.reverse else 'OFF'}",
                    f"Handbrake: {'ON' if control.hand_brake else 'OFF'}",
                    f"Recording: {'ON' if recording else 'OFF'}",
                    f"Frames: {frame_id}",
                    f"NPC actors: {len(actors_to_clean)}",
                    "",
                    "W/S = gas/brake",
                    "A/D = steer",
                    "X = reverse  V = view",
                    "C = record   T = traffic",
                    "SPACE = handbrake  Q = quit",
                ]
                for i, line in enumerate(lines):
                    draw_label(line, info_x, info_y + 10 + i * 22,
                               (0, 255, 0) if i < 7 else (200, 200, 200))

            elif view_mode == 1 and rgb_buf[0] is not None:
                big = pygame.transform.scale(
                    pygame.surfarray.make_surface(rgb_buf[0].swapaxes(0, 1)),
                    (screen_w, screen_h))
                screen.blit(big, (0, 0))
                draw_label("RGB Camera (fullscreen) | Q=quit V=view", 0, 0)

            elif view_mode == 2 and sem_buf[0] is not None:
                big = pygame.transform.scale(
                    pygame.surfarray.make_surface(sem_buf[0].swapaxes(0, 1)),
                    (screen_w, screen_h))
                screen.blit(big, (0, 0))
                draw_label("Semantic Segmentation (fullscreen) | Q=quit V=view", 0, 0)

            elif view_mode == 3:
                lid_big = draw_lidar_bev(lidar_buf[0])
                big = pygame.transform.scale(
                    pygame.surfarray.make_surface(lid_big.swapaxes(0, 1)),
                    (screen_w, screen_h))
                screen.blit(big, (0, 0))
                draw_label("LiDAR BEV (fullscreen) | Q=quit V=view", 0, 0)

            pygame.display.flip()

            # Update title
            vel = vehicle.get_velocity()
            spd = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            rec = " [REC]" if recording else ""
            pygame.display.set_caption(f"CARLA Multi-Sensor | {spd:.0f} km/h{rec}")

    finally:
        print("\nCleaning up...")
        rgb_cam.stop()
        sem_cam.stop()
        lidar_cam.stop()
        for actor in actors_to_clean:
            actor.destroy()
        rgb_cam.destroy()
        sem_cam.destroy()
        lidar_cam.destroy()
        vehicle.destroy()
        pygame.quit()
        if frame_id > 0:
            print(f"Recorded {frame_id} frames -> {record_dir}")
        print("Done!")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

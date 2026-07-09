"""
CARLA Demo - Vehicle + Camera + Keyboard Control (Terminal Input)
Uses msvcrt for keyboard (no pygame keyboard focus issues on Windows)
pygame only for camera display.
"""
import carla
import pygame
import numpy as np
import msvcrt
import sys

CARLA_HOST = "localhost"
CARLA_PORT = 2000
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600

def main():
    # 1. Connect to CARLA
    print("[1/5] Connecting to CARLA server...")
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(10.0)
    world = client.get_world()
    print(f"      Map: {world.get_map().name}")

    # 2. Spawn vehicle
    print("[2/5] Spawning vehicle...")
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    print(f"      Tesla Model 3 @ ({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f})")

    # 3. Attach camera
    print("[3/5] Attaching camera...")
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(WINDOW_WIDTH))
    camera_bp.set_attribute("image_size_y", str(WINDOW_HEIGHT))
    camera_bp.set_attribute("fov", "110")
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # Latest image buffer
    latest_image = [None]

    def camera_callback(image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]
        latest_image[0] = array

    camera.listen(camera_callback)

    # 4. Init pygame window (display only, no keyboard)
    print("[4/5] Initializing display window...")
    pygame.init()
    display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SWSURFACE)
    pygame.display.set_caption("CARLA Demo")

    # 5. Terminal-based control
    print("[5/5] Ready! Controls (type in THIS terminal window):")
    print("      w=gas  s=brake  a=left  d=right  x=reverse  space=handbrake  q=quit")
    print("      >>> Click here and type keys <<<")
    print()

    control = carla.VehicleControl()
    import time
    last_time = time.time()

    try:
        while True:
            # Process pygame events (keep window alive)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            # Read terminal keyboard (non-blocking)
            while msvcrt.kbhit():
                key = msvcrt.getch()
                if key == b'\xe0':  # arrow key prefix
                    key2 = msvcrt.getch()
                    if key2 == b'H':    # Up
                        control.throttle = min(1.0, control.throttle + 0.15)
                        control.brake = 0.0
                        print(f"  ^ throttle={control.throttle:.2f}")
                    elif key2 == b'P':  # Down
                        control.throttle = 0.0
                        control.brake = min(1.0, control.brake + 0.3)
                        print(f"  v brake={control.brake:.2f}")
                    elif key2 == b'K':  # Left
                        control.steer = max(-1.0, control.steer - 0.15)
                        print(f"  < steer={control.steer:.2f}")
                    elif key2 == b'M':  # Right
                        control.steer = min(1.0, control.steer + 0.15)
                        print(f"  > steer={control.steer:.2f}")
                else:
                    ch = key.decode('ascii', errors='ignore').lower()
                    if ch == 'w':
                        control.throttle = min(1.0, control.throttle + 0.15)
                        control.brake = 0.0
                        print(f"  W throttle={control.throttle:.2f}")
                    elif ch == 's':
                        control.throttle = 0.0
                        control.brake = min(1.0, control.brake + 0.3)
                        print(f"  S brake={control.brake:.2f}")
                    elif ch == 'a':
                        control.steer = max(-1.0, control.steer - 0.15)
                        print(f"  A steer={control.steer:.2f}")
                    elif ch == 'd':
                        control.steer = min(1.0, control.steer + 0.15)
                        print(f"  D steer={control.steer:.2f}")
                    elif ch == ' ':
                        control.hand_brake = not control.hand_brake
                        print(f"  SPACE handbrake={'ON' if control.hand_brake else 'OFF'}")
                    elif ch == 'r':
                        # Release all
                        control.throttle = 0.0
                        control.brake = 0.0
                        control.steer = 0.0
                        control.hand_brake = False
                        print("  R released all controls")
                    elif ch == 'x':
                        control.reverse = not control.reverse
                        print(f"  X reverse={'ON' if control.reverse else 'OFF'}")
                    elif ch == 'q':
                        print("  Q - quitting...")
                        return

            # Decay throttle/brake/steer slightly
            now = time.time()
            dt = now - last_time
            last_time = now

            if control.throttle > 0 and not msvcrt.kbhit():
                control.throttle = max(0.0, control.throttle - dt * 0.5)
            if control.brake > 0 and not msvcrt.kbhit():
                control.brake = max(0.0, control.brake - dt * 0.8)
            if control.steer != 0 and not msvcrt.kbhit():
                if control.steer > 0:
                    control.steer = max(0.0, control.steer - dt * 2.0)
                else:
                    control.steer = min(0.0, control.steer + dt * 2.0)

            # Apply to vehicle
            vehicle.apply_control(control)

            # Render camera
            if latest_image[0] is not None:
                surface = pygame.surfarray.make_surface(latest_image[0].swapaxes(0, 1))
                display.blit(surface, (0, 0))
            pygame.display.flip()

            # Update title
            velocity = vehicle.get_velocity()
            speed_kmh = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            pygame.display.set_caption(f"CARLA | {speed_kmh:.0f} km/h")

            time.sleep(0.03)  # ~30 fps

    finally:
        print("\nCleaning up...")
        camera.stop()
        camera.destroy()
        vehicle.destroy()
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

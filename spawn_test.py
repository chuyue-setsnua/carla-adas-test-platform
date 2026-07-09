"""Quick test: verify 3-vehicle spawn logic without pygame UI."""
import carla
import time
import math
import sys

INITIAL_GAP = 30

def main():
    print("Connecting to CARLA...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    print(f"Map: {carla_map.name}")
    print(f"Spawn points: {len(spawn_points)}")

    # Clear old actors
    for a in world.get_actors().filter("vehicle.*"):
        a.destroy()
    time.sleep(0.5)

    ego_bp = bp_lib.filter("vehicle.*")[0]
    lead_bp = bp_lib.filter("vehicle.*")[1]
    cutin_bp = bp_lib.filter("vehicle.*")[2]

    fail_count = {"ego": 0, "lead": 0, "cutin": 0}
    
    for sp_idx, sp in enumerate(spawn_points):
        # Step 1: Spawn EGO at spawn point
        try:
            ev = world.spawn_actor(ego_bp, sp)
        except RuntimeError:
            fail_count["ego"] += 1
            continue

        ev_tf = ev.get_transform()
        ev_loc = ev_tf.location
        ev_yaw = ev_tf.rotation.yaw
        ev_yaw_rad = math.radians(ev_yaw)
        ev_wp = carla_map.get_waypoint(ev_loc, project_to_road=True)

        # Check ego wasn't silently relocated
        ego_drift = ev_loc.distance(sp.location)
        if ego_drift > 5.0:
            if sp_idx < 3:
                print(f"  [skip#{sp_idx}] ego drift={ego_drift:.0f}m actual=({ev_loc.x:.0f},{ev_loc.y:.0f}) expected=({sp.location.x:.0f},{sp.location.y:.0f})")
            ev.destroy()
            fail_count["ego"] += 1
            continue

        # Step 2: Geometric forward projection for LEAD
        lv = None
        offsets = [INITIAL_GAP, INITIAL_GAP+2, INITIAL_GAP-2, INITIAL_GAP+5, INITIAL_GAP-5, INITIAL_GAP+8, INITIAL_GAP-8]
        for offset_m in offsets:
            lx = ev_loc.x + offset_m * math.cos(ev_yaw_rad)
            ly = ev_loc.y + offset_m * math.sin(ev_yaw_rad)
            lt = carla.Transform(carla.Location(x=lx, y=ly, z=ev_loc.z), ev_tf.rotation)
            try:
                lv = world.spawn_actor(lead_bp, lt)
            except RuntimeError:
                continue

            lv_loc = lv.get_transform().location
            actual_gap = ev_loc.distance(lv_loc)
            if actual_gap < 5 or actual_gap > INITIAL_GAP * 2.5:
                lv.destroy()
                lv = None
                continue

            lv_wp = carla_map.get_waypoint(lv_loc, project_to_road=True)
            if lv_wp.road_id != ev_wp.road_id or lv_wp.lane_id != ev_wp.lane_id:
                lv.destroy()
                lv = None
                continue

            break

        if lv is None:
            if sp_idx < 3:
                tx = ev_loc.x + INITIAL_GAP * math.cos(ev_yaw_rad)
                ty = ev_loc.y + INITIAL_GAP * math.sin(ev_yaw_rad)
                tp = carla_map.get_waypoint(carla.Location(x=tx, y=ty, z=ev_loc.z), project_to_road=True)
                print(f"  [skip#{sp_idx}] ego=({ev_loc.x:.0f},{ev_loc.y:.0f}) yaw={ev_yaw:.0f} ego_wp=(r{ev_wp.road_id} l{ev_wp.lane_id})")
                print(f"            target_lead=({tx:.0f},{ty:.0f}) -> wp=(r{tp.road_id if tp else 'None'} l{tp.lane_id if tp else 'None'})")
            ev.destroy()
            fail_count["lead"] += 1
            continue

        lv_tf = lv.get_transform()

        # Step 3: Cut-in from spawn points
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
            y2 = sp2_wp.transform.rotation.yaw
            yd = abs(y2 - ev_wp.transform.rotation.yaw) % 360
            if yd > 180: yd = 360 - yd
            if yd > 90: continue

            try:
                cv = world.spawn_actor(cutin_bp, sp2)
            except RuntimeError:
                continue

            cv_loc = cv.get_transform().location
            cv_drift = cv_loc.distance(sp2.location)
            if cv_drift > 10.0 or cv_loc.length() < 5.0:
                cv.destroy()
                continue

            break

        if cv is None:
            if sp_idx < 3:
                print(f"  [skip#{sp_idx}] no valid cutin found")
            ev.destroy()
            lv.destroy()
            fail_count["cutin"] += 1
            continue

        # SUCCESS!
        print(f"\n=== SUCCESS at sp#{sp_idx} ===")
        print(f"  Ego:    ({ev_loc.x:.1f}, {ev_loc.y:.1f}) yaw={ev_yaw:.0f}")
        print(f"  Lead:   ({lv_tf.location.x:.1f}, {lv_tf.location.y:.1f})")
        print(f"  Cutin:  ({cv.get_transform().location.x:.1f}, {cv.get_transform().location.y:.1f})")
        print(f"  Gap(ego-lead):     {ev_loc.distance(lv_tf.location):.1f}m")
        print(f"  Gap(ego-cutin):    {ev_loc.distance(cv.get_transform().location):.1f}m")
        print(f"  Gap(lead-cutin):   {lv_tf.location.distance(cv.get_transform().location):.1f}m")

        print(f"  Ego lane:    road={ev_wp.road_id} lane={ev_wp.lane_id}")
        lv_wp_final = carla_map.get_waypoint(lv_tf.location, project_to_road=True)
        cv_wp_final = carla_map.get_waypoint(cv.get_transform().location, project_to_road=True)
        print(f"  Lead lane:   road={lv_wp_final.road_id} lane={lv_wp_final.lane_id}")
        print(f"  Cutin lane:  road={cv_wp_final.road_id} lane={cv_wp_final.lane_id}")

        ev.destroy()
        lv.destroy()
        cv.destroy()
        return

    print(f"\n=== ALL {len(spawn_points)} FAILED ===")
    print(f"  ego={fail_count['ego']} lead={fail_count['lead']} cutin={fail_count['cutin']}")

if __name__ == "__main__":
    main()

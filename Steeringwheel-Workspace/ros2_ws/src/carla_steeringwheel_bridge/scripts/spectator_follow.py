"""
spectator_follow.py

Makes the CARLA spectator camera follow the hero vehicle from behind.
Run this in a separate terminal while carla_vehicle_bridge is running.

Usage:
    python3 spectator_follow.py
"""

import math
import time
import carla


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    spectator = world.get_spectator()

    # Find the hero vehicle
    hero = None
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == 'hero':
            hero = actor
            break

    if hero is None:
        print("ERROR: No vehicle with role_name='hero' found in CARLA.")
        print("Make sure vw_bus_spawner is running first.")
        return

    print(f"Following: {hero.type_id} (id={hero.id})")
    print("Press Ctrl+C to stop.")

    while True:
        try:
            tf = hero.get_transform()
            yaw_rad = math.radians(tf.rotation.yaw)

            # Camera: 8m behind, 5m above, looking slightly down
            cam_loc = carla.Location(
                x=tf.location.x - math.cos(yaw_rad) * 8.0,
                y=tf.location.y - math.sin(yaw_rad) * 8.0,
                z=tf.location.z + 5.0,
            )
            cam_rot = carla.Rotation(pitch=-20.0, yaw=tf.rotation.yaw, roll=0.0)
            spectator.set_transform(carla.Transform(cam_loc, cam_rot))
            time.sleep(0.05)   # 20 Hz
        except KeyboardInterrupt:
            print("\nSpectator follow stopped.")
            break
        except Exception as e:
            print(f"Lost vehicle: {e}")
            break


if __name__ == '__main__':
    main()

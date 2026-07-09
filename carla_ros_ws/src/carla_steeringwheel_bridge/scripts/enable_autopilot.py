"""
Enables CARLA autopilot on the hero vehicle so it drives itself.
Run after vw_bus_spawner has spawned the bus.

Usage:
    python3 enable_autopilot.py
"""

import carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Switch to async mode in case it was set to sync
settings = world.get_settings()
if settings.synchronous_mode:
    print("Disabling synchronous mode...")
    settings.synchronous_mode = False
    world.apply_settings(settings)

# Connect to Traffic Manager
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(False)
tm.set_global_distance_to_leading_vehicle(2.0)
tm.global_percentage_speed_difference(-30.0)  # 30% faster than speed limit

hero = None
for actor in world.get_actors().filter('vehicle.*'):
    if actor.attributes.get('role_name') == 'hero':
        hero = actor
        break

if hero is None:
    print("ERROR: No hero vehicle found. Run vw_bus_spawner first.")
else:
    hero.set_autopilot(True, 8000)
    print(f"Autopilot enabled on {hero.type_id} (id={hero.id})")
    print("Bus should start moving within a few seconds.")

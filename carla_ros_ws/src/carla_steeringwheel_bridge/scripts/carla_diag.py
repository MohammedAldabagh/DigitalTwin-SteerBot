import carla
import time

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# World settings
settings = world.get_settings()
print(f"Synchronous mode : {settings.synchronous_mode}")
print(f"Fixed delta secs : {settings.fixed_delta_seconds}")
print(f"No rendering mode: {settings.no_rendering_mode}")

# Force async mode
if settings.synchronous_mode:
    print(">>> Disabling sync mode...")
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = None
    world.apply_settings(settings)
    print("    Done.")

# Find hero
hero = None
for actor in world.get_actors().filter('vehicle.*'):
    if actor.attributes.get('role_name') == 'hero':
        hero = actor
        break

if hero is None:
    print("ERROR: No hero vehicle found!")
else:
    t = hero.get_transform()
    v = hero.get_velocity()
    c = hero.get_control()
    speed = (v.x**2 + v.y**2 + v.z**2) ** 0.5
    print(f"\nHero: {hero.type_id} id={hero.id}")
    print(f"  Location : ({t.location.x:.1f}, {t.location.y:.1f}, {t.location.z:.1f})")
    print(f"  Speed    : {speed:.2f} m/s")
    print(f"  Control  : throttle={c.throttle:.2f} steer={c.steer:.3f} brake={c.brake:.2f}")

    # Force move: apply throttle manually for 3 seconds
    print("\n>>> Applying manual throttle for 3 seconds...")
    for _ in range(60):
        ctrl = carla.VehicleControl()
        ctrl.throttle = 0.6
        ctrl.steer = 0.0
        ctrl.brake = 0.0
        hero.apply_control(ctrl)
        time.sleep(0.05)

    v = hero.get_velocity()
    speed = (v.x**2 + v.y**2 + v.z**2) ** 0.5
    print(f"Speed after throttle: {speed:.2f} m/s")

    if speed < 0.1:
        print("WARNING: Bus still not moving — may be stuck or braked by physics.")
    else:
        print("Bus is moving! Enabling autopilot...")
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(False)
        hero.set_autopilot(True, 8000)
        print("Autopilot ON.")

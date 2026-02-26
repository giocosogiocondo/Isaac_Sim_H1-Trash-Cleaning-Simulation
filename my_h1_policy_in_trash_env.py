from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import argparse
import os, sys
import carb
import carb.input
import omni.appwindow
import omni.usd
import numpy as np
from pxr import UsdGeom, Usd

from isaacsim.core.api import World
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

# --- local imports (trash_env folder) ---
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.insert(0, THIS_DIR)

from env_gen import spawn_room_walls, spawn_internal_walls, spawn_trash  # noqa: E402
from trash_utils import sample_free_xy  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--num-robots", type=int, default=1)
parser.add_argument("--seed", type=int, default=None, help="omit for random")
args = parser.parse_args()

print(f"num_robots={args.num_robots}, seed={args.seed}")

first_step = True
reset_needed = False
robots = []
base_command = np.zeros(3, dtype=float)

# -----------------------
# Arrow-key teleop
# base_command = [vx, vy, yaw_rate]
# -----------------------
keys_down = {"UP": False, "DOWN": False, "LEFT": False, "RIGHT": False, "SHIFT": False}

LIN_V = 0.7      # forward/back speed
YAW_V = 0.9      # turning speed
BOOST = 1.8      # shift boost


def _recompute_base_command():
    global base_command
    mul = BOOST if keys_down["SHIFT"] else 1.0

    vx = 0.0
    yaw = 0.0

    if keys_down["UP"]:
        vx += LIN_V
    if keys_down["DOWN"]:
        vx -= LIN_V

    if keys_down["LEFT"]:
        yaw += YAW_V
    if keys_down["RIGHT"]:
        yaw -= YAW_V

    base_command = np.array([vx * mul, 0.0, yaw * mul], dtype=float)


def _key_event_handler(event, *args, **kwargs):
    global reset_needed

    et = event.type
    key = event.input

    is_down = (et == carb.input.KeyboardEventType.KEY_PRESS)
    is_up = (et == carb.input.KeyboardEventType.KEY_RELEASE)
    if not (is_down or is_up):
        return

    val = is_down

    if key == carb.input.KeyboardInput.UP:
        keys_down["UP"] = val
    elif key == carb.input.KeyboardInput.DOWN:
        keys_down["DOWN"] = val
    elif key == carb.input.KeyboardInput.LEFT:
        keys_down["LEFT"] = val
    elif key == carb.input.KeyboardInput.RIGHT:
        keys_down["RIGHT"] = val
    elif key in (carb.input.KeyboardInput.LEFT_SHIFT, carb.input.KeyboardInput.RIGHT_SHIFT):
        keys_down["SHIFT"] = val
    elif is_down and key == carb.input.KeyboardInput.SPACE:
        # emergency stop
        keys_down.update({"UP": False, "DOWN": False, "LEFT": False, "RIGHT": False})
    elif is_down and key == carb.input.KeyboardInput.R:
        # reset world
        reset_needed = True

    _recompute_base_command()


def prim_world_xy(prim_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_],
        useExtentsHint=True
    )
    bbox = bbox_cache.ComputeWorldBound(prim)

    # Range3d 중심 계산 (버전 호환)
    r = bbox.ComputeAlignedBox()
    mn = r.GetMin()
    mx = r.GetMax()
    cx = (float(mn[0]) + float(mx[0])) * 0.5
    cy = (float(mn[1]) + float(mx[1])) * 0.5
    return np.array([cx, cy], dtype=float)


def remove_trash_prim(prim_path: str):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        stage.RemovePrim(prim_path)
        print(f"[Trash] Removed {prim_path}")


def update_trash_removal(dt: float, robot_xy: np.ndarray):
    for p in list(trash_dwell.keys()):
        txy = prim_world_xy(p)
        if txy is None:
            trash_dwell.pop(p, None)
            continue

        dist = float(np.linalg.norm(robot_xy - txy))
        if dist <= APPROACH_RADIUS:
            trash_dwell[p] += dt
            if trash_dwell[p] >= WAIT_DURATION:
                remove_trash_prim(p)
                trash_dwell.pop(p, None)
        else:
            trash_dwell[p] = 0.0


def on_physics_step(step_size) -> None:
    global first_step, reset_needed, base_command
    if first_step:
        for robot in robots:
            robot.initialize()
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
        base_command[:] = 0.0

        # reset trash timers too
        for p in list(trash_dwell.keys()):
            trash_dwell[p] = 0.0
    else:
        # 1) move robots
        for robot in robots:
            robot.forward(step_size, base_command)

        # 2) trash 제거 로직 (H1_0 기준)
        robot_xy = prim_world_xy("/World/H1_0")
        if robot_xy is not None:
            update_trash_removal(step_size, robot_xy)


# --- World ---
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=8 / 200)

# Ground plane (physics collider)
my_world.scene.add_default_ground_plane()

# --- Random environment ---
rng = np.random.default_rng(args.seed)
occupied = []  # AABB list for internal walls, trash, and robot spawn avoidance

room_w, room_h = spawn_room_walls(my_world, rng)
spawn_internal_walls(my_world, rng, room_w, room_h, occupied_aabbs=occupied)
trash_paths, trash_positions = spawn_trash(my_world, rng, room_w, room_h, occupied_aabbs=occupied, n_trash=10)

APPROACH_RADIUS = 1.0   # meters
WAIT_DURATION   = 3.0   # seconds

# trash_path -> time_in_radius
trash_dwell = {p: 0.0 for p in trash_paths}

print(f"Room size: W={room_w:.2f}, H={room_h:.2f}")
print("Trash count:", len(trash_paths))

# --- Spawn robot(s) on free space ---
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    raise RuntimeError("assets_root_path is None")

robot_footprint = (1.0, 1.0)  # (x,y) footprint AABB for spawn clearance
robot_margin = 1.2            # keep away from outer walls

for i in range(args.num_robots):
    (x, y), (mn, mx) = sample_free_xy(
        rng=rng,
        room_w=room_w,
        room_h=room_h,
        margin=robot_margin,
        occupied_aabbs=occupied,
        obj_size_xy=robot_footprint,
    )
    occupied.append((mn, mx))

    h1 = H1FlatTerrainPolicy(
        prim_path=f"/World/H1_{i}",
        name=f"H1_{i}",
        usd_path=assets_root_path + "/Isaac/Robots/Unitree/H1/h1.usd",
        position=np.array([x, y, 1.05], dtype=float),
    )
    robots.append(h1)

# --- Run ---
my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# keyboard subscription (one-time)
app_window = omni.appwindow.get_default_app_window()
keyboard = app_window.get_keyboard()
input_iface = carb.input.acquire_input_interface()
kb_sub = input_iface.subscribe_to_keyboard_events(keyboard, _key_event_handler)

try:
    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_stopped():
            reset_needed = True
finally:
    # clean up subscription
    try:
        input_iface.unsubscribe_from_keyboard_events(keyboard, kb_sub)
    except Exception:
        pass

    simulation_app.close()
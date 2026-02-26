# env_gen.py (Isaac Sim 4.5.0)
import numpy as np
import omni.usd
from pxr import UsdPhysics, PhysxSchema

from omni.isaac.core.objects import VisualCuboid
from trash_utils import sample_free_xy


# -----------------------
# USD / PhysX helpers
# -----------------------
def _stage():
    return omni.usd.get_context().get_stage()

def enable_collision(prim_path: str):
    """Force-enable collision on a prim."""
    prim = _stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim)

def _pos(x, y, z):
    return (float(x), float(y), float(z))

def _scale(x, y, z):
    return (float(x), float(y), float(z))

def _color_np(color):
    # VisualCuboid 내부에서 color.tolist()를 쓰므로 numpy array 필요
    return np.array([float(color[0]), float(color[1]), float(color[2])], dtype=float)

_BASE_CUBE_SIZE = 1.0  # VisualCuboid.size는 스칼라 (정육면체 변 길이), 직육면체는 scale로


# -----------------------
# Spawn: outer room walls
# -----------------------
def spawn_room_walls(
    world,
    rng: np.random.Generator,
    room_size_range=((8.0, 14.0), (6.0, 12.0)),
    wall_thickness=0.2,
    wall_height=2.2,
    color=(0.7, 0.7, 0.7),
    root_path="/World/Walls",
):
    room_w = float(rng.uniform(*room_size_range[0]))
    room_h = float(rng.uniform(*room_size_range[1]))
    t = float(wall_thickness)
    h = float(wall_height)
    zc = h / 2.0

    def add(name, center_xyz, dims_xyz):
        prim_path = f"{root_path}/{name}"
        prim = VisualCuboid(
            prim_path=prim_path,
            name=name,
            position=_pos(*center_xyz),
            size=float(_BASE_CUBE_SIZE),
            scale=_scale(*dims_xyz),
            color=_color_np(color),
        )
        world.scene.add(prim)
        enable_collision(prim_path)

    # dims_xyz = 실제 (X, Y, Z) 치수
    add("north", (0.0,  room_h/2 + t/2, zc), (room_w + 2*t, t, h))
    add("south", (0.0, -room_h/2 - t/2, zc), (room_w + 2*t, t, h))
    add("east",  ( room_w/2 + t/2, 0.0, zc), (t, room_h, h))
    add("west",  (-room_w/2 - t/2, 0.0, zc), (t, room_h, h))

    return room_w, room_h


# -----------------------
# Spawn: internal walls
# -----------------------
def spawn_internal_walls(
    world,
    rng: np.random.Generator,
    room_w: float,
    room_h: float,
    occupied_aabbs: list,
    n_walls_range=(3, 8),
    wall_length_range=(1.5, 4.0),
    wall_thickness_range=(0.2, 0.35),
    margin_from_outer=1.0,
    wall_height=2.2,
    color=(0.75, 0.75, 0.75),
    root_path="/World/InternalWalls",
):
    prim_paths = []
    h = float(wall_height)
    zc = h / 2.0

    n = int(rng.integers(n_walls_range[0], n_walls_range[1]))
    for i in range(n):
        is_horizontal = bool(rng.integers(0, 2))
        length = float(rng.uniform(*wall_length_range))
        thickness = float(rng.uniform(*wall_thickness_range))

        size_xy = (length, thickness) if is_horizontal else (thickness, length)

        (x, y), (mn, mx) = sample_free_xy(
            rng=rng,
            room_w=room_w,
            room_h=room_h,
            margin=margin_from_outer,
            occupied_aabbs=occupied_aabbs,
            obj_size_xy=size_xy,
        )
        occupied_aabbs.append((mn, mx))

        prim_path = f"{root_path}/wall_{i}"
        prim = VisualCuboid(
            prim_path=prim_path,
            name=f"wall_{i}",
            position=_pos(x, y, zc),
            size=float(_BASE_CUBE_SIZE),
            scale=_scale(size_xy[0], size_xy[1], h),
            color=_color_np(color),
        )
        world.scene.add(prim)
        enable_collision(prim_path)
        prim_paths.append(prim_path)

    return prim_paths


# -----------------------
# Spawn: trash
# -----------------------
def spawn_trash(
    world,
    rng: np.random.Generator,
    room_w: float,
    room_h: float,
    occupied_aabbs: list,
    n_trash=10,
    trash_size_xy=(0.15, 0.15),
    trash_height=0.10,
    margin_from_walls=1.0,
    color=(0.2, 0.8, 0.2),
    root_path="/World/Trash",
):
    paths = []
    positions = []

    sx, sy = float(trash_size_xy[0]), float(trash_size_xy[1])
    hz = float(trash_height)
    zc = hz / 2.0  # 바닥 위에 얹히게 중심 z를 height/2로

    for k in range(int(n_trash)):
        (x, y), (mn, mx) = sample_free_xy(
            rng=rng,
            room_w=room_w,
            room_h=room_h,
            margin=margin_from_walls,
            occupied_aabbs=occupied_aabbs,
            obj_size_xy=(sx, sy),
        )
        occupied_aabbs.append((mn, mx))

        prim_path = f"{root_path}/trash_{k}"
        prim = VisualCuboid(
            prim_path=prim_path,
            name=f"trash_{k}",
            position=_pos(x, y, zc),
            size=float(_BASE_CUBE_SIZE),
            scale=_scale(sx, sy, hz),
            color=_color_np(color),
        )
        world.scene.add(prim)
        #enable_collision(prim_path)  콜리젼 킬라면 필요하긴한데 일단 베타 수준이니깐 꺼뒀습니다. 필요하면 키세용용

        paths.append(prim_path)
        positions.append((float(x), float(y)))

    return paths, positions
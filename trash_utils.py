# utils.py
import numpy as np

def aabb_overlap(a_min, a_max, b_min, b_max) -> bool:
    """2D AABB overlap check (x,y only)."""
    return (
        a_min[0] <= b_max[0] and a_max[0] >= b_min[0] and
        a_min[1] <= b_max[1] and a_max[1] >= b_min[1]
    )

def make_aabb(center_xy, size_xy):
    """Return (min_xy, max_xy) AABB for 2D rectangle."""
    cx, cy = float(center_xy[0]), float(center_xy[1])
    sx, sy = float(size_xy[0]), float(size_xy[1])
    half = np.array([sx / 2.0, sy / 2.0], dtype=float)
    mn = np.array([cx, cy], dtype=float) - half
    mx = np.array([cx, cy], dtype=float) + half
    return mn, mx

def sample_free_xy(
    rng: np.random.Generator,
    room_w: float,
    room_h: float,
    margin: float,
    occupied_aabbs: list,
    obj_size_xy,
    max_tries: int = 5000,
):
    """
    Sample a free (x,y) inside the room bounds with margin,
    rejecting if overlaps with any occupied AABBs.
    """
    obj_size_xy = (float(obj_size_xy[0]), float(obj_size_xy[1]))
    for _ in range(max_tries):
        x = float(rng.uniform(-room_w / 2 + margin, room_w / 2 - margin))
        y = float(rng.uniform(-room_h / 2 + margin, room_h / 2 - margin))
        mn, mx = make_aabb((x, y), obj_size_xy)

        ok = True
        for (omn, omx) in occupied_aabbs:
            if aabb_overlap(mn, mx, omn, omx):
                ok = False
                break

        if ok:
            return (x, y), (mn, mx)

    raise RuntimeError(
        "Failed to sample a free position. Reduce density, increase room size, "
        "decrease object size, or reduce margin."
    )
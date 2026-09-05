"""
RV pantry interlocking shelf for Bambu Lab A1 (256^3 mm).

Glue-free mechanical lock:
  - Dovetail tabs for XY registration
  - Bowtie keys (press-fit) in seam slots
  - Center cross key + underside snap retainer for Z lock at the middle
  - Support-free print orientation (no mid-height overhangs)
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

# ---------------------------------------------------------------------------
# Dimensions (mm)
# ---------------------------------------------------------------------------
IN = 25.4

WIDTH = 14.5 * IN  # 368.3 — finished outer width including side lips
DEPTH = 18 * IN  # 457.2 — finished outer depth
DECK_T = 0.3 * IN  # 7.62
FRONT_LIP_H = 0.5 * IN  # 12.7
FRONT_LIP_T = 3.0
SIDE_LIP_H = 31.3  # above deck; total tile height ≈ DECK_T + SIDE_LIP_H
SIDE_LIP_T = 4.0

# Dovetails
TAB_PROJ = 8.0
TAB_NECK = 18.0
TAB_HEAD = 26.0
TAB_CLEAR = 0.30

# Half-lap Z interlock along seams
LAP_OVER = 6.0
LAP_CLEAR = 0.25

# Bowtie keys (seam locks)
BOWTIE_LEN = 38.0
BOWTIE_END = 20.0
BOWTIE_WAIST = 10.0
# Key is oversized vs slot (slot clearance alone never changed the key STL)
BOWTIE_KEY_SCALE = 1.05  # waist 10.5 mm
BOWTIE_SLOT_SCALE = 1.00  # waist 10.0 mm → 0.5 mm press-fit on waist

# Center cross key
CENTER_ARM = 42.0  # length of each arm from center
CENTER_ARM_W = 18.0

# Min center-to-center gap along a seam: half dovetail head + half bowtie length + pad
SEAM_FEATURE_GAP = TAB_HEAD / 2 + BOWTIE_LEN / 2 + 6.0  # ~38 mm
# Keep dovetails/bowties fully clear of the center cross (arm + half tab head + pad)
CENTER_KEEPOUT = CENTER_ARM + TAB_HEAD / 2 + 10.0  # ~65 mm from mid-seam
CENTER_SLOT_CLEAR = 0.25
CENTER_POST_R = 5.2          # bulb radius at tip
CENTER_POST_NECK_R = 4.3     # groove the retainer seats in
CENTER_POST_H = 12.0         # total post below deck
RETAINER_T = 5.0
RETAINER_ARM = 30.0
RETAINER_ARM_W = 22.0
RETAINER_HUB_R = 14.0
# Hole between neck and bulb so it flexes over the tip then locks
SNAP_HOLE_R = 4.85

# Command strips
STRIP_H = 93.0
STRIP_W = 19.0
STRIP_GUIDE_MARGIN = 1.5
STRIP_ENGRAVE = 0.35

# Underside ribs
RIB_H = 8.0
RIB_W = 4.0

BED = 256.0
OUT = Path(__file__).resolve().parent / "exports"

MID_X = WIDTH / 2
MID_Y = DEPTH / 2


def _dovetail_tab(cx: float, axis: str, male: bool, sign: float) -> Polygon:
    clear = -TAB_CLEAR if male else TAB_CLEAR
    neck = TAB_NECK + 2 * clear
    head = TAB_HEAD + 2 * clear
    proj = TAB_PROJ + (clear if not male else -clear)
    proj = max(proj, 0.5)
    u0, u1 = cx - neck / 2, cx + neck / 2
    uh0, uh1 = cx - head / 2, cx + head / 2

    if axis == "x":
        x0, x1 = 0.0, sign * proj
        if sign > 0:
            return Polygon([(x0, u0), (x0, u1), (x1, uh1), (x1, uh0)])
        return Polygon([(x0, u0), (x1, uh0), (x1, uh1), (x0, u1)])
    y0, y1 = 0.0, sign * proj
    if sign > 0:
        return Polygon([(u0, y0), (u1, y0), (uh1, y1), (uh0, y1)])
    return Polygon([(u0, y0), (uh0, y1), (uh1, y1), (u1, y0)])


def _alternating_seam_features(
    lo: float, hi: float, keepout_center: float, keepout_radius: float
) -> tuple[list[float], list[float]]:
    """Place dovetails and bowties on one seam segment, no overlaps.

    Pattern per open span (outside center keepout): DT — BT — DT
    Returns (dovetail_centers, bowtie_centers) along the seam axis.
    """
    edge_margin = max(TAB_HEAD / 2 + 8.0, BOWTIE_LEN / 2 + 8.0)
    keep_lo = keepout_center - keepout_radius
    keep_hi = keepout_center + keepout_radius

    spans: list[tuple[float, float]] = []
    if lo + edge_margin < keep_lo - 2:
        spans.append((lo + edge_margin, min(hi - edge_margin, keep_lo - 2)))
    if hi - edge_margin > keep_hi + 2:
        spans.append((max(lo + edge_margin, keep_hi + 2), hi - edge_margin))

    dovetails: list[float] = []
    bowties: list[float] = []
    for a, b in spans:
        if b - a < 2 * SEAM_FEATURE_GAP:
            # Short span: one dovetail only
            dovetails.append((a + b) / 2)
            continue
        # DT at ends of span, BT in the middle (guaranteed clear of both)
        d0, d1 = a, b
        bt = (a + b) / 2
        # If span is long enough, add a second bowtie + third dovetail
        if b - a >= 4 * SEAM_FEATURE_GAP:
            d_mid = (a + b) / 2
            bt0 = (a + d_mid) / 2
            bt1 = (d_mid + b) / 2
            dovetails.extend([d0, d_mid, d1])
            bowties.extend([bt0, bt1])
        else:
            dovetails.extend([d0, d1])
            bowties.append(bt)
    return dovetails, bowties


def vertical_seam_dovetails(qy: int) -> list[float]:
    """Dovetail Y centers on the vertical seam for this half."""
    dts, _ = _alternating_seam_features(0.0, DEPTH, MID_Y, CENTER_KEEPOUT)
    y0 = 0.0 if qy == 0 else MID_Y
    y1 = MID_Y if qy == 0 else DEPTH
    return [y for y in dts if y0 + 1 < y < y1 - 1]


def horizontal_seam_dovetails(qx: int) -> list[float]:
    """Dovetail X centers on the horizontal seam for this half."""
    dts, _ = _alternating_seam_features(0.0, WIDTH, MID_X, CENTER_KEEPOUT)
    x0 = 0.0 if qx == 0 else MID_X
    x1 = MID_X if qx == 0 else WIDTH
    return [x for x in dts if x0 + 1 < x < x1 - 1]


def bowtie_polygon(cx: float, cy: float, angle_deg: float, scale: float = 1.0) -> Polygon:
    """Hourglass / bowtie centered at (cx, cy). angle 0 → long axis along +X."""
    L = BOWTIE_LEN * scale / 2
    E = BOWTIE_END * scale / 2
    W = BOWTIE_WAIST * scale / 2
    # Local coords: long axis = u, short = v
    pts = [
        (-L, -E),
        (-W, -W),
        (W, -W),
        (L, -E),
        (L, E),
        (W, W),
        (-W, W),
        (-L, E),
    ]
    rad = np.deg2rad(angle_deg)
    c, s = np.cos(rad), np.sin(rad)
    world = []
    for u, v in pts:
        x = cx + u * c - v * s
        y = cy + u * s + v * c
        world.append((x, y))
    return Polygon(world)


def center_cross_polygon(clear: float = 0.0) -> Polygon:
    """Plus shape centered on shelf middle. clear > 0 enlarges (slot)."""
    a = CENTER_ARM + clear
    w = CENTER_ARM_W / 2 + clear
    # Union of two rectangles
    return unary_union(
        [
            box(MID_X - a, MID_Y - w, MID_X + a, MID_Y + w),
            box(MID_X - w, MID_Y - a, MID_X + w, MID_Y + a),
        ]
    )


def deck_polygon(qx: int, qy: int) -> Polygon:
    x0 = 0.0 if qx == 0 else MID_X
    x1 = MID_X if qx == 0 else WIDTH
    y0 = 0.0 if qy == 0 else MID_Y
    y1 = MID_Y if qy == 0 else DEPTH

    poly = box(x0, y0, x1, y1)

    # Vertical seam dovetails (coordinated with bowtie placement)
    if qx == 0:
        for cy in vertical_seam_dovetails(qy):
            tab = _dovetail_tab(cy, axis="x", male=True, sign=+1.0)
            tab = Polygon([(MID_X + x, y) for x, y in tab.exterior.coords])
            poly = unary_union([poly, tab])
    else:
        for cy in vertical_seam_dovetails(qy):
            tab = _dovetail_tab(cy, axis="x", male=False, sign=+1.0)
            tab = Polygon([(MID_X + x, y) for x, y in tab.exterior.coords])
            poly = poly.difference(tab)

    # Horizontal seam dovetails
    if qy == 0:
        for cx in horizontal_seam_dovetails(qx):
            tab = _dovetail_tab(cx, axis="y", male=True, sign=+1.0)
            tab = Polygon([(x, MID_Y + y) for x, y in tab.exterior.coords])
            poly = unary_union([poly, tab])
    else:
        for cx in horizontal_seam_dovetails(qx):
            tab = _dovetail_tab(cx, axis="y", male=False, sign=+1.0)
            tab = Polygon([(x, MID_Y + y) for x, y in tab.exterior.coords])
            poly = poly.difference(tab)

    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly


def extrude_polygon(poly: Polygon, z0: float, z1: float) -> trimesh.Trimesh:
    if poly.is_empty:
        return trimesh.Trimesh()
    if poly.geom_type == "MultiPolygon":
        meshes = [extrude_polygon(p, z0, z1) for p in poly.geoms if not p.is_empty]
        return trimesh.util.concatenate(meshes) if meshes else trimesh.Trimesh()
    mesh = trimesh.creation.extrude_polygon(poly, height=z1 - z0)
    mesh.apply_translation([0, 0, z0])
    return mesh


def box_mesh(x0, y0, z0, x1, y1, z1) -> trimesh.Trimesh:
    extents = [x1 - x0, y1 - y0, z1 - z0]
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(
        [x0 + extents[0] / 2, y0 + extents[1] / 2, z0 + extents[2] / 2]
    )
    return mesh


def cyl_mesh(x: float, y: float, z0: float, z1: float, r: float) -> trimesh.Trimesh:
    h = z1 - z0
    m = trimesh.creation.cylinder(radius=r, height=h, sections=48)
    m.apply_translation([x, y, z0 + h / 2])
    return m


def union_meshes(meshes: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    meshes = [m for m in meshes if m is not None and len(getattr(m, "faces", [])) > 0]
    if not meshes:
        return trimesh.Trimesh()
    if len(meshes) == 1:
        return meshes[0].copy()
    out = meshes[0]
    for m in meshes[1:]:
        out = trimesh.boolean.union([out, m], engine="manifold")
    return out


def difference_meshes(a: trimesh.Trimesh, b: trimesh.Trimesh) -> trimesh.Trimesh:
    return trimesh.boolean.difference([a, b], engine="manifold")


def strip_centers_y() -> list[float]:
    margin = 20.0 + STRIP_W / 2
    mid = DEPTH / 2 + (STRIP_W / 2 + 8.0)
    return [margin, mid, DEPTH - margin]


def bowtie_slot_centers_vertical(qy: int) -> list[tuple[float, float]]:
    """Bowtie centers on vertical seam, already clear of dovetails + center cross."""
    _, bts = _alternating_seam_features(0.0, DEPTH, MID_Y, CENTER_KEEPOUT)
    y0 = 0.0 if qy == 0 else MID_Y
    y1 = MID_Y if qy == 0 else DEPTH
    return [(MID_X, cy) for cy in bts if y0 + 1 < cy < y1 - 1]


def bowtie_slot_centers_horizontal(qx: int) -> list[tuple[float, float]]:
    """Bowtie centers on horizontal seam, already clear of dovetails + center cross."""
    _, bts = _alternating_seam_features(0.0, WIDTH, MID_X, CENTER_KEEPOUT)
    x0 = 0.0 if qx == 0 else MID_X
    x1 = MID_X if qx == 0 else WIDTH
    return [(cx, MID_Y) for cx in bts if x0 + 1 < cx < x1 - 1]


def all_bowtie_slots() -> list[tuple[float, float, float]]:
    """Unique (cx, cy, angle_deg) for every bowtie on the shelf."""
    slots: list[tuple[float, float, float]] = []
    seen: set[tuple[int, int]] = set()
    for qy in (0, 1):
        for cx, cy in bowtie_slot_centers_vertical(qy):
            key = (int(round(cx * 10)), int(round(cy * 10)))
            if key in seen:
                continue
            seen.add(key)
            slots.append((cx, cy, 90.0))  # long axis along Y (seam is vertical)
    for qx in (0, 1):
        for cx, cy in bowtie_slot_centers_horizontal(qx):
            key = (int(round(cx * 10)), int(round(cy * 10)))
            if key in seen:
                continue
            seen.add(key)
            slots.append((cx, cy, 0.0))  # long axis along X
    return slots


def half_lap_meshes(qx: int, qy: int) -> tuple[list[trimesh.Trimesh], list[trimesh.Trimesh]]:
    """No mid-height half-laps — those overhang when printed flat (Bambu cantilever warning).

    Z-lock is provided by bowtie flanges + center key/retainer instead.
    """
    return [], []


def key_slot_cutters(qx: int, qy: int) -> list[trimesh.Trimesh]:
    """Through-deck cutters for bowties + this tile's share of the center cross."""
    cutters: list[trimesh.Trimesh] = []
    x0 = 0.0 if qx == 0 else MID_X
    x1 = MID_X if qx == 0 else WIDTH
    y0 = 0.0 if qy == 0 else MID_Y
    y1 = MID_Y if qy == 0 else DEPTH
    tile_box = box(x0 - 1, y0 - 1, x1 + 1, y1 + 1)

    # Bowties on seams belonging to this tile
    for cx, cy, ang in all_bowtie_slots():
        slot = bowtie_polygon(cx, cy, ang, scale=BOWTIE_SLOT_SCALE)
        # Only cut the portion overlapping this tile
        local = slot.intersection(tile_box)
        if local.is_empty or local.area < 1.0:
            continue
        cutters.append(extrude_polygon(local, -0.2, DECK_T + 0.2))

    # Center cross — each tile gets its quadrant of the plus
    cross = center_cross_polygon(clear=CENTER_SLOT_CLEAR)
    local = cross.intersection(tile_box)
    if not local.is_empty and local.area > 1.0:
        cutters.append(extrude_polygon(local, -0.2, DECK_T + 0.2))

    return cutters


def side_lip_mesh(qx: int, qy: int) -> trimesh.Trimesh | None:
    """Tall side lip only — no horizontal tongues (those print as cantilevers)."""
    y0 = 0.0 if qy == 0 else MID_Y
    y1 = MID_Y if qy == 0 else DEPTH
    z1 = DECK_T + SIDE_LIP_H
    if qx == 0:
        x0, x1 = 0.0, SIDE_LIP_T
    else:
        x0, x1 = WIDTH - SIDE_LIP_T, WIDTH
    return box_mesh(x0, y0, 0.0, x1, y1, z1)


def front_lip_mesh(qx: int) -> trimesh.Trimesh | None:
    """Front retaining lip only — butt joint at center, no horizontal tongue."""
    if qx == 0:
        x0, x1 = SIDE_LIP_T, MID_X
    else:
        x0, x1 = MID_X, WIDTH - SIDE_LIP_T
    z0, z1 = DECK_T, DECK_T + FRONT_LIP_H
    return box_mesh(x0, 0.0, z0, x1, FRONT_LIP_T, z1)


def rib_meshes(qx: int, qy: int) -> list[trimesh.Trimesh]:
    """No hanging underside ribs — they leave the side lips floating above the bed.

    Stiffness comes from deck thickness + tall side lips + slicer infill.
    """
    return []


def clear_rib_under_keys(body: trimesh.Trimesh) -> trimesh.Trimesh:
    return body


def strip_guide_cutters(qx: int, qy: int) -> list[trimesh.Trimesh]:
    """Guides for Command strips laid horizontally (long axis along depth).

    Short side lips can't fit a 93 mm strip vertically; orient strips along the lip.
    """
    y0 = 0.0 if qy == 0 else MID_Y
    y1 = MID_Y if qy == 0 else DEPTH
    guides = []
    # Vertical extent on lip face ≈ strip width; length along depth ≈ strip length
    gh = STRIP_W + 2 * STRIP_GUIDE_MARGIN
    glen = STRIP_H + 2 * STRIP_GUIDE_MARGIN
    lip_top = DECK_T + SIDE_LIP_H
    z_mid = (DECK_T + lip_top) / 2
    z0 = max(z_mid - gh / 2, DECK_T + 1.0)
    z1 = min(z_mid + gh / 2, lip_top - 1.0)
    for cy in strip_centers_y():
        gy0, gy1 = cy - glen / 2, cy + glen / 2
        oy0, oy1 = max(gy0, y0 + 1), min(gy1, y1 - 1)
        if oy1 - oy0 < 8:
            continue
        if qx == 0:
            guides.append(box_mesh(-0.1, oy0, z0, STRIP_ENGRAVE, oy1, z1))
        else:
            guides.append(box_mesh(WIDTH - STRIP_ENGRAVE, oy0, z0, WIDTH + 0.1, oy1, z1))
    return guides


def build_tile(qx: int, qy: int) -> trimesh.Trimesh:
    name = {(0, 0): "FL", (1, 0): "FR", (0, 1): "BL", (1, 1): "BR"}[(qx, qy)]
    print(f"  Building {name}...")

    deck = extrude_polygon(deck_polygon(qx, qy), 0.0, DECK_T)
    parts: list[trimesh.Trimesh] = [deck]

    lip = side_lip_mesh(qx, qy)
    if lip is not None:
        parts.append(lip)
    if qy == 0:
        flip = front_lip_mesh(qx)
        if flip is not None:
            parts.append(flip)

    body = union_meshes(parts)

    for c in key_slot_cutters(qx, qy):
        body = difference_meshes(body, c)
    for c in strip_guide_cutters(qx, qy):
        body = difference_meshes(body, c)

    # Flat on bed: deck bottom at z=0, lips rising (no hanging geometry)
    body.apply_translation([0, 0, -body.bounds[0][2]])
    body.process(validate=True)

    extents = body.bounding_box.extents
    print(f"    {name}: bounds {extents[0]:.1f} x {extents[1]:.1f} x {extents[2]:.1f} mm")
    if any(v > BED + 0.5 for v in extents):
        raise RuntimeError(f"{name} exceeds A1 bed {BED} mm: {extents}")
    body.metadata["name"] = name
    return body


def build_bowtie_key() -> trimesh.Trimesh:
    """Press-fit bowtie — KEY is oversized relative to the tile slot."""
    poly = bowtie_polygon(0.0, 0.0, 0.0, scale=BOWTIE_KEY_SCALE)
    key = extrude_polygon(poly, 0.0, DECK_T)
    key.process(validate=True)
    e = key.bounding_box.extents
    print(
        f"    bowtie_key: bounds {e[0]:.2f} x {e[1]:.2f} x {e[2]:.2f} mm "
        f"(scale {BOWTIE_KEY_SCALE}, slot scale {BOWTIE_SLOT_SCALE})"
    )
    key.metadata["name"] = "bowtie_key"
    return key


def build_center_key() -> trimesh.Trimesh:
    """Plus-shaped center key with underside snap post.

    Printed with the cross flat on the bed and the snap post pointing up.
    Post: bulb at tip → long neck (fits thick retainer) → shaft to deck.
    """
    poly = center_cross_polygon(clear=0.0)
    body = extrude_polygon(poly, 0.0, DECK_T)
    flange = center_cross_polygon(clear=1.0)
    flange_mesh = extrude_polygon(flange, DECK_T - 0.9, DECK_T)

    # Assembly coords: deck at z=0..DECK_T, post hangs below (negative z)
    bulb_h = 2.0
    neck_h = RETAINER_T + 1.5  # retainer must fully seat in the groove
    shaft_top = 0.3
    z_bulb0 = -CENTER_POST_H
    z_bulb1 = z_bulb0 + bulb_h
    z_neck1 = z_bulb1 + neck_h

    bulb = cyl_mesh(MID_X, MID_Y, z_bulb0, z_bulb1, CENTER_POST_R)
    neck_m = cyl_mesh(MID_X, MID_Y, z_bulb1 - 0.2, z_neck1, CENTER_POST_NECK_R)
    shaft = cyl_mesh(MID_X, MID_Y, z_neck1 - 0.2, shaft_top, CENTER_POST_R - 0.3)
    # Lead-in chamfer tip (slightly pointed) for easier start through the hole
    tip = cyl_mesh(MID_X, MID_Y, z_bulb0 - 0.01, z_bulb0 + 0.8, CENTER_POST_NECK_R)

    key = union_meshes([body, flange_mesh, shaft, neck_m, bulb, tip])

    key.apply_translation([-MID_X, -MID_Y, 0.0])
    flip = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    key.apply_transform(flip)
    key.apply_translation([0.0, 0.0, -key.bounds[0][2]])
    key.process(validate=True)
    e = key.bounding_box.extents
    print(f"    center_key: bounds {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm")
    key.metadata["name"] = "center_key"
    return key


def build_center_retainer() -> trimesh.Trimesh:
    """Underside X-plate that snaps onto the center key post — locks all 4 tiles in Z."""
    arm = RETAINER_ARM
    w = RETAINER_ARM_W / 2
    plate = unary_union(
        [
            box(-arm, -w, arm, w),
            box(-w, -arm, w, arm),
            Polygon(
                [
                    (RETAINER_HUB_R * np.cos(t), RETAINER_HUB_R * np.sin(t))
                    for t in np.linspace(0, 2 * np.pi, 48, endpoint=False)
                ]
            ),
        ]
    )
    body = extrude_polygon(plate, 0.0, RETAINER_T)
    # Thru hole sized to flex over bulb, then lock on neck
    hole = cyl_mesh(0.0, 0.0, -0.1, RETAINER_T + 0.1, SNAP_HOLE_R)
    # Entry chamfer (larger on top face when printed flat)
    chamfer = cyl_mesh(0.0, 0.0, RETAINER_T - 1.2, RETAINER_T + 0.1, SNAP_HOLE_R + 0.55)
    body = difference_meshes(body, hole)
    body = difference_meshes(body, chamfer)
    # Flex kerfs — long enough to open over the bulb
    for ang in (45, 135, 225, 315):
        rad = np.deg2rad(ang)
        dx, dy = 7.5 * np.cos(rad), 7.5 * np.sin(rad)
        kerf = box_mesh(dx - 0.8, dy - 4.0, -0.1, dx + 0.8, dy + 4.0, RETAINER_T + 0.1)
        body = difference_meshes(body, kerf)

    body.process(validate=True)
    e = body.bounding_box.extents
    print(f"    center_retainer: bounds {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f} mm")
    print(
        f"    snap fit: bulb r={CENTER_POST_R:.2f} hole r={SNAP_HOLE_R:.2f} "
        f"neck r={CENTER_POST_NECK_R:.2f} neck_h={RETAINER_T + 1.5:.1f}"
    )
    body.metadata["name"] = "center_retainer"
    return body


def place_on_bed(mesh: trimesh.Trimesh, x: float, y: float) -> trimesh.Trimesh:
    """Put mesh on z=0 with its XY min corner at (x, y)."""
    m = mesh.copy()
    b = m.bounds
    m.apply_translation([x - b[0][0], y - b[0][1], -b[0][2]])
    return m


def write_single_mesh_3mf(mesh: trimesh.Trimesh, path: Path, name: str) -> None:
    m = place_on_bed(mesh, 5.0, 5.0)
    scene = trimesh.Scene()
    scene.add_geometry(m, geom_name=name, node_name=name)
    scene.export(path)


def pack_hardware_plate(meshes: dict[str, trimesh.Trimesh]) -> trimesh.Scene:
    """Pack center key, retainer, and 4 bowties onto one A1 plate with gaps."""
    scene = trimesh.Scene()
    x, y = 5.0, 5.0
    row_h = 0.0
    gap = 8.0
    bed_limit = BED - 5.0

    parts: list[tuple[str, trimesh.Trimesh]] = []
    if "center_key" in meshes:
        parts.append(("center_key", meshes["center_key"]))
    if "center_retainer" in meshes:
        parts.append(("center_retainer", meshes["center_retainer"]))
    for name, mesh in meshes.items():
        if name.startswith("bowtie_"):
            parts.append((name, mesh))

    for name, mesh in parts:
        e = mesh.bounding_box.extents
        if x + e[0] > bed_limit:
            x = 5.0
            y += row_h + gap
            row_h = 0.0
        if y + e[1] > bed_limit:
            raise RuntimeError(f"Hardware plate overflow placing {name}")
        m = place_on_bed(mesh, x, y)
        scene.add_geometry(m, geom_name=name, node_name=name)
        x += e[0] + gap
        row_h = max(row_h, e[1])
    return scene


def write_print_plates(meshes: dict[str, trimesh.Trimesh], out_dir: Path) -> list[Path]:
    """Write one 3MF per print plate — avoids Bambu 'overlap' on a single bed."""
    plates_dir = out_dir / "plates"
    plates_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    for name in ("FL", "FR", "BL", "BR"):
        path = plates_dir / f"plate_{name}.3mf"
        write_single_mesh_3mf(meshes[name], path, name)
        written.append(path)
        print(f"  Wrote plates/{path.name}")

    hw_path = plates_dir / "plate_hardware.3mf"
    pack_hardware_plate(meshes).export(hw_path)
    written.append(hw_path)
    print(f"  Wrote plates/{hw_path.name}")
    return written


def write_bambu_project(meshes: dict[str, trimesh.Trimesh], path: Path) -> None:
    """Single Bambu Studio project with one selectable plate per tile + hardware."""
    from bambu3mf import write_bambu_3mf

    def part(name: str, mesh: trimesh.Trimesh, pos=(0.0, 0.0), **settings):
        return {
            "name": name,
            "mesh": mesh,
            "pos": pos,
            "obj_settings": settings,
        }

    tile_settings = {
        "wall_loops": "4",
        "top_shell_layers": "5",
        "bottom_shell_layers": "5",
        "sparse_infill_density": "45%",
    }
    key_settings = {
        "wall_loops": "4",
        "sparse_infill_density": "100%",
        "top_shell_layers": "4",
        "bottom_shell_layers": "4",
    }

    plates = [
        {
            "name": "FL",
            "parts": [part("FL", meshes["FL"], **tile_settings)],
        },
        {
            "name": "FR",
            "parts": [part("FR", meshes["FR"], **tile_settings)],
        },
        {
            "name": "BL",
            "parts": [part("BL", meshes["BL"], **tile_settings)],
        },
        {
            "name": "BR",
            "parts": [part("BR", meshes["BR"], **tile_settings)],
        },
        {
            "name": "Hardware",
            "parts": [
                part("center_key", meshes["center_key"], pos=(-50.0, 45.0), **key_settings),
                part(
                    "center_retainer",
                    meshes["center_retainer"],
                    pos=(55.0, 45.0),
                    **key_settings,
                ),
                *[
                    part(
                        f"bowtie_{i+1}",
                        meshes[f"bowtie_{i+1}"],
                        pos=(-75.0 + i * 50.0, -55.0),
                        **key_settings,
                    )
                    for i in range(4)
                ],
            ],
        },
    ]

    overrides = {
        "wall_loops": "4",
        "sparse_infill_density": "45%",
        "top_shell_layers": "5",
        "bottom_shell_layers": "5",
        "layer_height": "0.2",
        "filament_type": ["PLA"],
    }
    write_bambu_3mf(str(path), plates, overrides)


def write_assembly_preview(meshes: dict[str, trimesh.Trimesh], path: Path) -> None:
    parts = []
    for name, mesh in meshes.items():
        if name.startswith("bowtie_") or name in ("center_key", "center_retainer"):
            continue
        parts.append(mesh.copy())
    # Show center key in place (world coords)
    if "center_key" in meshes:
        # Print mesh has deck on bed / post up; flip back for assembled preview
        ck = meshes["center_key"].copy()
        flip = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        ck.apply_transform(flip)
        b = ck.bounds
        # Seat deck top flush with tile deck top (z = DECK_T in assembly with ribs undone)
        ck.apply_translation([MID_X - (b[0][0] + b[1][0]) / 2, MID_Y - (b[0][1] + b[1][1]) / 2, DECK_T - b[1][2]])
        parts.append(ck)
    if "center_retainer" in meshes:
        cr = meshes["center_retainer"].copy()
        cr.apply_translation([MID_X, MID_Y, -RETAINER_T - 0.5])
        parts.append(cr)
    assembled = trimesh.util.concatenate(parts)
    assembled.export(path)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    print("Generating glue-free interlocking RV pantry shelf...")
    print(
        f"  Outer: {WIDTH:.1f} x {DEPTH:.1f} mm | deck {DECK_T:.1f} | "
        f"side lip {SIDE_LIP_H:.0f}H | dovetails + bowties + center lock"
    )

    tiles = {
        "FL": build_tile(0, 0),
        "FR": build_tile(1, 0),
        "BL": build_tile(0, 1),
        "BR": build_tile(1, 1),
    }

    print("  Building lock hardware...")
    slots = all_bowtie_slots()
    print(f"    {len(slots)} bowtie slots")
    center_key = build_center_key()
    retainer = build_center_retainer()
    bowtie = build_bowtie_key()

    meshes: dict[str, trimesh.Trimesh] = dict(tiles)
    meshes["center_key"] = center_key
    meshes["center_retainer"] = retainer
    for i, _ in enumerate(slots):
        meshes[f"bowtie_{i+1}"] = bowtie.copy()

    for name, mesh in meshes.items():
        if name.startswith("bowtie_"):
            continue
        stl_path = OUT / f"shelf_{name}.stl"
        mesh.export(stl_path)
        print(f"  Wrote {stl_path.name}")

    # One bowtie STL; print qty = number of slots
    bowtie.export(OUT / "shelf_bowtie_key.stl")
    print(f"  Wrote shelf_bowtie_key.stl  (print qty: {len(slots)})")

    preview = OUT / "shelf_assembly_preview.stl"
    write_assembly_preview(meshes, preview)
    print(f"  Wrote {preview.name}")

    print("  Writing per-plate 3MFs (backup)...")
    write_print_plates(meshes, OUT)

    threemf = OUT / "rv_pantry_shelf.3mf"
    print("  Writing Bambu multi-plate project...")
    write_bambu_project(meshes, threemf)
    print(f"  Wrote {threemf.name}  (open this in Bambu Studio — 5 plates)")

    print("\nFit check:")
    for name, mesh in tiles.items():
        e = mesh.bounding_box.extents
        ok = all(v <= BED + 0.01 for v in e)
        print(f"  {name}: {e[0]:.1f} x {e[1]:.1f} x {e[2]:.1f}  {'OK' if ok else 'FAIL'}")

    print("\nAssembly (no glue):")
    print("  1. Mate dovetails: FL-FR, BL-BR, then fronts onto backs.")
    print("  2. Press bowtie keys into each seam slot.")
    print("  3. Drop center cross key into the middle plus-slot (post down).")
    print("  4. Snap the underside retainer onto the center post.")
    print("\nDone.")


if __name__ == "__main__":
    main()

"""
레이아웃 실측 — simple_factory_layout.usda 를 열어 선반/매거진/로봇의 실제
치수와 좌표를 USD 에서 직접 뽑는다. 도면이나 눈대중이 아니라 씬 자체가
근거다.

    isaac_python isaacpjt/tools/measure_layout.py

무엇을 재나:
  1. 선반 (Shelf_01/Shelf_02)
       - 각 단(Level_1/Level_2) 상판의 윗면/아랫면 z, 판 두께
       - 선반 깊이(y) / 폭(x), 기둥 안쪽 유효 깊이
       - 단과 윗단 사이의 개구 높이 (매거진이 들어갈 수 있는 높이)
       - 통로쪽 전면(front plane) y 좌표와 기둥 안쪽면
  2. 매거진 (8 x 2 선반)
       - world pose, AABB, 바닥면/플랜지 윗면 z
       - 상판과의 간격 (물리 시작 시 떨어질 거리)
       - 전면 기둥과의 여유
       - QR 라벨 네 꼭짓점 -> 중심 world 좌표와 라벨 법선
  3. 로봇 (nova_carter1)
       - chassis_link -> m0609/base_link 장착 오프셋 (TF base_link->m0609_base 의 근거)
       - link_6 -> short_gripper -> rsd455 카메라 오프셋 (TF tool0->camera_link 의 근거)
       - 팔 베이스 높이 대비 각 선반 단의 상대 높이
  4. 파생 판정
       - 매거진 높이 vs 개구 높이
       - 팔 도달거리(스펙 0.9 m) 대비 선반 단까지의 수평/수직 거리

결과는 콘솔에 표로 찍고, 같은 내용을 YAML 로도 떨군다.
    isaacpjt/tools/out/layout_measured.yaml
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import json
import math
import sys
from pathlib import Path

# kit 은 fastShutdown(os._exit) 으로 끝나서 블록 버퍼에 남은 stdout 이 통째로
# 날아간다. 파일로 리다이렉트해도 끝까지 남도록 줄 단위 버퍼로 바꾼다.
sys.stdout.reconfigure(line_buffering=True)

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom


THIS_DIR     = Path(__file__).resolve().parent
ISAACPJT_DIR = THIS_DIR.parent
WORLD_USD    = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
OUT_PATH     = THIS_DIR / "out/layout_measured.yaml"

# M0609 카탈로그 스펙 (도달거리). 실측이 아니라 제조사 값이라 별도로 표기한다.
M0609_REACH_M = 0.900

SHELVES = {
    "Shelf_01": "/World/Environment/PickupZone/Shelf_01",
    "Shelf_02": "/World/Environment/PickupZone/Shelf_02",
}

MAGAZINE_GROUPS = {
    "shelf_1/top":    "/World/Magazines/shelf_1_magaines/top_magazines",
    "shelf_1/bottom": "/World/Magazines/shelf_1_magaines/bottom_magazines",
    "shelf_2/top":    "/World/Magazines/shelf_2_magaines/top_magazines",
    "shelf_2/bottom": "/World/Magazines/shelf_2_magaines/bottom_magazines",
}

# 어느 매거진 그룹이 어느 선반의 어느 단에 얹혀 있는지
GROUP_TO_LEVEL = {
    "shelf_1/top":    ("Shelf_01", "Level_2"),
    "shelf_1/bottom": ("Shelf_01", "Level_1"),
    "shelf_2/top":    ("Shelf_02", "Level_2"),
    "shelf_2/bottom": ("Shelf_02", "Level_1"),
}

ROBOT       = "/World/Robots/nova_carter1"
CHASSIS     = f"{ROBOT}/chassis_link"
ARM         = f"{ROBOT}/m0609"
ARM_BASE    = f"{ARM}/base_link"
ARM_LINK6   = f"{ARM}/link_6"
GRIPPER     = f"{ARM}/short_gripper"
GRIPPER_TIP = f"{GRIPPER}/gripper_tip"
CAMERA_ROOT = f"{GRIPPER}/rsd455"


# ══════════════════════════════════════════════════════════════
#  USD 조회 도우미
# ══════════════════════════════════════════════════════════════
def make_caches(stage):
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=False,
    )
    return xform_cache, bbox_cache


def world_xform(xform_cache, stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    return xform_cache.GetLocalToWorldTransform(prim)


def world_pos(xform_cache, stage, path):
    m = world_xform(xform_cache, stage, path)
    return None if m is None else np.array(m.ExtractTranslation())


def world_aabb(bbox_cache, stage, path):
    """world 축정렬 바운딩박스 -> (min[3], max[3]). 없으면 None"""
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    bound = bbox_cache.ComputeWorldBound(prim)
    rng = bound.ComputeAlignedRange()
    if rng.IsEmpty():
        return None
    return np.array(rng.GetMin()), np.array(rng.GetMax())


def relative_xform(xform_cache, stage, parent_path, child_path):
    """parent 프레임에서 본 child 의 (translation[3], quat_wxyz[4], rpy_deg[3])"""
    mp = world_xform(xform_cache, stage, parent_path)
    mc = world_xform(xform_cache, stage, child_path)
    if mp is None or mc is None:
        return None
    rel = mc * mp.GetInverse()
    t = np.array(rel.ExtractTranslation())
    q = rel.ExtractRotationQuat().GetNormalized()
    quat = np.array([q.GetReal(), *q.GetImaginary()])
    rpy = np.array(Gf.Rotation(q).Decompose(Gf.Vec3d(0, 0, 1),
                                            Gf.Vec3d(0, 1, 0),
                                            Gf.Vec3d(1, 0, 0)))[::-1]
    return t, quat, rpy


def f3(v, nd=4):
    return [round(float(x), nd) for x in v]


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — ROS 규약 맞추기
# ══════════════════════════════════════════════════════════════
def quat_to_mat(q):
    """(w,x,y,z) -> 3x3"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def mat_to_rpy(m):
    """R = Rz(yaw) Ry(pitch) Rx(roll) 분해 — static_transform_publisher 의
    --roll/--pitch/--yaw 와 같은 규약(고정축 XYZ)이다. 반환은 라디안."""
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(m[2, 0]) < 0.999999:
        roll = math.atan2(m[2, 1], m[2, 2])
        yaw  = math.atan2(m[1, 0], m[0, 0])
    else:                                   # 짐벌락
        roll = math.atan2(-m[1, 2], m[1, 1])
        yaw  = 0.0
    return np.array([roll, pitch, yaw])


def rpy_to_mat(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


# USD 카메라는 자기 프레임의 -Z 를 본다(+X 오른쪽, +Y 위).
# ROS 광학 프레임은 +Z 가 앞, +X 오른쪽, +Y 아래. 둘 사이는 X축 180도.
R_USDCAM_TO_OPTICAL = rpy_to_mat([math.pi, 0.0, 0.0])

# REP-103: camera_link(+X 앞, +Y 왼쪽, +Z 위) -> 광학 프레임은 rpy(-90, 0, -90)
R_CAMLINK_TO_OPTICAL = rpy_to_mat([-math.pi / 2, 0.0, -math.pi / 2])


def tf_entry(t, R, note=""):
    q = mat_to_quat(R)
    rpy = mat_to_rpy(R)
    e = {
        "xyz":       f3(t, 6),
        "quat_xyzw": f3([q[1], q[2], q[3], q[0]], 6),   # ROS 순서
        "rpy_rad":   f3(rpy, 6),
        "rpy_deg":   f3(np.degrees(rpy), 4),
        "static_transform_publisher_args": (
            f"--x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} "
            f"--roll {rpy[0]:.6f} --pitch {rpy[1]:.6f} --yaw {rpy[2]:.6f}"),
    }
    if note:
        e["note"] = note
    return e


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼
# ══════════════════════════════════════════════════════════════
# SurfaceGripper 프림에서 읽을 속성들. Isaac Sim 5.x 의 스키마 이름이다.
SURFACE_GRIPPER_ATTRS = [
    "isaac:maxGripDistance",
    "isaac:coaxialForceLimit",
    "isaac:shearForceLimit",
    "isaac:retryInterval",
    "isaac:gripThreshold",
    "isaac:status",
    "isaac:forwardAxis",
]


def measure_gripper(bbox_cache, xform_cache, stage):
    """흡착 컵의 실제 치수와 SurfaceGripper 설정값을 씬에서 읽는다"""
    out = {}

    tip = world_aabb(bbox_cache, stage, GRIPPER_TIP)
    link6 = world_xform(xform_cache, stage, ARM_LINK6)
    if tip is not None and link6 is not None:
        lo, hi = tip
        size = hi - lo
        # 흡착면은 link_6 +Z 를 향한다. 지금 자세에서 world +Z 와 같은 방향이라
        # AABB 를 그대로 써도 되지만, 혹시 몰라 link_6 로컬로 변환해 확인한다.
        inv = link6.GetInverse()
        corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                                      for y in (lo[1], hi[1])
                                      for z in (lo[2], hi[2])])
        local = np.array([list(inv.Transform(Gf.Vec3d(*c))) for c in corners])
        llo, lhi = local.min(axis=0), local.max(axis=0)
        out["gripper_tip"] = {
            "prim": GRIPPER_TIP,
            "world_size": f3(size),
            "link6_local_min": f3(llo, 5),
            "link6_local_max": f3(lhi, 5),
            # 컵은 원형이라 x/y 폭 중 큰 쪽을 지름으로 본다
            "cup_diameter_m": round(float(max(lhi[0] - llo[0], lhi[1] - llo[1])), 5),
            "suction_face_z_from_link6": round(float(lhi[2]), 5),
            "cup_depth_m": round(float(lhi[2] - llo[2]), 5),
        }

    # SurfaceGripper 프림 찾기 — 에셋 어디에 있는지 모르니 그리퍼 서브트리를 훑는다
    root = stage.GetPrimAtPath(GRIPPER)
    if root and root.IsValid():
        for prim in Usd.PrimRange(root):
            tname = prim.GetTypeName()
            if "SurfaceGripper" not in str(tname) and prim.GetName() != "SurfaceGripper":
                continue
            attrs = {}
            for a in prim.GetAttributes():
                n = a.GetName()
                if n.startswith("isaac:") or n in SURFACE_GRIPPER_ATTRS:
                    v = a.Get()
                    attrs[n] = v if isinstance(v, (int, float, str, type(None))) else str(v)
            out.setdefault("surface_gripper_prims", {})[str(prim.GetPath())] = {
                "type": str(tname),
                "attrs": attrs,
            }
    return out


# ══════════════════════════════════════════════════════════════
#  선반
# ══════════════════════════════════════════════════════════════
def measure_shelf(bbox_cache, xform_cache, stage, name, path):
    out = {"prim": path}
    origin = world_pos(xform_cache, stage, path)
    out["origin_world"] = f3(origin)

    levels = {}
    for lvl in ("Level_1", "Level_2"):
        box = world_aabb(bbox_cache, stage, f"{path}/{lvl}")
        if box is None:
            continue
        lo, hi = box
        levels[lvl] = {
            "top_z":       round(float(hi[2]), 4),
            "bottom_z":    round(float(lo[2]), 4),
            "thickness":   round(float(hi[2] - lo[2]), 4),
            "x_range":     [round(float(lo[0]), 4), round(float(hi[0]), 4)],
            "y_range":     [round(float(lo[1]), 4), round(float(hi[1]), 4)],
            "width_x":     round(float(hi[0] - lo[0]), 4),
            "depth_y":     round(float(hi[1] - lo[1]), 4),
        }
    out["levels"] = levels

    posts = {}
    for p in ("Post_FL", "Post_FR", "Post_BL", "Post_BR"):
        box = world_aabb(bbox_cache, stage, f"{path}/{p}")
        if box is None:
            continue
        lo, hi = box
        posts[p] = {
            "x_range": [round(float(lo[0]), 4), round(float(hi[0]), 4)],
            "y_range": [round(float(lo[1]), 4), round(float(hi[1]), 4)],
            "z_range": [round(float(lo[2]), 4), round(float(hi[2]), 4)],
        }
    out["posts"] = posts

    # 통로(= world y=0) 쪽이 전면이다. 선반 원점 y 의 부호로 어느 쪽이
    # 통로를 향하는지 정해진다.
    aisle_sign = -1.0 if origin[1] > 0 else 1.0
    out["aisle_side"] = "-y" if aisle_sign < 0 else "+y"

    if levels:
        any_level = next(iter(levels.values()))
        y_lo, y_hi = any_level["y_range"]
        out["plate_front_y"] = round(y_lo if aisle_sign < 0 else y_hi, 4)
        out["plate_back_y"]  = round(y_hi if aisle_sign < 0 else y_lo, 4)

    if posts:
        # 통로쪽에 가까운 기둥 두 개의 '안쪽면'(선반 속을 향하는 면)
        # 통로쪽이 -y 면 y 가 작은 기둥이, +y 면 y 가 큰 기둥이 통로에 가깝다
        front_posts = sorted(posts.items(),
                            key=lambda kv: -aisle_sign * sum(kv[1]["y_range"]) / 2.0)[:2]
        inner = [(p[1]["y_range"][1] if aisle_sign < 0 else p[1]["y_range"][0])
                 for p in front_posts]
        out["front_posts"] = [p[0] for p in front_posts]
        out["front_post_inner_y"] = round(max(inner) if aisle_sign < 0 else min(inner), 4)

        # 뒤쪽 기둥의 안쪽면 — 앞뒤 기둥 사이가 팔이 실제로 들어갈 수 있는 깊이다
        back_posts = sorted(posts.items(),
                            key=lambda kv: aisle_sign * sum(kv[1]["y_range"]) / 2.0)[:2]
        binner = [(p[1]["y_range"][0] if aisle_sign < 0 else p[1]["y_range"][1])
                  for p in back_posts]
        out["back_posts"] = [p[0] for p in back_posts]
        out["back_post_inner_y"] = round(min(binner) if aisle_sign < 0 else max(binner), 4)
        out["usable_depth_between_posts"] = round(
            abs(out["back_post_inner_y"] - out["front_post_inner_y"]), 4)
        z_tops = [p["z_range"][1] for p in posts.values()]
        z_bots = [p["z_range"][0] for p in posts.values()]
        out["post_z_range"] = [round(min(z_bots), 4), round(max(z_tops), 4)]
        out["post_bottom_gap_to_floor"] = round(min(z_bots), 4)

    # 각 단의 개구 높이 = 그 단 윗면부터 바로 윗단 아랫면까지.
    # 맨 윗단은 위가 트여 있으므로 기둥 상단까지를 적어 둔다.
    opening = {}
    ordered = sorted(levels.items(), key=lambda kv: kv[1]["top_z"])
    for i, (lvl, info) in enumerate(ordered):
        if i + 1 < len(ordered):
            above = ordered[i + 1][1]
            opening[lvl] = {
                "clear_height": round(above["bottom_z"] - info["top_z"], 4),
                "limited_by":   ordered[i + 1][0],
            }
        else:
            top = out.get("post_z_range", [0, info["top_z"]])[1]
            opening[lvl] = {
                "clear_height": None,          # 위가 트여 있다
                "limited_by":   "open (no upper level)",
                "post_top_above_level": round(top - info["top_z"], 4),
            }
    out["opening"] = opening
    return out


# ══════════════════════════════════════════════════════════════
#  매거진
# ══════════════════════════════════════════════════════════════
def measure_magazine(bbox_cache, xform_cache, stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None

    out = {"prim": path}
    m = world_xform(xform_cache, stage, path)
    out["origin_world"] = f3(m.ExtractTranslation())
    q = m.ExtractRotationQuat().GetNormalized()
    out["orient_wxyz"] = f3([q.GetReal(), *q.GetImaginary()])

    box = world_aabb(bbox_cache, stage, path)
    if box is not None:
        lo, hi = box
        out["aabb_min"] = f3(lo)
        out["aabb_max"] = f3(hi)
        out["size"] = f3(hi - lo)

    flange = world_aabb(bbox_cache, stage, f"{path}/flange_plate")
    if flange is not None:
        lo, hi = flange
        out["flange_top_z"]    = round(float(hi[2]), 4)
        out["flange_center"]   = f3((lo + hi) / 2.0)
        out["flange_size"]     = f3(hi - lo)
        # 흡착 목표점 = 플랜지 윗면 중심
        out["suction_point"]   = f3([(lo[0] + hi[0]) / 2.0,
                                     (lo[1] + hi[1]) / 2.0,
                                     hi[2]])

    base = world_aabb(bbox_cache, stage, f"{path}/base")
    if base is not None:
        out["bottom_z"] = round(float(base[0][2]), 4)

    # QR 라벨 — 메시 점을 world 로 옮겨 중심과 법선을 낸다
    for label in ("qr_label_ny", "qr_label_py"):
        lp = f"{path}/{label}"
        lprim = stage.GetPrimAtPath(lp)
        if not lprim or not lprim.IsValid():
            continue
        mesh = UsdGeom.Mesh(lprim)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        lm = world_xform(xform_cache, stage, lp)
        wp = np.array([list(lm.Transform(Gf.Vec3d(*p))) for p in pts])
        center = wp.mean(axis=0)
        n = np.cross(wp[1] - wp[0], wp[3] - wp[0])
        n = n / (np.linalg.norm(n) + 1e-12)
        side = np.linalg.norm(wp[1] - wp[0])
        out.setdefault("qr", {})[label] = {
            "center_world": f3(center),
            "normal_world": f3(n),
            "side_m":       round(float(side), 4),
        }
    return out


# ══════════════════════════════════════════════════════════════
#  본체
# ══════════════════════════════════════════════════════════════
def main():
    ctx = omni.usd.get_context()
    ctx.open_stage(WORLD_USD)
    for _ in range(60):
        simulation_app.update()

    stage = ctx.get_stage()
    stage.Load()                      # 모든 payload 를 편다
    for _ in range(60):
        simulation_app.update()

    xform_cache, bbox_cache = make_caches(stage)
    report = {"world_usd": WORLD_USD,
              "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
              "up_axis": str(UsdGeom.GetStageUpAxis(stage))}

    # ---- 선반 ----------------------------------------------------
    print("\n" + "=" * 72)
    print("  선반 실측")
    print("=" * 72)
    shelves = {}
    for name, path in SHELVES.items():
        s = measure_shelf(bbox_cache, xform_cache, stage, name, path)
        shelves[name] = s
        print(f"\n[{name}]  origin {s['origin_world']}   통로쪽 = {s['aisle_side']}")
        for lvl, info in sorted(s["levels"].items()):
            op = s["opening"][lvl]
            ch = op["clear_height"]
            ch_s = f"{ch*1000:7.1f} mm  (윗단 {op['limited_by']})" if ch is not None \
                   else f"   트임    (기둥이 상판보다 {op['post_top_above_level']*1000:.1f} mm 위)"
            print(f"   {lvl}  윗면 z = {info['top_z']*1000:7.1f} mm   "
                  f"두께 {info['thickness']*1000:4.1f} mm   "
                  f"폭(x) {info['width_x']*1000:6.1f}   깊이(y) {info['depth_y']*1000:6.1f}")
            print(f"            개구 높이 {ch_s}")
        print(f"   상판 전면 y = {s['plate_front_y']:+.4f} m   "
              f"기둥 안쪽면 y = {s['front_post_inner_y']:+.4f} m   "
              f"(전면 기둥 {', '.join(s['front_posts'])})")
        print(f"   깊이  상판 {next(iter(s['levels'].values()))['depth_y']*1000:.1f} mm   "
              f"기둥 안쪽 유효 {s['usable_depth_between_posts']*1000:.1f} mm   "
              f"상판 전면~기둥 안쪽면 {abs(s['front_post_inner_y']-s['plate_front_y'])*1000:.1f} mm")
        print(f"   기둥 z 범위 {s['post_z_range'][0]*1000:.1f} ~ "
              f"{s['post_z_range'][1]*1000:.1f} mm  "
              f"(바닥에서 {s['post_bottom_gap_to_floor']*1000:+.1f} mm 떠 있음)")
    report["shelves"] = shelves

    # ---- 매거진 --------------------------------------------------
    print("\n" + "=" * 72)
    print("  매거진 실측")
    print("=" * 72)
    magazines = {}
    for gname, gpath in MAGAZINE_GROUPS.items():
        gprim = stage.GetPrimAtPath(gpath)
        if not gprim or not gprim.IsValid():
            print(f"\n[{gname}]  없음 ({gpath})")
            continue
        shelf_name, level_name = GROUP_TO_LEVEL[gname]
        level = shelves[shelf_name]["levels"][level_name]
        shelf = shelves[shelf_name]
        aisle_sign = -1.0 if shelf["aisle_side"] == "-y" else 1.0

        print(f"\n[{gname}]  -> {shelf_name}/{level_name}  "
              f"(상판 윗면 z = {level['top_z']*1000:.1f} mm)")
        for child in gprim.GetChildren():
            info = measure_magazine(bbox_cache, xform_cache, stage, str(child.GetPath()))
            if info is None:
                continue
            key = f"{gname}/{child.GetName()}"
            # 상판과의 간격 (물리 시작 시 이만큼 떨어진다)
            drop = info["aabb_min"][2] - level["top_z"]
            info["drop_to_level"] = round(drop, 4)
            # 전면 기둥 안쪽면과의 여유
            # 매거진의 통로쪽 면이 전면 기둥 안쪽면보다 얼마나 들어가 있나.
            # 양수 = 기둥보다 안쪽, 음수 = 기둥 면보다 통로쪽으로 튀어나옴.
            # 이것만으로는 간섭 판정이 안 된다(x 가 안 겹치면 튀어나와도 무관).
            mag_front_y = info["aabb_min"][1] if aisle_sign < 0 else info["aabb_max"][1]
            info["front_post_setback"] = round(
                aisle_sign * (shelf["front_post_inner_y"] - mag_front_y), 4)

            # 실제 간섭은 AABB 끼리 x/y/z 세 축 모두 겹쳐야 성립한다.
            mn = np.array(info["aabb_min"]); mx = np.array(info["aabb_max"])
            hits = {}
            for pname, pinfo in shelf["posts"].items():
                pmn = np.array([pinfo["x_range"][0], pinfo["y_range"][0], pinfo["z_range"][0]])
                pmx = np.array([pinfo["x_range"][1], pinfo["y_range"][1], pinfo["z_range"][1]])
                # 축별 겹침량. 모두 양수면 관통, 하나라도 음수면 그 축 간격이 여유다.
                overlap = np.minimum(mx, pmx) - np.maximum(mn, pmn)
                if (overlap > 0).all():
                    hits[pname] = {"penetration": f3(overlap, 4)}
                else:
                    # 가장 크게 떨어져 있는 축 = 이 기둥과의 실질 여유
                    ax = int(np.argmin(overlap))
                    hits[pname] = {"gap_axis": "xyz"[ax],
                                   "gap": round(float(-overlap[ax]), 4)}
            info["post_interference"] = hits
            info["min_post_gap"] = round(
                min(h["gap"] for h in hits.values() if "gap" in h), 4) \
                if any("gap" in h for h in hits.values()) else 0.0
            info["post_collision"] = [k for k, h in hits.items() if "penetration" in h]
            info["height"] = round(info["size"][2], 4)
            magazines[key] = info

            print(f"   {child.GetName():24s} xyz {info['origin_world']}  "
                  f"높이 {info['height']*1000:5.1f} mm  "
                  f"플랜지 윗면 z {info['flange_top_z']*1000:6.1f} mm")
            coll = ("  기둥 관통: " + ", ".join(info["post_collision"])
                    if info["post_collision"] else "")
            print(f"   {'':24s} 상판과의 간격 {drop*1000:+6.1f} mm   "
                  f"전면 기둥 안쪽면 대비 {info['front_post_setback']*1000:+6.1f} mm   "
                  f"기둥 최소 간격 {info['min_post_gap']*1000:6.1f} mm{coll}")
            for lab, q in info.get("qr", {}).items():
                print(f"   {'':24s} {lab}: 중심 {q['center_world']}  "
                      f"법선 {q['normal_world']}  한 변 {q['side_m']*1000:.0f} mm")
    report["magazines"] = magazines

    # ---- 로봇 ----------------------------------------------------
    print("\n" + "=" * 72)
    print("  로봇 장착 오프셋  (TF 발행값의 근거)")
    print("=" * 72)
    robot = {}
    pairs = [
        ("base_link->m0609_base", CHASSIS, ARM_BASE),
        ("m0609_base->link_6",    ARM_BASE, ARM_LINK6),
        ("link_6->short_gripper", ARM_LINK6, GRIPPER),
        ("link_6->gripper_tip",   ARM_LINK6, GRIPPER_TIP),
        ("link_6->camera",        ARM_LINK6, CAMERA_ROOT),
        ("short_gripper->camera", GRIPPER, CAMERA_ROOT),
    ]
    for label, parent, child in pairs:
        rel = relative_xform(xform_cache, stage, parent, child)
        if rel is None:
            print(f"   {label:24s} (프림 없음: {child})")
            continue
        t, quat, rpy = rel
        robot[label] = {"xyz": f3(t, 6), "quat_wxyz": f3(quat, 6), "rpy_deg": f3(rpy, 4)}
        print(f"   {label:24s} xyz {f3(t, 5)}   rpy_deg {f3(rpy, 3)}")

    for label, path in (("chassis_link", CHASSIS), ("m0609/base_link", ARM_BASE),
                        ("link_6", ARM_LINK6), ("gripper_tip", GRIPPER_TIP),
                        ("rsd455", CAMERA_ROOT)):
        p = world_pos(xform_cache, stage, path)
        if p is not None:
            robot.setdefault("world_pose", {})[label] = f3(p, 5)
            print(f"   world  {label:18s} {f3(p, 4)}")

    # 카메라 프림 (UsdGeom.Camera) 과 내부 파라미터
    cams = [p for p in Usd.PrimRange(stage.GetPrimAtPath(CAMERA_ROOT))
            if p.IsA(UsdGeom.Camera)] if stage.GetPrimAtPath(CAMERA_ROOT) else []
    for cam in cams:
        c = UsdGeom.Camera(cam)
        f = c.GetFocalLengthAttr().Get()
        ha = c.GetHorizontalApertureAttr().Get()
        va = c.GetVerticalApertureAttr().Get()
        rel = relative_xform(xform_cache, stage, ARM_LINK6, str(cam.GetPath()))
        entry = {"focal_length_mm": f, "horiz_aperture_mm": ha, "vert_aperture_mm": va}
        if rel is not None:
            entry["link_6_to_camera"] = {"xyz": f3(rel[0], 6), "rpy_deg": f3(rel[2], 4)}
        robot.setdefault("cameras", {})[str(cam.GetPath())] = entry
        print(f"   camera {cam.GetName():18s} f={f} mm  aperture {ha} x {va} mm")
        if rel is not None:
            print(f"          link_6 기준 xyz {f3(rel[0], 5)}  rpy_deg {f3(rel[2], 3)}")

    report["robot"] = robot

    # ---- 흡착 그리퍼 ---------------------------------------------
    print("\n" + "=" * 72)
    print("  흡착 그리퍼 (short_gripper)")
    print("=" * 72)
    grip = measure_gripper(bbox_cache, xform_cache, stage)
    tip = grip.get("gripper_tip")
    if tip:
        print(f"   컵 지름        {tip['cup_diameter_m']*1000:6.1f} mm")
        print(f"   컵 두께(깊이)  {tip['cup_depth_m']*1000:6.1f} mm")
        print(f"   흡착면 z       link_6 로컬 +Z 로 {tip['suction_face_z_from_link6']*1000:.1f} mm")
        print(f"   tip AABB       link_6 로컬 {tip['link6_local_min']} ~ {tip['link6_local_max']}")
    else:
        print(f"   gripper_tip 프림을 못 찾았다: {GRIPPER_TIP}")
    for path, info in grip.get("surface_gripper_prims", {}).items():
        print(f"   SurfaceGripper {path}  (type={info['type']})")
        for k, v in sorted(info["attrs"].items()):
            print(f"       {k:32s} = {v}")
    if not grip.get("surface_gripper_prims"):
        print("   SurfaceGripper 프림 없음 — 런타임에 만들어 붙이는 구조다")
        print("   (10_suction_magazine.py / 14_place_test_pse.py 의 _attach_surface_gripper 참고)")
    report["gripper"] = grip

    # ---- TF 발행값 -----------------------------------------------
    # 씬에서 잰 변환을 ROS 규약(고정축 XYZ rpy)으로 바꿔 그대로 쓸 수 있게 낸다.
    print("\n" + "=" * 72)
    print("  TF 정적 변환 발행값  (frames.yaml 의 근거)")
    print("=" * 72)
    tf = {}

    rel = relative_xform(xform_cache, stage, CHASSIS, ARM_BASE)
    if rel is not None:
        t, quat, _ = rel
        tf["base_link__m0609_base"] = tf_entry(
            t, quat_to_mat(quat),
            "USD arm_mount_joint(FixedJoint)의 로컬 변환. 실측이 아니라 씬 정의값.")

    # 카메라 — 컬러 센서를 기준으로 삼는다. tool0 은 URDF 상 link_6 와 동일.
    color_cam = None
    for path in robot.get("cameras", {}):
        if "Color" in path:
            color_cam = path
            break
    if color_cam:
        rel = relative_xform(xform_cache, stage, ARM_LINK6, color_cam)
        t, quat, _ = rel
        R_usdcam = quat_to_mat(quat)
        R_optical = R_usdcam @ R_USDCAM_TO_OPTICAL
        R_camlink = R_optical @ R_CAMLINK_TO_OPTICAL.T
        tf["tool0__camera_link"] = tf_entry(
            t, R_camlink,
            f"tool0 == link_6 (URDF joint_6-tool0 가 0 오프셋). 출처 {color_cam}")
        tf["tool0__camera_color_optical_frame"] = tf_entry(
            t, R_optical,
            "REP-103 광학 프레임(+Z 앞, +X 오른쪽, +Y 아래). camera_info 의 K 가 "
            "가리키는 프레임이 이것이다.")
        tf["camera_link__camera_color_optical_frame"] = tf_entry(
            np.zeros(3), R_CAMLINK_TO_OPTICAL,
            "REP-103 고정 규약. 어느 씬에서나 같다.")

    for name, e in tf.items():
        print(f"   {name}")
        print(f"       xyz     {e['xyz']}")
        print(f"       rpy_deg {e['rpy_deg']}")
        print(f"       args    {e['static_transform_publisher_args']}")
    report["tf"] = tf

    # ---- 파생 판정 -----------------------------------------------
    print("\n" + "=" * 72)
    print("  판정")
    print("=" * 72)
    verdicts = []

    arm_base = world_pos(xform_cache, stage, ARM_BASE)
    if arm_base is not None:
        for sname, s in shelves.items():
            for lvl, info in sorted(s["levels"].items()):
                dz = info["top_z"] - arm_base[2]
                dy = abs(s["plate_front_y"] - arm_base[1])
                verdicts.append(
                    f"{sname}/{lvl}: 상판 윗면이 팔 베이스보다 {dz*1000:+7.1f} mm "
                    f"(음수=아래)")

    mag_h = [m["height"] for m in magazines.values()]
    if mag_h:
        hmax = max(mag_h)
        for sname, s in shelves.items():
            for lvl, op in sorted(s["opening"].items()):
                if op["clear_height"] is None:
                    verdicts.append(f"{sname}/{lvl}: 위가 트여 있음 — 높이 제약 없음")
                else:
                    margin = op["clear_height"] - hmax
                    verdicts.append(
                        f"{sname}/{lvl}: 개구 {op['clear_height']*1000:.1f} mm - "
                        f"매거진 {hmax*1000:.1f} mm = 여유 {margin*1000:+.1f} mm "
                        f"{'OK' if margin > 0 else '간섭!'}")

    for k, v in sorted(magazines.items(), key=lambda kv: kv[1]["min_post_gap"]):
        if v["post_collision"]:
            verdicts.append(f"{k}: 기둥 {', '.join(v['post_collision'])} 과 관통 — 씬 수정 필요")
        elif v["min_post_gap"] < 0.020:
            verdicts.append(f"{k}: 기둥과 최소 간격 {v['min_post_gap']*1000:.1f} mm "
                            f"— 접근 경로/허용오차가 이보다 작아야 한다")
    protruding = [(k, v["front_post_setback"]) for k, v in magazines.items()
                  if v["front_post_setback"] < 0]
    for k, c in sorted(protruding, key=lambda kv: kv[1]):
        verdicts.append(f"{k}: 기둥 안쪽면보다 {-c*1000:.1f} mm 통로쪽으로 나옴 "
                        f"(x 가 안 겹쳐 간섭은 없음 — 위 최소 간격 참고)")

    floaters = [(k, v["drop_to_level"]) for k, v in magazines.items()
                if abs(v["drop_to_level"]) > 0.001]
    for k, d in sorted(floaters, key=lambda kv: -abs(kv[1])):
        verdicts.append(f"{k}: 물리 시작 시 {d*1000:+.1f} mm 낙하 — GT pose 는 "
                        f"안정화 후 다시 읽어야 한다")

    verdicts.append(f"M0609 카탈로그 도달거리 {M0609_REACH_M*1000:.0f} mm "
                    f"(실측 아님 — 제조사 스펙)")

    for v in verdicts:
        print(f"   - {v}")
    report["verdicts"] = verdicts

    # ---- 기록 ----------------------------------------------------
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    try:
        import yaml
        text = yaml.safe_dump(report, allow_unicode=True, sort_keys=False)
    except Exception:
        text = json.dumps(report, ensure_ascii=False, indent=2)
    OUT_PATH.write_text(text, encoding="utf-8")
    print(f"\n기록: {OUT_PATH}")


try:
    main()
finally:
    simulation_app.close()

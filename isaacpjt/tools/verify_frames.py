"""
좌표계 완료 기준 검증 —
"매거진 prim 의 ground truth 위치를 로봇 좌표로 변환했을 때 오차 5 mm 이내"

    isaac_python isaacpjt/tools/verify_frames.py

무엇을 하나:
  1. 씬을 열고 물리를 돌려 매거진을 안정화시킨다.
     (매거진들이 상판 위 4.6~5.6 mm 떠 있게 저작돼 있어서, 물리 전 USD xform 을
      그대로 GT 로 쓰면 그 자체가 5 mm 급 오차다 — 이게 이 검증의 핵심 함정이다.)
  2. 두 경로로 같은 값을 구해 비교한다.
       (A) USD 직접 :  T(m0609_base_link <- world) * T(world <- magazine)
       (B) frames.yaml :  T(base_link <- m0609_base_link)^-1 * T(base_link <- world)
                          * T(world <- magazine)
     (B) 는 ROS 가 실제로 쓰게 될 경로다. 둘의 차이가 곧 frames.yaml 값의 오차다.
  3. 정적 변환(tool0->camera_link, tool0->tcp_suction)도 USD 와 대조한다.
  4. 파지 목표점(플랜지 윗면 중심)을 팔 베이스 좌표로 찍어 준다. 픽 파이프라인이
     실제로 필요로 하는 값이 이것이다.

물리를 돌리지 않고 저작값만 보고 싶으면:
    VERIFY_SETTLE=0 isaac_python isaacpjt/tools/verify_frames.py
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys
from pathlib import Path

import numpy as np
import omni.usd
import yaml
from pxr import Gf, Usd, UsdGeom

from isaacsim.core.api import World

sys.stdout.reconfigure(line_buffering=True)


THIS_DIR     = Path(__file__).resolve().parent
ISAACPJT_DIR = THIS_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD  = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"

SETTLE_STEPS = int(os.environ.get("VERIFY_SETTLE", "180"))

ROBOT    = "/World/Robots/nova_carter1"
CHASSIS  = f"{ROBOT}/chassis_link"          # ROS 의 base_link
ARM_BASE = f"{ROBOT}/m0609/base_link"       # ROS 의 m0609_base_link
LINK6    = f"{ROBOT}/m0609/link_6"          # ROS 의 m0609_tool0
CAMERA   = (f"{ROBOT}/m0609/short_gripper/rsd455/RSD455/"
            f"Camera_OmniVision_OV9782_Color")
TIP      = f"{ROBOT}/m0609/short_gripper/gripper_tip"

MAGAZINE_GROUPS = [
    "/World/Magazines/shelf_1_magaines/top_magazines",
    "/World/Magazines/shelf_1_magaines/bottom_magazines",
    "/World/Magazines/shelf_2_magaines/top_magazines",
    "/World/Magazines/shelf_2_magaines/bottom_magazines",
]

PASS_MM = 5.0


# ══════════════════════════════════════════════════════════════
#  변환 유틸 — 4x4 동차행렬로 통일해서 다룬다
# ══════════════════════════════════════════════════════════════
def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def homog(R, t):
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = t
    return m


def usd_world(cache, stage, path):
    """prim 의 world 변환을 4x4 로. USD 는 행벡터 규약이라 전치해서 맞춘다."""
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    m = cache.GetLocalToWorldTransform(prim)
    arr = np.array(m).T                      # 4x4, 마지막 열이 translation
    return arr


def tf_from_yaml(entry):
    return homog(quat_xyzw_to_mat(entry["quat_xyzw"]), np.array(entry["xyz"]))


def pose_err(a, b):
    """두 4x4 사이의 (위치오차 m, 각도오차 deg)"""
    dp = np.linalg.norm(a[:3, 3] - b[:3, 3])
    dR = a[:3, :3].T @ b[:3, :3]
    cos = (np.trace(dR) - 1.0) / 2.0
    ang = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
    return dp, ang


def main():
    cfg = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    st = cfg["static_transforms"]

    ctx = omni.usd.get_context()
    ctx.open_stage(WORLD_USD)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()
    stage.Load()
    for _ in range(60):
        simulation_app.update()

    print("=" * 72)
    print("  좌표계 검증")
    print("=" * 72)
    print(f"   씬      {WORLD_USD}")
    print(f"   설정    {FRAMES_YAML}")

    # 물리 돌리기 전 저작값을 떠 둔다 — 안정화로 얼마나 움직이는지 보려고
    pre_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    authored = {}
    for gpath in MAGAZINE_GROUPS:
        gprim = stage.GetPrimAtPath(gpath)
        if gprim and gprim.IsValid():
            for child in gprim.GetChildren():
                mp = str(child.GetPath())
                m = usd_world(pre_cache, stage, mp)
                if m is not None:
                    authored[mp] = m[:3, 3].copy()

    if SETTLE_STEPS > 0:
        world = World(stage_units_in_meters=1.0)
        world.reset()
        for _ in range(SETTLE_STEPS):
            world.step(render=False)
        print(f"   물리    {SETTLE_STEPS} 스텝 안정화 후 측정")
    else:
        print("   물리    돌리지 않음 (VERIFY_SETTLE=0) — USD 저작값 그대로")
    print()

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    cache.Clear()

    T_w_base = usd_world(cache, stage, CHASSIS)      # world <- base_link
    T_w_arm  = usd_world(cache, stage, ARM_BASE)     # world <- m0609_base_link
    if T_w_base is None or T_w_arm is None:
        sys.exit("로봇 프림을 못 찾았다")

    # ── 1. 정적 변환 자체를 USD 와 대조 ────────────────────────────
    print("-" * 72)
    print("  정적 변환 대조 (frames.yaml vs USD)")
    print("-" * 72)
    checks = [
        ("base_link__m0609_base_link", CHASSIS, ARM_BASE),
        ("m0609_tool0__camera_link",   LINK6,   None),   # 회전 규약이 달라 위치만 본다
        ("m0609_tool0__tcp_suction",   LINK6,   None),
    ]
    worst_static = 0.0
    for key, parent, child in checks:
        entry = st[key]
        T_yaml = tf_from_yaml(entry)
        if child is not None:
            T_p = usd_world(cache, stage, parent)
            T_c = usd_world(cache, stage, child)
            T_usd = np.linalg.inv(T_p) @ T_c
            dp, da = pose_err(T_usd, T_yaml)
            print(f"   {key:34s} 위치차 {dp*1000:6.3f} mm   각도차 {da:6.3f} deg")
        else:
            # camera_link / tcp_suction 은 USD 에 같은 이름의 프림이 없다.
            # 원점 위치만 각각의 근거 프림과 맞춰 본다.
            src = CAMERA if "camera" in key else TIP
            T_p = usd_world(cache, stage, parent)
            T_s = usd_world(cache, stage, src)
            t_usd = (np.linalg.inv(T_p) @ T_s)[:3, 3]
            if "tcp_suction" in key:
                # gripper_tip 은 컵의 중심이라 흡착면(바깥면)보다 컵두께/2 만큼 안쪽
                half = cfg["suction_gripper"]["cup_depth_m"] / 2.0
                t_usd = t_usd + np.array([0.0, 0.0, half])
            dp = np.linalg.norm(t_usd - T_yaml[:3, 3])
            print(f"   {key:34s} 위치차 {dp*1000:6.3f} mm   "
                  f"(USD {np.round(t_usd, 5).tolist()})")
            da = 0.0
        worst_static = max(worst_static, dp * 1000)

    # ── 2. 매거진 GT -> 로봇 좌표 ──────────────────────────────────
    print()
    print("-" * 72)
    print("  매거진 GT -> m0609_base_link 좌표   (완료 기준 5 mm)")
    print("-" * 72)

    # frames.yaml 이 주장하는 base_link <- m0609_base_link
    T_base_arm_yaml = tf_from_yaml(st["base_link__m0609_base_link"])
    # 그 주장대로라면 world <- m0609_base_link 는 이렇게 된다
    T_w_arm_yaml = T_w_base @ T_base_arm_yaml

    dp, da = pose_err(T_w_arm, T_w_arm_yaml)
    print(f"   base_link  world {np.round(T_w_base[:3, 3], 4).tolist()}")
    print(f"   팔 베이스  world {np.round(T_w_arm[:3, 3], 4).tolist()}")
    print(f"   팔 베이스 world pose:  USD 직접 vs frames.yaml 경유 "
          f"-> {dp*1000:.4f} mm, {da:.4f} deg")
    print()
    print("   주의 — 로봇은 지금 도킹존에 서 있고, 관절 구동 목표를 안 준 채로")
    print("   물리를 돌려서 팔이 중력에 주저앉은 상태다. 아래 '파지점' 좌표가")
    print("   큰 건 그 때문이고, 검증에는 문제가 없다. 오히려 관절값이 아무래도")
    print("   상관없이 체인이 맞는다는 걸 보여준다.")
    print()

    worst = 0.0
    n = 0
    for gpath in MAGAZINE_GROUPS:
        gprim = stage.GetPrimAtPath(gpath)
        if not gprim or not gprim.IsValid():
            continue
        label = "/".join(gpath.split("/")[-2:])
        print(f"   [{label}]")
        for child in gprim.GetChildren():
            mp = str(child.GetPath())
            T_w_mag = usd_world(cache, stage, mp)
            if T_w_mag is None:
                continue
            # (A) USD 직접
            gt_A = np.linalg.inv(T_w_arm) @ T_w_mag
            # (B) frames.yaml 경유
            gt_B = np.linalg.inv(T_w_arm_yaml) @ T_w_mag
            dp, da = pose_err(gt_A, gt_B)
            worst = max(worst, dp * 1000)
            n += 1

            # 파지 목표점 = 플랜지 윗면 중심 (매거진 로컬 -> 팔 베이스)
            fl = stage.GetPrimAtPath(f"{mp}/flange_plate")
            grasp = ""
            if fl and fl.IsValid():
                bb = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                ).ComputeWorldBound(fl).ComputeAlignedRange()
                top = np.array([(bb.GetMin()[0] + bb.GetMax()[0]) / 2.0,
                                (bb.GetMin()[1] + bb.GetMax()[1]) / 2.0,
                                bb.GetMax()[2], 1.0])
                g = (np.linalg.inv(T_w_arm) @ top)[:3]
                grasp = f"   파지점(팔베이스) {np.round(g, 4).tolist()}"

            flag = "OK " if dp * 1000 <= PASS_MM else "NG "
            print(f"      {flag}{child.GetName():22s} 오차 {dp*1000:7.4f} mm / "
                  f"{da:6.4f} deg{grasp}")

    # ── 3. 안정화로 매거진이 얼마나 움직였나 ──────────────────────
    if SETTLE_STEPS > 0 and authored:
        print()
        print("-" * 72)
        print("  물리 안정화로 매거진이 움직인 거리")
        print("-" * 72)
        moves = []
        for gpath in MAGAZINE_GROUPS:
            gprim = stage.GetPrimAtPath(gpath)
            if not gprim or not gprim.IsValid():
                continue
            for child in gprim.GetChildren():
                mp = str(child.GetPath())
                if mp not in authored:
                    continue
                now = usd_world(cache, stage, mp)
                if now is None:
                    continue
                d = now[:3, 3] - authored[mp]
                moves.append((mp, d))
        if moves:
            dz = [m[1][2] for m in moves]
            dxy = [np.linalg.norm(m[1][:2]) for m in moves]
            print(f"   z 변화   최소 {min(dz)*1000:+.2f} mm   최대 {max(dz)*1000:+.2f} mm")
            print(f"   xy 변화  최대 {max(dxy)*1000:.2f} mm")
            print()
            print("   저작값(USD xform)을 GT 로 쓰면 이만큼이 그대로 오차가 된다.")
            print("   완료 기준이 5 mm 라 무시할 수 없다 — GT 는 반드시 안정화 뒤에 읽어라.")

    print()
    print("=" * 72)
    print(f"   정적 변환 최대 오차   {worst_static:.4f} mm")
    print(f"   매거진 GT 최대 오차   {worst:.4f} mm   (대상 {n} 개)")
    ok = worst <= PASS_MM and worst_static <= PASS_MM
    print(f"   완료 기준 {PASS_MM:.0f} mm  ->  {'통과' if ok else '미달'}")
    print("=" * 72)
    if SETTLE_STEPS > 0:
        print()
        print("   참고 — 여기서 재는 건 frames.yaml 값이 씬과 맞는지다.")
        print("   실제 파이프라인 오차에는 QR 자세추정 오차가 더 붙는다.")
        print("   그건 인식 단계를 붙인 뒤에 따로 재야 한다.")
    return 0 if ok else 1


code = 1
try:
    code = main()
finally:
    simulation_app.close()
sys.exit(code)

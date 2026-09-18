"""
팔을 한 자세로 고정하고 베이스가 선반을 따라 지나가며 QR 을 훑는다 — 검증.

    SEED=shelf_1_top_close_centered isaac_python isaacpjt/tools/sweep_scan.py

설계 의도:
  한 층당 관측 자세 하나만 티칭하고, 훑는 건 AMR 주행으로 한다.
  2층 자세로 한 방향 훑고, 1층 자세로 바꿔 반대로 훑는다.
  이러면 팔 도달거리(0.9 m)가 x 커버리지를 제한하지 않는다 — 베이스가 움직이니까.

이 스크립트가 확인하는 것:
  베이스 x 를 훑으면서 매 지점 손목 카메라를 렌더하고 cv2 로 디코드한다.
    - 매거진마다 '디코드되는 x 구간'이 얼마나 되나 (주행 속도/샘플 주기의 근거)
    - 한 번 지나가는 동안 그 층 4 개가 전부 읽히나
    - 주황/파랑이 QR 평면 y 와 높이 z 가 달라도 같은 자세로 둘 다 읽히나
      (주황 y 1.9295 z 0.675 / 파랑 y 1.8995 z 0.635 — 30 mm, 40 mm 차이)

결과: out/sweep_scan.yaml, 프레임은 out/sweep_frames/
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys
from pathlib import Path

import numpy as np
import omni.usd
import yaml
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

sys.stdout.reconfigure(line_buffering=True)

THIS_DIR     = Path(__file__).resolve().parent
ISAACPJT_DIR = THIS_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD   = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
MEASURED    = THIS_DIR / "out/layout_measured.yaml"
TAUGHT      = THIS_DIR / "out/taught_poses.yaml"
OUT_YAML    = THIS_DIR / "out/sweep_scan.yaml"
FRAME_DIR   = THIS_DIR / "out/sweep_frames"

ROBOT    = "/World/Robots/nova_carter1"
CHASSIS  = f"{ROBOT}/chassis_link"
ARM      = f"{ROBOT}/m0609"
LINK6    = f"{ARM}/link_6"
CAMERA   = f"{ARM}/short_gripper/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

SEED_NAME = os.environ.get("SEED", "shelf_1_top_close_centered")
SHELF     = os.environ.get("SHELF", "shelf_1")
LEVEL     = os.environ.get("LEVEL", "top")
# 베이스 y / yaw 를 덮어쓸 수 있게 해 둔다. 기본은 시드 자세를 뜬 자리.
#
# shelf_2 는 통로가 +y 라 팔이 반대로 뻗어야 한다. 그런데 베이스를 180도
# 돌리면 팔 전체가 같이 돌아가므로 관절값은 그대로 쓸 수 있다 — 회전은
# 강체 변환이라 관절 공간이 안 바뀐다. BASE_YAW=180 이 그걸 시험하는 것이다.
BASE_Y    = os.environ.get("BASE_Y")          # 비우면 시드 자세의 값
BASE_YAW  = float(os.environ.get("BASE_YAW", "0"))
X_START   = float(os.environ.get("X_START", "-7.00"))
X_END     = float(os.environ.get("X_END",   "-4.30"))
X_STEP    = float(os.environ.get("X_STEP",  "0.10"))
SAVE_ALL  = os.environ.get("SAVE_ALL", "0") == "1"

CARTER_Z = 0.07963398335074101
RESOLUTION = (1280, 720)
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e5, 1e4, 2700.0


def yaw_quat_wxyz(deg):
    h = np.radians(deg) / 2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def world_pos(path):
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    t = m.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])


def world_rot(path):
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    return np.array(m).T[:3, :3].copy()


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w]); x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def configure_drives():
    stage = omni.usd.get_context().get_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ARM)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        d = UsdPhysics.DriveAPI.Get(prim, "angular") or \
            UsdPhysics.DriveAPI.Apply(prim, "angular")
        d.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
        d.CreateDampingAttr().Set(DRIVE_DAMPING)
        d.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)


def main():
    import cv2

    cfg    = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    meas   = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
    taught = yaml.safe_load(TAUGHT.read_text(encoding="utf-8"))
    if SEED_NAME not in taught:
        sys.exit(f"'{SEED_NAME}' 가 없다. 있는 것: {list(taught)}")

    joints = np.array(taught[SEED_NAME]["joints_deg"], dtype=float)
    base_y = float(BASE_Y) if BASE_Y else float(taught[SEED_NAME]["base_link_world"][1])
    K = cfg["wrist_camera"]["camera_info_observed"]["k"]
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    W, H = RESOLUTION
    qr_side = cfg["layout"]["magazine_1_orange"]["qr_side_m"]
    R_l6_cam = quat_xyzw_to_mat(
        cfg["static_transforms"]["m0609_tool0__camera_link"]["quat_xyzw"])
    R_cam_opt = quat_xyzw_to_mat(
        cfg["static_transforms"]["camera_link__camera_color_optical_frame"]["quat_xyzw"])
    # 매거진 종류별 기대 코드 — 에셋 doc 기준 (주황 '1', 파랑 '2')
    def expected_code(name):
        return "1" if "orange" in name else "2"
    aisle_sign = -1.0 if base_y > 0 else 1.0

    targets = []
    for key, m in sorted(meas["magazines"].items()):
        s, lvl, name = key.split("/")
        if s != SHELF or lvl != LEVEL:
            continue
        for lab, q in m["qr"].items():
            if np.sign(q["normal_world"][1]) != aisle_sign:
                continue
            targets.append((name, np.array(q["center_world"])))
    targets.sort(key=lambda t: t[1][0])

    ctx = omni.usd.get_context()
    ctx.open_stage(WORLD_USD)
    for _ in range(60):
        simulation_app.update()
    ctx.get_stage().Load()
    for _ in range(60):
        simulation_app.update()

    configure_drives()
    world = World(stage_units_in_meters=1.0)
    robot = world.scene.add(SingleArticulation(prim_path=CHASSIS, name="carter"))
    world.reset()
    idx = np.array([robot.get_dof_index(j) for j in ARM_JOINTS])

    q = robot.get_joint_positions()
    q[idx] = np.deg2rad(joints)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    robot.apply_action(ArticulationAction(joint_positions=np.deg2rad(joints),
                                          joint_indices=idx))
    for _ in range(60):
        world.step(render=False)

    import omni.replicator.core as rep
    rp = rep.create.render_product(CAMERA, RESOLUTION)
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    detector = cv2.QRCodeDetector()
    FRAME_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 84)
    print(f"  주행 스캔 검증 — 자세 '{SEED_NAME}' 고정, 베이스 x 훑기")
    print("=" * 84)
    print(f"   선반/층   {SHELF}/{LEVEL}   통로 {'-y' if aisle_sign < 0 else '+y'}")
    print(f"   관절 고정 {joints.round(2).tolist()}")
    print(f"   베이스 y  {base_y:.4f}  yaw {BASE_YAW:.0f}도   "
          f"x {X_START} -> {X_END} 간격 {X_STEP}")
    print(f"   표적      " + ", ".join(f"{n}(x={c[0]:.3f})" for n, c in targets))
    print()
    print(f"   {'base_x':>8} {'카메라x':>9} {'가장가까운QR':>20} {'거리':>7} {'px':>7} {'디코드':>8}")
    print("   " + "-" * 70)

    rows = []
    xs = np.round(np.arange(X_START, X_END + 1e-9, X_STEP), 3)
    for bx in xs:
        robot.set_world_pose(position=np.array([float(bx), base_y, CARTER_Z]),
                             orientation=yaw_quat_wxyz(BASE_YAW))
        try:
            robot.set_velocities(np.zeros(6))
        except Exception:
            pass
        for _ in range(70):
            world.step(render=True)

        cam = world_pos(CAMERA)
        nearest = min(targets, key=lambda t: np.linalg.norm(t[1] - cam))
        dist = float(np.linalg.norm(nearest[1] - cam))
        px = fx * qr_side / dist

        rgb = annot.get_data()
        img = np.asarray(rgb)[:, :, :3][:, :, ::-1].copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 화면에 QR 이 여러 개 있을 수 있다. 하나만 돌려주는 detectAndDecode 를
        # 쓰면 "가장 가까운 매거진" 과 실제로 읽힌 코드가 어긋난다 — 실제로
        # 파랑 매거진 자리에서 주황 코드('1')가 잡혔다. 전부 받아서 위치로 귀속한다.
        try:
            okm, infos, pts, _ = detector.detectAndDecodeMulti(gray)
        except Exception:
            okm, infos, pts = False, [], None
        found = []
        if okm and pts is not None:
            for text_i, quad in zip(infos, np.asarray(pts)):
                if not text_i:
                    continue
                found.append((text_i, np.asarray(quad).reshape(-1, 2).mean(axis=0)))

        # 기대 QR 을 화면에 투영해서 검출된 것과 짝짓는다
        R_opt = world_rot(LINK6) @ R_l6_cam @ R_cam_opt
        assigned = {}
        for name, c in targets:
            v = c - cam
            p3 = R_opt.T @ v
            if p3[2] <= 0.01:
                continue
            u = fx * p3[0] / p3[2] + cx
            w_ = fy * p3[1] / p3[2] + cy
            if not (0 <= u < W and 0 <= w_ < H):
                continue
            if not found:
                continue
            t_i, ctr = min(found, key=lambda f: np.hypot(*(f[1] - [u, w_])))
            if np.hypot(*(ctr - [u, w_])) <= 80:      # 같은 라벨로 볼 만한 거리
                assigned[name] = t_i

        if assigned or SAVE_ALL:
            cv2.imwrite(str(FRAME_DIR / f"{SHELF}_{LEVEL}_x{bx:+.2f}.png"), img)

        shown = ", ".join(f"{n.replace('magazine_','')}={t!r}"
                          for n, t in sorted(assigned.items())) or "X"
        print(f"   {bx:8.2f} {cam[0]:9.3f} {nearest[0][-12:]:>20} {dist:7.3f} "
              f"{px:7.1f}  {shown}")
        rows.append(dict(base_x=float(bx), camera_x=round(float(cam[0]), 4),
                         nearest=nearest[0], dist_m=round(dist, 4),
                         qr_px=round(float(px), 1), decoded=assigned))

    # 매거진별 디코드 구간
    print()
    print("=" * 84)
    print("   매거진별 디코드 구간 (베이스 x)")
    hit = {}
    for r in rows:
        for name, text in r["decoded"].items():
            hit.setdefault(name, []).append((r["base_x"], text))
    for name, _c in targets:
        exp = expected_code(name)
        if name not in hit:
            print(f"      {name:22s} 디코드 안 됨")
            continue
        v = [x for x, _ in hit[name]]
        codes = sorted({t for _, t in hit[name]})
        bad = [c for c in codes if c != exp]
        print(f"      {name:22s} x {min(v):+.2f} ~ {max(v):+.2f}  "
              f"(폭 {max(v)-min(v)+X_STEP:.2f} m, {len(v)} 지점)  "
              f"코드 {codes}  기대 {exp!r}" + ("  << 불일치" if bad else ""))
    print()
    ok_n = sum(1 for n, _ in targets
               if n in hit and all(t == expected_code(n) for _, t in hit[n]))
    print(f"   {ok_n} / {len(targets)} 개가 기대한 코드로 정확히 읽혔다")

    OUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    prev = yaml.safe_load(OUT_YAML.read_text(encoding="utf-8")) if OUT_YAML.exists() else {}
    prev[f"{SHELF}/{LEVEL}"] = dict(seed=SEED_NAME, base_y=base_y, base_yaw_deg=BASE_YAW,
                                    joints_deg=[round(float(v), 4) for v in joints],
                                    rows=rows)
    OUT_YAML.write_text(yaml.safe_dump(prev, allow_unicode=True, sort_keys=False),
                        encoding="utf-8")
    print(f"   기록 {OUT_YAML}   프레임 {FRAME_DIR}")


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
finally:
    sys.stdout.flush()
    os._exit(0)

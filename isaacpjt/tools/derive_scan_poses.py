"""
티칭한 자세 하나에서 같은 층의 나머지 자세를 IK 로 뽑아낸다.

    isaac_python isaacpjt/tools/derive_scan_poses.py
    SEED=shelf_1_top_close_centered STOP=shelf_1_a \\
        isaac_python isaacpjt/tools/derive_scan_poses.py

왜 이렇게 하나:
  같은 층의 매거진들은 x 만 다르다(간격 0.58~0.62 m). 자세가 본질적으로 다른
  게 아니라 카메라를 x 로 옮긴 것뿐이다. 그러니 층마다 하나만 티칭하고 나머지는
  풀면 된다.

  앞서 find_scan_poses.py 의 IK 가 팔꿈치 뒤집힌 해를 준 건 warm start 를
  0 으로 줬기 때문이다. 티칭한 관절값을 시드로 주면 그 근처 해로 수렴한다.
  아래 '시드와의 거리'가 그걸 확인하는 숫자다 — 크면 다른 형상으로 튄 것이다.

검증:
  푼 자세를 실제로 적용해서 손목 카메라를 렌더하고 cv2 로 디코드까지 해 본다.
  풀렸다(SOLVED)는 것과 실제로 QR 이 읽힌다는 건 다른 얘기다.

결과: out/derived_poses.yaml, 프레임은 out/derived_frames/
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
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

sys.stdout.reconfigure(line_buffering=True)

THIS_DIR     = Path(__file__).resolve().parent
ISAACPJT_DIR = THIS_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD    = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
URDF_PATH    = str(ISAACPJT_DIR / "M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESC_PATH    = str(ISAACPJT_DIR / "M0609/descriptor/m0609_description.yaml")
FRAMES_YAML  = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
MEASURED     = THIS_DIR / "out/layout_measured.yaml"
TAUGHT       = THIS_DIR / "out/taught_poses.yaml"
OUT_YAML     = THIS_DIR / "out/derived_poses.yaml"
FRAME_DIR    = THIS_DIR / "out/derived_frames"

ROBOT    = "/World/Robots/nova_carter1"
CHASSIS  = f"{ROBOT}/chassis_link"
ARM      = f"{ROBOT}/m0609"
ARM_BASE = f"{ARM}/base_link"
LINK6    = f"{ARM}/link_6"
CAMERA   = f"{ARM}/short_gripper/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
EE_FRAME = "link_6"

SEED_NAME = os.environ.get("SEED", "shelf_1_top_close_centered")
STOP_NAME = os.environ.get("STOP", "shelf_1_a")
LEVEL     = os.environ.get("LEVEL", "top")      # top | bottom
STANDOFF  = float(os.environ.get("STANDOFF", "0.30"))

CARTER_Z = 0.07963398335074101
RESOLUTION = (1280, 720)
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e5, 1e4, 2700.0


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w]); x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def quat_wxyz_to_mat(q):
    return quat_xyzw_to_mat([q[1], q[2], q[3], q[0]])


def mat_to_quat_wxyz(m):
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t+1.0)*2
        q = [0.25*s, (m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s]
    elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:
        s = np.sqrt(1.0+m[0,0]-m[1,1]-m[2,2])*2
        q = [(m[2,1]-m[1,2])/s, 0.25*s, (m[0,1]+m[1,0])/s, (m[0,2]+m[2,0])/s]
    elif m[1,1] > m[2,2]:
        s = np.sqrt(1.0+m[1,1]-m[0,0]-m[2,2])*2
        q = [(m[0,2]-m[2,0])/s, (m[0,1]+m[1,0])/s, 0.25*s, (m[1,2]+m[2,1])/s]
    else:
        s = np.sqrt(1.0+m[2,2]-m[0,0]-m[1,1])*2
        q = [(m[1,0]-m[0,1])/s, (m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s, 0.25*s]
    q = np.array(q); return q/np.linalg.norm(q)


def yaw_quat_wxyz(deg):
    h = np.radians(deg)/2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def world_mat(path):
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    a = np.array(m).T
    return a[:3, 3].copy(), a[:3, :3].copy()


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
        sys.exit(f"시드 자세 '{SEED_NAME}' 가 없다. 있는 것: {list(taught)}")
    seed = taught[SEED_NAME]
    seed_joints_deg = np.array(seed["joints_deg"], dtype=float)

    stops = cfg["scan_stops"]
    shelf = "_".join(STOP_NAME.split("_")[:2])
    letter = STOP_NAME.split("_")[-1]
    base_y = stops[shelf]["base_y"]
    base_x = stops[shelf]["stops"][letter]["x"]
    aisle_sign = -1.0 if stops[shelf]["aisle_side"] == "-y" else 1.0

    fx = cfg["wrist_camera"]["camera_info_observed"]["k"][0]
    qr_side = cfg["layout"]["magazine_1_orange"]["qr_side_m"]
    decode_min_px = cfg["qr"]["thresholds_px"]["decode_min"]

    R_l6_cam = quat_xyzw_to_mat(
        cfg["static_transforms"]["m0609_tool0__camera_link"]["quat_xyzw"])
    p_l6_cam = np.array(cfg["static_transforms"]["m0609_tool0__camera_link"]["xyz"])
    R_cam_opt = quat_xyzw_to_mat(
        cfg["static_transforms"]["camera_link__camera_color_optical_frame"]["quat_xyzw"])

    # 표적 — 이 선반 이 층의, 통로를 향한 QR 라벨
    targets = []
    for key, m in sorted(meas["magazines"].items()):
        s, lvl, name = key.split("/")
        if s != shelf or lvl != LEVEL:
            continue
        for lab, q in m["qr"].items():
            n = np.array(q["normal_world"])
            if np.sign(n[1]) != aisle_sign:
                continue
            targets.append((name, lab, np.array(q["center_world"]), n))

    # ── 씬 ────────────────────────────────────────────────────────
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

    def goto(joints_deg, steps=120, render=False):
        q = robot.get_joint_positions()
        q[idx] = np.deg2rad(joints_deg)
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        robot.apply_action(ArticulationAction(
            joint_positions=np.deg2rad(joints_deg), joint_indices=idx))
        for _ in range(steps):
            world.step(render=render)

    # 시드 자세로 세운 뒤 베이스를 정차 지점으로
    goto(seed_joints_deg, steps=30)
    robot.set_world_pose(position=np.array([base_x, base_y, CARTER_Z]),
                         orientation=yaw_quat_wxyz(0.0))
    try:
        robot.set_velocities(np.zeros(6))
    except Exception:
        pass
    for _ in range(120):
        world.step(render=False)

    arm_pos, _ = world_mat(ARM_BASE)
    _, R_l6_seed = world_mat(LINK6)
    # 시드 자세의 광학 프레임 회전 — 모든 표적에 이 방향을 그대로 쓴다.
    # 같은 선반의 QR 은 법선이 전부 같으므로 방향을 바꿀 이유가 없다.
    R_opt_seed = R_l6_seed @ R_l6_cam @ R_cam_opt

    lula = LulaKinematicsSolver(robot_description_path=DESC_PATH, urdf_path=URDF_PATH)
    lula.set_robot_base_pose(robot_position=arm_pos,
                             robot_orientation=np.array([1.0, 0.0, 0.0, 0.0]))

    import omni.replicator.core as rep
    rp = rep.create.render_product(CAMERA, RESOLUTION)
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    detector = cv2.QRCodeDetector()
    FRAME_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 84)
    print(f"  자세 파생 — 시드 '{SEED_NAME}' -> {shelf}/{LEVEL} @ {STOP_NAME}")
    print("=" * 84)
    print(f"   정차       base ({base_x}, {base_y})   팔 베이스 {arm_pos.round(3).tolist()}")
    print(f"   시드 관절  {seed_joints_deg.round(2).tolist()}")
    print(f"   표적거리   {STANDOFF:.2f} m   디코드 기준 {decode_min_px} px")
    print()
    print(f"   {'매거진':22s} {'IK':>6} {'시드와의 거리':>12} {'거리':>7} "
          f"{'px':>7} {'디코드':>8}")
    print("   " + "-" * 74)

    out = {}
    warm_starts = [seed_joints_deg]
    for name, lab, c, n in targets:
        cam_want = c + n * STANDOFF
        # 광학 프레임 -> link_6
        R_l6 = R_opt_seed @ (R_l6_cam @ R_cam_opt).T
        p_l6 = cam_want - R_l6 @ p_l6_cam
        # Lula IK 는 warm start 에서 가장 가까운 해로 수렴한다. 시드 하나만 쓰면
        # 반대쪽 표적에서 팔꿈치가 뒤집힌 해로 빠진다(실측 226도 차이). 그래서
        # 시드 + 지금까지 채택한 해들을 전부 warm start 로 넣어 보고, 시드에서
        # 가장 덜 벗어난 해를 고른다.
        q_target = mat_to_quat_wxyz(R_l6)
        best = None
        for ws_deg in warm_starts:
            sol, ok = lula.compute_inverse_kinematics(
                frame_name=EE_FRAME,
                target_position=p_l6,
                target_orientation=q_target,
                warm_start=np.deg2rad(ws_deg),
            )
            if not ok:
                continue
            d = np.degrees(np.asarray(sol, dtype=float))
            drift_ = float(np.max(np.abs(d - seed_joints_deg)))
            if best is None or drift_ < best[1]:
                best = (d, drift_)
        if best is None:
            print(f"   {name:22s} {'FAIL':>6}")
            out[name] = dict(solved=False)
            continue

        deg, drift = best
        warm_starts.append(deg)

        goto(deg, steps=150, render=True)
        cam_p, _ = world_mat(CAMERA)
        dist = float(np.linalg.norm(c - cam_p))
        px = fx * qr_side / dist

        rgb = annot.get_data()
        img = np.asarray(rgb)[:, :, :3][:, :, ::-1].copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        text, pts, _ = detector.detectAndDecode(gray)
        cv2.imwrite(str(FRAME_DIR / f"{STOP_NAME}_{LEVEL}_{name}.png"), img)

        print(f"   {name:22s} {'OK':>6} {drift:11.1f}도 {dist:7.3f} {px:7.1f} "
              f"{repr(text) if text else 'X':>8}")
        out[name] = dict(solved=True, joints_deg=[round(float(v), 4) for v in deg],
                         max_drift_from_seed_deg=round(drift, 3),
                         camera_world=[round(float(v), 5) for v in cam_p],
                         qr=f"{shelf}/{LEVEL}/{name}:{lab}",
                         distance_m=round(dist, 4), qr_px=round(float(px), 1),
                         decoded=text or "")

    OUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    prev = yaml.safe_load(OUT_YAML.read_text(encoding="utf-8")) if OUT_YAML.exists() else {}
    prev[f"{STOP_NAME}/{LEVEL}"] = dict(seed=SEED_NAME, stop=STOP_NAME, level=LEVEL,
                                        standoff_m=STANDOFF, poses=out)
    OUT_YAML.write_text(yaml.safe_dump(prev, allow_unicode=True, sort_keys=False),
                        encoding="utf-8")

    good = [v for v in out.values() if v.get("decoded")]
    print()
    print("=" * 84)
    print(f"   디코드 성공 {len(good)} / {len(targets)}")
    print(f"   기록 {OUT_YAML}   프레임 {FRAME_DIR}")


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
finally:
    sys.stdout.flush()
    os._exit(0)

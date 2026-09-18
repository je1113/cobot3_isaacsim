"""
QR 이 실제로 몇 미터까지 디코드되는지 렌더해서 확인한다.

    isaac_python isaacpjt/tools/qr_decode_range.py
    QR_POSE=shelf_1_top_scan isaac_python isaacpjt/tools/qr_decode_range.py

왜 필요한가:
  "화면에서 몇 픽셀" 만으로는 디코드 여부를 못 정한다. 이 씬의 QR 은
  qr_1.png 가 464x464 이고 29 박스 x 16 px = version 1 (21 모듈 + 여백 4 모듈)
  이다. 즉 라벨 50 mm 중 데이터 영역은 21/29 * 50 = 36.2 mm 뿐이고,
  화면상 라벨 크기가 N px 이면 모듈당 해상도는 (N * 21/29) / 21 = N/29 px 다.
  모듈당 2~3 px 이 디코드 하한이라고들 하는데, 실제로 그런지 렌더해서 본다.

방법:
  taught_poses.yaml 의 자세를 그대로 쓰고 베이스만 통로 방향(y)으로 밀면서
  거리를 바꾼다. 팔 자세를 안 건드리니 시야각/구도는 그대로고 거리만 변한다.
  매 지점에서 손목 카메라를 렌더해 cv2.QRCodeDetector 로 디코드를 시도한다.

결과:
  콘솔 표 + out/qr_frames/*.png (눈으로 확인용) + out/qr_decode_range.yaml
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys
from pathlib import Path

import numpy as np
import omni.usd
import yaml
from pxr import UsdGeom

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation

sys.stdout.reconfigure(line_buffering=True)

THIS_DIR     = Path(__file__).resolve().parent
ISAACPJT_DIR = THIS_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD    = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
FRAMES_YAML  = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
TAUGHT_YAML  = THIS_DIR / "out/taught_poses.yaml"
FRAME_DIR    = THIS_DIR / "out/qr_frames"
OUT_YAML     = THIS_DIR / "out/qr_decode_range.yaml"

ROBOT    = "/World/Robots/nova_carter1"
CHASSIS  = f"{ROBOT}/chassis_link"
ARM      = f"{ROBOT}/m0609"
CAMERA   = (f"{ARM}/short_gripper/rsd455/RSD455/"
            f"Camera_OmniVision_OV9782_Color")
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

POSE_NAME = os.environ.get("QR_POSE", "shelf_1_top_scan")

# 베이스를 선반 쪽으로 이 값들만큼 당긴다 (0 = 티칭한 자리, 클수록 가까이).
Y_OFFSETS = [0.00, 0.15, 0.30, 0.45, 0.55, 0.65, 0.72, 0.78, 0.84, 0.90]

# QR 규격 — qr_1.png 에서 읽은 값
QR_MODULES      = 21      # version 1
QR_QUIET        = 4       # 여백 (한쪽)
QR_LABEL_SIDE_M = 0.050

DRIVE_STIFFNESS = 1e5
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 2700.0

RESOLUTION = (1280, 720)


def yaw_quat_wxyz(deg):
    h = np.radians(deg) / 2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def configure_drives():
    from pxr import Usd, UsdPhysics
    stage = omni.usd.get_context().get_stage()
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ARM)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular") or \
                UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(DRIVE_DAMPING)
        drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)


def world_pos(path):
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    t = m.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])


def main():
    import cv2

    if not TAUGHT_YAML.exists():
        sys.exit(f"{TAUGHT_YAML} 가 없다. capture_pose.py 로 자세를 먼저 떠라.")
    taught = yaml.safe_load(TAUGHT_YAML.read_text(encoding="utf-8"))
    if POSE_NAME not in taught:
        sys.exit(f"'{POSE_NAME}' 자세가 없다. 있는 것: {list(taught)}")
    pose = taught[POSE_NAME]
    joints_deg = pose["joints_deg"]
    base_xy = pose["base_link_world"][:2]

    cfg = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    fx = cfg["wrist_camera"]["camera_info_observed"]["k"][0]

    # 이 자세에서 보인 QR 의 world 좌표 (거리 계산 기준)
    if not pose["qr_in_view"]:
        sys.exit(f"'{POSE_NAME}' 에는 보이는 QR 이 없다. 다른 자세를 골라라.")
    meas = yaml.safe_load((THIS_DIR / "out/layout_measured.yaml").read_text(encoding="utf-8"))
    qr_key, qr_lab = pose["qr_in_view"][0]["name"].split(":")
    qr_center = np.array(meas["magazines"][qr_key]["qr"][qr_lab]["center_world"])

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
    q[idx] = np.deg2rad(joints_deg)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    # SingleArticulation 에는 set_joint_position_targets 가 없다. apply_action 을 쓴다.
    from isaacsim.core.utils.types import ArticulationAction
    robot.apply_action(ArticulationAction(
        joint_positions=np.deg2rad(joints_deg), joint_indices=idx))
    for _ in range(30):
        world.step(render=False)

    # 렌더 프로덕트 + rgb 애노테이터
    import omni.replicator.core as rep
    rp = rep.create.render_product(CAMERA, RESOLUTION)
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])

    # 통로 방향 = QR 법선 방향 (선반에서 멀어지는 쪽)
    qr_normal = np.array(meas["magazines"][qr_key]["qr"][qr_lab]["normal_world"])
    aisle_sign = float(np.sign(qr_normal[1]))     # -1 이면 통로가 -y 쪽

    detector = cv2.QRCodeDetector()
    FRAME_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 78)
    print(f"  QR 디코드 가능 거리 — 자세 '{POSE_NAME}'")
    print("=" * 78)
    print(f"   QR      {qr_key}:{qr_lab}   world {np.round(qr_center, 4).tolist()}")
    print(f"   규격    version 1, {QR_MODULES} 모듈 + 여백 {QR_QUIET} "
          f"(라벨 {QR_LABEL_SIDE_M*1000:.0f} mm 중 데이터 영역 "
          f"{QR_LABEL_SIDE_M*QR_MODULES/(QR_MODULES+2*QR_QUIET)*1000:.1f} mm)")
    print(f"   관절    {[round(v,2) for v in joints_deg]}")
    print()
    print(f"   {'베이스 y':>9} {'거리':>7} {'라벨px':>7} {'모듈px':>7}  "
          f"{'검출':>4} {'디코드':>8}")
    print("   " + "-" * 60)

    rows = []
    for off in Y_OFFSETS:
        # aisle_sign 은 통로가 놓인 방향. 선반 쪽으로 가려면 그 반대로 민다.
        by = base_xy[1] - aisle_sign * off
        robot.set_world_pose(position=np.array([base_xy[0], by, 0.0]),
                             orientation=yaw_quat_wxyz(0.0))
        for attr, arg in (("set_velocities", np.zeros(6)),):
            fn = getattr(robot, attr, None)
            if fn:
                try:
                    fn(arg)
                except Exception:
                    pass
        # 텔레포트 뒤 팔이 완전히 멎어야 한다. 덜 멎은 프레임을 잡으면
        # 경계 근처에서 디코드가 들쭉날쭉해진다.
        for _ in range(120):
            world.step(render=True)

        cam_p = world_pos(CAMERA)
        dist = float(np.linalg.norm(qr_center - cam_p))
        label_px = fx * QR_LABEL_SIDE_M / dist
        module_px = label_px / (QR_MODULES + 2 * QR_QUIET)

        rgb = annot.get_data()
        if rgb is None or getattr(rgb, "size", 0) == 0:
            print(f"   {by:9.3f} {dist:7.3f}  렌더 실패")
            continue
        img = np.asarray(rgb)[:, :, :3][:, :, ::-1].copy()   # RGBA -> BGR
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        text, pts, _ = detector.detectAndDecode(gray)
        found = pts is not None and len(np.asarray(pts).reshape(-1)) > 0
        decoded = text if text else ""

        name = FRAME_DIR / f"{POSE_NAME}_y{off:+.2f}_{dist:.3f}m.png"
        cv2.imwrite(str(name), img)

        print(f"   {by:9.3f} {dist:7.3f} {label_px:7.1f} {module_px:7.2f}  "
              f"{'O' if found else 'X':>4} "
              f"{repr(decoded) if decoded else 'X':>8}")
        rows.append(dict(base_y=round(by, 4), offset=off,
                         distance_m=round(dist, 4),
                         label_px=round(float(label_px), 2),
                         module_px=round(float(module_px), 3),
                         detected=bool(found), decoded=decoded,
                         frame=str(name.relative_to(WS_ROOT))))

    ok = [r for r in rows if r["decoded"]]
    print()
    print("=" * 78)
    if ok:
        far = max(ok, key=lambda r: r["distance_m"])
        print(f"   디코드 성공 최대 거리   {far['distance_m']:.3f} m  "
              f"(라벨 {far['label_px']:.0f} px, 모듈당 {far['module_px']:.2f} px)")
        near_fail = [r for r in rows if not r["decoded"]
                     and r["distance_m"] > far["distance_m"]]
        if near_fail:
            f0 = min(near_fail, key=lambda r: r["distance_m"])
            print(f"   첫 실패 거리            {f0['distance_m']:.3f} m  "
                  f"(라벨 {f0['label_px']:.0f} px, 모듈당 {f0['module_px']:.2f} px)")
    else:
        print("   전 구간 디코드 실패")
    print(f"   프레임 이미지           {FRAME_DIR}")
    OUT_YAML.write_text(
        yaml.safe_dump(dict(pose=POSE_NAME, qr=f"{qr_key}:{qr_lab}", rows=rows),
                       allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"   기록                    {OUT_YAML}")


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
finally:
    sys.stdout.flush()
    os._exit(0)

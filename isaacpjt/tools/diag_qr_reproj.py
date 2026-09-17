"""
QR 꼭짓점 재투영 진단 — eval_qr_pose.py 의 4.4 mm 횡오차가 어디서 오는지 가른다.

    isaac_python isaacpjt/tools/diag_qr_reproj.py
    DIAG_POSE=shelf_1_top_close_centered DIAG_TARGET=shelf_1/top/magazine_1_orange \\
        isaac_python isaacpjt/tools/diag_qr_reproj.py

방법:
  정답 QR 코드영역 꼭짓점 4 개를 world 에서 만들고, K 와 카메라 pose 로 이미지에
  투영해서 cv2 가 검출한 꼭짓점과 픽셀 단위로 비교한다.

  정답과 카메라를 각각 두 가지로 만들어 조합별로 본다.
    GT      static : layout_measured.yaml 의 label center (물리 전 저작값)
            live   : 물리 안정화 뒤 매거진 리지드바디 pose x 라벨 메시 로컬 점
    camera  prim   : 카메라 prim 의 world 변환 (USD 카메라 -Z 시선 -> 광학 프레임)
            tf     : link_6 world x frames.yaml 정적변환 (eval_qr_pose 와 같은 경로)
  K 도 두 가지:
            info   : frames.yaml camera_info_observed
            usd    : 카메라 prim 의 focalLength / aperture 에서 직접 계산

판정:
  잔차가 네 점 모두 같은 방향으로 쏠려 있으면 GT/카메라/K 중 하나가 틀린 것이다.
  어떤 조합에서 잔차가 1 px 미만으로 떨어지는지가 곧 원인이다.

출력: 표 + out/qr_reproj_diag.yaml, 프레임은 out/qr_reproj_frames/
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import math
import sys
from pathlib import Path

import numpy as np
import omni.usd
import yaml
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.types import ArticulationAction

sys.stdout.reconfigure(line_buffering=True)

THIS_DIR = Path(__file__).resolve().parent
ISAACPJT = THIS_DIR.parent
WS_ROOT  = ISAACPJT.parent

WORLD_USD   = str(ISAACPJT / "worlds/simple_factory_layout.usda")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
MEASURED    = THIS_DIR / "out/layout_measured.yaml"
TAUGHT      = THIS_DIR / "out/taught_poses.yaml"
OUT_YAML    = THIS_DIR / "out/qr_reproj_diag.yaml"
FRAME_DIR   = THIS_DIR / "out/qr_reproj_frames"

ROBOT   = "/World/Robots/nova_carter1"
CHASSIS = f"{ROBOT}/chassis_link"
ARM     = f"{ROBOT}/m0609"
LINK6   = f"{ARM}/link_6"
CAMERA  = f"{ARM}/short_gripper/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

POSE_NAME  = os.environ.get("DIAG_POSE", "shelf_1_top_close_centered")
TARGET_KEY = os.environ.get("DIAG_TARGET", "shelf_1/top/magazine_1_orange")
LABEL      = os.environ.get("DIAG_LABEL", "qr_label_ny")
# 로봇이 멎었는지도 같이 본다 — 안정화 스텝을 늘려 가며 같은 측정을 반복한다.
SETTLE_LIST = [int(v) for v in os.environ.get("DIAG_SETTLE", "110,400").split(",")]
FRAMES = int(os.environ.get("DIAG_FRAMES", "3"))

CARTER_Z = 0.07963398335074101
RESOLUTION = (1280, 720)
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e5, 1e4, 2700.0

# 라벨 이미지는 29 모듈 중 코드가 가운데 21 모듈이다 (frames.yaml qr 절)
DATA_RATIO = 21.0 / 29.0

# USD 카메라(-Z 시선, +Y 위) -> ROS 광학(+Z 시선, +Y 아래)
R_USDCAM_TO_OPT = np.diag([1.0, -1.0, -1.0])


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w]); x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def quat_wxyz_to_mat(q):
    w, x, y, z = q
    return quat_xyzw_to_mat([x, y, z, w])


def yaw_quat_wxyz(deg):
    h = np.radians(deg)/2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def world_mat(path):
    """USD 의 local-to-world. 회전에 스케일이 섞여 있으면 정규화한다."""
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    a = np.array(m).T
    R = a[:3, :3].copy()
    R /= np.linalg.norm(R, axis=0)
    return a[:3, 3].copy(), R


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


def label_local_points(stage, label_path):
    """라벨 메시 점을 st 로 정렬해 [TL, TR, BR, BL] (이미지 기준) 로 돌려준다.

    cv2.QRCodeDetector 는 코드 방향 기준 TL, TR, BR, BL 순서로 꼭짓점을 준다.
    USD st 는 v 가 위로 증가하므로 이미지 TL = (u=0, v=1).
    """
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(label_path))
    pts = np.array(mesh.GetPointsAttr().Get(), dtype=float)
    st = np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st").Get(), dtype=float)
    want = {(0, 1): 0, (1, 1): 1, (1, 0): 2, (0, 0): 3}
    out = np.zeros((4, 3))
    for p, uv in zip(pts, st):
        out[want[(int(round(uv[0])), int(round(uv[1])))]] = p
    return out


def shrink_to_data(corners):
    c = corners.mean(axis=0)
    return c + DATA_RATIO * (corners - c)


def project(K, R_wo, p_wo, pts_world):
    """world 점 -> 픽셀. R_wo/p_wo 는 world <- optical"""
    pc = (pts_world - p_wo) @ R_wo           # = R_wo^T (p - t)
    uv = pc[:, :2] / pc[:, 2:3]
    return uv @ K[:2, :2].T + K[:2, 2]


def usd_intrinsics(stage):
    cam = UsdGeom.Camera(stage.GetPrimAtPath(CAMERA))
    f  = cam.GetFocalLengthAttr().Get()
    ha = cam.GetHorizontalApertureAttr().Get()
    va = cam.GetVerticalApertureAttr().Get()
    ho = cam.GetHorizontalApertureOffsetAttr().Get() or 0.0
    vo = cam.GetVerticalApertureOffsetAttr().Get() or 0.0
    W, H = RESOLUTION
    fx = f / ha * W
    # Isaac 렌더러는 정사각 픽셀을 강제한다 — fy 는 va 가 아니라 fx 를 따른다.
    # va 로 계산한 값도 같이 찍어 둬서, 둘이 다르면 눈에 띄게 한다.
    fy_va = f / va * H if va else float("nan")
    K = np.array([[fx, 0.0, W/2.0 + ho / ha * W],
                  [0.0, fx, H/2.0 + vo / ha * W],
                  [0.0, 0.0, 1.0]])
    return K, dict(focal_mm=f, h_ap_mm=ha, v_ap_mm=va, h_off_mm=ho, v_off_mm=vo,
                   fx=fx, fy_from_vap=fy_va)


def main():
    import cv2

    cfg    = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    meas   = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
    taught = yaml.safe_load(TAUGHT.read_text(encoding="utf-8"))

    ci = cfg["wrist_camera"]["camera_info_observed"]
    K_info = np.array(ci["k"], dtype=float).reshape(3, 3)
    R_l6_cam  = quat_xyzw_to_mat(
        cfg["static_transforms"]["m0609_tool0__camera_link"]["quat_xyzw"])
    t_l6_cam  = np.array(cfg["static_transforms"]["m0609_tool0__camera_link"]["xyz"])
    R_cam_opt = quat_xyzw_to_mat(
        cfg["static_transforms"]["camera_link__camera_color_optical_frame"]["quat_xyzw"])

    pose = taught[POSE_NAME]
    joints = np.array(pose["joints_deg"], dtype=float)
    base_y = float(pose["base_link_world"][1])

    tgt = meas["magazines"][TARGET_KEY]
    mag_prim = tgt["prim"]
    label_prim = f"{mag_prim}/{LABEL}"
    static_center = np.array(tgt["qr"][LABEL]["center_world"])

    # ── 씬 ───────────────────────────────────────────────────────
    ctx = omni.usd.get_context()
    ctx.open_stage(WORLD_USD)
    for _ in range(60):
        simulation_app.update()
    stage = ctx.get_stage()
    stage.Load()
    for _ in range(60):
        simulation_app.update()

    configure_drives()
    world = World(stage_units_in_meters=1.0)
    robot = world.scene.add(SingleArticulation(prim_path=CHASSIS, name="carter"))
    mag = world.scene.add(SingleRigidPrim(prim_path=mag_prim, name="diag_magazine"))
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

    cam_dx = (float(pose["camera_optical_world"][0])
              - float(pose["base_link_world"][0]))
    base_x = float(os.environ.get("DIAG_BASE_X", str(static_center[0] - cam_dx)))
    robot.set_world_pose(position=np.array([base_x, base_y, CARTER_Z]),
                         orientation=yaw_quat_wxyz(0.0))

    import omni.replicator.core as rep
    rp = rep.create.render_product(CAMERA, RESOLUTION)
    annot = rep.AnnotatorRegistry.get_annotator("rgb")
    annot.attach([rp])
    detector = cv2.QRCodeDetector()
    FRAME_DIR.mkdir(parents=True, exist_ok=True)

    K_usd, usd_k_info = usd_intrinsics(stage)

    # 라벨 로컬 점 -> 매거진 로컬. 라벨이 매거진 바로 아래 자식이라 그대로 쓴다.
    # (payload 루트가 매거진 prim 에 별칭되므로 중간 xform 이 없다. 그래도
    #  혹시 모를 중간 변환은 static 경로와 비교해서 드러난다.)
    local_corners = label_local_points(stage, label_prim)
    data_local = shrink_to_data(local_corners)

    print("=" * 88)
    print("  QR 꼭짓점 재투영 진단")
    print("=" * 88)
    print(f"   자세   {POSE_NAME}   표적 {TARGET_KEY}:{LABEL}   베이스 x {base_x:.4f}")
    print(f"   K info  fx {K_info[0,0]:.3f} fy {K_info[1,1]:.3f} "
          f"cx {K_info[0,2]:.2f} cy {K_info[1,2]:.2f}")
    print(f"   K usd   fx {K_usd[0,0]:.3f} (va 로 계산한 fy {usd_k_info['fy_from_vap']:.3f}) "
          f"cx {K_usd[0,2]:.2f} cy {K_usd[1,2]:.2f}   {usd_k_info}")

    records = []
    settled_so_far = 0
    for settle in SETTLE_LIST:
        for _ in range(max(0, settle - settled_so_far)):
            world.step(render=True)
        settled_so_far = max(settled_so_far, settle)

        for fi in range(FRAMES):
            for _ in range(3):
                world.step(render=True)

            # ── 이 프레임 시점의 모든 pose 를 한 번에 읽는다 ──
            mag_p, mag_q = mag.get_world_pose()
            R_mag = quat_wxyz_to_mat(mag_q)
            live_corners = (R_mag @ data_local.T).T + mag_p
            usd_p, usd_R = world_mat(label_prim)
            usd_corners = (usd_R @ shrink_to_data(local_corners).T).T + usd_p
            static_corners = live_corners - live_corners.mean(axis=0) + static_center

            cam_p, cam_R = world_mat(CAMERA)
            R_opt_prim = cam_R @ R_USDCAM_TO_OPT
            l6_p, R_l6 = world_mat(LINK6)
            R_opt_tf = R_l6 @ R_l6_cam @ R_cam_opt
            p_opt_tf = l6_p + R_l6 @ t_l6_cam

            rgb = np.asarray(annot.get_data())[:, :, :3][:, :, ::-1].copy()
            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            ok, infos, pts, _ = detector.detectAndDecodeMulti(gray)
            want = "1" if "orange" in TARGET_KEY else "2"
            quad = None
            if ok and pts is not None:
                for text, qd in zip(infos, np.asarray(pts)):
                    if text == want:
                        quad = np.asarray(qd, dtype=np.float64).reshape(4, 2)
            if quad is None:
                cv2.imwrite(str(FRAME_DIR / f"fail_settle{settle}_f{fi}.png"), rgb)
                print(f"\n   settle {settle} frame {fi}: 검출 실패  ok={ok} infos={infos}")
                continue
            quad_sub = cv2.cornerSubPix(
                gray, quad.astype(np.float32).reshape(-1, 1, 2), (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01)
            ).reshape(4, 2).astype(np.float64)

            rec = dict(settle=settle, frame=fi,
                       magazine_live_pos=[round(float(v), 5) for v in mag_p],
                       gt_live_minus_static_mm=[round(float(v)*1000, 3) for v in
                                                live_corners.mean(axis=0) - static_center],
                       gt_live_minus_usd_mm=[round(float(v)*1000, 3) for v in
                                             live_corners.mean(axis=0) - usd_corners.mean(axis=0)],
                       cam_prim_minus_tf_mm=[round(float(v)*1000, 3) for v in cam_p - p_opt_tf],
                       cam_rot_prim_vs_tf_deg=round(math.degrees(math.acos(np.clip(
                           (np.trace(R_opt_prim.T @ R_opt_tf) - 1) / 2, -1, 1))), 4),
                       detected_px=quad.round(2).tolist(),
                       subpix_shift_px=round(float(np.abs(quad_sub - quad).max()), 3),
                       combos={})

            if fi == 0:
                vis = rgb.copy()
                p_draw = project(K_info, R_opt_prim, cam_p, live_corners)
                p_stat = project(K_info, R_opt_prim, cam_p, static_corners)
                for a, b, c in zip(quad, p_draw, p_stat):
                    cv2.circle(vis, (int(a[0]), int(a[1])), 5, (0, 0, 255), -1)
                    cv2.drawMarker(vis, (int(b[0]), int(b[1])), (0, 255, 0),
                                   cv2.MARKER_CROSS, 14, 2)
                    cv2.drawMarker(vis, (int(c[0]), int(c[1])), (255, 0, 0),
                                   cv2.MARKER_TILTED_CROSS, 14, 2)
                cv2.imwrite(str(FRAME_DIR / f"settle{settle}.png"), vis)

            print(f"\n   ── settle {settle}  frame {fi} ──")
            print(f"      GT live-static {rec['gt_live_minus_static_mm']} mm   "
                  f"live-usd {rec['gt_live_minus_usd_mm']} mm")
            print(f"      cam prim-tf   {rec['cam_prim_minus_tf_mm']} mm   "
                  f"회전차 {rec['cam_rot_prim_vs_tf_deg']}°   subpix 이동 {rec['subpix_shift_px']} px")
            print(f"      {'GT':>7} {'cam':>5} {'K':>5}   "
                  f"{'평균 du':>8} {'평균 dv':>8} {'RMS':>7} {'최대':>7}")
            for gname, gpts in (("static", static_corners), ("live", live_corners)):
                for cname, (Rw, pw) in (("prim", (R_opt_prim, cam_p)),
                                        ("tf", (R_opt_tf, p_opt_tf))):
                    for kname, K in (("info", K_info), ("usd", K_usd)):
                        proj = project(K, Rw, pw, gpts)
                        res = quad_sub - proj
                        mean = res.mean(axis=0)
                        rms = float(np.sqrt((res ** 2).sum(axis=1).mean()))
                        mx = float(np.linalg.norm(res, axis=1).max())
                        key = f"{gname}/{cname}/{kname}"
                        rec["combos"][key] = dict(
                            mean_du=round(float(mean[0]), 3), mean_dv=round(float(mean[1]), 3),
                            rms_px=round(rms, 3), max_px=round(mx, 3),
                            per_corner=res.round(3).tolist())
                        print(f"      {gname:>7} {cname:>5} {kname:>5}   "
                              f"{mean[0]:+8.2f} {mean[1]:+8.2f} {rms:7.2f} {mx:7.2f}")
            records.append(rec)

    # ── 요약: 조합별 평균 RMS ─────────────────────────────────────
    print("\n" + "=" * 88)
    print("  요약 — 조합별 RMS 평균 (px)")
    print("=" * 88)
    summary = {}
    if records:
        for key in records[0]["combos"]:
            v = [r["combos"][key]["rms_px"] for r in records if key in r["combos"]]
            summary[key] = round(float(np.mean(v)), 3)
        for key, v in sorted(summary.items(), key=lambda kv: kv[1]):
            print(f"   {key:<22} {v:7.3f}")

    OUT_YAML.write_text(yaml.safe_dump(
        dict(pose=POSE_NAME, target=f"{TARGET_KEY}:{LABEL}",
             k_info=K_info.round(4).tolist(), k_usd=K_usd.round(4).tolist(),
             usd_camera=usd_k_info, summary_rms_px=summary, records=records),
        allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"\n   기록 {OUT_YAML}   프레임 {FRAME_DIR}")


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
finally:
    sys.stdout.flush()
    os._exit(0)

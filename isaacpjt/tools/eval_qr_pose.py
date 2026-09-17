"""
QR 자세추정 정확도 평가 — GT 대비 위치/yaw 오차와 반복편차를 잰다.

    isaac_python isaacpjt/tools/eval_qr_pose.py
    EVAL_POSE=shelf_1_top_close_centered EVAL_TARGET=magazine_1_orange \\
        isaac_python isaacpjt/tools/eval_qr_pose.py

어떻게 테스트가 되나:
  씬이니까 정답을 안다. USD 에서 QR 의 실제 world pose 를 읽을 수 있다.
  그래서 이렇게 닫힌 루프를 만든다.

      손목 카메라 렌더
        -> cv2.QRCodeDetector 로 꼭짓점 4 개 + ID
        -> cv2.solvePnP(SOLVEPNP_IPPE_SQUARE) 로 T_cam_QR
        -> 카메라 world pose 로 곱해서 T_world_QR (추정)
        -> USD 의 T_world_QR (정답) 과 비교

  카메라 world pose 와 K 는 이미 검증된 값을 쓴다 (TF 체인 오차 0.11 mm,
  K 는 /wrist_camera/color/camera_info 실측). 그래서 여기서 나오는 오차는
  대부분 자세추정 자체의 오차다.

한 변 길이 s 를 뭘로 넣을 것인가 — 이게 제일 큰 함정이다:
  이 씬의 QR 라벨 메시는 50 mm 인데 그건 여백(quiet zone) 4 모듈까지 포함한
  크기다. 실제 코드 영역은 21/29 * 50 = 36.2 mm 다.
  cv2 의 QRCodeDetector 는 코드 영역의 꼭짓점을 돌려주므로 s = 36.2 를 써야 한다.
  50 을 넣으면 거리가 29/21 = 1.38 배로 나온다. 두 값을 다 돌려서 보여준다.

출력: 표 + out/qr_pose_eval.yaml, 프레임은 out/qr_pose_frames/
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

THIS_DIR = Path(__file__).resolve().parent
ISAACPJT = THIS_DIR.parent
WS_ROOT  = ISAACPJT.parent

WORLD_USD   = str(ISAACPJT / "worlds/simple_factory_layout.usda")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
MEASURED    = THIS_DIR / "out/layout_measured.yaml"
TAUGHT      = THIS_DIR / "out/taught_poses.yaml"
OUT_YAML    = THIS_DIR / "out/qr_pose_eval.yaml"
FRAME_DIR   = THIS_DIR / "out/qr_pose_frames"

ROBOT   = "/World/Robots/nova_carter1"
CHASSIS = f"{ROBOT}/chassis_link"
ARM     = f"{ROBOT}/m0609"
LINK6   = f"{ARM}/link_6"
CAMERA  = f"{ARM}/short_gripper/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

POSE_NAME   = os.environ.get("EVAL_POSE", "shelf_1_top_close_centered")
TARGET_KEY  = os.environ.get("EVAL_TARGET", "shelf_1/top/magazine_1_orange")
LABEL       = os.environ.get("EVAL_LABEL", "qr_label_ny")
# 디코드 창 안에서 베이스 x 를 흔들어 가며 표본을 모은다. 반복편차를 보려면
# 완전히 같은 조건만 보면 안 된다 — 실제로는 정차 위치가 매번 조금씩 다르다.
X_JITTER = [float(v) for v in
            os.environ.get("EVAL_X_JITTER", "-0.04,-0.02,0.0,0.02,0.04").split(",")]
FRAMES_PER_POSE = int(os.environ.get("EVAL_FRAMES", "3"))

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


def yaw_quat_wxyz(deg):
    h = np.radians(deg)/2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def world_mat(path):
    stage = omni.usd.get_context().get_stage()
    m = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    a = np.array(m).T
    return a[:3, 3].copy(), a[:3, :3].copy()


def qr_frame_from_normal(n):
    """grasp.yaml 과 같은 규약: x=이미지 오른쪽, y=이미지 아래, z=라벨 안쪽"""
    z = -np.asarray(n, dtype=float); z /= np.linalg.norm(z)
    y = np.array([0.0, 0.0, -1.0])
    x = np.cross(y, z); x /= np.linalg.norm(x)
    return np.column_stack([x, y, z])


def yaw_diff_deg(R_a, R_b):
    """두 회전의 world z 축 둘레 각도 차"""
    # 각 프레임의 x 축을 xy 평면에 투영해 방위각을 비교한다
    ax, bx = R_a[:, 0], R_b[:, 0]
    a = math.atan2(ax[1], ax[0])
    b = math.atan2(bx[1], bx[0])
    d = math.degrees(a - b)
    return (d + 180.0) % 360.0 - 180.0


import math  # yaw_diff_deg 에서 쓴다


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

    K = cfg["wrist_camera"]["camera_info_observed"]["k"]
    Kmat = np.array(K, dtype=float).reshape(3, 3)
    dist = np.array(cfg["wrist_camera"]["camera_info_observed"]["d"], dtype=float)

    qr_cfg = cfg["qr"]
    S_LABEL = qr_cfg["label_side_m"]        # 0.050  (여백 포함)
    S_DATA  = qr_cfg["data_side_m"]         # 0.0362 (코드 영역만)

    pose = taught[POSE_NAME]
    joints = np.array(pose["joints_deg"], dtype=float)
    base_y = float(pose["base_link_world"][1])

    tgt = meas["magazines"][TARGET_KEY]
    qr_gt_c = np.array(tgt["qr"][LABEL]["center_world"])
    qr_gt_n = np.array(tgt["qr"][LABEL]["normal_world"])
    R_gt = qr_frame_from_normal(qr_gt_n)

    R_l6_cam  = quat_xyzw_to_mat(
        cfg["static_transforms"]["m0609_tool0__camera_link"]["quat_xyzw"])
    R_cam_opt = quat_xyzw_to_mat(
        cfg["static_transforms"]["camera_link__camera_color_optical_frame"]["quat_xyzw"])

    # ── 씬 ───────────────────────────────────────────────────────
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

    # 베이스 x 기준점 — 이 표적이 디코드되는 창의 중앙 부근
    # 티칭 자세에서 카메라가 베이스보다 x 로 얼마나 앞/뒤에 있었는지.
    # 그만큼 빼야 카메라가 QR 바로 앞에 선다.
    cam_dx = (float(pose["camera_optical_world"][0])
              - float(pose["base_link_world"][0]))
    base_x0 = float(os.environ.get("EVAL_BASE_X", str(qr_gt_c[0] - cam_dx)))

    print("=" * 84)
    print("  QR 자세추정 정확도 평가")
    print("=" * 84)
    print(f"   자세     {POSE_NAME}   관절 {joints.round(2).tolist()}")
    print(f"   표적     {TARGET_KEY}:{LABEL}")
    print(f"   GT       중심 {qr_gt_c.round(5).tolist()}   법선 {qr_gt_n.tolist()}")
    print(f"   베이스   x {base_x0:.4f} (지터 {X_JITTER})  y {base_y:.4f}")
    print(f"   K        fx {K[0]:.2f}  cx {K[2]:.1f}  cy {K[5]:.1f}   왜곡 {dist.tolist()}")
    print(f"   s 후보   라벨 {S_LABEL*1000:.1f} mm / 코드영역 {S_DATA*1000:.1f} mm")

    # SOLVEPNP_IPPE_SQUARE 는 3D 점 순서가 문서에 못박혀 있다:
    #     0: [-s/2, +s/2, 0]   1: [+s/2, +s/2, 0]
    #     2: [+s/2, -s/2, 0]   3: [-s/2, -s/2, 0]
    # 즉 y 가 '위'인 프레임이다. 우리 QR 프레임 규약(y 가 아래)대로 넣으면
    # 조용히 쓰레기 값을 돌려준다 — 실제로 tvec 이 0.006 m 로 나왔다
    # (정답 0.271 m). 예외도 안 나고 ok=True 라 놓치기 쉽다.
    # 그래서 여기서는 OpenCV 규약대로 넣고, 결과를 우리 규약으로 돌린다.
    def obj_points(s):
        h = s / 2.0
        return np.array([[-h,  h, 0.0], [h,  h, 0.0],
                         [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float64)

    # OpenCV 마커 프레임(x 오른쪽, y 위) -> 우리 QR 프레임(x 오른쪽, y 아래, z 안쪽).
    # x 축 둘레 180도.
    R_CV_TO_QR = np.diag([1.0, -1.0, -1.0])

    samples = {"data": [], "label": []}
    rows = []
    for jx in X_JITTER:
        robot.set_world_pose(position=np.array([base_x0 + jx, base_y, CARTER_Z]),
                             orientation=yaw_quat_wxyz(0.0))
        try:
            robot.set_velocities(np.zeros(6))
        except Exception:
            pass
        for _ in range(110):
            world.step(render=True)

        cam_p, _ = world_mat(CAMERA)
        _, R_l6 = world_mat(LINK6)
        R_opt = R_l6 @ R_l6_cam @ R_cam_opt          # world <- optical

        for fi in range(FRAMES_PER_POSE):
            for _ in range(3):
                world.step(render=True)
            rgb = annot.get_data()
            img = np.asarray(rgb)[:, :, :3][:, :, ::-1].copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ok, infos, pts, _ = detector.detectAndDecodeMulti(gray)
            if not ok or pts is None:
                rows.append(dict(jitter=jx, frame=fi, detected=False))
                continue
            # 기대 ID 와 맞는 것을 고른다
            want = "1" if "orange" in TARGET_KEY else "2"
            quad = None
            for text, qd in zip(infos, np.asarray(pts)):
                if text == want:
                    quad = np.asarray(qd, dtype=np.float64).reshape(4, 2)
                    break
            if quad is None:
                rows.append(dict(jitter=jx, frame=fi, detected=False))
                continue

            if fi == 0:
                vis = img.copy()
                for i, (u, v) in enumerate(quad):
                    cv2.circle(vis, (int(u), int(v)), 6, (0, 0, 255), -1)
                    cv2.putText(vis, str(i), (int(u) + 8, int(v) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imwrite(str(FRAME_DIR / f"{TARGET_KEY.replace('/','_')}"
                                            f"_x{jx:+.2f}.png"), vis)

            if os.environ.get("EVAL_DEBUG") and fi == 0 and jx == X_JITTER[0]:
                print(f"\n   [디버그] quad(px)\n{np.round(quad, 1)}")
                print(f"   [디버그] quad 변 길이 {np.linalg.norm(quad[1]-quad[0]):.1f}, "
                      f"{np.linalg.norm(quad[2]-quad[1]):.1f}, "
                      f"{np.linalg.norm(quad[3]-quad[2]):.1f}, "
                      f"{np.linalg.norm(quad[0]-quad[3]):.1f} px")
                print(f"   [디버그] 카메라 world {cam_p.round(4).tolist()}  "
                      f"GT 까지 {np.linalg.norm(qr_gt_c-cam_p):.4f} m")
                _o, _r, _t = cv2.solvePnP(obj_points(S_DATA), quad, Kmat, dist,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
                print(f"   [디버그] IPPE_SQUARE tvec {np.asarray(_t).reshape(3).round(5).tolist()}")
                _o2, _r2, _t2 = cv2.solvePnP(obj_points(S_DATA), quad, Kmat, dist,
                                             flags=cv2.SOLVEPNP_ITERATIVE)
                print(f"   [디버그] ITERATIVE   tvec {np.asarray(_t2).reshape(3).round(5).tolist()}\n")

            row = dict(jitter=jx, frame=fi, detected=True)
            for tag, s in (("data", S_DATA), ("label", S_LABEL)):
                okp, rvec, tvec = cv2.solvePnP(
                    obj_points(s), quad, Kmat, dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not okp:
                    continue
                R_pnp, _ = cv2.Rodrigues(rvec)
                c_est = R_opt @ tvec.reshape(3) + cam_p
                R_est = R_opt @ R_pnp @ R_CV_TO_QR
                pos_err = float(np.linalg.norm(c_est - qr_gt_c))
                yaw_err = yaw_diff_deg(R_est, R_gt)
                dist_est = float(np.linalg.norm(tvec))
                # 오차를 시선축(광학 +Z)과 그에 수직인 성분으로 쪼갠다.
                # 거의 전부가 시선축이면 스케일(= s 값) 문제이고,
                # 횡방향이면 꼭짓점 위치나 캘리브 문제다.
                e = c_est - qr_gt_c
                view = R_opt[:, 2]                       # world 에서의 광학 +Z
                e_along = float(np.dot(e, view))
                e_lat = float(np.linalg.norm(e - e_along * view))
                dist_gt = float(np.linalg.norm(qr_gt_c - cam_p))
                samples[tag].append((pos_err, yaw_err, c_est, dist_est,
                                     e_along, e_lat))
                row[tag] = dict(pos_err_mm=round(pos_err * 1000, 3),
                                yaw_err_deg=round(yaw_err, 4),
                                range_m=round(dist_est, 4),
                                range_gt_m=round(dist_gt, 4),
                                err_along_view_mm=round(e_along * 1000, 3),
                                err_lateral_mm=round(e_lat * 1000, 3))
            rows.append(row)

    print()
    print(f"   {'지터':>6} {'프레임':>6}  "
          f"{'오차':>8}{'시선축':>9}{'횡':>8}{'yaw':>9}"
          f"{'추정/정답 거리':>16}  {'s=50mm':>8}")
    print("   " + "-" * 82)
    for r in rows:
        if not r.get("detected"):
            print(f"   {r['jitter']:+6.2f} {r['frame']:6d}  검출 실패")
            continue
        d, l = r.get("data"), r.get("label")
        print(f"   {r['jitter']:+6.2f} {r['frame']:6d}  "
              f"{d['pos_err_mm']:8.2f}{d['err_along_view_mm']:+9.2f}"
              f"{d['err_lateral_mm']:8.2f}{d['yaw_err_deg']:+8.2f}°"
              f"{d['range_m']:8.3f}/{d['range_gt_m']:.3f}  {l['pos_err_mm']:8.2f}")

    print()
    print("=" * 84)
    print("  결과")
    print("=" * 84)
    summary = {}
    for tag, label in (("data", "s = 코드영역 36.2 mm"), ("label", "s = 라벨 50 mm")):
        v = samples[tag]
        if not v:
            continue
        pe = np.array([x[0] for x in v]) * 1000
        ye = np.array([x[1] for x in v])
        cs = np.array([x[2] for x in v])
        ea = np.array([x[4] for x in v]) * 1000
        el = np.array([x[5] for x in v]) * 1000
        print(f"\n   [{label}]   표본 {len(v)}")
        print(f"      위치오차   평균 {pe.mean():7.2f} mm   최대 {pe.max():7.2f} mm")
        print(f"      yaw 오차   평균 {ye.mean():+7.3f}°   최대 {np.abs(ye).max():7.3f}°")
        print(f"      시선축 성분 평균 {ea.mean():+7.2f} mm   횡 성분 평균 {el.mean():7.2f} mm")
        print(f"      반복편차   위치 1σ {cs.std(axis=0).max()*1000:6.3f} mm   "
              f"yaw 1σ {ye.std():6.3f}°")
        summary[tag] = dict(
            s_m=(S_DATA if tag == "data" else S_LABEL), n=len(v),
            pos_err_mm_mean=round(float(pe.mean()), 3),
            pos_err_mm_max=round(float(pe.max()), 3),
            yaw_err_deg_mean=round(float(ye.mean()), 4),
            yaw_err_deg_absmax=round(float(np.abs(ye).max()), 4),
            err_along_view_mm_mean=round(float(ea.mean()), 3),
            err_lateral_mm_mean=round(float(el.mean()), 3),
            pos_repeat_1sigma_mm=round(float(cs.std(axis=0).max() * 1000), 4),
            yaw_repeat_1sigma_deg=round(float(ye.std()), 4))

    print()
    print("   완료 기준: 위치 3 mm, yaw 1도 (반복편차도 동일 수준)")
    for tag in ("data", "label"):
        if tag not in summary:
            continue
        s_ = summary[tag]
        ok = s_["pos_err_mm_max"] <= 3.0 and s_["yaw_err_deg_absmax"] <= 1.0
        print(f"      s={s_['s_m']*1000:.1f} mm -> {'통과' if ok else '미달'}")

    OUT_YAML.write_text(yaml.safe_dump(
        dict(pose=POSE_NAME, target=f"{TARGET_KEY}:{LABEL}",
             gt_center=[float(v) for v in qr_gt_c],
             summary=summary, rows=rows),
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

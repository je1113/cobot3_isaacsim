"""
근거리 QR 자세추정 평가 (깊이 평면 기반) — 체크리스트 3-2 완료 기준 검증.

    isaac_python isaacpjt/tools/eval_qr_pose_depth.py
    EVAL_POSE=shelf_1_top_close_centered EVAL_TARGET=magazine_1_orange \\
        isaac_python isaacpjt/tools/eval_qr_pose_depth.py

무엇을 검증하나:
  cobot3_perception.qr_pose.estimate_qr_pose() 를 실제 렌더(RGB+깊이)에 돌려
  GT 대비 오차를 잰다. eval_qr_pose.py(순수 PnP)와 같은 자리·같은 GT 정정
  방식(물리 안정화 뒤 리지드바디 pose, docs/08 §2-4 의 함정 참고)을 쓰되,
  이번엔 QR 프레임 회전을 PnP 가 아니라 깊이로 맞춘 벽 평면에서 가져온다.

  다중 프레임 평균 + 이상치 제거(aggregate_qr_poses)까지 포함해서, 최종
  "한 번 정지해서 얻는 값"이 완료 기준(위치 3 mm, yaw 1 도)을 만족하는지 본다.

완료 기준 (project-plan 3-2): GT 대비 위치 오차 3 mm, yaw 1 도 이내,
반복 편차도 동일 수준.

출력: 표 + out/qr_pose_depth_eval.yaml, 프레임은 out/qr_pose_depth_frames/
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
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.types import ArticulationAction

sys.stdout.reconfigure(line_buffering=True)

THIS_DIR = Path(__file__).resolve().parent
ISAACPJT = THIS_DIR.parent
WS_ROOT  = ISAACPJT.parent

sys.path.insert(0, str(WS_ROOT / "src/cobot3_perception"))
from cobot3_perception.qr_pose import estimate_qr_pose, aggregate_qr_poses, draw_observation

WORLD_USD   = str(ISAACPJT / "worlds/simple_factory_layout.usda")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
MEASURED    = THIS_DIR / "out/layout_measured.yaml"
TAUGHT      = THIS_DIR / "out/taught_poses.yaml"
OUT_YAML    = THIS_DIR / "out/qr_pose_depth_eval.yaml"
FRAME_DIR   = THIS_DIR / "out/qr_pose_depth_frames"

ROBOT   = "/World/Robots/nova_carter1"
CHASSIS = f"{ROBOT}/chassis_link"
ARM     = f"{ROBOT}/m0609"
LINK6   = f"{ARM}/link_6"
CAMERA  = f"{ARM}/short_gripper/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

POSE_NAME  = os.environ.get("EVAL_POSE", "shelf_1_top_close_centered")
TARGET_KEY = os.environ.get("EVAL_TARGET", "shelf_1/top/magazine_1_orange")
LABEL      = os.environ.get("EVAL_LABEL", "qr_label_ny")
# 정차 위치가 매번 정확히 같지 않다 — 베이스 x 를 흔들어 가며 표본을 모은다.
X_JITTER = [float(v) for v in
            os.environ.get("EVAL_X_JITTER", "-0.04,-0.02,0.0,0.02,0.04").split(",")]
FRAMES_PER_POSE = int(os.environ.get("EVAL_FRAMES", "3"))
DEPTH_NOISE_M = float(os.environ.get("EVAL_DEPTH_NOISE_MM", "0")) / 1000.0

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


def main():
    cfg    = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    meas   = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
    taught = yaml.safe_load(TAUGHT.read_text(encoding="utf-8"))

    K = np.array(cfg["wrist_camera"]["camera_info_observed"]["k"], dtype=float).reshape(3, 3)
    dist = np.array(cfg["wrist_camera"]["camera_info_observed"]["d"], dtype=float)

    pose = taught[POSE_NAME]
    joints = np.array(pose["joints_deg"], dtype=float)
    base_y = float(pose["base_link_world"][1])

    tgt = meas["magazines"][TARGET_KEY]
    mag_prim = tgt["prim"]
    expected_id = "1" if "orange" in TARGET_KEY else "2"

    R_l6_cam  = quat_xyzw_to_mat(
        cfg["static_transforms"]["m0609_tool0__camera_link"]["quat_xyzw"])
    t_l6_cam  = np.array(cfg["static_transforms"]["m0609_tool0__camera_link"]["xyz"])
    R_cam_opt = quat_xyzw_to_mat(
        cfg["static_transforms"]["camera_link__camera_color_optical_frame"]["quat_xyzw"])

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
    mag = world.scene.add(SingleRigidPrim(prim_path=mag_prim, name="eval_magazine"))
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
    rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annot.attach([rp])
    depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_annot.attach([rp])
    FRAME_DIR.mkdir(parents=True, exist_ok=True)

    cam_dx = (float(pose["camera_optical_world"][0])
              - float(pose["base_link_world"][0]))
    base_x0 = float(os.environ.get("EVAL_BASE_X", str(
        float(tgt["qr"][LABEL]["center_world"][0]) - cam_dx)))

    rng = np.random.default_rng(0)

    print("=" * 88)
    print("  근거리 QR 자세추정 평가 (깊이 평면)")
    print("=" * 88)
    print(f"   자세     {POSE_NAME}   표적 {TARGET_KEY} (id '{expected_id}')")
    print(f"   베이스   x {base_x0:.4f} (지터 {X_JITTER})  y {base_y:.4f}")
    print(f"   깊이 노이즈 {DEPTH_NOISE_M*1000:.1f} mm")

    per_jitter_results = []
    all_obs = []
    all_gt = []
    for jx in X_JITTER:
        robot.set_world_pose(position=np.array([base_x0 + jx, base_y, CARTER_Z]),
                             orientation=yaw_quat_wxyz(0.0))
        try:
            robot.set_velocities(np.zeros(6))
        except Exception:
            pass
        for _ in range(110):
            world.step(render=True)

        obs_list = []
        gt_list = []
        for fi in range(FRAMES_PER_POSE):
            for _ in range(3):
                world.step(render=True)
            rgb = np.asarray(rgb_annot.get_data())[:, :, :3][:, :, ::-1].copy()
            depth = np.asarray(depth_annot.get_data(), dtype=np.float64)
            depth = depth.reshape(rgb.shape[:2])
            if DEPTH_NOISE_M > 0:
                depth = depth + rng.normal(0, DEPTH_NOISE_M, depth.shape)

            l6_p, R_l6 = world_mat(LINK6)
            R_opt = R_l6 @ R_l6_cam @ R_cam_opt
            p_opt = l6_p + R_l6 @ t_l6_cam

            # 촬영 시점 GT (docs/08 §2-4: 물리 전 좌표를 GT 로 쓰면 안 된다)
            mag_p, mag_q = mag.get_world_pose()
            w_, x_, y_, z_ = mag_q
            R_mag = quat_xyzw_to_mat([x_, y_, z_, w_])
            qr_local_c = np.array(tgt["qr"][LABEL]["center_world"]) - np.array(tgt["origin_world"])
            qr_local_n = np.array(tgt["qr"][LABEL]["normal_world"])
            gt_c = R_mag @ qr_local_c + mag_p
            gt_n = R_mag @ qr_local_n
            import math as _m
            gt_yaw = _m.atan2((-np.cross([0, 0, -1.0], -gt_n))[1],
                              (-np.cross([0, 0, -1.0], -gt_n))[0])
            # (위 한 줄은 qr_pose.estimate_qr_pose 와 동일한 프레임 관례로 GT yaw 를 만든다)
            x_qr_gt = np.cross([0.0, 0.0, -1.0], -gt_n)
            x_qr_gt /= np.linalg.norm(x_qr_gt)
            gt_yaw = _m.atan2(x_qr_gt[1], x_qr_gt[0])

            obs = estimate_qr_pose(rgb, depth, K, dist, R_opt, p_opt, expected_id=expected_id)
            if fi == 0:
                import cv2
                cv2.imwrite(str(FRAME_DIR / f"{TARGET_KEY.replace('/','_')}_x{jx:+.2f}.png"),
                           draw_observation(rgb, obs, K, dist, R_opt, p_opt))
            obs_list.append(obs)
            gt_list.append((gt_c, gt_yaw))
            all_obs.append(obs)
            all_gt.append((gt_c, gt_yaw))

            if obs.ok:
                e_xy = float(np.linalg.norm(obs.center_world - gt_c) * 1000)
                e_yaw = _m.degrees(((obs.yaw_rad - gt_yaw + _m.pi) % (2 * _m.pi)) - _m.pi)
                print(f"   x{jx:+.2f} frame{fi}  xy {e_xy:6.3f} mm  yaw {e_yaw:+7.4f}°  "
                      f"tilt {obs.wall_tilt_deg:.2f}°  n_plane {obs.n_plane_points:5d}  "
                      f"rms {obs.plane_rms_mm:.3f} mm  cross-check {obs.cross_check_mm:.1f} mm")
            else:
                print(f"   x{jx:+.2f} frame{fi}  실패: {obs.reason}")

        agg = aggregate_qr_poses(obs_list)
        gt_mean_c = np.mean([g[0] for g in gt_list], axis=0)
        gt_mean_yaw = gt_list[len(gt_list) // 2][1]
        if agg.ok:
            import math as _m
            e_xy = float(np.linalg.norm(agg.center_world - gt_mean_c) * 1000)
            e_yaw = _m.degrees(((agg.yaw_rad - gt_mean_yaw + _m.pi) % (2 * _m.pi)) - _m.pi)
            print(f"   -> 지터 {jx:+.2f} 집계: 검출 {agg.n_used}/{agg.n_total}  "
                  f"xy 오차 {e_xy:.3f} mm  yaw 오차 {e_yaw:+.4f}°  "
                  f"pos_std {agg.pos_std_mm.round(3).tolist()} mm  yaw_std {agg.yaw_std_deg:.4f}°")
            per_jitter_results.append(dict(jitter=jx, **agg.as_dict(),
                                           gt_center=gt_mean_c.round(5).tolist(),
                                           xy_err_mm=round(e_xy, 4), yaw_err_deg=round(e_yaw, 4)))
        else:
            print(f"   -> 지터 {jx:+.2f} 집계 실패: {agg.reason}")
            per_jitter_results.append(dict(jitter=jx, ok=False, reason=agg.reason))

    print()
    print("=" * 88)
    print("  전체 결과 (매 프레임, 지터 통합)")
    print("=" * 88)
    ok_obs = [(o, g) for o, g in zip(all_obs, all_gt) if o.ok]
    summary = {}
    if ok_obs:
        import math as _m
        pos_err = np.array([np.linalg.norm(o.center_world - g[0]) * 1000 for o, g in ok_obs])
        yaw_err = np.array([_m.degrees(((o.yaw_rad - g[1] + _m.pi) % (2 * _m.pi)) - _m.pi)
                            for o, g in ok_obs])
        print(f"   프레임별(단일 관측)  표본 {len(ok_obs)}/{len(all_obs)}")
        print(f"      위치오차   평균 {pos_err.mean():7.3f} mm   최대 {pos_err.max():7.3f} mm")
        print(f"      yaw 오차   평균 {yaw_err.mean():+7.4f}°   최대(abs) {np.abs(yaw_err).max():7.4f}°")
        print(f"      반복편차   yaw 1σ {yaw_err.std():6.4f}°")

        agg_ok = [r for r in per_jitter_results if r.get("ok")]
        if agg_ok:
            axy = np.array([r["xy_err_mm"] for r in agg_ok])
            ayw = np.array([r["yaw_err_deg"] for r in agg_ok])
            print(f"\n   지터별 집계(다중 프레임 평균)  표본 {len(agg_ok)}/{len(per_jitter_results)}")
            print(f"      위치오차   평균 {axy.mean():7.3f} mm   최대 {axy.max():7.3f} mm")
            print(f"      yaw 오차   평균 {ayw.mean():+7.4f}°   최대(abs) {np.abs(ayw).max():7.4f}°")
            summary["aggregated"] = dict(
                n=len(agg_ok), pos_err_mm_mean=round(float(axy.mean()), 4),
                pos_err_mm_max=round(float(axy.max()), 4),
                yaw_err_deg_mean=round(float(ayw.mean()), 4),
                yaw_err_deg_absmax=round(float(np.abs(ayw).max()), 4))
        summary["per_frame"] = dict(
            n=len(ok_obs), n_total=len(all_obs),
            pos_err_mm_mean=round(float(pos_err.mean()), 4),
            pos_err_mm_max=round(float(pos_err.max()), 4),
            yaw_err_deg_mean=round(float(yaw_err.mean()), 4),
            yaw_err_deg_absmax=round(float(np.abs(yaw_err).max()), 4),
            yaw_repeat_1sigma_deg=round(float(yaw_err.std()), 4))

    print()
    print("   완료 기준 (3-2): 위치 3 mm, yaw 1 도 이내 (반복편차 동일 수준)")
    for key in ("per_frame", "aggregated"):
        if key not in summary:
            continue
        s = summary[key]
        ok = s["pos_err_mm_max"] <= 3.0 and s["yaw_err_deg_absmax"] <= 1.0
        print(f"      {key:12s} -> {'통과' if ok else '미달'}  "
              f"(위치 최대 {s['pos_err_mm_max']} mm, yaw 최대 {s['yaw_err_deg_absmax']}°)")

    OUT_YAML.write_text(yaml.safe_dump(
        dict(pose=POSE_NAME, target=f"{TARGET_KEY}:{LABEL}", depth_noise_mm=DEPTH_NOISE_M * 1000,
             summary=summary, per_jitter=per_jitter_results),
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

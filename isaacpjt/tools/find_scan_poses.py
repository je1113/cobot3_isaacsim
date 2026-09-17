"""
스캔 자세 / 근거리 관측 자세 관절값 산출.

    isaac_python isaacpjt/tools/find_scan_poses.py

두 자세를 뭐라고 정의하나:
  스캔 자세      선반 한 단을 멀리서 훑어 QR 이 어디 있는지 찾는 자세.
                 QR 이 화면에서 최소 SCAN_MIN_PX 픽셀은 돼야 검출된다.
  근거리 관측    특정 매거진의 QR 을 확실히 디코드하고 자세를 추정하는 자세.
                 CLOSE_MIN_PX 픽셀 이상.

거리는 눈대중이 아니라 실제 K 에서 역산한다.
    화면상 QR 크기(px) = fx * QR한변(m) / 거리(m)
fx 는 src/cobot3_bringup/config/frames.yaml 의 camera_info_observed 값
(= 실제 /wrist_camera/color/camera_info 에서 받은 것)을 쓴다.

카메라 pose 를 잡고 나면 link_6(=m0609_tool0) pose 로 바꿔 Lula IK 에 넣는다.
카메라와 link_6 사이 변환은 frames.yaml 의 m0609_tool0__camera_link 다.

출력은 관절값(deg)과, 그 자세에서 화면에 들어오는 매거진 목록이다.

이 도구의 역할 —
  거리/화각 계산과 도달 가능 여부 확인까지다. 여기서 나오는 관절값을 그대로
  쓰지 마라. Lula IK 는 자기 충돌만 보고 선반/매거진은 안 보며, 해가 여러 개인
  자세에서 팔꿈치가 뒤집힌 해를 고르기도 한다. 실제로 쓸 관절값은 GUI 에서
  직접 티칭해 capture_pose.py 로 뜨는 게 맞다.
"""

import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys
from pathlib import Path

import numpy as np
import yaml

from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

sys.stdout.reconfigure(line_buffering=True)


THIS_DIR     = Path(__file__).resolve().parent
ISAACPJT_DIR = THIS_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

URDF_PATH        = str(ISAACPJT_DIR / "M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(ISAACPJT_DIR / "M0609/descriptor/m0609_description.yaml")
FRAMES_YAML      = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
MEASURED_YAML    = THIS_DIR / "out/layout_measured.yaml"

EE_FRAME = "link_6"          # ROS 이름으로는 m0609_tool0

# QR 이 화면에서 이만큼은 돼야 한다는 기준
SCAN_MIN_PX  = 60.0          # 검출만
CLOSE_MIN_PX = 120.0         # 디코드 + 자세추정

# 베이스 waypoint — 12_place_test.py 가 IK 스윕으로 검증한 WP_PICK 을 그대로 쓴다.
# shelf_2 쪽은 통로를 사이에 두고 대칭이라 같은 이격거리로 미러링했다.
CARTER_Z = 0.07963398335074101
# shelf_1 은 통로가 -y 쪽이라 베이스가 선반 전면(y=1.8667)보다 0.4167 m 앞
# (작은 y)에 선다. shelf_2 는 통로가 +y 쪽이라 부호가 반대다 —
# 전면 y=-2.0592 에서 같은 거리만큼 +y 로 나와야 한다.
BASE_WAYPOINTS = {
    "shelf_1": dict(xy=(-6.5,  1.4500), yaw_deg=0.0),
    "shelf_2": dict(xy=(-6.5, -1.6425), yaw_deg=0.0),
}


# ══════════════════════════════════════════════════════════════
#  회전 유틸
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


def mat_to_quat_wxyz(m):
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def yaw_quat_wxyz(deg):
    h = np.radians(deg) / 2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def optical_frame_looking_along(view_dir):
    """광학 프레임 회전행렬을 만든다.

    ROS 광학 규약: +Z 앞, +X 오른쪽, +Y 아래.
    view_dir 을 +Z 로 삼고, 화면 아래쪽이 월드 아래(-Z)가 되게 맞춘다.
    """
    z = np.array(view_dir, dtype=float)
    z /= np.linalg.norm(z)
    down = np.array([0.0, 0.0, -1.0])
    x = np.cross(down, z)                 # 오른쪽 = 아래 x 앞
    if np.linalg.norm(x) < 1e-6:          # 수직으로 내려다보는 경우
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


# ══════════════════════════════════════════════════════════════
def main():
    cfg = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    if not MEASURED_YAML.exists():
        sys.exit(f"{MEASURED_YAML} 가 없다. measure_layout.py 를 먼저 돌려라.")
    meas = yaml.safe_load(MEASURED_YAML.read_text(encoding="utf-8"))

    K = cfg["wrist_camera"]["camera_info_observed"]
    fx, W, H = K["k"][0], K["width"], K["height"]
    qr_side = cfg["layout"]["magazine_1_orange"]["qr_side_m"]

    d_scan  = fx * qr_side / SCAN_MIN_PX
    d_close = fx * qr_side / CLOSE_MIN_PX
    hfov = 2 * np.degrees(np.arctan(W / (2 * fx)))
    vfov = 2 * np.degrees(np.arctan(H / (2 * fx)))

    print("=" * 72)
    print("  스캔 / 근거리 관측 자세")
    print("=" * 72)
    print(f"   K          fx = {fx:.2f}  ({W}x{H}, camera_info 실측)")
    print(f"   화각       H {hfov:.1f}도   V {vfov:.1f}도")
    print(f"   QR 한 변   {qr_side*1000:.0f} mm")
    print(f"   스캔 거리  {d_scan:.3f} m   (QR 이 {SCAN_MIN_PX:.0f} px)")
    print(f"   근거리     {d_close:.3f} m   (QR 이 {CLOSE_MIN_PX:.0f} px)")
    print(f"   가로 커버  스캔 {2*d_scan*np.tan(np.radians(hfov/2)):.3f} m   "
          f"근거리 {2*d_close*np.tan(np.radians(hfov/2)):.3f} m")
    print()

    # link_6 -> camera_link, 그리고 camera_link -> optical
    t_cam = cfg["static_transforms"]["m0609_tool0__camera_link"]
    R_l6_cam = quat_xyzw_to_mat(t_cam["quat_xyzw"])
    p_l6_cam = np.array(t_cam["xyz"])
    t_opt = cfg["static_transforms"]["camera_link__camera_color_optical_frame"]
    R_cam_opt = quat_xyzw_to_mat(t_opt["quat_xyzw"])
    R_l6_opt = R_l6_cam @ R_cam_opt
    p_l6_opt = p_l6_cam            # 두 프레임은 같은 점에 있다

    lula = LulaKinematicsSolver(robot_description_path=DESCRIPTION_PATH,
                                urdf_path=URDF_PATH)
    arm_off = np.array(cfg["static_transforms"]["base_link__m0609_base_link"]["xyz"])

    # 매거진을 선반/단별로 묶어 둔다
    groups = {}
    for key, m in meas["magazines"].items():
        shelf_group, level, name = key.split("/")
        shelf = "shelf_1" if shelf_group == "shelf_1" else "shelf_2"
        groups.setdefault((shelf, level), []).append((name, m))

    results = {}
    for (shelf, level), mags in sorted(groups.items()):
        wp = BASE_WAYPOINTS[shelf]
        base_pos = np.array([wp["xy"][0], wp["xy"][1], CARTER_Z])
        base_quat = yaw_quat_wxyz(wp["yaw_deg"])
        # Lula 는 팔 베이스(base_link) 기준이다. 카터 base_link 에 장착 오프셋을 더한다.
        arm_pos = base_pos + quat_xyzw_to_mat(
            [base_quat[1], base_quat[2], base_quat[3], base_quat[0]]) @ arm_off
        lula.set_robot_base_pose(robot_position=arm_pos, robot_orientation=base_quat)

        # 통로를 향한 QR 라벨을 고른다 (법선의 y 부호로 판별)
        aisle_dir = -1.0 if shelf == "shelf_1" else 1.0   # 통로가 놓인 방향
        qrs = []
        for name, m in mags:
            for lab, q in m.get("qr", {}).items():
                if np.sign(q["normal_world"][1]) == aisle_dir:
                    qrs.append((name, np.array(q["center_world"]),
                                np.array(q["normal_world"])))
        if not qrs:
            continue

        centers = np.array([c for _, c, _ in qrs])
        qr_plane_y = centers[:, 1].mean()
        qr_z = centers[:, 2].mean()
        view_dir = np.array([0.0, -aisle_dir, 0.0])      # 카메라가 보는 방향 (선반 쪽)

        print("-" * 72)
        print(f"  {shelf} / {level}   베이스 {tuple(np.round(base_pos, 3))} "
              f"yaw {wp['yaw_deg']:.0f}도")
        print(f"   QR 평면 y = {qr_plane_y:+.4f}   QR 높이 z = {qr_z:.4f}   "
              f"(라벨 {len(qrs)} 개)")

        for mode, dist, min_px in (("스캔", d_scan, SCAN_MIN_PX),
                                   ("근거리", d_close, CLOSE_MIN_PX)):
            # 카메라는 베이스 바로 앞, QR 평면에서 dist 만큼 떨어진 곳
            cam_pos = np.array([arm_pos[0],
                                qr_plane_y + aisle_dir * dist,
                                qr_z])
            R_opt = optical_frame_looking_along(view_dir)

            # 광학 프레임 pose -> link_6 pose
            R_l6 = R_opt @ R_l6_opt.T
            p_l6 = cam_pos - R_l6 @ p_l6_opt
            q_l6 = mat_to_quat_wxyz(R_l6)

            # LulaKinematicsSolver 는 ArticulationKinematicsSolver 와 달리
            # (joint_positions: np.ndarray, success: bool) 튜플을 돌려준다.
            q_sol, ok = lula.compute_inverse_kinematics(
                frame_name=EE_FRAME,
                target_position=p_l6,
                target_orientation=q_l6,
                warm_start=np.zeros(6),
            )
            if ok:
                q = np.degrees(np.asarray(q_sol, dtype=float))
                fk_p, fk_R = lula.compute_forward_kinematics(EE_FRAME, q_sol)
                err = np.linalg.norm(np.array(fk_p) - p_l6) * 1000
                joints = "[" + ", ".join(f"{v:7.2f}" for v in q) + "]"
                print(f"   {mode:4s} d={dist:.3f} m  카메라 {np.round(cam_pos,3).tolist()}"
                      f"  -> SOLVED  FK오차 {err:.3f} mm")
                print(f"        관절(deg) {joints}")
            else:
                print(f"   {mode:4s} d={dist:.3f} m  카메라 {np.round(cam_pos,3).tolist()}"
                      f"  -> FAILED (IK 미해)")
                q = None

            # 이 자세에서 화면에 들어오는 QR
            half_w = dist * np.tan(np.radians(hfov / 2))
            half_h = dist * np.tan(np.radians(vfov / 2))
            inside = [n for n, c, _ in qrs
                      if abs(c[0] - cam_pos[0]) <= half_w
                      and abs(c[2] - cam_pos[2]) <= half_h]
            print(f"        화면 안 QR {len(inside)}/{len(qrs)}: "
                  f"{', '.join(inside) if inside else '없음'}")

            results[f"{shelf}/{level}/{mode}"] = dict(
                base_xy=[float(v) for v in wp["xy"]], base_yaw_deg=wp["yaw_deg"],
                camera_xyz=[float(v) for v in cam_pos], distance_m=float(dist),
                joints_deg=None if q is None else [float(v) for v in q],
                qr_in_view=inside, qr_total=len(qrs),
            )
        print()

    out = THIS_DIR / "out/scan_poses.yaml"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(results, allow_unicode=True, sort_keys=False),
                   encoding="utf-8")
    print("=" * 72)
    solved = sum(1 for r in results.values() if r["joints_deg"] is not None)
    print(f"   {solved}/{len(results)} 자세가 IK 로 풀렸다")
    print(f"   기록: {out}")
    print()
    print("   주의 — Lula IK 는 자기 충돌만 본다. 선반/매거진과의 충돌은 안 본다.")
    print("   여기서 SOLVED 라고 나온 자세도 GUI 로 한 번 확인해야 한다.")


# kit 의 종료가 이 구성(스테이지를 안 연 채 lula 만 쓰는)에서 몇 분씩 매달린다.
# 결과는 이미 다 찍고 파일로도 남겼으니 그냥 끊는다. 예외는 먼저 보여준다.
import traceback

code = 0
try:
    main()
except Exception:
    traceback.print_exc()
    code = 1
finally:
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)

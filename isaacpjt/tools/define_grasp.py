"""
파지 자세 정의 T_QR_grasp — 실측 지오메트리에서 계산한다.

    python3 isaacpjt/tools/define_grasp.py          (Isaac 불필요)

무엇을 정하나:
  1. 흡착점    무게중심에 가까운 평평한 면 = flange_plate 윗면 중심
  2. 접근 축   그 면의 법선 (+Z, 위쪽) -> 툴은 반대로 내려온다
  3. QR 프레임 기준 오프셋 T_QR_grasp
  4. pre-grasp / 진입 깊이 / 들어올림 높이

QR 프레임 규약 (여기서 못박는다. 라이브러리 규약을 물려받지 않는다):
      x_qr = 라벨 이미지의 오른쪽
      y_qr = 라벨 이미지의 아래
      z_qr = x * y = 라벨 안쪽 (= -바깥법선)
  ROS 광학 프레임(x 오른쪽, y 아래, z 앞)과 같은 손 방향이라, 카메라가 라벨을
  정면으로 보면 두 프레임이 거의 겹친다. solvePnP 결과를 그대로 쓰기 좋다.

검증하는 것:
  - 흡착점이 QR 영역과 겹치지 않는지
  - 흡착 컵(Ø50)이 플랜지 위에 온전히 얹히는지 (짧은 변 기준 여유)
  - 매거진의 -Y 라벨과 +Y 라벨에서 T_QR_grasp 가 같은 값으로 나오는지
    (같아야 한다. 안 같으면 QR 프레임 정의나 에셋이 잘못된 것이다)
  - 라벨 텍스처가 좌우 반전돼 있지 않은지 (에셋의 points/st 직접 확인)
"""

import math
import re
import sys
from pathlib import Path

import numpy as np
import yaml

THIS_DIR  = Path(__file__).resolve().parent
ISAACPJT  = THIS_DIR.parent
WS_ROOT   = ISAACPJT.parent
MEASURED  = THIS_DIR / "out/layout_measured.yaml"
ASSET_DIR = ISAACPJT / "assets"
OUT_YAML  = WS_ROOT / "src/cobot3_bringup/config/grasp.yaml"

CUP_D = 0.050          # 흡착 컵 지름 (gripper_tip 실측)

# 접근/진입/들어올림 — 전부 12_pick_test.py 에서 실제로 돌려 검증된 값이다.
# 새로 지어내지 않고 그 스크립트의 상수를 그대로 가져온다.
PRE_GRASP_M   = 0.150      # = APPROACH_HEIGHT_OFFSET.
                           # 원래 0.25 였는데 WP_PICK 에서 IK 여유가 안 나와 0.15 로 낮춘 값이다.
APPROACH_SLOW_M = 0.030    # 이 지점부터 저속 + 접촉 감시
GRIP_GAPS_M   = [0.005, 0.002, 0.000, -0.003]   # 재시도 사다리 (아래 주석 참고)
LIFT_M        = 0.100      # = LIFT_HEIGHT_OFFSET.
                           # 0.23 으로 두면 LIFT 목표가 IK 경계 밖이라 절반쯤 실패한다.

# 플랜지 상방 관측 — QR 은 ID 와 대략 위치만, 파지 목표 xy / 윗면 z / yaw 는
# 손목 카메라로 플랜지를 위에서 찍어 정한다.
# 검출 코드: src/cobot3_perception/cobot3_perception/flange_topview.py
# 검증:      PICK_VISION=1 PICK_TRIALS=10 PICK_HEADLESS=1 isaac_python 12_pick_test.py
FLANGE_VISION = {
    "why": (
        "QR 라벨은 옆벽에 서 있어 매거진 yaw 가 '화면 밖으로 기우는' 회전이 된다. "
        "36 mm 코드를 273 mm 에서 보면 1도에 좌우 변 길이 차가 0.2 px 인데 "
        "cv2 꼭짓점 잔차가 RMS 2.1 px 라 yaw 가 1σ 2.3도로 흔들린다 "
        "(diag_qr_reproj.py, eval_qr_pose.py). 위에서 보면 yaw 는 화면 안 회전이고 "
        "80 mm 변 전체에 직선을 맞추며, 컵을 놓을 바로 그 면을 잰다."),
    # 카메라 광학 중심이 prior 윗면에서 이만큼 위
    "observe_cam_height_m": 0.25,
    # prior 가 화면의 (중앙 + 이 값) 픽셀에 오게 관측 자세를 잡는다.
    # 흡착 컵이 화면 아래쪽(광학 +y)을 가려서 플랜지를 위쪽에 둔다.
    "observe_image_offset_px": [0, -110],
    "settle_steps": 30,          # 관측 자세 도착 후 팔이 멎을 때까지
    "render_steps": 4,           # 캡처 전 렌더 (annotator 가 최신 프레임을 주게)
    "search_radius_m": 0.05,     # prior 주변 탐색 반경
    # 깊이로 윗면만 자른다. 다음 면(bridge 윗면)이 24 mm 아래이고 prior z 오차가
    # 수 mm 라 ±12 mm 면 양쪽 다 여유가 있다. 색은 깊이가 없을 때만 쓴다.
    "use_depth": True,
    "depth_band_m": 0.012,
    "size_tol": 0.10,            # 변 길이 허용 비율
    "min_fill": 0.90,            # 성분 면적 / 맞춘 사각형 면적
    "max_depth_correction_m": 0.015,
}


def parse_label_uv(asset_path, label_name):
    """에셋에서 라벨 메시의 points 와 st 를 읽어 u 축이 어느 월드 축인지 낸다.

    QR 이 좌우 반전돼 있으면 절대 디코드가 안 되므로, 규약을 못박기 전에
    에셋이 실제로 어떻게 생겼는지 확인한다.
    """
    text = asset_path.read_text(encoding="utf-8")
    m = re.search(rf'def Mesh "{label_name}".*?\n}}', text, re.S)
    if not m:
        return None
    blk = m.group(0)
    pts = re.search(r"point3f\[\] points = \[(.*?)\]\s*\n", blk, re.S)
    st  = re.search(r"texCoord2f\[\] primvars:st = \[(.*?)\]", blk, re.S)
    if not pts or not st:
        return None
    P = [tuple(float(v) for v in t.split(","))
         for t in re.findall(r"\(([^)]+)\)", pts.group(1))]
    S = [tuple(float(v) for v in t.split(","))
         for t in re.findall(r"\(([^)]+)\)", st.group(1))]
    P, S = np.array(P), np.array(S)
    # u 가 1 만큼 늘 때 로컬 좌표가 어느 방향으로 가나
    i0 = int(np.argmin(S[:, 0] + S[:, 1]))          # st (0,0)
    i1 = int(np.argmin(np.abs(S[:, 0] - 1) + S[:, 1]))  # st (1,0)
    i3 = int(np.argmin(S[:, 0] + np.abs(S[:, 1] - 1)))  # st (0,1)
    du = P[i1] - P[i0]
    dv = P[i3] - P[i0]
    return dict(u_dir=du / np.linalg.norm(du), v_dir=dv / np.linalg.norm(dv),
                side=float(np.linalg.norm(du)))


def qr_frame(normal_world):
    """바깥 법선에서 QR 프레임 축을 만든다 (월드 기준).

    매거진이 전부 똑바로 서 있고(orient 단위) 라벨이 수직 벽면이라,
    이미지 아래 = 월드 -Z 다.
    """
    z = -np.asarray(normal_world, dtype=float)      # 라벨 안쪽
    z = z / np.linalg.norm(z)
    y = np.array([0.0, 0.0, -1.0])                  # 이미지 아래
    x = np.cross(y, z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, y, z])               # 열이 x,y,z 축


def main():
    if not MEASURED.exists():
        sys.exit(f"{MEASURED} 가 없다. measure_layout.py 를 먼저 돌려라.")
    meas = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))

    print("=" * 78)
    print("  파지 자세 정의  T_QR_grasp")
    print("=" * 78)

    # ── 0. 에셋의 라벨 텍스처 방향 확인 ───────────────────────────
    print("\n[0] 라벨 텍스처 방향 (좌우 반전 여부)")
    for asset in ("magazine_1_orange_qr.usda", "magazine_2_blue_qr.usda"):
        for lab in ("qr_label_ny", "qr_label_py"):
            uv = parse_label_uv(ASSET_DIR / asset, lab)
            if uv is None:
                print(f"   {asset}:{lab}  읽기 실패")
                continue
            # 이 라벨을 통로쪽에서 볼 때 관측자의 오른쪽
            n = np.array([0.0, -1.0, 0.0]) if lab.endswith("ny") else np.array([0.0, 1.0, 0.0])
            fwd = -n                                     # 관측자의 시선
            right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
            ok = float(np.dot(uv["u_dir"], right)) > 0.99
            print(f"   {asset.split('_qr')[0]:18s} {lab}  u축 {uv['u_dir'].round(2).tolist()}  "
                  f"관측자 오른쪽 {right.round(2).tolist()}  -> "
                  f"{'정상' if ok else '좌우 반전!'}   한 변 {uv['side']*1000:.0f} mm")

    # ── 1~3. 매거진 종류별 T_QR_grasp ─────────────────────────────
    print("\n[1] 흡착점 / 접근축 / T_QR_grasp")
    seen, results = {}, {}
    for key, m in sorted(meas["magazines"].items()):
        kind = key.rsplit("/", 1)[1].replace("_02", "")
        if kind in seen:
            continue
        seen[kind] = True

        fl_c = np.array(m["flange_center"])
        fl_s = np.array(m["flange_size"])
        grasp = np.array(m["suction_point"])          # 플랜지 윗면 중심
        short_side = float(min(fl_s[0], fl_s[1]))
        margin = (short_side - CUP_D) / 2.0

        print(f"\n   [{kind}]")
        print(f"      흡착점(월드)   {grasp.round(4).tolist()}   = 플랜지 윗면 중심")
        print(f"      플랜지         {fl_s[0]*1000:.0f} x {fl_s[1]*1000:.0f} x "
              f"{fl_s[2]*1000:.0f} mm")
        print(f"      접근 축        -Z (플랜지 법선 +Z 의 반대). 툴 +Z 가 아래를 향한다")
        print(f"      컵 여유        짧은변 {short_side*1000:.0f} - 컵 {CUP_D*1000:.0f} "
              f"-> 편측 {margin*1000:+.1f} mm"
              + ("   << 부족" if margin < 0.010 else ""))

        per_label = {}
        for lab, q in sorted(m["qr"].items()):
            c = np.array(q["center_world"])
            R = qr_frame(q["normal_world"])
            d = grasp - c
            t = R.T @ d                                # QR 프레임 성분

            # 접근 축(월드 -Z)을 QR 프레임으로
            approach_qr = R.T @ np.array([0.0, 0.0, -1.0])

            # 흡착점과 QR 영역이 겹치나 — QR 평면에 투영해서 본다
            side = q["side_m"]
            in_plane = np.hypot(t[0], t[1])
            overlap = abs(t[0]) <= side / 2 and abs(t[1]) <= side / 2
            per_label[lab] = dict(
                t=t, approach=approach_qr, overlap=overlap, in_plane=in_plane,
                qr_center=c)
            print(f"      {lab}  T_QR_grasp xyz "
                  f"[{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}]  "
                  f"접근축 {approach_qr.round(3).tolist()}")
            print(f"      {'':14s} 해석: 오른쪽 {t[0]*1000:+.1f} mm, "
                  f"{'위' if t[1] < 0 else '아래'} {abs(t[1])*1000:.1f} mm, "
                  f"안쪽 {t[2]*1000:+.1f} mm")
            print(f"      {'':14s} QR 영역과 겹침: {'그렇다 <<문제' if overlap else '아니다'}"
                  f"  (평면상 거리 {in_plane*1000:.1f} mm, QR 반변 {side/2*1000:.1f} mm)")

        labs = list(per_label)
        if len(labs) == 2:
            diff = np.abs(per_label[labs[0]]["t"] - per_label[labs[1]]["t"]).max()
            print(f"      두 라벨 일치   최대 차 {diff*1000:.3f} mm  "
                  f"{'OK' if diff < 1e-4 else '<< 불일치'}")

        results[kind] = dict(
            color="orange" if "orange" in kind else "blue",   # flange_topview.HSV_RANGES 키
            flange_size=[float(v) for v in fl_s],
            cup_margin_m=round(margin, 5),
            T_QR_grasp_xyz=[round(float(v), 5) for v in per_label[labs[0]]["t"]],
            approach_axis_in_qr=[round(float(v), 4)
                                 for v in per_label[labs[0]]["approach"]],
            qr_overlap=bool(per_label[labs[0]]["overlap"]),
            labels_agree_mm=round(float(diff * 1000), 4) if len(labs) == 2 else None,
        )

    # ── 4. 접근 파라미터 ──────────────────────────────────────────
    print("\n[2] 접근 / 진입 / 들어올림")
    print(f"      pre-grasp      접근축 전방 {PRE_GRASP_M*1000:.0f} mm")
    print(f"      감속 시작      {APPROACH_SLOW_M*1000:.0f} mm 앞")
    print(f"      진입 깊이      {[f'{g*1000:+.0f}' for g in GRIP_GAPS_M]} mm "
          f"(순서대로 재시도, 음수=누름)")
    print(f"      들어올림       {LIFT_M*1000:.0f} mm")

    OUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    doc = {
        "meta": {
            "generated_by": "isaacpjt/tools/define_grasp.py",
            "source": "isaacpjt/tools/out/layout_measured.yaml",
            "units": "m, rad",
        },
        "qr_frame_convention": {
            "x": "라벨 이미지의 오른쪽",
            "y": "라벨 이미지의 아래",
            "z": "x cross y = 라벨 안쪽 (= -바깥법선)",
            "note": "ROS 광학 프레임과 같은 손 방향. solvePnP 결과를 그대로 쓸 수 있다.",
        },
        "suction_point": "flange_plate 윗면 중심",
        "approach_axis_world": [0.0, 0.0, -1.0],
        "cup_diameter_m": CUP_D,
        "pre_grasp_m": PRE_GRASP_M,
        "approach_slow_from_m": APPROACH_SLOW_M,
        "grip_gaps_m": GRIP_GAPS_M,
        "grip_gaps_note": (
            "첫 칸(+5 mm)은 주황에서 한 번도 안 붙는다. 그래서 헛도는 줄 알고 빼 봤더니 "
            "주황이 10/10 -> 2/10 으로 무너졌다 (파랑은 9/10 유지). "
            "즉 +5 는 흡착을 노리는 칸이 아니라 단계 역할을 한다 — 실패한 +5 시도가 "
            "접근고도(145 mm)에서 곧장 내려오는 대신 3 mm 만 더 내려가게 만들고, "
            "그 사이 물리가 안정된다. 빼지 마라."),
        "lift_m": LIFT_M,
        "flange_vision": FLANGE_VISION,
        "verified": {
            "tool": "PICK_TRIALS=10 PICK_HEADLESS=1 [PICK_TARGET=...] "
                    "isaac_python isaacpjt/M0609/lula_ik/12_pick_test.py",
            "magazine_1_orange": {
                "result": "10 / 10 성공",
                "cycle_time_s": {"mean": 2.04, "min": 1.99, "max": 2.06},
                "lift_rise_mm_min": 88.6, "tilt_deg_max": 0.10,
            },
            "magazine_2_blue": {
                "result": "10 / 10 성공  (플랜지를 80x80 으로 고친 뒤)",
                "cycle_time_s": {"mean": 2.04, "min": 2.01, "max": 2.06},
                "lift_rise_mm_min": 88.4, "tilt_deg_max": 0.10,
                "base_x": -5.9233,
            },
            "flange_vision": {
                "tool": "PICK_VISION=1 PICK_TRIALS=10 PICK_HEADLESS=1 [PICK_DEPTH_NOISE_MM=2] "
                        "isaac_python isaacpjt/M0609/lula_ik/12_pick_test.py",
                "setup": "매거진 ±10 mm / ±5도 교란, prior(QR 대역) 오차 ≤5 mm / ±7도, 관측 높이 250 mm",
                "measurement_2026_09_17": {
                    "orange":         {"detected": "10/10", "xy_mm_max": 0.28, "z_mm_max": 0.00, "yaw_deg_max": 0.018},
                    "orange_noise2mm": {"detected": "10/10", "xy_mm_max": 0.29, "z_mm_max": 0.02, "yaw_deg_max": 0.011},
                    "blue":           {"detected": "10/10", "xy_mm_max": 0.27, "z_mm_max": 0.00, "yaw_deg_max": 0.032},
                    "blue_noise2mm":  {"detected": "10/10", "xy_mm_max": 0.27, "z_mm_max": 0.02, "yaw_deg_max": 0.029},
                },
                "pick_result": "주황 9/10, 파랑 10/10. 주황 실패 1 건(trial 8)은 인식 오차 0.26 mm 였고 "
                               "PICK_MAG_DELTA=6.65,5.74,-2.61 로 GT 파지를 해도 3/3 실패한다 — 인식 문제가 아니다. "
                               "frames.yaml open_issues pick-magazine-pushed-on-descent 참고.",
                "caveat": "시뮬 깊이는 가장자리가 칼같고, 가우시안 노이즈는 실물의 flying pixel / 가장자리 "
                          "번짐을 흉내 내지 못한다. xy 0.3 mm 는 실물에서 그대로 기대할 수 없다.",
            },
            "criteria": "rise >= 5 mm, tilt <= 5 deg",
            "note": "GT 플랜지 pose 로 파지한다. QR 인식 오차는 아직 안 들어가 있다.",
            "harness_fix": (
                "반복 실행이 되게 하려고 12_pick_test.py 를 두 군데 고쳤다. "
                "(1) 매거진 리셋이 USD xform 쓰기라 시뮬 중에는 먹지 않았다 — "
                "SingleRigidPrim 으로 옮기고 속도까지 턴다. "
                "(2) 리셋을 베이스 텔레포트/팔 스냅보다 먼저 하고 있어서 그 뒤에 "
                "다시 흔들렸다. y 진폭이 7->11->21->30->60->100 mm 로 커져 "
                "IK 가 안 풀리는 트라이얼이 나왔다. 로봇을 다 세운 뒤 맨 마지막에 "
                "매거진을 놓도록 순서를 바꿨고, 지금은 10 회가 전부 같은 자리에서 시작한다."),
        },
        "magazines": results,
    }
    OUT_YAML.write_text(yaml.safe_dump(doc, allow_unicode=True, sort_keys=False),
                        encoding="utf-8")
    print(f"\n   기록: {OUT_YAML}")


if __name__ == "__main__":
    main()

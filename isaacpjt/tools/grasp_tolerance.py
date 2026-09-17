"""
허용 오차 기준 산출 — 실측 지오메트리에서 역산한다.

    python3 isaacpjt/tools/grasp_tolerance.py

입력은 measure_layout.py 가 떨군 isaacpjt/tools/out/layout_measured.yaml 이다.
Isaac 을 띄울 필요는 없다.

무엇을 계산하나:
  흡착 컵(Ø50 mm)이 매거진 플랜지 위에 온전히 얹히려면 컵 중심이 플랜지
  중심에서 얼마나 벗어나도 되는지가 위치 허용오차의 상한이다. 플랜지가
  정사각이 아니면 짧은 변이 상한을 정한다.

      허용 횡오차 = (플랜지 짧은 변 - 컵 지름) / 2

  그리고 매거진 pose 를 QR 로 추정하는 이상, yaw 오차는 QR 에서 플랜지까지의
  레버암을 타고 횡오차로 바뀐다.

      yaw 오차가 만드는 횡오차 = r * yaw_err,  r = |QR중심 - 플랜지중심| (수평)

  이 둘에 캘리브레이션 오차와 제어 추종 오차를 더한 값이 허용 횡오차 안에
  들어와야 한다. 그래서 "위치 2 cm, yaw 5도" 같은 값을 먼저 정하는 게 아니라
  예산을 나눠 갖는 식으로 정해야 한다.
"""

import math
import sys
from pathlib import Path

import yaml


THIS_DIR = Path(__file__).resolve().parent
MEASURED = THIS_DIR / "out/layout_measured.yaml"

# 예산에서 미리 떼어 두는 몫 (checklist 의 완료 기준 및 경험값)
CAL_ERR_M    = 0.005   # TF/캘리브레이션 — checklist "완료 기준: 5 mm"
CTRL_ERR_M   = 0.003   # IK 해 + 관절 추종 오차. 12_place_test 의 배치오차 판정이 2 mm 라
                       # 실제로는 이보다 작지만, 선반 앞 자세는 더 불리해서 3 mm 로 잡는다.
SAFETY       = 0.80    # 남은 몫을 다 쓰지 않고 80 %만 쓴다


def horizontal_lever(qr_center, flange_center):
    dx = qr_center[0] - flange_center[0]
    dy = qr_center[1] - flange_center[1]
    return math.hypot(dx, dy)


def main():
    if not MEASURED.exists():
        sys.exit(f"{MEASURED} 가 없다. 먼저 isaac_python isaacpjt/tools/measure_layout.py 를 돌려라.")
    d = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))

    cup = d["gripper"]["gripper_tip"]["cup_diameter_m"]
    print("=" * 72)
    print("  허용 오차 기준 산출")
    print("=" * 72)
    print(f"흡착 컵 지름  {cup*1000:.1f} mm  (short_gripper/gripper_tip 실측)")
    print()

    # 매거진 종류별로 한 번씩만 본다 (같은 에셋이 여러 번 배치돼 있다)
    seen = {}
    for key, m in d["magazines"].items():
        name = key.rsplit("/", 1)[1].replace("_02", "")
        if name in seen:
            continue
        seen[name] = m

    rows = []
    for name, m in sorted(seen.items()):
        fx, fy, _ = m["flange_size"]
        short_side = min(fx, fy)
        lateral_budget = (short_side - cup) / 2.0

        levers = {}
        for lab, q in m.get("qr", {}).items():
            levers[lab] = horizontal_lever(q["center_world"], m["flange_center"])
        r = max(levers.values()) if levers else 0.0

        # 예산 배분: 전체 - 캘리브 - 제어 = 인식(위치+yaw) 몫
        remain = lateral_budget - CAL_ERR_M - CTRL_ERR_M
        usable = remain * SAFETY
        # 인식 몫을 위치/yaw 가 절반씩 나눠 갖는다
        pos_tol = usable / 2.0
        yaw_tol = (usable / 2.0) / r if r > 1e-6 else float("inf")

        rows.append(dict(
            name=name, flange=(fx, fy), short_side=short_side,
            lateral_budget=lateral_budget, lever=r, remain=remain,
            pos_tol=pos_tol, yaw_tol_rad=yaw_tol,
        ))

        print(f"[{name}]")
        print(f"   플랜지            {fx*1000:.0f} x {fy*1000:.0f} mm  (짧은 변 {short_side*1000:.0f} mm)")
        print(f"   허용 횡오차 상한  ({short_side*1000:.0f} - {cup*1000:.0f}) / 2 = "
              f"{lateral_budget*1000:.1f} mm")
        print(f"   QR->플랜지 레버암 {r*1000:.1f} mm")
        print(f"   빼는 몫           캘리브 {CAL_ERR_M*1000:.0f} mm + 제어 {CTRL_ERR_M*1000:.0f} mm")
        print(f"   인식에 남는 몫    {remain*1000:+.1f} mm  "
              f"(안전율 {SAFETY:.0%} 적용 후 {remain*SAFETY*1000:+.1f} mm)")
        if remain <= 0:
            print(f"   >>> 예산 초과. 이 매거진은 지금 컵/플랜지 조합으로는 "
                  f"캘리브 오차만으로도 흡착면을 벗어난다.")
        else:
            print(f"   => 위치 허용오차  ±{pos_tol*1000:.1f} mm")
            print(f"      yaw 허용오차   ±{math.degrees(yaw_tol):.2f}도  "
                  f"(레버암 {r*1000:.0f} mm 에서 {pos_tol*1000:.1f} mm 를 만드는 각도)")
        print()

    # 전체 기준 = 가장 빡빡한 매거진에 맞춘다
    ok = [r for r in rows if r["remain"] > 0]
    print("=" * 72)
    if not ok:
        print("  결론: 어떤 매거진도 현재 조합으로 기준을 세울 수 없다")
        return
    worst = min(ok, key=lambda r: r["pos_tol"])
    worst_yaw = min(ok, key=lambda r: r["yaw_tol_rad"])
    print("  결론 — 가장 빡빡한 매거진 기준으로 통일")
    print("=" * 72)
    print(f"   위치 허용오차   ±{worst['pos_tol']*1000:.0f} mm    (지배: {worst['name']}, "
          f"플랜지 짧은 변 {worst['short_side']*1000:.0f} mm)")
    print(f"   yaw 허용오차    ±{math.degrees(worst_yaw['yaw_tol_rad']):.1f}도   "
          f"(지배: {worst_yaw['name']}, 레버암 {worst_yaw['lever']*1000:.0f} mm)")
    print(f"   캘리브 허용오차 ±{CAL_ERR_M*1000:.0f} mm    (이미 예산에서 뺀 값)")
    print()
    print("   z 방향은 흡착 컵이 표면에 닿아야 하므로 SurfaceGripper 의")
    print("   maxGripDistance 가 상한이다. 아래 '흡착 조건' 참고.")
    print()

    # 처음 제안값과 비교
    print("=" * 72)
    print("  최초 제안값(위치 2 cm, yaw 5도) 검토")
    print("=" * 72)
    for r in rows:
        err_from_yaw = r["lever"] * math.radians(5.0)
        total = 0.020 + err_from_yaw + CAL_ERR_M + CTRL_ERR_M
        verdict = "OK" if total <= r["lateral_budget"] else "초과"
        print(f"   {r['name']:18s} 20.0 + yaw5도 {err_from_yaw*1000:5.1f} + "
              f"{CAL_ERR_M*1000:.0f} + {CTRL_ERR_M*1000:.0f} = {total*1000:5.1f} mm "
              f"vs 상한 {r['lateral_budget']*1000:4.1f} mm  -> {verdict}")
    print()
    print("   최초 제안값은 위치 항 하나만으로도 상한을 넘는다. 위 '결론' 값을 쓴다.")

    # 흡착 조건
    sg = d["gripper"].get("surface_gripper_prims", {})
    print()
    print("=" * 72)
    print("  유효 흡착 조건 (Isaac Sim Surface Gripper)")
    print("=" * 72)
    for path, info in sg.items():
        print(f"   에셋 기본값 — {path}")
        for k, v in sorted(info["attrs"].items()):
            print(f"       {k:30s} = {v}")
    print()
    print("   에셋 기본값은 coaxial/shear 가 0 이라 그대로 두면 아무것도 못 든다.")
    print("   런타임에 덮어써야 한다 (10_suction_magazine.py 가 하는 일).")


if __name__ == "__main__":
    main()

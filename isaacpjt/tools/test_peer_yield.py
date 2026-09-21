"""
task_manager 의 로더 차선 양보 판단 단위테스트 — Isaac · ROS 불필요.

    python3 isaacpjt/tools/test_peer_yield.py

두 로봇이 같은 로더로 갈 때 한 대만 차선에 들어가게 하는 판단이
task_manager.py 의 peer_busy() · peer_frozen() 에 들어 있다. py_trees 와 rclpy
없이 그 판단만 시험한다 — ast 로 실제 소스에서 상수와 메서드 본문을 꺼내
쓰므로 복사본이 아니라 원본을 돌린다. 소스를 고치면 이 테스트가 같이 따라간다.

확인하는 것
  1. 상대를 한 번도 못 받았으면 조율 없이 직행한다 (한 대만 띄웠을 때)
  2. 단계별 양보 판단이 양쪽 설정과 맞는다
  3. 소식이 끊겨도 마지막으로 들은 단계를 그대로 쓴다
  4. 상대가 차선 안에서 얼어붙으면 즉시 실패로 올리고, 차선 밖(대기 자리)에서
     얼어붙으면 양보를 푼다
  5. 둘 다 줄 서 있는 상태로 서로를 기다리는 교착 조합이 없다
  6. mission_nodes.launch.py 의 설정이 소스 기본값과 어긋나지 않는다

5번이 이 설계의 핵심 성질이다. 두 로봇이 같은 목록을 쓰므로, 줄 서 있는 상태
('approach' · 'wait')를 목록에 넣는 순간 양쪽이 서로의 대기를 기다려 아무도
움직이지 않는다. 그 둘이 목록 밖에 있다는 것이 교착 부재의 근거다.
"""

import ast
import io
import itertools
import sys
import time
from pathlib import Path

WS = Path(__file__).resolve().parent.parent.parent
TM = WS / "src/cobot3_orchestrator/cobot3_orchestrator/task_manager.py"
LAUNCH = WS / "src/cobot3_bringup/launch/mission_nodes.launch.py"

# 판단에 쓰는 메서드. 이 넷만 꺼낸다.
WANTED = {"_on_peer_state", "peer_busy", "peer_frozen", "peer_stage"}


def _top_level_consts(tree, extra=None):
    """최상위 단순 대입을 순서대로 eval 한다.

    뒤 항목이 앞 이름을 참조하므로(LOGGED_STAGES = (PICK, ...)) 순서가 중요하다.
    eval 이 안 되는 것(함수 호출 등)은 건너뛴다 — 필요한 건 리터럴뿐이다.
    """
    ns = {"__builtins__": __builtins__}
    ns.update(extra or {})
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                ns[node.targets[0].id] = eval(
                    compile(ast.Expression(node.value), "<const>", "eval"), ns)
            except Exception:
                pass
    return ns


def _extract_peer_class(tree, ns):
    """TaskManager 에서 판단 메서드만 떼어 빈 클래스에 붙인다."""
    methods = [m for node in tree.body
               if isinstance(node, ast.ClassDef) and node.name == "TaskManager"
               for m in node.body
               if isinstance(m, ast.FunctionDef) and m.name in WANTED]
    missing = WANTED - {m.name for m in methods}
    assert not missing, f"task_manager.py 에서 못 찾은 메서드: {missing}"
    cls = ast.ClassDef(name="Peer", bases=[], keywords=[], body=methods,
                       decorator_list=[])
    mod = ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[]))
    exec(compile(mod, "<extracted>", "exec"), ns)
    return ns["Peer"]


class _Msg:
    """std_msgs/String 대역. _on_peer_state 는 .data 만 본다."""

    def __init__(self, data):
        self.data = data


def main():
    tm_tree = ast.parse(io.open(TM, encoding="utf-8").read())
    ns = _top_level_consts(tm_tree, {"time": time})
    Peer = _extract_peer_class(tm_tree, ns)

    WAIT = ns["WAIT"]
    lane_like = [ns["NAV"], ns["PUSH"], ns["PLACE"], ns["RETURN"]]
    all_stages = [ns["PATROL"], ns["HOLD"], ns["SCAN"], ns["PICK"],
                  ns["APPROACH"], WAIT] + lane_like

    launch_ns = _top_level_consts(ast.parse(io.open(LAUNCH, encoding="utf-8").read()))
    # 두 로봇이 같은 목록을 쓴다. 순서를 정하는 장치는 없다.
    busy_set = list(launch_ns["LANE_STAGES"])

    def make(busy_stages, seen=None, failed=False, age=0.0):
        """상대 상태를 주입한 판단기. seen=None 이면 한 번도 못 받은 상태."""
        p = Peer()
        p._peer_busy_stages = tuple(busy_stages)
        p._peer_stage, p._peer_failed, p._peer_last_rx = None, False, 0.0
        if seen is not None:
            parts = [f"state={seen}", "detected=False"]
            if failed:
                parts += ["FAILED", f"fail_stage={seen}", "fail_reason=X"]
            # _publish_state 가 실제로 만드는 문자열 형식 그대로 넣는다.
            p._on_peer_state(_Msg(" | ".join(parts)))
            p._peer_last_rx = time.monotonic() - age
        return p

    failures = []

    def check(cond, label):
        print(f"  {'PASS' if cond else 'FAIL'}  {label}")
        if not cond:
            failures.append(label)

    print("── 1. 상대를 한 번도 못 받았으면 직행 ──")
    check(make(busy_set).peer_busy()[0] is False, "미수신 -> 직행")

    print("\n── 2. 단계별 양보 판단 ──")
    for st in all_stages:
        got = make(busy_set, seen=st).peer_busy()[0]
        want = st in busy_set
        check(got == want, f"state={st:9s} 양보={got!s:5s} (기대 {want!s:5s})")

    print("\n── 3. 소식이 끊겨도 마지막 단계를 그대로 쓴다 ──")
    # 수신 시각으로 "묵었으면 양보" 하는 규칙은 없다. 위험한 방향은 마지막
    # 단계가 이미 막아 주고, 그 규칙을 두면 순찰 중에 죽은 상대 때문에
    # 살아 있는 로봇이 얼어붙는다.
    for age in (0.0, 30.0):
        check(make(busy_set, seen=ns["PATROL"], age=age).peer_busy()[0] is False,
              f"마지막이 patrol, {age:.0f}s 전 -> 직행")
        check(make(busy_set, seen=ns["PLACE"], age=age).peer_busy()[0] is True,
              f"마지막이 place,  {age:.0f}s 전 -> 양보")

    print("\n── 4. 상대가 얼어붙은 경우 ──")
    for st in lane_like:
        check(make(busy_set, seen=st, failed=True).peer_frozen() is True,
              f"차선({st}) 에서 정지 -> 즉시 실패")
    check(make(busy_set, seen=WAIT, failed=True).peer_frozen() is False,
          "대기 자리에서 정지 -> 실패 아님 (차선은 비었다)")
    check(make(busy_set, seen=WAIT, failed=True).peer_busy()[0] is False,
          "대기 자리에서 정지 -> 양보 안 함 (차선은 비었다)")
    check(make(busy_set, seen=ns["PICK"], failed=True).peer_frozen() is False,
          "차선 밖(pick) 에서 정지 -> 실패 아님")

    print("\n── 5. 교착 조합이 없어야 한다 ──")
    # 줄 서 있는 두 상태의 모든 조합에서, 양쪽이 동시에 양보하면 교착이다.
    queued = [ns["APPROACH"], WAIT]
    dead = [(a, b) for a, b in itertools.product(queued, repeat=2)
            if make(busy_set, seen=b).peer_busy()[0]
            and make(busy_set, seen=a).peer_busy()[0]]
    check(not dead, f"둘 다 줄 서 있는데 서로 양보하는 조합 없음 {dead}")

    print("\n── 6. launch 설정이 소스 기본값과 어긋나지 않는가 ──")
    check(busy_set == list(ns["DEFAULT_PEER_BUSY_STAGES"]),
          f"LANE_STAGES == DEFAULT_PEER_BUSY_STAGES {busy_set}")
    staging = launch_ns["STAGING_BY_ROBOT"]
    check(staging["robot1"] != staging["robot2"],
          f"대기 자리가 로봇별로 다름 {staging}")
    check(list(ns["DEFAULT_STAGING_POSE"]) in staging.values(),
          "소스 기본 대기자리가 launch 값 중 하나와 같다")
    check(WAIT not in busy_set and ns["APPROACH"] not in busy_set,
          "LANE_STAGES 에 wait · approach 가 없다 — 있으면 5번 교착이 생긴다")
    check(ns["RETURN"] in busy_set,
          "LANE_STAGES 에 return 이 있다 — 빼면 이탈하는 로봇과 정면으로 만난다")

    print("\n── 7. 거리 기반 진입 설정 ──")
    capped = list(ns["PEER_CAPPED_STAGES"])
    check(all(st in busy_set for st in capped),
          f"PEER_CAPPED_STAGES {capped} 가 전부 양보 목록 안에 있다 — "
          f"양보하지 않는 단계에 상한을 둬도 의미가 없다")
    check(float(ns["DEFAULT_LANE_CLEAR_DIST_M"]) > 0,
          f"lane_clear_dist_m 기본값 {ns['DEFAULT_LANE_CLEAR_DIST_M']} m > 0")
    src = io.open(LAUNCH, encoding="utf-8").read()
    check(("peer_pose_topic" in src) and ("amcl_pose" in src),
          "launch 가 peer_pose_topic 을 넘긴다 — 없으면 거리 판정을 못 하고 "
          "상대의 return 이 끝날 때까지 기다린다")

    print()
    if failures:
        print(f"실패 {len(failures)}건")
        return 1
    print("전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())

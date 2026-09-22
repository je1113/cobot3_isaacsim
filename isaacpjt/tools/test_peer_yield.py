"""
task_manager 의 로더 차선 양보·순서 판단 단위테스트 — Isaac · ROS 불필요.

    python3 isaacpjt/tools/test_peer_yield.py

로봇 두 대가 같은 로더로 갈 때 한 대만 차선에 들어가게 하는 판단이
task_manager.py 의 peer_busy() · peer_frozen() · peer_outranks_me() 에 들어
있다. py_trees 와 rclpy 없이 그 판단만 시험한다 — ast 로 실제 소스에서 상수와
메서드 본문을 꺼내 쓰므로 복사본이 아니라 원본을 돌린다. 소스를 고치면 이
테스트가 같이 따라간다.

확인하는 것
  1. 상대를 한 번도 못 받았으면 조율 없이 직행한다 (한 대만 띄웠을 때)
  2. 단계별 양보 판단이 양쪽 설정과 맞는다
  3. 소식이 끊겨도 마지막으로 들은 단계를 그대로 쓴다
  4. 상대가 차선 안에서 얼어붙으면 즉시 실패로 올리고, 차선 밖(대기 자리)에서
     얼어붙으면 양보를 푼다
  5. 둘 다 줄 서 있는 상태로 서로를 기다리는 교착 조합이 없다
  6. 순서(lane_priority)는 한쪽만 양보시킨다
  7. 반경 판정과 HOLD_BACK 의 단계 목록이 양보 목록과 맞는다
  8. mission_nodes.launch.py 의 표가 소스 기본값과 어긋나지 않는다

5번과 6번이 이 설계의 핵심 성질이다. 두 로봇이 같은 목록을 쓰므로, 줄 서 있는
상태('approach' · 'wait' · 'hold_back')를 목록에 넣는 순간 양쪽이 서로의 대기를
기다려 아무도 움직이지 않는다. 그 셋이 목록 밖에 있다는 것이 교착 부재의
근거이고, 순서는 전순서인 lane_priority 가 정한다.
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

# 판단에 쓰는 메서드. 이것들만 꺼낸다.
WANTED = {"_on_peer_state", "peer_busy", "peer_frozen", "peer_stage", "peer_seen",
          "peer_prio", "peer_outranks_me", "peer_contending"}


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

    WAIT, APPROACH, HOLD_BACK = ns["WAIT"], ns["APPROACH"], ns["HOLD_BACK"]
    queued = [APPROACH, WAIT, HOLD_BACK]      # 줄 서 있는 상태들
    lane_like = [ns["NAV"], ns["PUSH"], ns["PLACE"], ns["STACK_NAV"],
                 ns["STACK_SCAN"], ns["STACK_PICK"], ns["STACK_DELIVER"],
                 ns["STACK_PLACE"]]
    all_stages = [ns["START"], ns["SCAN"], ns["PICK"], ns["DOCK"],
                  ns["DONE"]] + queued + lane_like

    launch_ns = _top_level_consts(ast.parse(io.open(LAUNCH, encoding="utf-8").read()))
    # 두 로봇이 같은 목록을 쓴다.
    busy_set = list(launch_ns["LANE_STAGES"])

    def make(seen=None, failed=False, age=0.0, my_prio=0, peer_prio=0,
             busy_stages=None):
        """상대 상태를 주입한 판단기. seen=None 이면 한 번도 못 받은 상태."""
        p = Peer()
        p._peer_busy_stages = tuple(busy_set if busy_stages is None else busy_stages)
        p._peer_stage, p._peer_failed, p._peer_last_rx = None, False, 0.0
        p._peer_prio, p.lane_priority = 0, my_prio
        if seen is not None:
            parts = [f"state={seen}"]
            if peer_prio:
                parts.append(f"prio={peer_prio}")
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
    check(make().peer_busy()[0] is False, "미수신 -> 직행")
    check(make().peer_seen() is False, "미수신 -> peer_seen False")

    print("\n── 2. 단계별 양보 판단 ──")
    for st in all_stages:
        got = make(seen=st).peer_busy()[0]
        want = st in busy_set
        check(got == want, f"state={st:14s} 양보={got!s:5s} (기대 {want!s:5s})")

    print("\n── 3. 소식이 끊겨도 마지막 단계를 그대로 쓴다 ──")
    # 수신 시각으로 "묵었으면 양보" 하는 규칙은 없다. 위험한 방향은 마지막
    # 단계가 이미 막아 주고, 그 규칙을 두면 차선 밖에서 죽은 상대 때문에
    # 살아 있는 로봇이 얼어붙는다.
    for age in (0.0, 30.0):
        check(make(seen=ns["SCAN"], age=age).peer_busy()[0] is False,
              f"마지막이 scan,  {age:.0f}s 전 -> 직행")
        check(make(seen=ns["PLACE"], age=age).peer_busy()[0] is True,
              f"마지막이 place, {age:.0f}s 전 -> 양보")

    print("\n── 4. 상대가 얼어붙은 경우 ──")
    for st in lane_like:
        check(make(seen=st, failed=True).peer_frozen() is True,
              f"차선({st}) 에서 정지 -> 즉시 실패")
    for st in queued:
        check(make(seen=st, failed=True).peer_frozen() is False,
              f"{st} 에서 정지 -> 실패 아님 (차선은 비었다)")
        check(make(seen=st, failed=True).peer_busy()[0] is False,
              f"{st} 에서 정지 -> 양보 안 함")
    check(make(seen=ns["PICK"], failed=True).peer_frozen() is False,
          "차선 밖(pick) 에서 정지 -> 실패 아님")

    print("\n── 5. 교착 조합이 없어야 한다 ──")
    # 줄 서 있는 상태의 모든 조합에서, 양쪽이 동시에 양보하면 교착이다.
    dead = [(a, b) for a, b in itertools.product(queued, repeat=2)
            if make(seen=b).peer_busy()[0] and make(seen=a).peer_busy()[0]]
    check(not dead, f"둘 다 줄 서 있는데 서로 양보하는 조합 없음 {dead}")

    print("\n── 6. 순서(lane_priority)는 한쪽만 양보시킨다 ──")
    contending = list(ns["CONTENDING_STAGES"])
    check(not any(st in busy_set for st in contending),
          f"CONTENDING_STAGES {contending} 는 양보 목록 밖이다 — 안 그러면 "
          f"prio 없이도 둘 다 양보한다")
    for st in contending:
        lo = make(seen=st, my_prio=2, peer_prio=1)
        hi = make(seen=st, my_prio=1, peer_prio=2)
        check(lo.peer_contending() and lo.peer_outranks_me(),
              f"후순위(prio 2): 상대 {st} -> 양보")
        check(hi.peer_contending() and not hi.peer_outranks_me(),
              f"선순위(prio 1): 상대 {st} -> 양보 안 함")
    for st in queued:
        check(not make(seen=st, my_prio=1, peer_prio=2).peer_contending(),
              f"상대가 {st}(이미 양보 중)이면 contending 아님 -> 선순위 직행")
    check(not make(seen=ns["PICK"], my_prio=2, peer_prio=0).peer_outranks_me()
          and not make(seen=ns["PICK"], my_prio=0, peer_prio=1).peer_outranks_me(),
          "한쪽이라도 prio 가 없으면(0) 순서 장치가 꺼진다")
    # ★ 전순서라 둘이 서로를 선순위로 볼 수 없다 — 6번 교착 부재의 근거.
    both = [(a, b) for a, b in itertools.permutations((1, 2), 2)
            if make(seen=ns["PICK"], my_prio=a, peer_prio=b).peer_outranks_me()
            and make(seen=ns["PICK"], my_prio=b, peer_prio=a).peer_outranks_me()]
    check(not both, f"서로를 선순위로 보는 조합 없음 {both}")

    print("\n── 7. 반경 판정과 HOLD_BACK 목록 ──")
    leaving = list(ns["PEER_LEAVING_STAGES"])
    check(all(st in busy_set for st in leaving),
          f"PEER_LEAVING_STAGES {leaving} 가 전부 양보 목록 안에 있다 — "
          f"양보하지 않는 단계에 반경 판정을 둬도 의미가 없다")
    holdback = list(ns["HOLD_BACK_STAGES"])
    check(all(st in busy_set for st in holdback),
          f"HOLD_BACK_STAGES {holdback} 가 전부 양보 목록 안에 있다")
    check(ns["PLACE"] not in holdback,
          "place 는 HOLD_BACK 대상이 아니다 — 상대가 로더에 멈춰 있는 그때가 "
          "대기 자리로 올라갈 유일한 때다. 넣으면 대기 자리가 쓸모없어진다")
    past = list(ns["PEER_PAST_LOADER_STAGES"])
    check(all(st in past for st in (ns["DOCK"], ns["DONE"])),
          "도킹·완료도 '로더를 지났다' 에 든다 — 안 그러면 둘 다 스택을 맡는다")
    check(all(st in busy_set for st in past
              if st not in (ns["DOCK"], ns["DONE"])),
          "로더를 지난 단계(dock·done 빼고)는 전부 양보 대상이다")
    check(float(ns["DEFAULT_LOADER_CLEAR_RADIUS_M"]) > 0,
          f"loader_clear_radius_m 기본값 {ns['DEFAULT_LOADER_CLEAR_RADIUS_M']} m > 0")

    print("\n── 8. launch 표가 소스 기본값과 어긋나지 않는가 ──")
    check(busy_set == list(ns["DEFAULT_PEER_BUSY_STAGES"]),
          f"LANE_STAGES == DEFAULT_PEER_BUSY_STAGES ({len(busy_set)}개)")
    scan_routes = launch_ns["SCAN_ROUTE_BY_ROBOT"]
    dock_routes = launch_ns["DOCK_ROUTE_BY_ROBOT"]
    row_exits = launch_ns["ROW_EXIT_BY_ROBOT"]
    staging = launch_ns["STAGING_BY_ROBOT"]
    prios = launch_ns["LANE_PRIORITY_BY_ROBOT"]
    check(set(scan_routes) == set(dock_routes) == set(row_exits)
          == set(staging) == set(prios),
          "표 다섯이 같은 로봇 집합을 안다")
    check(sorted(prios.values()) == [1, 2],
          f"lane_priority 가 1·2 로 갈린다 {prios}")
    check(staging["robot1"] != staging["robot2"],
          f"대기 자리가 로봇별로 다름 {staging}")
    check(dock_routes["robot1"][-3:] != dock_routes["robot2"][-3:],
          "도크가 로봇마다 다르다")
    check(list(ns["DEFAULT_SCAN_ROUTE"]) == scan_routes["robot1"],
          "소스 기본 scan_route 가 launch 의 robot1 값과 같다")
    check(list(ns["DEFAULT_DOCK_ROUTE"]) == dock_routes["robot1"],
          "소스 기본 dock_route 가 launch 의 robot1 값과 같다")
    check(list(ns["DEFAULT_APPROACH_ROUTE"])
          == row_exits["robot1"] + staging["robot1"],
          "소스 기본 approach_route 가 launch 의 robot1 (이탈점 + 대기자리) 와 같다")
    check(list(ns["DEFAULT_STAGING_POSE"]) in staging.values(),
          "소스 기본 대기자리가 launch 값 중 하나와 같다")
    for name, table in (("scan", scan_routes), ("dock", dock_routes),
                        ("row_exit", row_exits), ("staging", staging)):
        check(all(len(r) % 3 == 0 and len(r) >= 3 for r in table.values()),
              f"{name} 좌표가 (x, y, yaw) 3 의 배수다")
    src = io.open(LAUNCH, encoding="utf-8").read()
    check(("peer_pose_topic" in src) and ("amcl_pose" in src),
          "launch 가 peer_pose_topic 을 넘긴다 — 없으면 출발 게이트와 반경 "
          "판정을 거리로 못 재고 시간·상태로만 푼다")
    tm_src = io.open(TM, encoding="utf-8").read()
    check("prio=" in tm_src, "상태 문자열에 prio 토큰이 실린다 — 순서 장치에 필요")
    check("patrol" not in tm_src.split('"""')[2],
          "독스트링 밖에 patrol 이 남아 있지 않다 (이 브랜치는 순찰이 없다)")

    print()
    if failures:
        print(f"실패 {len(failures)}건")
        return 1
    print("전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""디스패처 — 큐에서 한 건 꺼내 로봇에게 보내고, 보고를 받아 표에 적는다.

WBS 4.7 · 4.8 이 여기다. 브리지에서 올라오는 사건을 표 5·6 에 반영하는
유일한 자리이기도 하다 — 라우터는 사람이 누른 것만 처리하고, 로봇이
스스로 보고한 것은 전부 이 파일을 지난다.

★ 로봇당 하나씩 도는 루프다. 브리지가 없으면(NullBridge) 조용히 쉰다 —
  큐에 쌓이기만 하고 아무 일도 안 일어난다. 그게 맞다. 로봇이 없는데
  작업을 RUNNING 으로 올려 두면 영원히 끝나지 않는 행이 남는다.
"""

from __future__ import annotations

import asyncio
import logging

from .. import shapes
from ..config import settings
from ..errors import ApiError
from ..ws import hub
from . import pickup, queue
from .rosbridge import bridge

log = logging.getLogger(__name__)

# ExecuteTask.Result 의 fail_reason 상수 — 액션 정의와 같은 순서여야 한다.
NONE, NO_TARGET, NOT_TAUGHT, SCAN_FAIL, PICK_FAIL, NAV_FAIL, PLACE_FAIL, CANCELED, NO_CARRIER = range(9)

# "실패로 적지 않는" 결과. 둘 다 로봇이 잘못한 게 아니다.
#   CANCELED   — 사람이 세웠다 (NavigateTo.action 주석과 같은 판단)
#   NO_CARRIER — 선반이 비어 있었다 (§8-4 "found=false 는 에러가 아니다")
BENIGN = {CANCELED, NO_CARRIER}


# 로봇별 '지금 확인해라' 신호. 작업이 끝나거나 새로 생기면 세운다.
_wake: dict[str, asyncio.Event] = {}

# 신호가 없어도 이만큼마다 한 번은 본다 — 신호를 놓쳐도 큐가 멈추지 않게.
_IDLE_POLL_SEC = 2.0


def wake(robot_id: str) -> None:
    """배차 루프를 즉시 깨운다.

    ★ 이게 없으면 캐리어 하나를 끝낼 때마다 폴링 주기만큼 로봇이 선반 앞에서
      논다. 작업 1건 = 캐리어 1개라 그 대기가 캐리어 수만큼 곱해진다.
    """
    ev = _wake.get(robot_id)
    if ev is not None:
        ev.set()


async def run(robot_id: str) -> None:
    """한 로봇의 배차 루프.

    큐가 비어 있으면 잔다. 작업이 끝나거나(_on_result) 새로 배정되면
    wake() 가 깨워서, 로봇이 복귀한 그 자리에서 곧바로 다음 goal 을 받는다.
    """
    ev = _wake.setdefault(robot_id, asyncio.Event())
    while True:
        try:
            if bridge.available:
                task = await queue.claim_next(robot_id)
                if task is not None:
                    await _send(robot_id, task)
                    continue  # 연달아 꺼내지 않는다 — 다음은 이 작업이 끝난 뒤다
        except asyncio.CancelledError:
            raise
        except Exception:
            log.exception("%s 배차 실패 — 다음 주기에 다시 시도한다", robot_id)

        ev.clear()
        try:
            await asyncio.wait_for(ev.wait(), timeout=_IDLE_POLL_SEC)
        except asyncio.TimeoutError:
            pass


async def _send(robot_id: str, task: dict) -> None:
    # 표 5 와 화면은 'AMR-01', ROS 는 'robot1' — 브리지로 넘어갈 때 번역한다.
    try:
        await bridge.execute_task(settings().to_ns(robot_id), task)
    except ApiError as e:
        # 보낼 수 없으면 RUNNING 으로 둘 수 없다 — 큐로 되돌린다.
        log.warning("task=%s 를 %s 에게 못 보냈다: %s", task["task_id"], robot_id, e.message)
        await queue.apply_result(task["task_id"], success=False, requeue=True)
        return
    log.info("배차 task=%s → %s (%s %s)", task["task_id"], robot_id, task["kind"], task["target_ref"])
    await hub.publish("task_changed", task, robot_id)


# ── 브리지 사건 처리 ─────────────────────────────────────────────────
async def on_bridge_event(ch: str, data: dict) -> None:
    """RclpyBridge 가 올리는 모든 사건의 입구. main 이 여기에 연결한다.

    ★ 브리지는 ROS 네임스페이스('robot1')로 올린다. 여기서 한 번만 화면/DB
      이름('AMR-01')으로 바꾸고, 아래로는 그 이름만 흐른다.
    """
    robot_id = settings().to_display(data.get("robot_id") or "")

    if ch == "task.feedback":
        row = await queue.apply_feedback(data["task_id"], data.get("run_id"), data.get("progress"))
        if row:
            await hub.publish("task_changed", {**row, "stage": data.get("stage"),
                                               "carrier_id": data.get("carrier_id")}, robot_id)
        return

    if ch == "task.result":
        await _on_result(data, robot_id)
        return

    if ch == "task.rejected":
        # 로봇이 goal 을 거절했다 — 큐로 되돌린다. 사람이 볼 수 있게 alert 도 띄운다.
        row = await queue.apply_result(data["task_id"], success=False, requeue=True)
        await hub.publish("task_changed", row, robot_id)
        await hub.publish("alert", {"severity": 1, "code": "TASK_REJECTED",
                                    "detail": f"task={data['task_id']} 를 로봇이 거절했다"}, robot_id)
        return

    # ── 실시간 상태 — 저장하지 않는다(§10-1). 화면으로 흘려보내기만 한다.
    #
    # ★ 위치와 상태가 ROS 에서는 다른 토픽(amcl_pose / orchestrator/state)으로
    #   오지만, 화면은 **하나의 robot_state 메시지**만 읽는다. 그래서 여기서
    #   합쳐 보낸다. 화면은 받은 칸만 이전 값과 병합하므로(`??`) 매번 둘 다
    #   실을 필요는 없다 — 다만 **null 로는 못 지운다**는 점에 주의.
    if ch == "robot.pose":
        await hub.publish_robot_state([shapes.robot_state_out(
            robot_id or "",
            pose={"x": data.get("x", 0.0), "y": data.get("y", 0.0), "theta": data.get("theta", 0.0)},
        )])
        return

    if ch == "robot.state":
        # /orchestrator/state 의 "state=pick | carrier=F1-MGZB-1 | ..." 를 판 dict 다.
        stage = data.get("state")
        carrier = data.get("carrier")
        # 값 없는 토큰(PAUSED · FAILED)은 parse_state 가 True 로 둔다. 화면은 이
        # 문자열에서 PAUSE · FAIL 을 부분일치로 찾아 색을 고르므로 뒤에 붙인다.
        if stage and data.get("PAUSED"):
            stage = f"{stage} PAUSED"
        if stage and data.get("FAILED"):
            stage = f"{stage} FAILED"
        await hub.publish_robot_state([shapes.robot_state_out(
            robot_id or "",
            # 화면은 이 문자열을 그대로 보여주고, 대문자 부분일치로 색을 고른다
            # (PAUSE / NAVIGAT / MOVING / RUNNING / FAIL / ERROR). 그래서 대문자로 낸다.
            state=str(stage).upper() if stage else None,
            task={"target_id": carrier} if carrier else None,
        )])
        return

    if ch in {"alert"}:
        await hub.publish(ch, data, robot_id)
        return

    # 조용히 흘리지 않는다 — 브리지가 내보내는 이름과 여기 이름이 어긋나면
    # 화면만 빈 채로 돌고 아무데도 흔적이 안 남는다(실제로 robot.pose 가
    # robot_pose 로 적혀 있어 위치·상태가 통째로 사라진 적이 있다).
    log.warning("브리지가 올린 모르는 채널: %s", ch)


async def _on_result(data: dict, robot_id: str | None) -> None:
    task_id = data["task_id"]
    reason = int(data.get("fail_reason") or 0)
    success = bool(data.get("success"))

    # NO_CARRIER 는 "할 일이 없었다" 라서 작업 자체는 정상 종료다.
    finished_ok = success or reason in BENIGN
    row = await queue.apply_result(task_id, success=finished_ok, run_id=data.get("run_id"))
    if row:
        await hub.publish("task_changed", {**row, "fail_detail": data.get("fail_detail")}, robot_id)

    # 회수 작업이었다면 짝이 되는 대기를 닫거나 재예약한다.
    # ★ 실패해도 산출물은 여전히 스테이션에 있다 — pickup.on_task_finished 가
    #   그래서 done() 이 아니라 retry() 를 부른다(§4-3 과 같은 구조).
    changed = await pickup.on_task_finished(task_id, success)
    if changed:
        await hub.publish("pickup_changed", changed, robot_id)

    # 선반에 아직 남아 있으면 곧바로 후속 작업을 건다.
    # 로봇은 이미 복귀해서 그 선반 앞에 서 있으므로 이동 없이 이어서 돈다.
    if data.get("more_at_target"):
        follow = await queue.follow_up(task_id)
        if follow:
            await hub.publish("task_changed", follow, robot_id)

    # 작업이 끝났으니 다음 것을 바로 꺼내라고 깨운다.
    if robot_id:
        wake(robot_id)

    if not finished_ok:
        await hub.publish(
            "alert",
            {
                "severity": 2,
                "code": _REASON_NAME.get(reason, f"FAIL_{reason}"),
                "detail": data.get("fail_detail") or "",
                "stage": "",
            },
            robot_id,
        )


_REASON_NAME = {
    NO_TARGET: "NO_TARGET",
    NOT_TAUGHT: "NOT_TAUGHT",
    SCAN_FAIL: "SCAN_FAIL",
    PICK_FAIL: "PICK_FAIL",
    NAV_FAIL: "NAV_FAIL",
    PLACE_FAIL: "PLACE_FAIL",
}


def start_all(tasks: list[asyncio.Task]) -> None:
    for robot in settings().robots:
        tasks.append(asyncio.create_task(run(robot), name=f"dispatch:{robot}"))

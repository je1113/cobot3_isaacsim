"""작업 큐 — 표 5 (WBS 4단계).

★ 이 라우터와 dispatcher 만 task 표에 쓴다(0.3). 여기는 사람이 누른 것,
  dispatcher 는 로봇이 보고한 것.
"""

from __future__ import annotations

from fastapi import APIRouter, Query

from ..config import settings
from ..errors import ApiError
from ..models import AutoDistribute, QueueReplace, TaskCreate, TaskPatch
from ..services import dispatcher, queue
from ..services.rosbridge import bridge
from ..ws import hub

router = APIRouter(tags=["tasks"])


@router.get("/tasks")
async def list_tasks(status: str | None = Query(None), limit: int = Query(200, ge=1, le=1000)):
    return await queue.list_tasks(status, limit)


@router.get("/tasks/{task_id}")
async def get_task(task_id: int):
    return await queue.get(task_id)


@router.post("/tasks", status_code=201)
async def create_task(body: TaskCreate):
    """배정 (4.1). 티칭 미완료 → 422, 같은 선반에 다른 로봇이 실행 중 → 409."""
    row = await queue.create(body.robot_id, body.target_ref, body.kind, body.queue_order)
    await hub.publish("task_changed", row, row["robot_id"])
    dispatcher.wake(row["robot_id"])   # 노는 로봇이면 즉시 나간다
    return row


@router.patch("/tasks/{task_id}")
async def patch_task(task_id: int, body: TaskPatch):
    """로봇 간 이관 (4.4). RUNNING 이관은 409."""
    row = await queue.patch(task_id, body.robot_id, body.queue_order)
    await hub.publish("task_changed", row, row["robot_id"])
    dispatcher.wake(row["robot_id"])
    return row


@router.delete("/tasks/{task_id}", status_code=204)
async def delete_task(task_id: int):
    """취소 (4.5) — 무슨 상태든 화면에서 누르면 지워진다.

    RUNNING 이면 정중하게(STOP 과 같은 경로, bridge.cancel_task) 먼저
    요청해 본 뒤, 결과를 기다리지 않고 표를 강제로 지운다 — goal_handle 이
    죽은 프로세스 것으로 남아 STOP 이 조용히 무시되는 경우(queue.cancel
    독스트링 참고)에도 화면에서 확실히 치워지게 하려는 것이다. bridge 가
    꺼져 있거나 handle 이 없어도 실패로 안 친다 — 정중한 요청은 "되면 좋고"
    지 이 취소의 필수 조건이 아니다.

    pending_pickup 정리(QUEUED→GAVE_UP, RUNNING→WAITING 재예약)는
    queue.cancel() 이 같은 트랜잭션 안에서 다 한다 — 여기서 따로 안 건드린다.
    """
    row = await queue.get(task_id)
    if row["status"] == "RUNNING":
        try:
            await bridge.cancel_task(settings().to_ns(row["robot_id"]))
        except ApiError:
            pass
    outcome = await queue.cancel(task_id)
    status = "CANCELED" if outcome == "QUEUED" else "FAILED"
    await hub.publish("task_changed", {"task_id": task_id, "status": status}, row["robot_id"])
    # queue.cancel() 이 RECOVER 작업이면 연결된 pending_pickup 도 같이 건드릴
    # 수 있다(GAVE_UP/WAITING) — 그 여부를 여기서 다시 조회하지 않고 화면에
    # 새로고침만 시킨다. "회수 대기" 패널은 내용과 무관하게 이 이벤트를
    # 받으면 통째로 다시 불러온다(MonitoringPage loadPendingPickups).
    await hub.publish("pickup_changed", {}, row["robot_id"])
    dispatcher.wake(row["robot_id"])


@router.get("/robots/{robot_id}/queue")
async def get_queue(robot_id: str, include_done: bool = False):
    return await queue.queue_of(robot_id, include_done)


@router.put("/robots/{robot_id}/queue")
async def put_queue(robot_id: str, body: QueueReplace):
    """순서 전체 교체 (4.3).

    부분 교체를 받지 않는다 — 화면이 낡은 목록을 들고 있으면 드래그 결과가
    절반만 반영되고, 그게 가장 알아차리기 어려운 실패다. 집합이 다르면 409.
    """
    rows = await queue.replace_order(robot_id, body.task_ids)
    await hub.publish("task_changed", {"robot_id": robot_id, "queue": rows}, robot_id)
    dispatcher.wake(robot_id)
    return rows


@router.post("/tasks/auto-distribute")
async def auto_distribute(body: AutoDistribute):
    """분배안 **계산만** 한다 (4.6).

    바로 적용하지 않는 이유: 화면이 결과를 먼저 보여주고 사람이 확인한 뒤
    PUT /robots/{id}/queue 로 적용한다. 적용 경로가 하나면 불변식 검사도 한 곳이다.
    """
    return {"plan": await queue.auto_distribute(body.robots)}


@router.post("/tasks/reset-sim", status_code=204)
async def reset_sim():
    """개발용 — 시뮬레이션을 새로 껐다 켰을 때 큐를 통째로 비운다.

    ★ 2026-09-25: Isaac Sim 재시작은 물리 세계를 리셋하지만 DB 큐는 "이전
      세계"를 가리킨 채 남는다(queue.reset_all 독스트링 참고) — 화면에서
      하나씩 강제삭제하던 걸 버튼 하나로 묶는다. run 기록(magazine_log/
      stack_log)은 안 건드린다.
    """
    await queue.reset_all()
    for robot_id in settings().robots:
        await hub.publish("task_changed", {"reset": True}, robot_id)
        dispatcher.wake(robot_id)
    await hub.publish("pickup_changed", {"reset": True})

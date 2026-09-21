"""작업 큐 — 표 5 (WBS 4단계).

★ 이 라우터와 dispatcher 만 task 표에 쓴다(0.3). 여기는 사람이 누른 것,
  dispatcher 는 로봇이 보고한 것.
"""

from __future__ import annotations

from fastapi import APIRouter, Query

from ..models import AutoDistribute, QueueReplace, TaskCreate, TaskPatch
from ..services import dispatcher, queue
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
    """취소 (4.5). QUEUED 만 지운다 — 실행 중인 것은 STOP 명령으로 세운다."""
    row = await queue.get(task_id)
    await queue.cancel(task_id)
    await hub.publish("task_changed", {"task_id": task_id, "status": "CANCELED"}, row["robot_id"])


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

"""제어 명령 — PAUSE / RESUME / SKIP / STOP (WBS 7단계).

★ 202 로 즉시 답한다. 실제 결과는 command.result 채널로 온다.
  ROS 서비스 응답을 기다렸다가 200 을 주면, DDS 가 느릴 때 화면이 굳는다.
  그래서 "받았다" 와 "됐다" 를 나눈다.

★ STOP 은 비상정지가 아니다(7.4).
  소프트웨어로 goal 을 거두는 것이라 노드가 죽어 있거나 DDS 가 끊기면 듣지
  못한다. 화면 라벨은 "전체 정지" 이고, 사람을 지키는 정지는 하드웨어
  E-Stop 이어야 한다. 그렇게 오해되지 않도록 경로 이름에도 emergency 를 안 썼다.
"""

from __future__ import annotations

import asyncio
import logging
import uuid

from fastapi import APIRouter

from ..config import settings
from ..errors import ApiError
from ..models import CommandAccepted, CommandRequest
from ..services import queue
from ..services.rosbridge import bridge
from ..ws import hub

log = logging.getLogger(__name__)
router = APIRouter(tags=["commands"])


async def _dispatch(robot_id: str, command: str, reason: str, cid: str) -> None:
    try:
        res = await bridge.command(robot_id, command, reason)
    except ApiError as e:
        await hub.publish("command_result", {"correlation_id": cid, "command": command,
                                             "accepted": False, "error": e.message}, robot_id)
        return

    await hub.publish("command_result", {"correlation_id": cid, "command": command, **res}, robot_id)

    # STOP / SKIP 은 실행 중이던 작업을 끝낸다. 로봇이 result(CANCELED)를 올리면
    # dispatcher 가 표를 정리하므로 여기서 표를 고치지 않는다 — writer 를 하나로 둔다.
    if res.get("accepted") and command in {"STOP", "SKIP"}:
        await bridge.cancel_task(robot_id)


@router.post("/robots/{robot_id}/commands", status_code=202, response_model=CommandAccepted)
async def send_command(robot_id: str, body: CommandRequest):
    cid = uuid.uuid4().hex[:12]
    asyncio.create_task(_dispatch(robot_id, body.command, body.reason or "", cid))
    log.info("명령 %s → %s (cid=%s)", body.command, robot_id, cid)
    return CommandAccepted(accepted=True, command=body.command, robot_id=robot_id, correlation_id=cid)


@router.post("/commands/stop-all", status_code=202)
async def stop_all():
    """7.2 — 전체 정지. 로봇마다 따로 보내고, 하나가 실패해도 나머지는 보낸다."""
    cid = uuid.uuid4().hex[:12]
    for robot in settings().robots:
        asyncio.create_task(_dispatch(robot, "STOP", "전체 정지", cid))
    return {"accepted": True, "robots": list(settings().robots), "correlation_id": cid}


@router.get("/robots")
async def list_robots():
    """화면이 로봇 목록을 어디선가 받아야 한다. 표가 아니라 설정에서 온다(§10-1).

    실시간 상태(위치·배터리)는 여기 없다 — robot.pose / robot.state 채널로만 간다.
    저장하지 않는 값이라 REST 로 줄 스냅샷이 없다.
    """
    out = []
    for robot in settings().robots:
        out.append({
            "robot_id": robot,
            "queue": await queue.queue_of(robot),
            "bridge": bridge.available,
        })
    return out

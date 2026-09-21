"""프론트가 **실제로 부르는** 엔드포인트.

앱 전체에서 서버를 부르는 곳은 세 군데뿐이다(src/api/*):

    POST {VITE_TASK_START_PATH}      TaskAssignmentPage  '작업 시작'
    GET  {VITE_LOGS_PATH}            LogsPage            '로그 불러오기'
    POST /api/robots/{id}/goal|pause|resume   MonitoringPage  지도 클릭 · 일시정지 · 재개

앞의 둘은 경로를 .env 로 바꿀 수 있지만 **로봇 제어 셋은 경로가 코드에
하드코딩**돼 있다(src/api/robots.js). 그래서 그 셋은 이름을 맞춰 준다.

★ 화면이 응답 본문을 **전부 무시한다.** 성공/실패만 본다(`API 요청 실패: {status}`).
  그래도 본문을 제대로 채운다 — 화면이 나중에 읽게 되고, 그전에도
  개발자가 /api/docs 로 확인한다.
"""

from __future__ import annotations

import logging
from typing import Any

from fastapi import APIRouter, Body, Query

from .. import db, shapes
from ..config import settings
from ..errors import Unprocessable
from ..services import configstore, dispatcher, queue
from ..services.rosbridge import bridge
from ..ws import hub

log = logging.getLogger(__name__)
router = APIRouter(tags=["frontend"])


# ── 메타 (화면이 상수를 박아 두지 않게) ──────────────────────────────
@router.get("/meta")
async def meta():
    """화면이 하드코딩하던 값들의 **단일 출처**.

    이게 없으면 같은 목록이 두 곳에 생긴다: 로봇 이름은 백엔드 설정과
    MonitoringPage.ROBOT_IDS 에, 완료 신호는 shapes.py 와 <select> 에.
    그러면 한쪽만 고쳤을 때 조용히 갈라진다 — 화면에는 선택지가 있는데
    서버가 422 로 거절하는 식이다(§5-2 와 같은 이유).

    ★ 파일도 DB 도 아니다. 전부 코드/설정에서 유도되는 값이라 저장할 것이 없다.
      로봇 대수를 바꾸려면 COBOT3_ROBOTS 환경변수를 고친다.
    """
    return {
        # 화면에 보이는 이름. ROS 네임스페이스는 내보내지 않는다 —
        # 화면이 알 필요가 없고, 알면 그걸로 API 를 부르려 든다.
        "robots": list(settings().robots),
        "enums": {
            "station_types": list(shapes.STATION_TYPES),
            "completion_signals": list(shapes.COMPLETION_SIGNALS),
            "scan_directions": list(shapes.DIRECTIONS),
            "final_destinations": list(shapes.FINAL_DESTINATIONS),
            "task_results": ["SUCCESS", "FAILED"],
        },
        # 라우팅 체인의 시작 센티널. 스테이션이 아니라 '선반에서 출발' 이라는 뜻이다.
        # 완료 신호가 뜻하는 바. 어휘와 그 설명을 한곳에 둔다 — 값만 서버가
        # 주고 설명은 화면이 들고 있으면, 신호를 하나 더할 때 두 곳을 고쳐야 하고
        # 한쪽을 잊으면 화면에 빈 설명이 뜬다.
        "completion_signal_help": {
            "TIMER": "설정한 처리 시간이 지나면 공정 완료로 판단합니다.",
            "VISION": "비전 인식 결과를 기준으로 공정 완료를 판단합니다.",
            "TIMER + VISION": "처리 시간과 비전 확인 조건을 함께 사용해 공정 완료를 판단합니다.",
            "EXTERNAL": "외부 완료 신호를 수신하면 공정 완료로 판단합니다.",
        },
        # 훑는 방향을 화살표로. 목록의 두 번째(BACKWARD)가 역방향이다.
        "scan_direction_arrows": {
            shapes.DIRECTIONS[0]: "→",
            shapes.DIRECTIONS[1]: "←",
        },
        "route_start": shapes.ROUTE_START,
        # 팔 축 수. arm_teach_pose / place_arm_pose 의 길이.
        "joints": shapes.JOINTS,
    }


# ── 작업 시작 (TaskAssignmentPage) ───────────────────────────────────
@router.post("/tasks/start")
async def start_tasks(body: dict = Body(...)):
    """화면이 보내는 것:

        {"robots": [{"robot_id": "AMR-01", "queue": ["SHELF-A", "SHELF-C"]},
                    {"robot_id": "AMR-02", "queue": ["SHELF-B"]}]}

    `queue` 는 **선반 id 문자열 배열**이고 순서가 곧 우선순위다. 두 로봇이
    큐가 비어 있어도 항상 실려 온다.

    ★ 화면의 큐가 서버의 진실이 된다 — 그 로봇의 대기 작업을 통째로 이 목록으로
      바꾼다. 화면에는 '추가' 개념만 있고 '동기화' 개념이 없어서, 부분 반영을
      하면 화면에서 지운 항목이 서버에 남는다.

    ★ 실행 중(RUNNING)인 작업은 건드리지 않는다. 이미 로봇이 그 선반에 가 있다.
    """
    robots = body.get("robots")
    if not isinstance(robots, list):
        raise Unprocessable("robots 는 배열이어야 한다", got=type(robots).__name__)

    known_shelves = set(configstore.shelf_ids())
    started: list[dict] = []

    for entry in robots:
        display = str(entry.get("robot_id") or "").strip()
        shelf_ids = [str(s) for s in (entry.get("queue") or [])]
        if not display:
            raise Unprocessable("robot_id 가 없다")
        if display not in settings().robots:
            raise Unprocessable(
                f"모르는 로봇: {display}", known=list(settings().robots)
            )

        for shelf_id in shelf_ids:
            if shelf_id not in known_shelves:
                raise Unprocessable(
                    f"등록되지 않은 선반이다: {shelf_id}", known=sorted(known_shelves)
                )
            shelf = configstore.shelf(shelf_id) or {}
            # 화면과 같은 규칙으로 막는다. 화면도 티칭 미완료는 배정 못 하게
            # 하지만, 화면의 판정과 서버의 판정이 갈라지면 여기서 걸린다.
            if not shapes.shelf_is_taught(shelf):
                raise Unprocessable(
                    f"{shelf_id} 는 티칭이 끝나지 않아 배정할 수 없다. "
                    "설정 > 선반에서 모든 층의 관절값을 채울 것.",
                    shelf_id=shelf_id,
                )

        rows = await queue.replace_queue(display, shelf_ids)
        started.append({"robot_id": display, "queued": len(rows)})
        dispatcher.wake(display)

    await hub.publish("task_changed", {"started": started})
    log.info("작업 시작 — %s", started)
    return {"accepted": True, "robots": started}


# ── 로그 (LogsPage) ──────────────────────────────────────────────────
@router.get("/logs")
async def logs(limit: int = Query(500, ge=1, le=5000)):
    """★ **최상위가 배열**이어야 한다.

        if (!Array.isArray(response)) throw new Error('로그 응답 형식이 배열이 아닙니다.')

    그래서 {items: [...]} 로 감싸면 화면이 거절한다. 페이징도 없다 —
    화면이 전부 받아서 클라이언트에서 거르고 집계한다. limit 이 그 상한이다.

    통계(총 작업·완료·실패·평균 소요)도 화면이 이 배열로 직접 계산한다.
    서버가 따로 집계를 주지 않는 이유가 그것이다.
    """
    rows = await db.fetch(
        "SELECT role, log_id, qr_payload, robot_id, stage, ended_at, "
        "       duration_sec, succeeded, port "
        "FROM carrier_log ORDER BY ended_at DESC, log_id DESC LIMIT %s",
        (limit,),
    )
    return [shapes.log_row_out(r) for r in rows]


# ── 로봇 제어 (MonitoringPage) ───────────────────────────────────────
# 경로가 src/api/robots.js 에 하드코딩돼 있다. 이름을 바꾸면 화면이 깨진다.


@router.post("/robots/{robot_id}/goal")
async def robot_goal(robot_id: str, body: dict = Body(...)):
    """지도 클릭. 화면이 보내는 것:

        {"goal": {"x_ratio": 0.12345, "y_ratio": 0.54321}}

    ★ **월드 좌표가 아니다.** 화면의 지도 div 안에서의 0~1 비율이고,
      그 div 는 지금 실제 맵 이미지가 아니라 개념 배치다
      ("실제 ROS Map 표시 영역 · 개념 배치 · 좌표 미사용").

      그래서 이 비율을 map 프레임으로 바꾸는 변환이 **아직 없다.** 없는 변환을
      추측해서 로봇을 보내는 것이 가장 위험하므로, 받아서 기록만 하고 422 로
      거절한다. 맵 이미지와 그 원점·해상도가 정해지면 그때 변환을 넣는다.
    """
    goal = body.get("goal") or {}
    log.info("지도 클릭 목표 (미변환) robot=%s %s", robot_id, goal)
    raise Unprocessable(
        "지도 클릭으로 이동은 아직 연결되지 않았다. "
        "화면이 보내는 값은 지도 div 안의 0~1 비율이고, 그것을 map 좌표로 바꾸려면 "
        "실제 맵 이미지와 그 원점·해상도(map.yaml)가 먼저 정해져야 한다.",
        received=goal,
        robot_id=robot_id,
    )


@router.post("/robots/{robot_id}/pause")
async def robot_pause(robot_id: str, body: dict = Body(default={})):
    return await _command(robot_id, "PAUSE")


@router.post("/robots/{robot_id}/resume")
async def robot_resume(robot_id: str, body: dict = Body(default={})):
    """화면은 마지막 목표를 같이 보내지만(`{goal: lastGoal}`), 위 goal 과 같은
    이유로 무시한다. 재개는 '세운 자리에서 계속' 이면 충분하다."""
    return await _command(robot_id, "RESUME")


async def _command(display: str, command: str) -> dict:
    """★ 202 가 아니라 **실제 결과**를 기다려서 답한다.

    화면은 이 호출을 await 하고 실패하면 `error.message` 를 띄운다. 그리고
    WebSocket 의 command_result 는 **읽지 않는다**(robot_state 만 처리한다).
    그러니 202 로 먼저 답하면 화면은 성공한 줄 알고 끝난다 — 로봇이 거절해도
    사람은 모른다. 브리지가 꺼져 있으면 여기서 503 이 그대로 올라간다.
    """
    if display not in settings().robots:
        raise Unprocessable(f"모르는 로봇: {display}", known=list(settings().robots))
    ns = settings().to_ns(display)
    res = await bridge.command(ns, command, "")   # 브리지 꺼져 있으면 503
    await hub.publish("command_result", {"robot_id": display, "command": command, **res})
    return {"accepted": res.get("accepted", False), "robot_id": display, "command": command}

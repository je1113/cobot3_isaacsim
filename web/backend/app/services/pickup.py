"""산출물 회수 — 표 6(pending_pickup) 과 스케줄러 (WBS 5단계).

고리는 이렇다:

    place 완료 (magazine_log INSERT)
      → pg_notify → notify.py → on_place_done()        5.1  행 생성
      → ready_at 도래 → tick()                          5.2  RECOVER task 생성
      → 가 보니 없더라 → retry()                        5.3  ready_at 재예약
      → 회수 성공 → done()

★ ROS 를 거치지 않는다. place 완료를 DB 트리거로 알기 때문에, 로봇이
  event_logger 에 평소대로 로그를 남기기만 하면 회수가 저절로 걸린다.
  웹이 죽어 있던 동안 쌓인 place 도 재시작 때 tick() 이 주워 간다 —
  ready_at 이 과거인 WAITING 행이 그대로 남아 있어서다. 그것이 이 표가
  메모리 타이머가 아니라 표여야 하는 이유다(§10-1).
"""

from __future__ import annotations

import asyncio
import logging
from datetime import datetime, timedelta, timezone

from .. import db
from ..config import settings
from ..errors import NotFound
from . import configstore, dispatcher, queue

log = logging.getLogger(__name__)

_COLS = """pending_id, station_ref, source_run_id::text AS source_run_id,
           ready_at, retry_count, status, task_id, created_at"""

# 산출물 페이로드는 저장하지 않고 조인으로 푼다 (§10-3).
_WITH_EXPECTED = f"""
SELECT p.pending_id, p.station_ref, p.source_run_id::text AS source_run_id,
       p.ready_at, p.retry_count, p.status, p.task_id, p.created_at,
       cp.stack_payload AS expected_payload,
       EXTRACT(EPOCH FROM (p.ready_at - now())) AS seconds_remaining
FROM pending_pickup p
LEFT JOIN magazine_log ml
       ON ml.run_id = p.source_run_id AND ml.stage = 'place'
LEFT JOIN carrier_pair cp
       ON cp.magazine_payload = ml.qr_payload
"""


async def list_pickups(status: str | None = None) -> list[dict]:
    if status:
        return await db.fetch(
            _WITH_EXPECTED + " WHERE p.status = %s ORDER BY p.ready_at", (status,)
        )
    return await db.fetch(_WITH_EXPECTED + " ORDER BY p.status, p.ready_at")


async def get(pending_id: int) -> dict:
    row = await db.fetchrow(_WITH_EXPECTED + " WHERE p.pending_id = %s", (pending_id,))
    if row is None:
        raise NotFound(f"그런 회수 대기가 없다: {pending_id}", pending_id=pending_id)
    return row


# ── 5.1 — place 완료 → 행 생성 ───────────────────────────────────────
async def on_place_done(run_id: str, station_ref: str | None, ended_at: datetime) -> dict | None:
    """magazine_log 에 성공한 place 행이 들어오면 호출된다.

    station_ref 가 없으면(§4-5 — task_manager 가 port 를 대부분 모른다)
    회수를 걸지 않는다. 어디서 꺼낼지 모르는 채로 로봇을 보낼 수는 없다.
    """
    if not station_ref:
        log.warning("place 완료(run=%s)에 port 가 없어 회수를 걸지 않는다 (§4-5)", run_id)
        return None
    if configstore.station(station_ref) is None:
        log.warning("stations.yaml 에 없는 port=%s — 회수를 걸지 않는다", station_ref)
        return None

    ready_at = ended_at + timedelta(seconds=configstore.process_sec(station_ref))
    # pickup_one_per_run 이 중복을 막는다 — 스풀 재적재로 같은 로그가 두 번 와도 안전하다.
    row = await db.fetchrow(
        "INSERT INTO pending_pickup (station_ref, source_run_id, ready_at) "
        "VALUES (%s, %s::uuid, %s) "
        "ON CONFLICT DO NOTHING "
        f"RETURNING {_COLS}",
        (station_ref, run_id, ready_at),
    )
    if row:
        log.info("회수 예약 %s — %s 부터", station_ref, ready_at.isoformat())
    return row


# ── 5.2 — 스케줄러 ───────────────────────────────────────────────────
async def tick() -> list[dict]:
    """ready_at 이 지난 WAITING 을 RECOVER 작업으로 바꾼다. 갱신된 행을 돌려준다."""
    changed: list[dict] = await _reconcile_orphaned_queued()

    due = await db.fetch(
        f"SELECT {_COLS} FROM pending_pickup "
        "WHERE status = 'WAITING' AND ready_at <= now() ORDER BY ready_at"
    )
    for row in due:
        try:
            robot = await _pick_recover_robot()
            task = await queue.create(robot, row["station_ref"], kind="RECOVER")
        except Exception as e:  # 배정 실패는 회수 자체를 버릴 이유가 아니다
            log.warning("회수 배차 실패 pending=%s: %s", row["pending_id"], e)
            continue
        updated = await db.fetchrow(
            "UPDATE pending_pickup SET status = 'QUEUED', task_id = %s "
            f"WHERE pending_id = %s AND status = 'WAITING' RETURNING {_COLS}",
            (task["task_id"], row["pending_id"]),
        )
        if updated:
            dispatcher.wake(robot)   # 노는 로봇이면 즉시 회수하러 간다
            log.info("회수 배차 pending=%s → task=%s (%s)", row["pending_id"], task["task_id"], robot)
            # ★ 2026-09-25: robot_id 를 같이 실어 보낸다 — main.py 가 이걸로
            #   task_changed 도 같이 쏴야, 화면 "작업 큐" 패널(task_changed 로만
            #   새로고침한다)이 스케줄러가 만든 RECOVER 를 곧바로 보여준다.
            #   실측: task=29 가 QUEUED 로 잡혔는데도 화면엔 안 떴었다 —
            #   여기서 pickup_changed 만 쏘고 task_changed 를 안 쏘던 게 원인.
            changed.append({**updated, "robot_id": robot})
    return changed


async def _pick_recover_robot() -> str:
    """RECOVER 하나를 어느 로봇에게 줄지 고른다.

    ★ 2026-09-25: 원래 queue.auto_distribute()(작업 할당 화면이 선반 큐를
      재분배할 때 쓰던 함수)를 그대로 빌려 썼다. 그 함수는 QUEUED 작업만
      다시 나누는 계획을 돌려주는데, RECOVER 는 배차되자마자 QUEUED 목록이
      비어서 매 tick 마다 두 로봇 다 부하 0 — 그 동점을 robot_id 문자열
      비교(min 의 튜플 정렬)로 깨다 보니 "robot1" 이 알파벳순으로 항상
      이겼다. 예전엔 "작업 할당" 화면이 SCAN 작업도 같이 큐에 넣어서 이
      편향이 부하 차이에 묻혔는데, 그 화면을 assigned_robot 저장 방식으로
      바꾸면서(TaskAssignmentPage) task 표에 RECOVER 밖에 안 남아 100%
      robot1 로 드러났다 — "robot1 에게만 쌓인다" 는 이게 원인이었다.

      여기서는 QUEUED+RUNNING 을 합친 실제 부하로 먼저 거르고(바쁜 로봇은
      피한다), 그래도 동점이면 RECOVER 를 가장 오래 안 받은 로봇을 고른다
      — 그래야 실제로 돌아가며 처리된다.
    """
    robots = list(settings().robots)
    rows = await db.fetch(
        "SELECT robot_id, count(*) AS n FROM task "
        "WHERE status IN ('QUEUED', 'RUNNING') AND robot_id = ANY(%s) GROUP BY robot_id",
        (robots,),
    )
    load = {r: 0 for r in robots}
    for row in rows:
        load[row["robot_id"]] = row["n"]

    min_load = min(load.values())
    tied = [r for r in robots if load[r] == min_load]
    if len(tied) == 1:
        return tied[0]

    last = await db.fetchrow(
        "SELECT robot_id FROM task WHERE kind = 'RECOVER' AND robot_id = ANY(%s) "
        "ORDER BY created_at DESC LIMIT 1",
        (tied,),
    )
    if last is not None:
        others = [r for r in tied if r != last["robot_id"]]
        if others:
            return others[0]
    return tied[0]


async def _reconcile_orphaned_queued() -> list[dict]:
    """QUEUED 인데 그 task 가 이미 끝나 있는 행을 정리한다 (고아 행).

    ★ 2026-09-25: on_task_finished(task_id, success) 는 dispatcher._on_result
      (ROS 쪽 "task.result" 이벤트)가 불러야 도는데, 그 이벤트 한 번이
      유실되면(브리지 재연결 타이밍, 웹 백엔드가 안 재시작된 채 ROS 쪽만
      재시작 등) pending_pickup 은 영원히 QUEUED 로 남는다 — task 는 DONE/
      FAILED 로 끝났는데 표 6 만 그걸 모른다. 스케줄러가 매 tick 마다
      "표 5 기준으로" 다시 맞춰 보는 안전망이다. 이벤트가 제때 왔으면 여기서
      할 일이 없다(이미 WAITING/DONE 으로 넘어가 있어 이 질의에 안 걸린다).
    """
    orphaned = await db.fetch(
        f"SELECT p.pending_id, t.status AS task_status "
        f"FROM pending_pickup p JOIN task t ON t.task_id = p.task_id "
        f"WHERE p.status = 'QUEUED' AND t.status IN ('DONE', 'FAILED')"
    )
    changed: list[dict] = []
    for row in orphaned:
        log.warning(
            "회수 대기 pending=%s 가 이미 끝난 task 를 들고 QUEUED 로 남아 있었다"
            " (task_status=%s) — 놓친 task.result 이벤트를 뒤늦게 반영한다",
            row["pending_id"], row["task_status"],
        )
        updated = await (
            done(row["pending_id"]) if row["task_status"] == "DONE"
            else retry(row["pending_id"])
        )
        if updated:
            changed.append(updated)
    return changed


# ── 5.3 — 가 보니 없더라 ─────────────────────────────────────────────
async def retry(pending_id: int, delay_sec: int | None = None) -> dict:
    """비전 확인 실패. retry_count 를 올리고 다시 기다린다.

    이 칸이 표 6 을 없앨 수 없게 만드는 이유다 — "언제 회수 가능한가" 는
    magazine_latest 에서 파생할 수 있지만, "가 봤는데 아직 없어서 미뤘다" 는
    파생할 근거가 어디에도 없다(§10-3 ②).
    """
    s = settings()
    delay = delay_sec if delay_sec is not None else s.pickup_retry_backoff_sec
    row = await get(pending_id)

    if row["retry_count"] + 1 > s.pickup_max_retry:
        given_up = await db.fetchrow(
            "UPDATE pending_pickup SET status = 'GAVE_UP', retry_count = retry_count + 1 "
            f"WHERE pending_id = %s RETURNING {_COLS}",
            (pending_id,),
        )
        log.warning("회수 포기 pending=%s (%d회 실패)", pending_id, given_up["retry_count"])
        return given_up

    return await db.fetchrow(
        "UPDATE pending_pickup SET status = 'WAITING', task_id = NULL, "
        "retry_count = retry_count + 1, ready_at = now() + make_interval(secs => %s) "
        f"WHERE pending_id = %s RETURNING {_COLS}",
        (delay, pending_id),
    )


async def done(pending_id: int) -> dict:
    return await db.fetchrow(
        f"UPDATE pending_pickup SET status = 'DONE' WHERE pending_id = %s RETURNING {_COLS}",
        (pending_id,),
    )


async def on_task_finished(task_id: int, success: bool) -> dict | None:
    """RECOVER 작업이 끝났을 때 짝이 되는 회수 대기를 닫는다.

    실패면 재예약한다 — 작업은 실패했지만 **산출물은 여전히 거기 있다**.
    한 행으로 합칠 수 없는 이유가 바로 이것이다(§4-3 과 같은 구조).
    """
    row = await db.fetchrow(
        f"SELECT {_COLS} FROM pending_pickup WHERE task_id = %s AND status = 'QUEUED'",
        (task_id,),
    )
    if row is None:
        return None
    return await (done(row["pending_id"]) if success else retry(row["pending_id"]))


# ── 루프 ─────────────────────────────────────────────────────────────
async def run_scheduler(on_change) -> None:
    """lifespan 이 띄우는 백그라운드 태스크. 예외가 나도 루프를 죽이지 않는다."""
    period = settings().pickup_tick_sec
    log.info("회수 스케줄러 시작 (%.1fs 주기)", period)
    while True:
        try:
            for row in await tick():
                await on_change(row)
        except asyncio.CancelledError:
            raise
        except Exception:
            log.exception("회수 스케줄러 tick 실패 — 다음 주기에 다시 시도한다")
        await asyncio.sleep(period)

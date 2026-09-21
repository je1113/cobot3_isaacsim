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
    due = await db.fetch(
        f"SELECT {_COLS} FROM pending_pickup "
        "WHERE status = 'WAITING' AND ready_at <= now() ORDER BY ready_at"
    )
    changed: list[dict] = []
    for row in due:
        try:
            plan = await queue.auto_distribute()
            robot = min(plan, key=lambda r: (len(plan[r]), r))
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

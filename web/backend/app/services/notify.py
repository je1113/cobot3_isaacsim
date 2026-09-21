"""LISTEN trace_appended — 로그 INSERT 를 실시간으로 받는다 (WBS 6.4 · 5.1).

★ event_logger 와 웹 사이에 ROS 연결이 없다. 로거는 평소대로 INSERT 만 하고,
  DB 트리거(sql/004_notify.sql)가 pg_notify 를 쏘고, 여기서 받는다.
  로거는 웹의 존재를 모르고, 웹이 죽어 있어도 로거는 평소대로 돈다 —
  docs/DB구성.md §1 의 "기록 실패가 로봇 동작을 막지 않는다" 와 같은 방향이다.

★ 풀에서 커넥션을 빌리지 않고 **전용 커넥션**을 따로 연다.
  LISTEN 은 세션에 붙는 상태라, 풀이 커넥션을 돌려주는 순간 구독이 끊긴다.
"""

from __future__ import annotations

import asyncio
import json
import logging
from datetime import datetime

import psycopg

from ..config import settings

log = logging.getLogger(__name__)

CHANNEL = "trace_appended"


async def run_listener(on_trace, on_place_done) -> None:
    """끊기면 다시 붙는다. DB 재시작이 웹을 죽이지 않도록.

    on_trace(payload)      — 모든 로그 행 (WebSocket trace.appended)
    on_place_done(payload) — 성공한 magazine place 행만 (회수 예약 5.1)
    """
    backoff = 1.0
    while True:
        try:
            await _listen_once(on_trace, on_place_done)
            backoff = 1.0
        except asyncio.CancelledError:
            raise
        except Exception as e:
            log.warning("LISTEN 끊김 (%s) — %.0fs 뒤 재접속", e, backoff)
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 30.0)


async def _listen_once(on_trace, on_place_done) -> None:
    async with await psycopg.AsyncConnection.connect(settings().dsn, autocommit=True) as conn:
        await conn.execute(f"LISTEN {CHANNEL}")
        log.info("LISTEN %s 시작", CHANNEL)
        async for note in conn.notifies():
            try:
                payload = json.loads(note.payload)
            except json.JSONDecodeError:
                log.warning("깨진 notify payload 무시: %r", note.payload[:200])
                continue
            await _dispatch(payload, on_trace, on_place_done)


async def _dispatch(payload: dict, on_trace, on_place_done) -> None:
    await on_trace(payload)

    # 5.1 — 매거진을 스테이션에 성공적으로 내려놓은 순간이 회수 타이머의 시작이다.
    # stage 를 소문자로 맞춰 비교한다: TraceEvent 는 소문자 계약이지만,
    # 옛 판(대문자 PLACE)으로 쌓인 행이 섞여 있어도 걸리도록.
    if (
        payload.get("role") == "MAGAZINE"
        and str(payload.get("stage", "")).lower() == "place"
        and payload.get("succeeded")
    ):
        ended_at = _parse_ts(payload.get("ended_at"))
        if ended_at is None:
            log.warning("place 행에 ended_at 이 없다 — 회수를 걸지 않는다: %s", payload)
            return
        await on_place_done(payload["run_id"], payload.get("port"), ended_at)


def _parse_ts(raw) -> datetime | None:
    if not raw:
        return None
    try:
        return datetime.fromisoformat(str(raw))
    except ValueError:
        return None

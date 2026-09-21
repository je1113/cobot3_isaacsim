"""PostgreSQL 커넥션 풀.

★ docs/DB구성.md §1 은 "DB 커넥션을 가진 노드가 event_logger 하나뿐" 이라고
  적었는데, 그건 **로깅 경로에 한해서** 다(§10-7). 표 5·6 은 제어 상태라
  /trace/event 를 타지 않고, 웹 백엔드가 직접 쓴다. 이 풀이 그 두 번째 커넥션이다.

  로거가 죽어도 웹은 돌고, 웹이 죽어도 로거는 돈다 — 둘이 쓰는 표가 겹치지
  않기 때문이다. 웹은 표 1~4 를 읽기만 한다.

★ **DB 가 없어도 백엔드는 뜬다.** 풀을 wait=False 로 열어 두고, 붙을 때까지
  백그라운드에서 재시도한다. 그동안 DB 를 쓰는 API 는 503 으로 거절하고,
  yaml 만 쓰는 설정 화면은 평소대로 동작한다.

  기동 시점에 DB 를 요구하면 컨테이너 기동 순서에 백엔드가 묶인다. 그리고
  운영 중 DB 가 잠깐 끊겼을 때 되살아나지 못하는 것도 같은 문제다 —
  §2-9 의 "기록 실패가 로봇 동작을 막지 않는다" 와 같은 방향으로 맞춘 것이다.
"""

from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from typing import Any, Iterable

import psycopg
from psycopg.rows import dict_row
from psycopg_pool import AsyncConnectionPool, PoolTimeout

from .config import settings
from .errors import DatabaseUnavailable

log = logging.getLogger(__name__)

_pool: AsyncConnectionPool | None = None

# 풀에서 커넥션을 기다리는 시간. 짧게 둔다 — DB 가 없을 때 화면이 굳는 것보다
# 빨리 503 을 받고 "DB 안 붙었음" 을 보여 주는 편이 낫다.
_ACQUIRE_TIMEOUT = 3.0


async def open_pool() -> AsyncConnectionPool:
    global _pool
    if _pool is None:
        s = settings()
        _pool = AsyncConnectionPool(
            s.dsn,
            min_size=s.pool_min,
            max_size=s.pool_max,
            kwargs={"row_factory": dict_row, "autocommit": True},
            open=False,
        )
        # wait=False — 못 붙어도 예외를 내지 않고 백그라운드에서 계속 시도한다.
        await _pool.open(wait=False)
        log.info("DB 풀 생성 (%s) — 연결은 백그라운드에서 붙는다", _redact(s.dsn))
    return _pool


async def close_pool() -> None:
    global _pool
    if _pool is not None:
        await _pool.close()
        _pool = None


def pool() -> AsyncConnectionPool:
    if _pool is None:
        raise DatabaseUnavailable("DB 풀이 아직 안 열렸다 — lifespan 을 거치지 않은 호출")
    return _pool


async def ping() -> bool:
    try:
        await fetchval("SELECT 1")
        return True
    except Exception:
        return False


def _redact(dsn: str) -> str:
    if "@" not in dsn:
        return dsn
    head, tail = dsn.rsplit("@", 1)
    return f"{head.split('://')[0]}://***@{tail}"


@asynccontextmanager
async def _conn():
    """커넥션 하나. 못 얻으면 503 으로 번역한다 — 스택트레이스 대신 문장을 준다."""
    try:
        async with pool().connection(timeout=_ACQUIRE_TIMEOUT) as conn:
            yield conn
    except PoolTimeout as e:
        raise DatabaseUnavailable(
            f"DB 에 연결할 수 없다 ({_redact(settings().dsn)}). "
            "컨테이너가 떠 있는지, COBOT3_DSN 이 맞는지 확인할 것."
        ) from e
    except psycopg.OperationalError as e:
        raise DatabaseUnavailable(f"DB 연결이 끊겼다: {e}") from e


# ── 질의 헬퍼 ────────────────────────────────────────────────────────
# 전부 dict_row 라 결과가 그대로 JSON 직렬화 대상이 된다.


async def fetch(sql: str, params: Iterable[Any] | None = None) -> list[dict]:
    async with _conn() as conn:
        cur = await conn.execute(sql, params)
        return await cur.fetchall()


async def fetchrow(sql: str, params: Iterable[Any] | None = None) -> dict | None:
    async with _conn() as conn:
        cur = await conn.execute(sql, params)
        return await cur.fetchone()


async def fetchval(sql: str, params: Iterable[Any] | None = None) -> Any:
    row = await fetchrow(sql, params)
    if row is None:
        return None
    return next(iter(row.values()))


async def execute(sql: str, params: Iterable[Any] | None = None) -> int:
    async with _conn() as conn:
        cur = await conn.execute(sql, params)
        return cur.rowcount


@asynccontextmanager
async def tx():
    """여러 문장을 한 트랜잭션으로 — 큐 재정렬(4.3)이 이걸 쓴다.

    autocommit 풀이라 명시적으로 transaction() 을 열어야 한다.
    """
    async with _conn() as conn:
        async with conn.transaction():
            yield conn

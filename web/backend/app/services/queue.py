"""작업 큐 — 표 5(task) 를 다루는 모든 쓰기 (WBS 4단계).

★ WBS 0.3 확정: 이 파일이 task 표의 **유일한 writer** 다.
  task_manager 는 ExecuteTask.action 의 feedback/result 로 보고만 하고,
  그 보고를 받아 행을 고치는 것도 여기(apply_feedback / apply_result)다.

★ task 1건 = 캐리어 1개다(ExecuteTask.action 참고). 선반에 매거진이 3개면
  작업도 3건이고, 같은 선반에 QUEUED 가 여럿인 것이 정상이다. 몇 개인지는
  배정 시점에 모르므로 로봇이 result.more_at_target 으로 알려주고 웹이
  후속 작업을 건다(on_more_at_target).

★ 선반은 잠그지 않는다. 두 로봇이 같은 선반에 붙어 매거진을 나눠 빼는 것이
  정상 운용이다. 배타는 '로봇당 실행 중 하나' 뿐이다.

★ 불변식은 SQL 에 있지 파이썬에 있지 않다(sql/003_task.sql):
    task_one_running_per_robot 한 로봇이 동시에 둘을 RUNNING 못 한다 → 409
    task_queue_order           같은 로봇 큐에서 순서가 겹칠 수 없다  → 409
  경합을 파이썬에서 막으려 하면 두 요청이 동시에 들어올 때 샌다.
  DB 가 거부하게 두고 예외를 사람이 읽을 문장으로 번역한다.
"""

from __future__ import annotations

import logging
from typing import Any

from psycopg import errors as pgerr

from .. import db
from ..errors import Conflict, NotFound, Unprocessable
from . import configstore

log = logging.getLogger(__name__)

_COLS = """task_id, robot_id, kind, target_ref, queue_order, status,
           run_id::text AS run_id, resume_progress,
           created_at, started_at, ended_at"""


# ── 조회 ─────────────────────────────────────────────────────────────
async def get(task_id: int) -> dict:
    row = await db.fetchrow(f"SELECT {_COLS} FROM task WHERE task_id = %s", (task_id,))
    if row is None:
        raise NotFound(f"그런 작업이 없다: {task_id}", task_id=task_id)
    return row


async def queue_of(robot_id: str, include_done: bool = False) -> list[dict]:
    if include_done:
        return await db.fetch(
            f"SELECT {_COLS} FROM task WHERE robot_id = %s "
            "ORDER BY status = 'DONE', queue_order, created_at DESC",
            (robot_id,),
        )
    return await db.fetch(
        f"SELECT {_COLS} FROM task WHERE robot_id = %s AND status IN ('QUEUED','RUNNING') "
        "ORDER BY status DESC, queue_order",
        (robot_id,),
    )


async def list_tasks(status: str | None = None, limit: int = 200) -> list[dict]:
    if status:
        return await db.fetch(
            f"SELECT {_COLS} FROM task WHERE status = %s ORDER BY task_id DESC LIMIT %s",
            (status, limit),
        )
    return await db.fetch(f"SELECT {_COLS} FROM task ORDER BY task_id DESC LIMIT %s", (limit,))


# ── 배정 (4.1) ───────────────────────────────────────────────────────
async def create(
    robot_id: str,
    target_ref: str,
    kind: str = "SCAN",
    queue_order: int | None = None,
    resume_progress: float | None = None,
) -> dict:
    _check_robot(robot_id)
    await _check_target(kind, target_ref)

    if queue_order is None:
        queue_order = await _tail(robot_id) if kind == "SCAN" else await _head(robot_id)

    try:
        row = await db.fetchrow(
            "INSERT INTO task (robot_id, kind, target_ref, queue_order, resume_progress) "
            f"VALUES (%s, %s, %s, %s, %s) RETURNING {_COLS}",
            (robot_id, kind, target_ref, queue_order, resume_progress),
        )
    except pgerr.UniqueViolation as e:
        raise _translate(e, robot_id, target_ref) from e
    log.info("배정 task=%s %s %s → %s", row["task_id"], kind, target_ref, robot_id)
    return row


# ── 순서 교체 (4.3) ──────────────────────────────────────────────────
async def replace_order(robot_id: str, task_ids: list[int]) -> list[dict]:
    """그 로봇의 QUEUED 집합과 정확히 일치해야 한다.

    부분 교체를 허용하면 화면이 낡은 목록을 들고 있을 때 조용히 일부만
    반영된다 — 드래그로 옮긴 결과가 절반만 남는 것이 가장 나쁘다.
    """
    async with db.tx() as conn:
        cur = await conn.execute(
            "SELECT task_id FROM task WHERE robot_id = %s AND status = 'QUEUED' FOR UPDATE",
            (robot_id,),
        )
        current = {r["task_id"] for r in await cur.fetchall()}
        given = set(task_ids)
        if current != given:
            raise Conflict(
                "큐가 그 사이에 바뀌었다. 화면을 새로 고친 뒤 다시 정렬할 것.",
                missing=sorted(current - given),
                unexpected=sorted(given - current),
            )

        # 부분 유니크 인덱스(robot_id, queue_order) 때문에 한 번에 못 바꾼다.
        # 음수로 피신시킨 뒤 최종값을 쓴다 — 음수끼리는 원래 순서가 달랐으니 겹치지 않는다.
        await conn.execute(
            "UPDATE task SET queue_order = -queue_order - 1 "
            "WHERE robot_id = %s AND status = 'QUEUED'",
            (robot_id,),
        )
        for i, task_id in enumerate(task_ids):
            await conn.execute(
                "UPDATE task SET queue_order = %s WHERE task_id = %s", (i, task_id)
            )
    return await queue_of(robot_id)


async def replace_queue(robot_id: str, target_refs: list[str], kind: str = "SCAN") -> list[dict]:
    """그 로봇의 대기 큐를 이 목록으로 **통째로 교체**한다.

    화면의 '작업 시작' 이 쓰는 경로다. 화면에는 '추가' 만 있고 '동기화' 가 없어서,
    부분 반영을 하면 화면에서 지운 항목이 서버에 남는다.

    ★ RUNNING 은 건드리지 않는다 — 로봇이 이미 그 선반에 가 있다.
      화면의 큐에 없더라도 끝날 때까지 둔다.
    """
    _check_robot(robot_id)
    for ref in target_refs:
        await _check_target(kind, ref)

    async with db.tx() as conn:
        await conn.execute(
            "DELETE FROM task WHERE robot_id = %s AND status = 'QUEUED'", (robot_id,)
        )
        for order, ref in enumerate(target_refs):
            await conn.execute(
                "INSERT INTO task (robot_id, kind, target_ref, queue_order) VALUES (%s, %s, %s, %s)",
                (robot_id, kind, ref, order),
            )
    log.info("큐 교체 %s ← %s", robot_id, target_refs or "(비움)")
    return await queue_of(robot_id)


# ── 이관 (4.4) ───────────────────────────────────────────────────────
async def patch(task_id: int, robot_id: str | None, queue_order: int | None) -> dict:
    row = await get(task_id)
    if robot_id is not None and robot_id != row["robot_id"]:
        if row["status"] != "QUEUED":
            raise Conflict(
                f"{row['status']} 인 작업은 다른 로봇으로 옮길 수 없다. "
                "먼저 정지(STOP)하거나 끝날 때까지 기다릴 것.",
                status=row["status"],
            )
        _check_robot(robot_id)
        if queue_order is None:
            queue_order = await _tail(robot_id)

    try:
        updated = await db.fetchrow(
            "UPDATE task SET robot_id = COALESCE(%s, robot_id), "
            "queue_order = COALESCE(%s, queue_order) "
            f"WHERE task_id = %s RETURNING {_COLS}",
            (robot_id, queue_order, task_id),
        )
    except pgerr.UniqueViolation as e:
        raise _translate(e, robot_id or row["robot_id"], row["target_ref"]) from e
    return updated


# ── 취소 (4.5) ───────────────────────────────────────────────────────
async def cancel(task_id: int) -> None:
    row = await get(task_id)
    if row["status"] != "QUEUED":
        raise Conflict(
            f"{row['status']} 인 작업은 취소할 수 없다. "
            "실행 중이면 STOP 명령으로 세운다.",
            status=row["status"],
        )
    await db.execute("DELETE FROM task WHERE task_id = %s", (task_id,))


# ── 자동 분배 (4.6) ──────────────────────────────────────────────────
async def auto_distribute(robots: list[str] | None = None) -> dict[str, list[int]]:
    """분배안만 계산한다. 적용은 PUT /robots/{id}/queue 를 다시 쓴다.

    계산해서 바로 적용하지 않는 이유: 화면이 결과를 먼저 보여주고 사람이
    확인한 뒤 적용하기 때문이다. 그리고 적용 경로가 하나면 불변식 검사도 한 곳이다.
    """
    targets = robots or list(_robots())
    for r in targets:
        _check_robot(r)

    rows = await db.fetch(
        "SELECT task_id, robot_id, kind, queue_order FROM task "
        "WHERE status = 'QUEUED' AND robot_id = ANY(%s) "
        "ORDER BY kind = 'SCAN', queue_order",  # RECOVER 먼저
        (targets,),
    )
    # RUNNING 인 작업은 옮길 수 없으니 부하 계산에만 넣는다.
    busy = await db.fetch(
        "SELECT robot_id, count(*) AS n FROM task "
        "WHERE status = 'RUNNING' AND robot_id = ANY(%s) GROUP BY robot_id",
        (targets,),
    )
    load = {r: 0 for r in targets}
    for b in busy:
        load[b["robot_id"]] = b["n"]

    plan: dict[str, list[int]] = {r: [] for r in targets}
    for row in rows:
        pick = min(targets, key=lambda r: (load[r], r))
        plan[pick].append(row["task_id"])
        load[pick] += 1
    return plan


# ── 디스패처가 쓰는 것 ───────────────────────────────────────────────
async def claim_next(robot_id: str) -> dict | None:
    """큐 맨 앞을 RUNNING 으로 올린다. 이미 RUNNING 이 있으면 None.

    SKIP LOCKED 를 쓰지 않는 이유: 로봇당 디스패처가 하나라 경합이 없고,
    혹시 둘이 되더라도 task_one_running_per_robot 이 두 번째를 거부한다.
    """
    async with db.tx() as conn:
        cur = await conn.execute(
            "SELECT 1 FROM task WHERE robot_id = %s AND status = 'RUNNING'", (robot_id,)
        )
        if await cur.fetchone():
            return None
        cur = await conn.execute(
            "UPDATE task SET status = 'RUNNING', started_at = now() "
            "WHERE task_id = (SELECT task_id FROM task "
            "                 WHERE robot_id = %s AND status = 'QUEUED' "
            "                 ORDER BY queue_order LIMIT 1) "
            f"RETURNING {_COLS}",
            (robot_id,),
        )
        return await cur.fetchone()


async def apply_feedback(task_id: int, run_id: str | None, progress: float | None) -> dict | None:
    """ExecuteTask feedback → 행 갱신 (4.8).

    run_id 는 한 번만 찍힌다(COALESCE) — 재개로 두 번째 run_id 가 와도
    첫 값을 유지할지는 §11 #2 가 정해야 한다. 기본은 '새 run_id' 이므로
    그때 이 COALESCE 를 빼야 한다.
    """
    return await db.fetchrow(
        "UPDATE task SET run_id = COALESCE(run_id, %s::uuid), "
        "resume_progress = COALESCE(%s, resume_progress) "
        f"WHERE task_id = %s RETURNING {_COLS}",
        (run_id, progress, task_id),
    )


async def apply_result(
    task_id: int, success: bool, run_id: str | None = None, requeue: bool = False
) -> dict | None:
    """ExecuteTask result → 종료 (4.8).

    requeue=True 는 STOP 으로 중단된 작업을 큐 맨 앞으로 되돌린다.
    되돌릴지 버릴지는 화면이 정한다 — 팔에 캐리어를 든 채 멈췄을 수 있어
    서버가 자동으로 재시도하면 안 된다.
    """
    if requeue:
        row = await get(task_id)
        head = await _head(row["robot_id"])
        return await db.fetchrow(
            "UPDATE task SET status = 'QUEUED', queue_order = %s, started_at = NULL "
            f"WHERE task_id = %s RETURNING {_COLS}",
            (head, task_id),
        )
    return await db.fetchrow(
        "UPDATE task SET status = %s, ended_at = now(), run_id = COALESCE(run_id, %s::uuid) "
        f"WHERE task_id = %s RETURNING {_COLS}",
        ("DONE" if success else "FAILED", run_id, task_id),
    )


async def follow_up(task_id: int) -> dict | None:
    """result.more_at_target=true 이면 같은 선반에 후속 작업을 건다.

    task 1건 = 캐리어 1개라서 필요한 고리다. 배정 시점에는 선반에 몇 개가
    있는지 모르므로, 로봇이 "아직 더 있다" 를 알려주면 웹이 한 건 더 만든다.
    사람이 매번 다시 배정하지 않아도 선반이 빌 때까지 돈다.

    ★ **진척도(resume_progress)는 넘기지 않는다.** 그 값은 '이 작업을 어디까지
      했나' 이고, 다음 캐리어는 **새 작업**이라 scan 부터 시작해야 한다.
      물려주면 로봇이 스캔도 파지도 건너뛰고 빈 팔로 place 로 간다.
      (resume_progress 의 쓰임은 웹 복구 하나뿐이다 — 끊긴 그 작업을 다시
       시킬 때만 쓴다. ExecuteTask.action 참고)

    같은 로봇에게 준다 — 이미 그 자리에 복귀해 있으니 이동 비용이 0이다.
    다른 로봇이 같은 선반에서 같이 일하고 있어도 상관없다(선반은 안 잠근다).
    부하 균형이 필요하면 화면에서 자동 분배를 다시 돌리면 된다.
    """
    row = await get(task_id)
    if row["kind"] != "SCAN":
        return None
    try:
        return await create(row["robot_id"], row["target_ref"], kind="SCAN")
    except Conflict as e:
        log.info("후속 작업 생성 건너뜀 (%s): %s", row["target_ref"], e.message)
        return None


# ── 내부 ─────────────────────────────────────────────────────────────
def _robots() -> tuple[str, ...]:
    from ..config import settings

    return settings().robots


def _check_robot(robot_id: str) -> None:
    if robot_id not in _robots():
        raise Unprocessable(
            f"모르는 로봇: {robot_id}. COBOT3_ROBOTS 에 등록된 것만 쓸 수 있다.",
            known=list(_robots()),
        )


async def _check_target(kind: str, target_ref: str) -> None:
    """티칭 미완료 선반은 배정할 수 없다 (4.1 → 422).

    참조 무결성을 DB 가 못 거는 자리다 — target_ref 는 yaml 의 id 라 FK 가 아니다
    (§10-2 ①). 그래서 배정 시점에 백엔드가 파일을 읽어 검사한다.

    ★ 판정 규칙은 화면(ShelfSettingsPage.isTeachingComplete)과 **같아야 한다.**
      갈라지면 화면은 "작업 할당 가능" 이라 하고 서버는 422 를 낸다.
      규칙 자체는 shapes.shelf_is_taught 한 곳에만 있다.
    """
    from .. import shapes

    if kind == "SCAN":
        shelf = configstore.shelf(target_ref)
        if shelf is None:
            raise Unprocessable(
                f"shelves.yaml 에 없는 선반: {target_ref}", known=configstore.shelf_ids()
            )
        if not shapes.shelf_is_taught(shelf):
            raise Unprocessable(
                f"{target_ref} 는 티칭이 끝나지 않아 배정할 수 없다. "
                "설정 > 선반에서 모든 층의 관절값을 채울 것.",
                target_ref=target_ref,
            )
    else:
        if configstore.station(target_ref) is None:
            raise Unprocessable(
                f"stations.yaml 에 없는 스테이션: {target_ref}",
                known=configstore.station_ids(),
            )


async def _tail(robot_id: str) -> int:
    v = await db.fetchval(
        "SELECT coalesce(max(queue_order), -1) + 1 FROM task "
        "WHERE robot_id = %s AND status = 'QUEUED'",
        (robot_id,),
    )
    return int(v)


async def _head(robot_id: str) -> int:
    """회수(RECOVER)는 SCAN 보다 먼저 배차된다 (5.2) — 큐의 맨 앞."""
    v = await db.fetchval(
        "SELECT coalesce(min(queue_order), 1) - 1 FROM task "
        "WHERE robot_id = %s AND status = 'QUEUED'",
        (robot_id,),
    )
    return int(v)


def _translate(e: pgerr.UniqueViolation, robot_id: str, target_ref: str) -> Conflict:
    name = getattr(e.diag, "constraint_name", "") or ""
    if "task_one_running_per_robot" in name:
        return Conflict(f"{robot_id} 가 이미 다른 작업을 실행 중이다.", robot_id=robot_id)
    if "task_queue_order" in name:
        return Conflict("큐 순서가 겹친다. 화면을 새로 고친 뒤 다시 시도할 것.")
    return Conflict(f"제약 위반: {name or e}")

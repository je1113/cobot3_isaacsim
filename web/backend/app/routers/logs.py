"""로그 · 통계 — 뷰 4개를 읽기만 한다 (WBS 2.2 ~ 2.6).

★ 화면이 지금 쓰는 로그 엔드포인트는 여기가 아니라 routers/compat.py 의
  GET /api/logs 다 — 최상위가 배열이어야 하고 페이징이 없다.
  여기 /logs/page 는 keyset 페이징이 붙은 판이고, 로그가 커져서 화면이
  전부 받을 수 없게 되면 그때 화면을 이쪽으로 옮긴다.

★ 이 라우터는 표 1~4 에 절대 쓰지 않는다. append-only 의 유일한 writer 는
  event_logger 다(docs/DB구성.md §1). 웹이 한 행이라도 고치면 §8-1 이 지키려던
  증거 능력이 사라진다.

★ 페이징은 keyset 이다. OFFSET 은 로그가 계속 쌓이는 표에서 페이지가 밀린다 —
  2페이지를 보는 동안 새 행이 들어오면 같은 행을 두 번 보거나 건너뛴다.
  정렬 기준은 (ended_at, role, log_id) 셋이고, 앞의 둘만으로는 유일하지 않아
  log_id 가 타이브레이커로 필요하다.
"""

from __future__ import annotations

import base64
import csv
import io
from datetime import datetime

from fastapi import APIRouter, Query
from fastapi.responses import StreamingResponse

from .. import db
from ..config import settings
from ..errors import NotFound, Unprocessable

router = APIRouter(tags=["logs"])

_LOG_COLS = """role, log_id, run_id::text AS run_id, qr_payload, kind_code, robot_id, stage,
               started_at, ended_at, duration_sec, succeeded, status::text AS status,
               fail_reason::text AS fail_reason, fail_detail, port"""


# ── 커서 ─────────────────────────────────────────────────────────────
def _encode(row: dict) -> str:
    raw = f"{row['ended_at'].isoformat()}|{row['role']}|{row['log_id']}"
    return base64.urlsafe_b64encode(raw.encode()).decode().rstrip("=")


def _decode(cursor: str) -> tuple[datetime, str, int]:
    try:
        pad = "=" * (-len(cursor) % 4)
        ts, role, log_id = base64.urlsafe_b64decode(cursor + pad).decode().split("|")
        return datetime.fromisoformat(ts), role, int(log_id)
    except Exception as e:
        raise Unprocessable(f"커서를 읽을 수 없다: {cursor}") from e


# ── 2.3 로그 목록 ────────────────────────────────────────────────────
@router.get("/logs/page")
async def list_logs(
    robot_id: str | None = None,
    role: str | None = Query(None, pattern="^(MAGAZINE|STACK)$"),
    stage: str | None = None,
    qr_payload: str | None = None,
    status: str | None = Query(None, pattern="^(IN_TRANSIT|COMPLETED|FAILED)$"),
    failed_only: bool = False,
    factory: str | None = Query(None, description="0.4 — 라인이 아니라 공장 단위다"),
    after: str | None = Query(None, description="이전 응답의 next_cursor"),
    limit: int = Query(None, ge=1),
):
    s = settings()
    limit = min(limit or s.page_limit_default, s.page_limit_max)

    where, params = ["TRUE"], []
    if robot_id:
        where.append("robot_id = %s"); params.append(robot_id)
    if role:
        where.append("role = %s"); params.append(role)
    if stage:
        where.append("lower(stage) = lower(%s)"); params.append(stage)
    if qr_payload:
        where.append("qr_payload = %s"); params.append(qr_payload)
    if status:
        where.append("status = %s::run_status"); params.append(status)
    if failed_only:
        where.append("NOT succeeded")
    if factory:
        # 0.4 — 페이로드에 라인이 없다(§11 #4). 'F1-MGZB-1' 의 첫 토큰이 공장이다.
        where.append("split_part(qr_payload, '-', 1) = %s"); params.append(factory)
    if after:
        ts, r, log_id = _decode(after)
        where.append("(ended_at, role, log_id) < (%s, %s, %s)")
        params += [ts, r, log_id]

    rows = await db.fetch(
        f"SELECT {_LOG_COLS} FROM carrier_log WHERE {' AND '.join(where)} "
        "ORDER BY ended_at DESC, role DESC, log_id DESC LIMIT %s",
        (*params, limit + 1),
    )
    has_more = len(rows) > limit
    rows = rows[:limit]
    return {
        "items": rows,
        "next_cursor": _encode(rows[-1]) if has_more and rows else None,
    }


# ── 2.3 미션 1회 ─────────────────────────────────────────────────────
@router.get("/runs/{run_id}")
async def get_run(run_id: str):
    rows = await db.fetch(
        f"SELECT {_LOG_COLS} FROM carrier_log WHERE run_id = %s::uuid ORDER BY log_id",
        (run_id,),
    )
    if not rows:
        raise NotFound(f"그런 미션이 없다: {run_id}", run_id=run_id)
    task = await db.fetchrow(
        "SELECT task_id, robot_id, kind::text AS kind, target_ref, status::text AS status "
        "FROM task WHERE run_id = %s::uuid",
        (run_id,),
    )
    # run_id 하나가 '배정' 과 '이력' 두 세계를 잇는다 (§10-5).
    return {"run_id": run_id, "task": task, "stages": rows}


@router.get("/carriers/{qr_payload}")
async def get_carrier(qr_payload: str):
    rows = await db.fetch(
        f"SELECT {_LOG_COLS} FROM carrier_log WHERE qr_payload = %s "
        "ORDER BY ended_at DESC, log_id DESC LIMIT 200",
        (qr_payload,),
    )
    if not rows:
        raise NotFound(f"그 캐리어의 기록이 없다: {qr_payload}", qr_payload=qr_payload)
    return {"qr_payload": qr_payload, "history": rows}


@router.get("/tracking")
async def tracking():
    """로트 현황 — 매거진·스택 한 줄 (production_tracking 뷰).

    한 번도 이송하지 않은 개체는 종류가 NULL 이다 — 종류는 로그 행이 들고 오고,
    carrier_pair 는 페이로드 두 개만 갖기 때문이다(§6).
    """
    return await db.fetch(
        "SELECT * FROM production_tracking ORDER BY magazine_at DESC NULLS LAST"
    )


# ── 2.3 통계 ─────────────────────────────────────────────────────────
@router.get("/stats")
async def stats():
    by_type = await db.fetch(
        "SELECT k.carrier_type, "
        "  count(*) FILTER (WHERE l.stage = 'pick') AS attempts, "
        "  count(*) FILTER (WHERE l.status = 'COMPLETED' AND l.stage = 'place') AS completed "
        "FROM carrier_log l LEFT JOIN carrier_kind k USING (kind_code) "
        "WHERE k.carrier_type IS NOT NULL GROUP BY k.carrier_type ORDER BY 1"
    )
    failures = await db.fetch(
        "SELECT stage, fail_reason::text AS fail_reason, fail_detail, count(*) AS n "
        "FROM carrier_log WHERE NOT succeeded "
        "GROUP BY 1,2,3 ORDER BY n DESC LIMIT 50"
    )
    durations = await db.fetch(
        "SELECT stage, round(avg(duration_sec)::numeric, 1) AS avg_sec, count(*) AS n "
        "FROM carrier_log WHERE succeeded AND duration_sec IS NOT NULL "
        "GROUP BY stage ORDER BY stage"
    )
    in_transit = await db.fetch(
        "SELECT role, robot_id, qr_payload, stage, ended_at FROM carrier_log "
        "WHERE status = 'IN_TRANSIT' AND stage IN ('pick','nav') ORDER BY ended_at DESC"
    )
    # ★ duration_sec 은 시뮬 시각 기준이다(§4-2). 벽시계와 2배 가까이 차이 나므로
    #   화면에 "사이클 타임" 으로 쓸 값은 이쪽이 맞다.
    return {
        "by_type": by_type,
        "failures": failures,
        "durations_sim_sec": durations,
        "in_transit": in_transit,
    }


# ── 2.2 종류 카탈로그 ────────────────────────────────────────────────
@router.get("/carrier-kinds")
async def carrier_kinds():
    return await db.fetch(
        "SELECT kind_code, family, color, carrier_type, "
        "       length_mm, width_mm, height_mm, mass_kg, spec_extra "
        "FROM carrier_kind ORDER BY family, color"
    )


@router.get("/carrier-pairs")
async def carrier_pairs():
    return await db.fetch("SELECT * FROM carrier_pair ORDER BY magazine_payload")


# ── 2.4 CSV ──────────────────────────────────────────────────────────
@router.get("/logs.csv")
async def logs_csv(robot_id: str | None = None, failed_only: bool = False, limit: int = 10000):
    where, params = ["TRUE"], []
    if robot_id:
        where.append("robot_id = %s"); params.append(robot_id)
    if failed_only:
        where.append("NOT succeeded")
    rows = await db.fetch(
        f"SELECT {_LOG_COLS} FROM carrier_log WHERE {' AND '.join(where)} "
        "ORDER BY ended_at DESC, log_id DESC LIMIT %s",
        (*params, min(limit, 100000)),
    )

    def gen():
        buf = io.StringIO()
        w = csv.writer(buf)
        header = [
            "role", "log_id", "run_id", "qr_payload", "kind_code", "robot_id", "stage",
            "started_at", "ended_at", "duration_sec_sim", "succeeded", "status",
            "fail_reason", "fail_detail", "port",
        ]
        w.writerow(header)
        yield buf.getvalue(); buf.seek(0); buf.truncate()
        for r in rows:
            w.writerow([r.get(k.replace("duration_sec_sim", "duration_sec")) for k in header])
            yield buf.getvalue(); buf.seek(0); buf.truncate()

    return StreamingResponse(
        gen(),
        media_type="text/csv; charset=utf-8",
        headers={"Content-Disposition": 'attachment; filename="carrier_log.csv"'},
    )

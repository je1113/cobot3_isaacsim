"""AMR 매거진 이송 관제 — 웹 백엔드.

    uvicorn app.main:app --reload --port 8000     (web/backend 에서)

★ 조각들이 어떻게 맞물리는가:

    화면 ──REST──> routers ──> services ──> PostgreSQL (표 5·6 쓰기, 표 1~4 읽기)
                                    │
                                    └──> rosbridge ──action/srv──> task_manager

    PostgreSQL ──NOTIFY──> notify.py ──┬──> hub (trace.appended)
     (event_logger 가 INSERT)          └──> pickup.on_place_done   5.1

    task_manager ──feedback──> rosbridge ──> dispatcher ──┬──> 표 5·6 갱신
                                                          └──> hub (task.changed)

★ ROS 없이도 뜬다(COBOT3_ROS=0). 읽기 API·설정 편집·큐 관리는 전부 동작하고,
  막히는 것은 '로봇에게 실제로 시키는' 부분뿐이며 그건 503 으로 명확히 거절한다.
"""

from __future__ import annotations

import asyncio
import logging

from contextlib import asynccontextmanager

from fastapi import APIRouter, FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from . import db
from .config import settings
from .errors import ApiError, api_error_handler
from .routers import commands, compat, logs, pickups, settings as settings_routes, tasks, ws_routes
from .services import dispatcher, notify, pickup
from .services.rosbridge import bridge
from .ws import hub

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-7s %(name)s — %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("cobot3.web")


@asynccontextmanager
async def lifespan(app: FastAPI):
    s = settings()
    background: list[asyncio.Task] = []

    # DB 가 아직 없어도 뜬다 — 붙을 때까지 백그라운드에서 재시도하고,
    # 그동안 DB 를 쓰는 API 만 503 을 낸다(db.py 주석 참고).
    await db.open_pool()
    if not await db.ping():
        log.warning("DB 에 아직 못 붙었다 — 설정 화면은 동작하고 나머지는 503 이다")

    bridge.on_event = dispatcher.on_bridge_event
    await bridge.start()

    # 6.4 + 5.1 — 로그 INSERT 를 pg_notify 로 받는다. ROS 를 거치지 않는다.
    background.append(
        asyncio.create_task(
            notify.run_listener(
                on_trace=lambda payload: hub.publish("trace_appended", payload, payload.get("robot_id")),
                on_place_done=_place_done,
            ),
            name="listen:trace",
        )
    )
    # 5.2 — ready_at 도래 감시
    background.append(
        asyncio.create_task(
            pickup.run_scheduler(lambda row: hub.publish("pickup_changed", row)),
            name="pickup:scheduler",
        )
    )
    # 4.7 — 로봇별 배차 루프
    dispatcher.start_all(background)

    log.info(
        "백엔드 시작 — 로봇 %s · ROS %s · 설정 %s",
        ", ".join(s.robots),
        "켜짐" if s.ros_enabled else "꺼짐",
        s.config_dir,
    )
    try:
        yield
    finally:
        for t in background:
            t.cancel()
        await asyncio.gather(*background, return_exceptions=True)
        await bridge.stop()
        await db.close_pool()
        log.info("백엔드 종료")


async def _place_done(run_id, station_ref, ended_at) -> None:
    row = await pickup.on_place_done(run_id, station_ref, ended_at)
    if row:
        await hub.publish("pickup_changed", row)


app = FastAPI(
    title="AMR 매거진 이송 관제 API",
    version="0.1.0",
    lifespan=lifespan,
    description=__doc__,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=list(settings().cors_origins),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["ETag"],   # 3.2 — 화면이 revision 을 읽어야 If-Match 를 보낼 수 있다
)

app.add_exception_handler(ApiError, api_error_handler)

# ── API 는 전부 /api 아래 ─────────────────────────────────────────────
# 프론트(web/frontend)가 src/api/robots.js 에서 "/api/robots/{id}/pause|resume" 을
# 하드코딩하고 있어 그 접두사에 맞춘다. 덤으로 "/" 를 SPA 에 통째로 내줄 수
# 있게 된다 — 라우트 이름과 화면 경로가 겹칠 걱정이 없어진다.
api = APIRouter(prefix="/api")
# ★ compat 먼저 — 화면이 부르는 경로(/tasks/start, /logs, /robots/{id}/pause)가
#   아래 일반 라우터의 /tasks/{task_id} 같은 패턴에 먹히지 않도록.
api.include_router(compat.router)
api.include_router(settings_routes.router)
api.include_router(logs.router)
api.include_router(tasks.router)
api.include_router(pickups.router)
api.include_router(commands.router)
app.include_router(api)

# WebSocket 은 /ws 그대로 둔다 — 프론트의 VITE_WS_BASE_URL 이 전체 URL 을
# 받으므로 접두사를 맞출 필요가 없고, 짧은 편이 프록시 설정도 쉽다.
app.include_router(ws_routes.router)


@app.get("/api/health", tags=["meta"])
async def health():
    """의존성 셋을 각각 따로 본다 — 하나가 죽어도 어느 것인지 바로 알 수 있게."""
    try:
        await db.fetchval("SELECT 1")
        database = "ok"
    except Exception as e:
        database = f"error: {e}"
    return {
        "database": database,
        "ros_bridge": "ok" if bridge.available else ("off" if not settings().ros_enabled else "starting"),
        "ws_clients": hub.client_count,
        "robots": list(settings().robots),
    }


# ── 프론트엔드 ───────────────────────────────────────────────────────
# React + Vite 앱의 **빌드 결과물**(web/frontend/dist)을 서빙한다.
#
#   개발 중  : npm run dev (5173) → 위 CORS 설정과 vite proxy 를 탄다
#   배포     : npm run build → dist/ 가 생기고 그때부터 여기서 같이 나간다
#
# ★ web/frontend 자체를 마운트하면 안 된다. 그 index.html 은 /src/main.jsx 를
#   가리키는 Vite 진입점이라 번들러 없이는 동작하지 않는다.
#
# ★ 반드시 **맨 마지막**에 마운트한다. "/" 마운트는 어떤 경로에나 걸리므로,
#   위 라우터들보다 먼저 등록하면 /tasks 같은 API 가 전부 가려진다.
_FRONTEND = (settings().root / "web/frontend/dist").resolve()
if _FRONTEND.is_dir():
    app.mount("/", StaticFiles(directory=_FRONTEND, html=True), name="frontend")
else:
    log.info("프론트엔드 빌드가 없다(%s) — 개발 중에는 npm run dev 로 5173 에서 띄운다", _FRONTEND)

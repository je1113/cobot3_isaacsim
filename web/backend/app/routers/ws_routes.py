"""WebSocket 엔드포인트.

프로토콜은 사실상 **단방향**이다. 프론트의 useWebSocket 은 보낼 수단을 노출하지
않는다(`send` 가 없다) — 연결하고 받기만 한다. 그래서:

  · 연결하면 곧바로 전부 받는다. 구독 요청을 기다리지 않는다.
  · 그래도 subscribe / ping 은 받아 둔다. 나중에 화면이 트래픽을 줄이고
    싶어질 때 서버를 다시 고치지 않아도 되도록.

★ 연결 직후 스냅샷을 밀지 않는다. 화면이 REST 로 현재 상태를 먼저 읽고 그
  다음부터 이 채널을 듣는 순서다. 여기서도 스냅샷을 보내면 REST 응답과 WS 첫
  메시지 중 어느 쪽이 최신인지 알 수 없게 된다.

★ 훅에 재연결이 없다(onClose 가 그냥 DISCONNECTED 로 두고 끝난다). 서버가
  멀쩡히 살아 있어도 한 번 끊기면 화면은 영영 조용해진다 — 프론트 쪽에
  재연결을 넣는 것이 맞고, 그때까지는 새로고침이 유일한 복구다.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..ws import TYPES, hub

log = logging.getLogger(__name__)
router = APIRouter()


@router.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    client = await hub.join(ws)
    pump = asyncio.create_task(hub.pump(client))
    try:
        while True:
            msg = await ws.receive_json()
            op = msg.get("op")
            if op == "subscribe":
                wanted = {t for t in (msg.get("types") or []) if t in TYPES}
                # 빈 목록은 '전부' 로 되돌린다 — 실수로 전부 끊기는 것보다 낫다.
                client.types = wanted or None
            elif op == "ping":
                await ws.send_json({"type": "pong", "seq": 0, "ts": None})
    except WebSocketDisconnect:
        pass
    except Exception:
        # 훅이 아무것도 안 보내므로 receive_json 은 보통 끊길 때까지 그냥 막혀 있다.
        log.debug("WS 연결 종료", exc_info=True)
    finally:
        pump.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await pump
        await hub.leave(client)

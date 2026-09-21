"""WebSocket 허브.

봉투는 **프론트가 파싱하는 모양**이다(hooks/useWebSocket.js + MonitoringPage):

    {"type": "robot_state", "seq": 41, "ts": "...", "robots": [...]}
     ^^^^                                            ^^^^^^
     화면이 분기하는 값                               최상위여야 한다

★ payload 를 `data` 로 감싸지 않는다. 화면이 `message.robots` 를 **최상위에서**
  찾기 때문이다(`message.robots || [message.robot] || [message]`).

★ 구독을 요구하지 않는다. 프론트는 subscribe 를 **보내지 않는다** — 연결만
  하고 받기만 한다. 그래서 기본이 '전부 수신' 이고, subscribe 는 굳이 줄이고
  싶을 때만 쓰는 선택지다. 기본을 '아무것도 안 보냄' 으로 두면 화면은 조용히
  빈 채로 돌고, 그게 가장 알아차리기 어려운 고장이다.

★ `seq` 와 `ts` 를 항상 싣는다. 훅이 `lastMessage` 에 **마지막 문자열 하나만**
  들고 있어서, 연속된 두 메시지가 완전히 같으면 React 가 리렌더를 건너뛴다.
  매번 달라지는 칸이 하나는 있어야 한다.

느린 클라이언트 하나가 전체를 막지 않도록 전송은 클라이언트별 큐를 거친다.
큐가 차면 그 클라이언트의 메시지를 버린다 — seq 결번으로 드러난다.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from datetime import datetime, timezone
from typing import Any

from fastapi import WebSocket

log = logging.getLogger(__name__)

# 메시지 타입. 화면이 지금 쓰는 것은 robot_state 하나뿐이고 나머지는 무시한다
# (`if (message.type !== 'robot_state') return`). 나머지는 화면이 붙을 자리다.
TYPES = {
    "robot_state",      # 로봇 위치·상태·현재 작업 — 저장하지 않는다
    "task_changed",     # 표 5 가 바뀌었다
    "pickup_changed",   # 표 6 이 바뀌었다
    "trace_appended",   # 로그가 한 줄 쌓였다 (pg_notify 경유)
    "alert",            # payload 없는 사건 (§4-6 구멍)
    "command_result",   # 제어 명령의 실제 결과 (REST 는 202 로 먼저 답한다)
    "config_changed",   # 설정 yaml 이 저장됐다 — 다른 탭이 열려 있을 때
}

_QUEUE_MAX = 256


class Client:
    def __init__(self, ws: WebSocket):
        self.ws = ws
        self.types: set[str] | None = None   # None = 전부 받는다 (기본)
        self.queue: asyncio.Queue[dict] = asyncio.Queue(maxsize=_QUEUE_MAX)
        self.dropped = 0

    def wants(self, msg_type: str) -> bool:
        return self.types is None or msg_type in self.types

    def offer(self, msg: dict) -> None:
        try:
            self.queue.put_nowait(msg)
        except asyncio.QueueFull:
            self.dropped += 1   # seq 결번으로 드러난다


class Hub:
    def __init__(self) -> None:
        self._clients: set[Client] = set()
        self._seq = 0
        self._lock = asyncio.Lock()

    async def join(self, ws: WebSocket) -> Client:
        client = Client(ws)
        async with self._lock:
            self._clients.add(client)
        return client

    async def leave(self, client: Client) -> None:
        async with self._lock:
            self._clients.discard(client)
        if client.dropped:
            log.warning("클라이언트 종료 — 큐 넘침으로 %d건 버림", client.dropped)

    async def publish(
        self, msg_type: str, payload: dict[str, Any] | None = None, robot_id: str | None = None
    ) -> None:
        """payload 의 키를 **최상위에 펼쳐서** 보낸다 (화면이 그렇게 읽는다).

        robot_id 를 따로 받는 것은 호출부 대부분이 "이 로봇에 대한 일" 을
        알리기 때문이다. payload 안에 이미 있으면 그쪽을 존중한다.
        """
        if msg_type not in TYPES:
            raise ValueError(f"모르는 메시지 타입: {msg_type}")
        self._seq += 1
        body = dict(payload or {})
        if robot_id is not None:
            body.setdefault("robot_id", robot_id)
        msg = {
            "type": msg_type,
            "seq": self._seq,
            "ts": datetime.now(timezone.utc).isoformat(),
            **body,
        }
        async with self._lock:
            targets = [c for c in self._clients if c.wants(msg_type)]
        for c in targets:
            c.offer(msg)

    async def publish_robot_state(self, robots: list[dict]) -> None:
        """MonitoringPage 가 실제로 소비하는 유일한 메시지."""
        if robots:
            await self.publish("robot_state", {"robots": robots})

    async def pump(self, client: Client) -> None:
        try:
            while True:
                msg = await client.queue.get()
                await client.ws.send_json(msg)
        except asyncio.CancelledError:
            raise
        except Exception:
            with contextlib.suppress(Exception):
                await client.ws.close()

    @property
    def client_count(self) -> int:
        return len(self._clients)


hub = Hub()

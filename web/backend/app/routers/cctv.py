"""CCTV — 씬 천장 카메라의 실시간 영상 (실시간 모니터링 Top View 아래 카드).

    GET /api/cctv.mjpeg    multipart/x-mixed-replace 스트림. <img src> 로 바로 본다.
    GET /api/cctv/status   영상이 오고 있는지. 화면이 "영상 없음" 을 가른다.

★ WebSocket 허브를 쓰지 않는다. 프레임이 크고(수십 KB) 초당 몇 장씩 오므로,
  같은 줄에 태우면 robot_state 같은 작은 메시지가 그 뒤에 밀린다.

★ 저장하지 않는다. 브리지가 마지막 한 장만 들고 있다(rosbridge._on_cctv).
  그래서 여러 탭이 봐도 인코딩은 한 번이고, 느린 탭은 중간 장을 건너뛴다.

★ 영상의 출처는 simple_factory_layout.usda 의 /World/Environment/CCTV_Camera
  → /World/Graph/ROS_CCTV(ROS2CameraHelper) → /cctv/image_raw 다.
  sim_backend 를 SIM_HEADLESS=1 로 띄우면 렌더를 안 하므로(world.step(render=False))
  영상이 안 나온다.
"""

from __future__ import annotations

import asyncio
import time

from fastapi import APIRouter, Request
from fastapi.responses import StreamingResponse

from ..config import settings
from ..errors import BridgeUnavailable
from ..services.rosbridge import bridge

router = APIRouter(tags=["cctv"])

_BOUNDARY = "cctvframe"
# 이 시간 넘게 새 장이 없으면 "끊겼다" 로 본다. 발행이 ~7 fps 라 넉넉하다.
_STALE_SEC = 3.0


def _require() -> None:
    if not settings().cctv_topic:
        raise BridgeUnavailable("CCTV 가 꺼져 있다 (COBOT3_CCTV_TOPIC 이 비었다)")
    if not bridge.available:
        raise BridgeUnavailable("ROS 브리지가 꺼져 있어 CCTV 영상이 없다 (COBOT3_ROS=1 로 켠다)")


@router.get("/cctv/status")
async def cctv_status():
    frame = bridge.latest_cctv()
    age = None if frame is None else round(time.monotonic() - frame.received, 2)
    return {
        "enabled": bool(settings().cctv_topic) and bridge.available,
        "topic": settings().cctv_topic,
        "live": frame is not None and age is not None and age < _STALE_SEC,
        "age_sec": age,
        "width": frame.width if frame else None,
        "height": frame.height if frame else None,
    }


@router.get("/cctv.mjpeg")
async def cctv_stream(request: Request):
    _require()
    period = 1.0 / max(settings().cctv_fps, 0.1)

    async def frames():
        last = None
        while not await request.is_disconnected():
            frame = bridge.latest_cctv()
            if frame is not None and frame.seq != last:
                last = frame.seq
                yield (
                    f"--{_BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame.jpeg)}\r\n\r\n"
                ).encode() + frame.jpeg + b"\r\n"
            # 새 장을 기다린다. 발행 주기의 절반마다 보면 지연이 반 주기 이하다.
            await asyncio.sleep(period / 2)

    return StreamingResponse(
        frames(),
        media_type=f"multipart/x-mixed-replace; boundary={_BOUNDARY}",
        headers={"Cache-Control": "no-store"},
    )

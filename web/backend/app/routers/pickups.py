"""산출물 회수 — 표 6 (WBS 2.5 · 5단계)."""

from __future__ import annotations

from fastapi import APIRouter, Query

from ..models import PickupRetry
from ..services import pickup
from ..ws import hub

router = APIRouter(prefix="/pending-pickups", tags=["pickups"])


@router.get("")
async def list_pickups(status: str | None = Query(None, pattern="^(WAITING|QUEUED|DONE|GAVE_UP)$")):
    """expected_payload 와 seconds_remaining 은 표에 없다 — 조인·계산해서 낸다(§10-3).

    화면의 카운트다운은 seconds_remaining 을 한 번 받아 브라우저가 세면 된다.
    서버가 매초 밀어 줄 이유가 없다.
    """
    return await pickup.list_pickups(status)


@router.get("/{pending_id}")
async def get_pickup(pending_id: int):
    return await pickup.get(pending_id)


@router.post("/{pending_id}/retry")
async def retry_pickup(pending_id: int, body: PickupRetry):
    """가 보니 아직 없더라 (5.3). retry_count 를 올리고 ready_at 을 미룬다.

    최대 횟수를 넘기면 GAVE_UP — 로봇이 같은 자리를 무한히 왕복하지 않게.
    """
    row = await pickup.retry(pending_id, body.delay_sec)
    await hub.publish("pickup_changed", row)
    return row


@router.post("/{pending_id}/done")
async def finish_pickup(pending_id: int):
    """사람이 직접 치웠을 때. 로봇이 회수했으면 dispatcher 가 알아서 닫는다."""
    row = await pickup.done(pending_id)
    await hub.publish("pickup_changed", row)
    return row

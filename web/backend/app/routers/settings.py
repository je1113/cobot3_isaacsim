"""설정 — 선반 / 스테이션 / 공정 흐름 (화면 3탭).

★ 화면이 들고 있는 모양 그대로 주고받는다. 필드 이름도, 숫자를 문자열로
  싣는 것도 전부 프론트 기준이다(app/shapes.py 가 번역한다).

★ **컬렉션 통째로** PUT 한다. 화면이 배열 하나를 통째로 쥐고 항목을
  추가·삭제·수정하기 때문이다 — 부분 PATCH 를 만들면 화면 쪽에 없는
  개념(어느 항목이 바뀌었는지 추적)을 새로 만들어야 한다.

★ 설정의 주인은 DB 가 아니라 파일이다(docs/DB구성.md §10-1 · §8-5).
  그래서 이 라우터는 DB 없이도 전부 동작한다.

★ 동시 편집은 revision(파일 내용 해시)으로 막는다. GET 이 준 revision 을
  PUT 이 If-Match 로 되돌려주지 않으면 428, 그 사이 파일이 바뀌었으면 412.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from fastapi import APIRouter, Body, Header, Response

from .. import shapes, yamlstore
from ..config import settings
from ..errors import ApiError, Unprocessable
from ..services import configstore
from ..services.rosbridge import bridge
from ..ws import hub

log = logging.getLogger(__name__)
router = APIRouter(tags=["settings"])


class MissingIfMatch(ApiError):
    status, code = 428, "if_match_required"


async def _save(path: Path, doc: Any, if_match: str | None, scope: str) -> str:
    if not if_match:
        raise MissingIfMatch(
            "If-Match 헤더에 GET 으로 받은 revision 을 실어야 한다. "
            "없이 저장하면 남의 편집을 덮어쓴다."
        )
    rev = yamlstore.save(path, doc, if_match.strip('"'))
    configstore.invalidate(path)
    await hub.publish("config_changed", {"scope": scope, "revision": rev})

    # 떠 있는 노드에 "다시 읽어라" 를 보낸다. 못 보내도 실패로 치지 않는다 —
    # 반영 방식이 아직 열려 있고(§11 #3), 다음 미션에서 어차피 다시 읽는다.
    for robot in settings().robots:
        try:
            res = await bridge.reload_config(settings().to_ns(robot), scope)
            if not res.get("reloaded"):
                log.info("%s reload 미반영: %s", robot, res.get("rejected_because"))
        except ApiError as e:
            log.info("%s 에 reload 를 못 보냈다: %s", robot, e.message)
    return rev


def _unique_ids(items: list[dict], key: str) -> list[str]:
    """화면과 **같은 규칙**으로 검사한다: trim 후 비어 있지 않고, 중복 없음."""
    ids = []
    for i, item in enumerate(items):
        raw = str(item.get(key) or "").strip()
        if not raw:
            raise Unprocessable(f"{i + 1}번째 항목에 {key} 가 없다", index=i)
        if raw in ids:
            raise Unprocessable(f"{key} 가 중복된다: {raw}", duplicate=raw)
        ids.append(raw)
    return ids


# ── 선반 ─────────────────────────────────────────────────────────────
@router.get("/shelves")
async def get_shelves(response: Response):
    doc, rev = configstore.doc(settings().shelves_yaml)
    response.headers["ETag"] = rev
    items = doc.get("shelves") or []
    return {
        "revision": rev,
        "shelves": [shapes.shelf_out(s.get("shelf_id", ""), s) for s in items],
    }


@router.put("/shelves")
async def put_shelves(
    body: dict = Body(...), if_match: str | None = Header(None, alias="If-Match")
):
    incoming = body.get("shelves")
    if not isinstance(incoming, list):
        raise Unprocessable("shelves 는 배열이어야 한다", got=type(incoming).__name__)
    ids = _unique_ids(incoming, "shelf_id")

    path = settings().shelves_yaml
    doc, _ = yamlstore.load(path)
    kept = {str(s.get("shelf_id")): s for s in (doc.get("shelves") or [])}

    merged = []
    for shelf_id, item in zip(ids, incoming):
        target = kept.get(shelf_id)
        converted = {"shelf_id": shelf_id, **shapes.shelf_in(item)}
        if target is not None:
            # ★ 로봇 쪽 필드는 화면이 안 보내면 기존 값을 지킨다. merge_into 는
            #   src 에 없는 키를 지우므로, 여기서 넘겨주지 않으면 저장 한 번에
            #   assigned_robot 이 사라지고 task_manager 가 자기 선반을 잃는다
            #   (그 노드의 _resolve_patrol_route ★★).
            if "assigned_robot" not in converted and "assigned_robot" in target:
                converted["assigned_robot"] = target["assigned_robot"]
            # 제자리 병합 — 갈아끼우면 항목에 붙은 주석이 날아간다(yamlstore 참고)
            merged.append(yamlstore.merge_into(target, converted))
        else:
            merged.append(converted)
    doc["shelves"] = merged

    rev = await _save(path, doc, if_match, "shelves")
    return {"revision": rev, "shelves": [shapes.shelf_out(s["shelf_id"], s) for s in merged]}


# ── 스테이션 ─────────────────────────────────────────────────────────
@router.get("/stations")
async def get_stations(response: Response):
    doc, rev = configstore.doc(settings().stations_yaml)
    response.headers["ETag"] = rev
    items = doc.get("stations") or []
    return {
        "revision": rev,
        "stations": [shapes.station_out(s.get("station_id", ""), s) for s in items],
    }


@router.put("/stations")
async def put_stations(
    body: dict = Body(...), if_match: str | None = Header(None, alias="If-Match")
):
    incoming = body.get("stations")
    if not isinstance(incoming, list):
        raise Unprocessable("stations 는 배열이어야 한다", got=type(incoming).__name__)
    ids = _unique_ids(incoming, "station_id")

    for item in incoming:
        st = item.get("station_type") or ""
        if st not in shapes.STATION_TYPES:
            raise Unprocessable(f"모르는 스테이션 유형: {st}", allowed=list(shapes.STATION_TYPES))
        cs = item.get("completion_signal") or ""
        if cs not in shapes.COMPLETION_SIGNALS:
            raise Unprocessable(f"모르는 완료 신호: {cs}", allowed=list(shapes.COMPLETION_SIGNALS))

    path = settings().stations_yaml
    doc, _ = yamlstore.load(path)
    kept = {str(s.get("station_id")): s for s in (doc.get("stations") or [])}

    merged = []
    for station_id, item in zip(ids, incoming):
        target = kept.get(station_id)
        converted = {"station_id": station_id, **shapes.station_in(item)}
        if target is not None:
            # ★ output_shelf 는 화면에 입력 칸이 없다 — 화면이 안 보내면
            #   파일의 기존 값을 지킨다(shelves.yaml assigned_robot 과 같은
            #   이유, put_shelves 주석 참고). 안 지키면 스테이션을 한 번
            #   저장할 때마다 RECOVER 매핑이 사라진다.
            if "output_shelf" not in converted and "output_shelf" in target:
                converted["output_shelf"] = target["output_shelf"]
            merged.append(yamlstore.merge_into(target, converted))
        else:
            merged.append(converted)
    doc["stations"] = merged

    rev = await _save(path, doc, if_match, "stations")
    return {"revision": rev, "stations": [shapes.station_out(s["station_id"], s) for s in merged]}


# ── 공정 흐름 ────────────────────────────────────────────────────────
@router.get("/routing")
async def get_routing(response: Response):
    doc, rev = configstore.doc(settings().routing_yaml)
    response.headers["ETag"] = rev
    return {"revision": rev, **shapes.routing_out(doc)}


@router.put("/routing")
async def put_routing(
    body: dict = Body(...), if_match: str | None = Header(None, alias="If-Match")
):
    parsed = shapes.routing_in(body)
    _check_chain(parsed)

    path = settings().routing_yaml
    doc, _ = yamlstore.load(path)
    yamlstore.merge_into(doc, parsed)

    rev = await _save(path, doc, if_match, "routing")
    return {"revision": rev, **shapes.routing_out(doc)}


def _check_chain(parsed: dict) -> None:
    """화면의 routingComplete 와 **같은 규칙**으로 검사한다.

    규칙이 어긋나면 화면은 "저장 가능" 이라 하고 서버는 422 를 낸다 —
    그런 불일치가 가장 고치기 어려운 종류의 버그다. 그래서 조건을 그대로 옮겼다:

        validStart        rules[0].from === 'SHELF'
        continuityValid   rules[i].from === rules[i-1].to
        stationLinksValid from/to 가 비어 있지 않고 등록된 스테이션일 것
        descriptions      description.trim() !== ''
    """
    rules = parsed["rules"]
    if not rules:
        return   # 빈 라우팅은 '아직 안 만든 것' 이지 잘못된 것이 아니다

    known = {str(s.get("station_id")) for s in (configstore.stations_list())}

    if rules[0]["from"] != shapes.ROUTE_START:
        raise Unprocessable(
            f"첫 규칙의 출발지는 '{shapes.ROUTE_START}' 여야 한다 (지금 '{rules[0]['from']}')."
        )
    for i, rule in enumerate(rules):
        if not rule["to"].strip():
            raise Unprocessable(f"{i + 1}번째 규칙에 도착지가 없다", index=i)
        if rule["to"] not in known:
            raise Unprocessable(
                f"등록되지 않은 스테이션이다: {rule['to']}", index=i, known=sorted(known)
            )
        if not rule["description"].strip():
            raise Unprocessable(f"{i + 1}번째 규칙에 설명이 없다", index=i)
        if i > 0 and rule["from"] != rules[i - 1]["to"]:
            raise Unprocessable(
                f"체인이 끊겼다: {i}번째가 '{rules[i - 1]['to']}' 에서 끝났는데 "
                f"{i + 1}번째는 '{rule['from']}' 에서 시작한다.",
                index=i,
            )

    fd = parsed["final_destination"]
    if fd not in shapes.FINAL_DESTINATIONS:
        raise Unprocessable(
            f"모르는 최종 목적지: {fd}", allowed=list(shapes.FINAL_DESTINATIONS)
        )

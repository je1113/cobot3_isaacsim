"""설정 yaml 캐시 — 배정 검사·스케줄러가 매 요청마다 파일을 열지 않게.

mtime 이 바뀌면 다시 읽는다. 백엔드가 직접 쓴 것뿐 아니라 **사람이 에디터로
고친 것도** 잡힌다 — 파일이 주인이라는 전제(§8-5)를 캐시가 깨면 안 되므로.

★ 이 캐시는 읽기 전용이다. 쓰기는 yamlstore.save() 한 곳뿐이고,
  저장 직후 invalidate() 를 부른다.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from .. import yamlstore
from ..config import settings

log = logging.getLogger(__name__)

_cache: dict[Path, tuple[float, Any, str]] = {}


def _read(path: Path) -> tuple[Any, str]:
    try:
        mtime = path.stat().st_mtime
    except OSError:
        _cache.pop(path, None)
        raise
    hit = _cache.get(path)
    if hit is not None and hit[0] == mtime:
        return hit[1], hit[2]
    data, rev = yamlstore.load(path)
    plain = yamlstore.plain(data)
    _cache[path] = (mtime, plain, rev)
    return plain, rev


def invalidate(path: Path | None = None) -> None:
    if path is None:
        _cache.clear()
    else:
        _cache.pop(path, None)


def doc(path: Path) -> tuple[dict, str]:
    data, rev = _read(path)
    return (data if isinstance(data, dict) else {}), rev


# ★ 화면이 배열을 쥐고 있어 yaml 도 **리스트**로 저장한다. 순서가 곧 화면 순서다.
#   그래서 조회 함수도 리스트를 돌려주고, id 로 찾는 것은 별도 함수로 둔다.


def shelves_list() -> list[dict]:
    data, _ = doc(settings().shelves_yaml)
    return data.get("shelves") or []


def stations_list() -> list[dict]:
    data, _ = doc(settings().stations_yaml)
    return data.get("stations") or []


def routing() -> dict:
    data, _ = doc(settings().routing_yaml)
    return {"rules": data.get("rules") or [], "final_destination": data.get("final_destination") or ""}


def shelf(shelf_id: str) -> dict | None:
    return next((s for s in shelves_list() if str(s.get("shelf_id")) == shelf_id), None)


def station(station_id: str) -> dict | None:
    return next((s for s in stations_list() if str(s.get("station_id")) == station_id), None)


def shelf_ids() -> list[str]:
    return [str(s.get("shelf_id")) for s in shelves_list()]


def station_ids() -> list[str]:
    return [str(s.get("station_id")) for s in stations_list()]


def process_sec(station_id: str, default: int = 60) -> int:
    """회수 대기 시각을 정하는 값 (5.1).

    화면의 `process_time`(초) 이 그대로 쓰인다. 너무 짧으면 회수하러 가서
    헛탕 치고 retry_count 만 오른다 — 그 재시도가 5.3 이다.
    """
    from .. import shapes

    st = station(station_id)
    if st is None:
        log.warning("stations.yaml 에 없는 스테이션: %s — %ss 로 대체", station_id, default)
        return default
    return shapes.process_seconds(st, default)


def taught_pose_names() -> set[str]:
    data, _ = doc(settings().taught_poses)
    return set(data)

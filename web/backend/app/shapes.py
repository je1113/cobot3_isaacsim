"""프론트엔드 계약 — 화면이 기대하는 모양과 파일/DB 의 모양을 잇는 곳.

★ 이 파일이 **유일한 번역 지점**이다. 라우터는 여기 함수만 부르고, 필드 이름을
  직접 만들지 않는다. 같은 변환이 두 곳에 생기면 §5-2 대로 갈라진다.

번역해야 하는 것이 셋이다:

1. **숫자 ↔ 문자열**
   화면의 좌표·관절값은 전부 `<input type="number">` 이고, React 는 그 값을
   **문자열**로 들고 있다(`event.target.value`). 미입력은 `''` 다.
   반면 yaml 은 로봇도 읽는 파일이라 숫자여야 한다 — `task_manager` 가
   `"−4.85"` 를 좌표로 쓸 수는 없다.
   그래서 **파일에는 숫자(또는 null), 화면에는 문자열**로 낸다.
   ★ null 을 그대로 내보내면 안 된다. React 의 controlled input 이
     uncontrolled 로 바뀌고, `arm_teach_pose` 는 `.trim()` 에서 터진다.

2. **로봇 이름**  'AMR-01'(화면) ↔ 'robot1'(ROS 네임스페이스·DB)

3. **로그 행**  carrier_log 뷰의 15칸 → 화면이 읽는 7칸
"""

from __future__ import annotations

from datetime import datetime
from typing import Any

from .config import settings

JOINTS = 6  # arm_teach_pose 는 항상 정확히 6칸이다

DIRECTIONS = ("FORWARD", "BACKWARD")
STATION_TYPES = ("", "PACKAGING", "TEST", "STORAGE")
# 'TIMER + VISION' 의 공백까지 화면 코드와 정확히 같아야 한다.
COMPLETION_SIGNALS = ("", "TIMER", "VISION", "TIMER + VISION", "EXTERNAL")
FINAL_DESTINATIONS = ("", "SHELF", "출하")
ROUTE_START = "SHELF"   # 라우팅 체인의 시작 센티널


# ── 숫자 ↔ 문자열 ────────────────────────────────────────────────────
def to_text(value: Any) -> str:
    """파일의 숫자 → 화면의 문자열. None 은 '' 로."""
    if value is None or value == "":
        return ""
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


def to_number(value: Any) -> float | int | None:
    """화면의 문자열 → 파일의 숫자. '' 는 None(미입력)으로.

    숫자로 못 읽는 값은 **버리지 않고 그대로 둔다.** 화면이 실수로 보낸
    문자열을 조용히 0 으로 바꾸면 로봇이 원점으로 간다.
    """
    if value is None:
        return None
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return value
    text = str(value).strip()
    if text == "":
        return None
    try:
        num = float(text)
    except ValueError:
        return text  # type: ignore[return-value]
    return int(num) if num.is_integer() and "." not in text and "e" not in text.lower() else num


def pose_out(pose: Any) -> dict[str, str]:
    p = pose if isinstance(pose, dict) else {}
    return {k: to_text(p.get(k)) for k in ("x", "y", "theta")}


def pose_in(pose: Any) -> dict[str, Any]:
    p = pose if isinstance(pose, dict) else {}
    return {k: to_number(p.get(k)) for k in ("x", "y", "theta")}


def joints_out(values: Any) -> list[str]:
    """항상 정확히 6칸. 모자라면 '' 로 채운다 — 화면이 길이 6을 가정한다."""
    src = list(values) if isinstance(values, (list, tuple)) else []
    return [to_text(src[i]) if i < len(src) else "" for i in range(JOINTS)]


def joints_in(values: Any) -> list[Any]:
    src = list(values) if isinstance(values, (list, tuple)) else []
    return [to_number(src[i]) if i < len(src) else None for i in range(JOINTS)]


# ── 선반 ─────────────────────────────────────────────────────────────
def shelf_out(shelf_id: str, data: dict) -> dict:
    return {
        "shelf_id": shelf_id,
        "waypoint_start": pose_out(data.get("waypoint_start")),
        "waypoint_end": pose_out(data.get("waypoint_end")),
        "standoff_distance": to_text(data.get("standoff_distance")),
        "scan_passes": [pass_out(p) for p in (data.get("scan_passes") or [])],
        # 화면은 없으면 '-' 를 그린다. 키 자체를 빼지 말고 null 로 낸다.
        "first_taught_at": data.get("first_taught_at"),
        # ★ 로봇 쪽 필드다 — task_manager 가 이 값으로 자기 선반을 고른다
        #   (그 노드의 _resolve_patrol_route). 화면은 아직 편집하지 않지만 그대로
        #   들고 있다가 저장 때 되돌려줘야 한다. 빠지면 저장 한 번에 배정이 사라진다.
        "assigned_robot": data.get("assigned_robot"),
    }


def pass_out(p: Any) -> dict:
    p = p if isinstance(p, dict) else {}
    level = p.get("level", p.get("pass_id", 1))
    return {
        "pass_id": int(p.get("pass_id", level) or 1),
        "level": int(level or 1),
        # 화면은 방향을 레벨 홀짝으로 만들지만, 삭제 뒤에는 다시 계산하지
        # 않는다. 그래서 저장된 값이 있으면 그대로 믿는다.
        "direction": p.get("direction") if p.get("direction") in DIRECTIONS
        else ("FORWARD" if int(level or 1) % 2 == 1 else "BACKWARD"),
        "arm_teach_pose": joints_out(p.get("arm_teach_pose")),
    }


def shelf_in(body: dict) -> dict:
    out = {
        "waypoint_start": pose_in(body.get("waypoint_start")),
        "waypoint_end": pose_in(body.get("waypoint_end")),
        "standoff_distance": to_number(body.get("standoff_distance")),
        "scan_passes": [
            {
                "pass_id": int(p.get("pass_id", p.get("level", 1)) or 1),
                "level": int(p.get("level", 1) or 1),
                "direction": p.get("direction", "FORWARD"),
                "arm_teach_pose": joints_in(p.get("arm_teach_pose")),
            }
            for p in (body.get("scan_passes") or [])
            if isinstance(p, dict)
        ],
        "first_taught_at": body.get("first_taught_at"),
    }
    # ★ 화면이 보낸 것만 쓴다. 키가 없으면(옛 화면 · 새로 만든 선반) 아예 넣지
    #   않는다 — routers/settings.py 가 파일의 기존 값을 그대로 둔다. 빈 문자열은
    #   "배정 해제" 라 null 로 저장한다.
    if "assigned_robot" in body:
        out["assigned_robot"] = to_text(body.get("assigned_robot")) or None
    return out


def shelf_is_taught(data: dict) -> bool:
    """화면의 '티칭 완료' 판정과 **같은 규칙**이어야 한다.

        scan_passes.length > 0 && every(pass => every(joint => joint.trim() !== ''))

    이게 어긋나면 화면은 "작업 할당 가능" 이라 하고 서버는 422 를 낸다.
    """
    passes = data.get("scan_passes") or []
    if not passes:
        return False
    return all(
        all(to_text(j) != "" for j in joints_out((p or {}).get("arm_teach_pose")))
        for p in passes
    )


# ── 스테이션 ─────────────────────────────────────────────────────────
def station_out(station_id: str, data: dict) -> dict:
    return {
        "station_id": station_id,
        "station_type": data.get("station_type") or "",
        "place_pose": pose_out(data.get("place_pose")),
        "process_time": to_text(data.get("process_time")),
        "output_type": data.get("output_type") or "",
        # ★ 2026-09-25: RECOVER 작업이 쓰는 station→shelf 매핑(stations.yaml
        #   머리주석 참고). 화면 입력 칸은 아직 없지만, 여기서 안 돌려주면
        #   routers/settings.py 의 merge_into 가 다음 저장 때 지운다
        #   (shelves.yaml assigned_robot 때 겪은 것과 같은 함정) — 그래서
        #   화면이 안 보내도 station_in 에서 원본 값을 그대로 지킨다.
        "output_shelf": data.get("output_shelf") or "",
        "completion_signal": data.get("completion_signal") or "",
        "capacity": to_text(data.get("capacity")),
        "updated_at": data.get("updated_at"),
    }


def station_in(body: dict) -> dict:
    out = {
        "station_type": body.get("station_type") or "",
        "place_pose": pose_in(body.get("place_pose")),
        "process_time": to_number(body.get("process_time")),
        "output_type": body.get("output_type") or "",
        "completion_signal": body.get("completion_signal") or "",
        "capacity": to_number(body.get("capacity")),
        "updated_at": body.get("updated_at") or datetime.now().astimezone().isoformat(),
    }
    # ★ 화면이 보낸 것만 쓴다(shelf_in 의 assigned_robot 과 같은 규칙 —
    #   그 함수 주석 참고). 키가 없으면 여기 안 넣는다 — 호출부
    #   (routers/settings.py put_stations) 가 파일의 기존 output_shelf 를
    #   지켜서 넣어준다. 여기서 ""로 채우면 화면에 입력 칸이 없는 지금은
    #   저장 한 번마다 RECOVER 매핑이 사라진다.
    if "output_shelf" in body:
        out["output_shelf"] = body.get("output_shelf") or ""
    return out


def process_seconds(station: dict, default: int = 60) -> int:
    """회수 타이머가 쓰는 값. 화면의 `process_time`(초)이 그대로 들어온다."""
    raw = to_number(station.get("process_time"))
    try:
        return int(raw) if raw is not None else default
    except (TypeError, ValueError):
        return default


# ── 라우팅 ───────────────────────────────────────────────────────────
def rule_out(r: Any) -> dict:
    r = r if isinstance(r, dict) else {}
    return {
        "rule_id": str(r.get("rule_id") or ""),
        # ★ 'from' 은 파이썬 예약어지만 화면이 그 이름을 쓴다. dict 로만 다룬다.
        "from": str(r.get("from") or ""),
        "to": str(r.get("to") or ""),
        "description": str(r.get("description") or ""),
    }


def routing_out(doc: dict) -> dict:
    return {
        "rules": [rule_out(r) for r in (doc.get("rules") or [])],
        "final_destination": doc.get("final_destination") or "",
    }


def routing_in(body: dict) -> dict:
    return {
        "rules": [rule_out(r) for r in (body.get("rules") or [])],
        "final_destination": body.get("final_destination") or "",
    }


# ── 로봇 이름 ────────────────────────────────────────────────────────
def to_ns(display: str) -> str:
    return settings().to_ns(display)


def to_display(ns: str) -> str:
    return settings().to_display(ns)


# ── 로그 행 ──────────────────────────────────────────────────────────
def log_row_out(r: dict) -> dict:
    """carrier_log 15칸 → 화면이 읽는 7칸.

    ★ 화면은 timestamp 를 **그대로 출력**한다(포맷 안 한다). 그래서 서버가
      사람이 읽는 문자열로 만들어 보낸다.
    ★ duration 은 **초**여야 한다 — 화면이 평균을 `${avg.toFixed(1)}초` 로
      라벨한다. 그리고 그 값은 시뮬 시각 기준이다(§4-2): 벽시계와 2배 가까이
      차이 나고, 사이클 타임으로 옳은 쪽은 시뮬이다.
    """
    ended = r.get("ended_at")
    return {
        # role 이 다르면 log_id 가 겹칠 수 있다(표가 둘이라). React key 라 유일해야 한다.
        "id": f"{r.get('role', '?')}-{r.get('log_id')}",
        "timestamp": ended.strftime("%Y-%m-%d %H:%M:%S") if isinstance(ended, datetime) else to_text(ended),
        "robot_id": to_display(r.get("robot_id") or ""),
        "task_type": str(r.get("stage") or "").upper(),
        "target_id": r.get("qr_payload") or r.get("port") or "",
        "result": "SUCCESS" if r.get("succeeded") else "FAILED",
        "duration": round(float(r["duration_sec"]), 1) if r.get("duration_sec") is not None else "",
    }


# ── 로봇 실시간 상태 (WebSocket) ─────────────────────────────────────
def robot_state_out(ns_or_display: str, *, state=None, task=None, pose=None) -> dict:
    """화면의 robot_state 항목 하나.

    ★ 화면은 없는 값을 `??` 로 이전 값과 병합한다 — 즉 **null 로는 못 지운다.**
      그러니 "모른다" 를 null 로 보내지 말고 아예 키를 빼는 편이 낫다.
    ★ position 은 반올림해서 보낸다. 화면이 `x: ${x}` 로 **가공 없이** 찍어서,
      안 하면 소수점 17자리가 그대로 나온다.
    """
    out: dict[str, Any] = {"robot_id": to_display(ns_or_display)}
    if state is not None:
        out["current_state"] = state
    if task is not None:
        out["current_task"] = task
    if pose is not None:
        out["position"] = {
            "x": round(float(pose.get("x", 0.0)), 2),
            "y": round(float(pose.get("y", 0.0)), 2),
            "theta": round(float(pose.get("theta", 0.0)), 3),
        }
    return out

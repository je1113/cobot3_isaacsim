"""요청/응답 스키마.

응답 모델은 일부러 느슨하게 뒀다 — 뷰 4개의 컬럼이 그대로 나가는 경로가
많아서, 모델을 좁게 박으면 DB구성.md 의 스키마가 바뀔 때 두 곳을 고쳐야 한다.
**입력** 은 반대로 좁게 박는다. 잘못된 쓰기가 표 5·6 에 들어가면 로봇이 움직인다.
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Literal

from pydantic import BaseModel, Field, field_validator

TaskKind = Literal["SCAN", "RECOVER"]
TaskStatus = Literal["QUEUED", "RUNNING", "DONE", "FAILED"]
PickupStatus = Literal["WAITING", "QUEUED", "DONE", "GAVE_UP"]
Command = Literal["PAUSE", "RESUME", "SKIP", "STOP"]


# ── 설정 (2.1 · 3.x) ─────────────────────────────────────────────────
class ConfigDoc(BaseModel):
    """yaml 한 덩어리 + revision. 화면은 revision 을 들고 있다가 PUT 에 싣는다."""

    revision: str
    path: str
    data: dict[str, Any]


class ConfigWrite(BaseModel):
    data: dict[str, Any]


class CapturePose(BaseModel):
    """현재 관절값을 taught_poses.yaml 에 append 하고 그 키를 설정에 단다 (3.3)."""

    pose_name: str = Field(min_length=1, max_length=64)
    robot_id: str


# ── 작업 큐 (4.x) ────────────────────────────────────────────────────
class TaskCreate(BaseModel):
    robot_id: str
    target_ref: str
    kind: TaskKind = "SCAN"
    # 생략하면 큐 맨 뒤. RECOVER 는 스케줄러가 음수로 넣어 앞을 선점한다.
    queue_order: int | None = None


class TaskPatch(BaseModel):
    """로봇 간 이관 (4.4). RUNNING 이관은 서버가 409 로 막는다."""

    robot_id: str | None = None
    queue_order: int | None = None


class QueueReplace(BaseModel):
    """순서 전체 교체 (4.3). 그 로봇의 QUEUED 집합과 정확히 일치해야 한다."""

    task_ids: list[int]

    @field_validator("task_ids")
    @classmethod
    def _no_dupes(cls, v: list[int]) -> list[int]:
        if len(set(v)) != len(v):
            raise ValueError("task_ids 에 중복이 있다")
        return v


class AutoDistribute(BaseModel):
    """분배안 계산만 한다 (4.6). 적용은 PUT /robots/{id}/queue 재사용."""

    robots: list[str] | None = None


class TaskRow(BaseModel):
    task_id: int
    robot_id: str
    kind: TaskKind
    target_ref: str
    queue_order: int
    status: TaskStatus
    run_id: str | None = None
    resume_progress: float | None = None
    created_at: datetime
    started_at: datetime | None = None
    ended_at: datetime | None = None


# ── 회수 (5.x) ───────────────────────────────────────────────────────
class PickupRow(BaseModel):
    pending_id: int
    station_ref: str
    source_run_id: str
    ready_at: datetime
    retry_count: int
    status: PickupStatus
    task_id: int | None = None
    # §10-3 조인으로 얻는 값 — 표에 저장하지 않는다
    expected_payload: str | None = None
    seconds_remaining: float | None = None


class PickupRetry(BaseModel):
    """도착해서 보니 아직 없더라 (5.3) — 재예약."""

    delay_sec: int | None = None


# ── 제어 (7.x) ───────────────────────────────────────────────────────
class CommandRequest(BaseModel):
    command: Command
    reason: str | None = None


class CommandAccepted(BaseModel):
    accepted: bool
    command: Command
    robot_id: str
    correlation_id: str

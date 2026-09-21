"""API 에러 — 코드 하나당 HTTP 상태 하나.

프론트가 분기해야 하는 실패는 전부 여기 있다. 메시지는 사람이 읽는 것이고,
분기 기준은 `code` 다. "무엇이 잘못됐고 어떻게 고치는지" 를 메시지에 적는다.
"""

from __future__ import annotations

from fastapi import Request
from fastapi.responses import JSONResponse


class ApiError(Exception):
    status = 500
    code = "internal"

    def __init__(self, message: str, **extra):
        super().__init__(message)
        self.message = message
        self.extra = extra

    def payload(self) -> dict:
        return {"code": self.code, "message": self.message, **self.extra}


class NotFound(ApiError):
    status, code = 404, "not_found"


class Conflict(ApiError):
    """이미 있는 것과 부딪힌다 — 중복 배정, RUNNING 이관 시도 등."""

    status, code = 409, "conflict"


class PreconditionFailed(ApiError):
    """If-Match 불일치. 다른 사람이 먼저 저장했다 (WBS 3.2)."""

    status, code = 412, "revision_mismatch"


class Unprocessable(ApiError):
    """형식은 맞지만 받아줄 수 없다 — 티칭 미완료 선반 배정 등 (WBS 4.1)."""

    status, code = 422, "unprocessable"


class BridgeUnavailable(ApiError):
    """ROS 쪽으로 못 넘긴다 — 브리지가 꺼져 있거나 인터페이스가 아직 없다."""

    status, code = 503, "bridge_unavailable"


class DatabaseUnavailable(ApiError):
    """DB 가 아직 안 붙었다. 백엔드는 이것 없이도 뜬다(db.py 참고) —
    설정 화면은 yaml 만 쓰므로 이 상태에서도 평소대로 동작한다."""

    status, code = 503, "database_unavailable"


async def api_error_handler(_: Request, exc: ApiError) -> JSONResponse:
    return JSONResponse(status_code=exc.status, content=exc.payload())

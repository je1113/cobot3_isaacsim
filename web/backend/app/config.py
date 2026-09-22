"""설정 — 환경변수 하나로 덮어쓸 수 있는 값만 모은다.

경로 기본값은 전부 리포지토리 기준 상대경로다. 다른 곳에 배포하면
COBOT3_* 환경변수로 덮는다(.env.example 참고).
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path


def _repo_root() -> Path:
    # web/backend/app/config.py → parents: app, backend, web, <repo>
    return Path(__file__).resolve().parents[3]


def _env_path(key: str, default: Path) -> Path:
    raw = os.environ.get(key)
    return Path(raw).expanduser().resolve() if raw else default


def _env_int(key: str, default: int) -> int:
    try:
        return int(os.environ[key])
    except (KeyError, ValueError):
        return default


def _env_bool(key: str, default: bool) -> bool:
    raw = os.environ.get(key)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class Settings:
    root: Path = field(default_factory=_repo_root)

    # ── DB ───────────────────────────────────────────────────────────
    dsn: str = os.environ.get(
        "COBOT3_DSN", "postgresql://cobot3@localhost:5432/cobot3"
    )
    pool_min: int = _env_int("COBOT3_POOL_MIN", 1)
    pool_max: int = _env_int("COBOT3_POOL_MAX", 8)

    # ── yaml (설정의 주인) ───────────────────────────────────────────
    config_dir: Path = field(
        default_factory=lambda: _env_path(
            "COBOT3_CONFIG_DIR", _repo_root() / "src/cobot3_bringup/config"
        )
    )
    taught_poses: Path = field(
        default_factory=lambda: _env_path(
            "COBOT3_TAUGHT_POSES", _repo_root() / "isaacpjt/tools/out/taught_poses.yaml"
        )
    )
    # MonitoringPage 의 Top View 좌표계 — Nav2 map_server 가 쓰는 그 정적 맵
    # 파일에서 origin/resolution/크기만 읽는다(multi_navigation.launch.py 의
    # map_file 과 같은 파일). 이미지 자체는 화면에 안 보여준다 — 점유격자
    # PNG 가 단순 도형이라 그대로 보여줘도 못 알아봤다. 대신 이 범위 안에
    # 선반·스테이션·로봇 위치를 직접 그린다(compat.py GET /map 참고). 새
    # 범위를 만들지 않는 이유: 로봇이 실제로 이 지도로 로컬라이즈하니,
    # 화면이 다른 범위를 쓰면 그린 위치가 실제 배치와 어긋난다.
    nav_map_yaml: Path = field(
        default_factory=lambda: _env_path(
            "COBOT3_NAV_MAP_YAML",
            _repo_root() / "src/cobot3_navigation/maps/simple_factory_layout.yaml",
        )
    )

    # ── 회수 스케줄러 (WBS 5.2) ──────────────────────────────────────
    pickup_tick_sec: float = float(os.environ.get("COBOT3_PICKUP_TICK_SEC", "5"))
    pickup_max_retry: int = _env_int("COBOT3_PICKUP_MAX_RETRY", 3)
    pickup_retry_backoff_sec: int = _env_int("COBOT3_PICKUP_RETRY_BACKOFF_SEC", 30)

    # ── ROS 경계 ─────────────────────────────────────────────────────
    # 0 이면 NullBridge — 백엔드는 ROS 없이 완전히 뜬다. 실시간 채널만 조용해진다.
    ros_enabled: bool = _env_bool("COBOT3_ROS", False)

    # ★ 로봇 이름이 두 벌이다.
    #     화면/API  : 'AMR-01'  — 프론트가 곳곳에 하드코딩하고 있다
    #                 (MonitoringPage.ROBOT_IDS, 작업 시작 payload, 로그 필터 버튼)
    #     ROS/DB    : 'robot1'  — 네임스페이스이고, magazine_log.robot_id 에
    #                 그대로 들어간다(task_manager 가 get_namespace() 로 찍는다)
    #
    #   둘 중 하나로 통일하지 않은 이유: 화면 쪽을 바꾸면 이미 만들어진 6개
    #   페이지를 전부 고쳐야 하고, ROS 쪽을 바꾸면 이미 쌓인 로그 행과 어긋난다.
    #   그래서 **경계에서만 번역**한다. 번역표는 여기 한 곳뿐이다.
    #   "AMR-01=robot1,AMR-02=robot2" 형식으로 덮어쓸 수 있다.
    robot_map: tuple[tuple[str, str], ...] = tuple(
        tuple(pair.split("=", 1)) if "=" in pair else (pair, pair)
        for pair in os.environ.get("COBOT3_ROBOTS", "AMR-01=robot1,AMR-02=robot2").split(",")
        if pair.strip()
    )
    pose_topic_tmpl: str = os.environ.get("COBOT3_POSE_TOPIC", "/{robot}/amcl_pose")
    state_topic_tmpl: str = os.environ.get("COBOT3_STATE_TOPIC", "/{robot}/orchestrator/state")
    pose_hz: float = float(os.environ.get("COBOT3_POSE_HZ", "8"))  # 6.1 솎아내기

    # ── HTTP ─────────────────────────────────────────────────────────
    cors_origins: tuple[str, ...] = tuple(
        o.strip()
        for o in os.environ.get("COBOT3_CORS", "http://localhost:5173").split(",")
        if o.strip()
    )
    page_limit_default: int = _env_int("COBOT3_PAGE_LIMIT", 100)
    page_limit_max: int = _env_int("COBOT3_PAGE_LIMIT_MAX", 1000)

    # ── 로봇 이름 번역 ───────────────────────────────────────────────
    @property
    def robots(self) -> tuple[str, ...]:
        """화면/API 쪽 이름. 'AMR-01' …"""
        return tuple(display for display, _ in self.robot_map)

    @property
    def namespaces(self) -> tuple[str, ...]:
        """ROS/DB 쪽 이름. 'robot1' …"""
        return tuple(ns for _, ns in self.robot_map)

    def to_ns(self, display: str) -> str:
        """'AMR-01' → 'robot1'. 모르는 이름은 그대로 통과시킨다 —
        설정에 없는 로봇을 조용히 다른 로봇으로 바꿔 보내는 것이 더 위험하다."""
        return dict(self.robot_map).get(display, display)

    def to_display(self, ns: str) -> str:
        """'robot1' → 'AMR-01'. 로그 행에 남아 있는 옛 이름도 그대로 나간다."""
        return {n: d for d, n in self.robot_map}.get(ns, ns)

    # ── 설정 파일 경로 ───────────────────────────────────────────────
    @property
    def shelves_yaml(self) -> Path:
        return self.config_dir / "shelves.yaml"

    @property
    def stations_yaml(self) -> Path:
        return self.config_dir / "stations.yaml"

    @property
    def routing_yaml(self) -> Path:
        return self.config_dir / "routing.yaml"

    @property
    def place_yaml(self) -> Path:
        return self.config_dir / "place.yaml"


@lru_cache(maxsize=1)
def settings() -> Settings:
    return Settings()

"""
event_logger — TraceEvent 를 PostgreSQL 에 append 하는 노드.

    ros2 run cobot3_orchestrator event_logger

스키마와 그 근거는 docs/DB구성.md 에 있다. 이 노드는 그 문서의 §9 배선이고,
표 2·3(magazine_log / stack_log)에 행을 넣는 유일한 쓰는 쪽이다.

    task_manager (robot1) ─┐
                           ├─ /trace/event ─→ event_logger ─→ PostgreSQL
    task_manager (robot2) ─┘                       └─ 막히면 runs/trace_spool.jsonl

★ 전역 1개다
  로봇마다 하나씩 뜨는 다른 노드들과 다르다. /trace/event 가 절대이름인 이유가
  이것이고, 그래서 robot_id 가 메시지에 실려 온다. launch 에서도 네임스페이스
  없이 띄운다(mission_nodes.launch.py).

★ DB 커넥션을 가진 노드가 이것 하나뿐인 것이 설계의 핵심이다
  task_manager 는 토픽에 던지고 끝이라, 이 노드가 안 떠 있어도 DB 가 죽어도
  로봇은 평소대로 돈다. docs/05 §2-9 의 "기록 실패가 로봇 동작을 막지 않는다.
  그렇다고 조용히 버리지도 않는다" 를 배선으로 보장한 것이다.
  그래서 여기서 지키는 두 가지가

      1. 콜백은 절대 블로킹하지 않는다 — 큐에 넣고 바로 돌아온다
      2. 큐가 넘치거나 DB 가 죽으면 파일로 흘린다. 버리지 않는다

★ 판단하지 않는다
  status(물체 상태)는 task_manager 가 정해서 실어 보낸다. 로거가 "이게 마지막
  단계인가" 를 추론하면 return 행에서 틀린다 — return 은 place 뒤라 실패해도
  물체는 이미 배달됐기 때문이다(TraceEvent.msg 의 status 주석).

★ 어느 표에 넣을지는 DB 에 물어본다
  매거진 로그와 스택 로그가 갈려 있으므로 원문에서 품목 코드를 뽑아야 하는데,
  파서를 여기 또 쓰면 carrier_code.py 와 두 벌이 된다. 대신 시작할 때
  carrier_kind 4행을 읽어 두고 "원문에 그 코드가 들어 있는가" 로 찾는다.
  자리(인덱스)를 안 보므로 나중에 로트 날짜가 끼어도(F1-260921-MGZO-1)
  그대로 동작한다 — carrier_code.kind_of() 와 같은 규칙이고,
  코드 목록은 DB 한 곳에만 있다.

파라미터
  db_dsn      (string)  기본값 = 환경변수 COBOT3_DB_DSN.
                        비밀번호는 넣지 않는다 — libpq 가 ~/.pgpass 에서 꺼낸다.
                        예: postgresql://cobot3@localhost:5432/cobot3
  spool_path  (string)  기본값 runs/trace_spool.jsonl
"""

import json
import os
import pathlib
import queue
import threading
import time
from datetime import datetime, timedelta, timezone

import psycopg2
import rclpy
from rclpy.node import Node

from cobot3_interfaces.msg import TraceEvent

# 표 2·3 은 컬럼이 완전히 같다. 이름만 갈아끼운다.
TABLE_BY_FAMILY = {"magazine": "magazine_log", "stack": "stack_log"}

# UNIQUE (run_id, stage, attempt) 가 있어서 재전송·중복이 무해하다.
# 순찰 가지를 되살리면 선점으로 같은 단계가 두 번 발행될 수 있는데, 그것도 여기서 흡수된다.
INSERT_SQL = """
INSERT INTO {table} (run_id, qr_payload, kind_code, robot_id, stage, attempt,
                     started_at, ended_at, started_sim, ended_sim,
                     succeeded, status, fail_reason, fail_detail, port)
VALUES (%(run_id)s, %(qr_payload)s, %(kind_code)s, %(robot_id)s, %(stage)s, %(attempt)s,
        %(started_at)s, %(ended_at)s, %(started_sim)s, %(ended_sim)s,
        %(succeeded)s, %(status)s::run_status, %(fail_reason)s::fail_reason,
        %(fail_detail)s, %(port)s)
ON CONFLICT (run_id, stage, attempt) DO NOTHING
"""
# plant_code 와 duration_sec 은 생성열이라 넣지 않는다 (넣으면 에러난다).

# ★ 옛 씬의 숫자 QR 폴백. 씬(과 스포너 에셋 magazine_*_qr.usda)이 아직 QR 에
#   숫자 하나("1"/"2")만 담는다. 그 원문에는 품목 코드가 없어서 _resolve 가
#   못 풀고, 모든 기록이 "미등록 페이로드" 로 스풀에만 쌓였다(DB 는 빈 채로).
#   task_manager.NUMERIC_TO_VARIANT 와 같은 대응이다 — "1" 주황, "2" 파랑 매거진.
#   원문(qr_payload)은 그대로 저장하고 품목 코드만 채운다.
#   씬을 새 페이로드(F1-MGZO-1 …)로 바꾸면 이 표와 _resolve 의 폴백을 지운다.
LEGACY_NUMERIC_KIND = {"1": "MGZO", "2": "MGZB"}

QUEUE_MAX = 10000
RECONNECT_MIN_S = 1.0
RECONNECT_MAX_S = 30.0


def _sim_sec(t):
    """builtin_interfaces/Time -> 초 단위 실수. 시뮬 시각이라 0 일 수 있다."""
    return float(t.sec) + float(t.nanosec) / 1e9


def _wall_iso(t):
    """builtin_interfaces/Time -> ISO8601 문자열.

    psycopg2 가 문자열을 그대로 보내고 PostgreSQL 이 timestamptz 로 캐스팅한다.
    JSON 으로도 그대로 떨어져서 스풀 파일이 단순해진다.
    sec 가 0 이면 시계를 못 읽은 것이라 None 을 돌려 DB 가 거부하게 둔다
    (started_at·ended_at 은 NOT NULL 이다 — 조용히 틀린 값이 들어가는 것보다 낫다).
    """
    if t.sec == 0 and t.nanosec == 0:
        return None
    return (datetime(1970, 1, 1, tzinfo=timezone.utc)
            + timedelta(seconds=t.sec, microseconds=t.nanosec // 1000)).isoformat()


class EventLogger(Node):

    def __init__(self):
        super().__init__("event_logger")
        self.declare_parameter("db_dsn", os.environ.get("COBOT3_DB_DSN", ""))
        self.declare_parameter("spool_path", "runs/trace_spool.jsonl")

        self.dsn = self.get_parameter("db_dsn").value
        if not self.dsn:
            self.get_logger().error(
                "db_dsn 이 비었다. 환경변수 COBOT3_DB_DSN 을 설정하거나 "
                "-p db_dsn:=postgresql://cobot3@localhost:5432/cobot3 로 넘겨라. "
                "노드는 계속 뜨지만 전부 스풀 파일로 흘러간다.")

        self.spool = pathlib.Path(self.get_parameter("spool_path").value)
        self.spool.parent.mkdir(parents=True, exist_ok=True)

        self.q = queue.Queue(maxsize=QUEUE_MAX)
        self._spool_lock = threading.Lock()
        self._kinds = []          # [(kind_code, family)] — 시작할 때 DB 에서 읽는다
        self._dropped = 0

        # ★ 절대이름. 로봇이 몇 대든 전부 여기로 모인다.
        self.create_subscription(TraceEvent, "/trace/event", self.on_event, 50)

        threading.Thread(target=self._writer_loop, daemon=True).start()
        self.get_logger().info(
            f"event_logger 시작 — /trace/event 구독, 스풀 {self.spool}")

    # ── 구독 콜백 — 절대 블로킹하지 않는다 ────────────────────────────
    def on_event(self, msg):
        row = {
            "run_id":      msg.run_id or None,
            "qr_payload":  msg.carrier_id,
            "robot_id":    msg.robot_id,
            "stage":       msg.stage,
            "attempt":     int(msg.attempt) or 1,
            "started_at":  _wall_iso(msg.started_wall),
            "ended_at":    _wall_iso(msg.wall_stamp),
            "started_sim": _sim_sec(msg.started_stamp),
            "ended_sim":   _sim_sec(msg.stamp),
            "succeeded":   bool(msg.success),
            "status":      msg.status,
            "fail_reason": msg.fail_reason or None,
            "fail_detail": msg.fail_detail or None,
            "port":        msg.port or None,
        }
        try:
            self.q.put_nowait(row)
        except queue.Full:
            # 큐가 넘쳐도 버리지 않는다 — 나중에 이 파일을 DB 로 밀어 넣는다
            self._to_spool(row, "큐 넘침")

    # ── 스풀 ──────────────────────────────────────────────────────────
    def _to_spool(self, row, why):
        try:
            with self._spool_lock, self.spool.open("a", encoding="utf-8") as f:
                f.write(json.dumps(row, ensure_ascii=False) + "\n")
        except Exception as exc:
            # 여기까지 실패하면 정말 버리는 수밖에 없다. 세어서 알리기라도 한다.
            self._dropped += 1
            self.get_logger().error(f"스풀 기록도 실패({exc}) — 누적 유실 {self._dropped}건")
        else:
            self.get_logger().warning(f"{why} — 스풀로 흘림: {row['qr_payload']} {row['stage']}")

    def _drain_spool(self, conn):
        """쌓여 있던 스풀을 DB 로 밀어 넣는다. 연결에 성공한 직후 한 번."""
        if not self.spool.exists() or self.spool.stat().st_size == 0:
            return
        with self._spool_lock:
            pending = self.spool.read_text(encoding="utf-8").splitlines()
            self.spool.write_text("", encoding="utf-8")
        ok, failed = 0, []
        for line in pending:
            if not line.strip():
                continue
            try:
                self._insert(conn, json.loads(line))
                ok += 1
            except Exception:
                failed.append(line)
        if failed:
            with self._spool_lock, self.spool.open("a", encoding="utf-8") as f:
                f.write("\n".join(failed) + "\n")
        self.get_logger().info(f"스풀 재적재 — 성공 {ok}건, 남김 {len(failed)}건")

    # ── 품목 코드 해석 ────────────────────────────────────────────────
    def _load_kinds(self, conn):
        with conn.cursor() as cur:
            cur.execute("SELECT kind_code, family FROM carrier_kind")
            self._kinds = [(k.upper(), f) for k, f in cur.fetchall()]
        self.get_logger().info(
            f"carrier_kind {len(self._kinds)}종 읽음: "
            + ", ".join(f"{k}->{f}" for k, f in self._kinds))

    def _resolve(self, payload):
        """원문 -> (kind_code, 표 이름). 못 풀면 (None, None).

        자리를 보지 않고 '아는 코드가 들어 있는가' 로 찾는다 —
        carrier_code.kind_of() 와 같은 규칙이고, 코드 목록은 DB 에만 있다.
        """
        up = (payload or "").strip().upper()
        for kind_code, family in self._kinds:
            if kind_code in up:
                return kind_code, TABLE_BY_FAMILY.get(family)
        # 옛 숫자 QR — LEGACY_NUMERIC_KIND 주석. 코드가 DB 에 있을 때만 쓴다.
        legacy = LEGACY_NUMERIC_KIND.get(up)
        if legacy is not None:
            for kind_code, family in self._kinds:
                if kind_code == legacy:
                    return kind_code, TABLE_BY_FAMILY.get(family)
        return None, None

    # ── 쓰기 ──────────────────────────────────────────────────────────
    def _insert(self, conn, row):
        kind_code, table = self._resolve(row["qr_payload"])
        if table is None:
            raise ValueError(f"미등록 페이로드 {row['qr_payload']!r} — 어느 표인지 모른다")
        with conn.cursor() as cur:
            cur.execute(INSERT_SQL.format(table=table), dict(row, kind_code=kind_code))

    def _writer_loop(self):
        conn, backoff = None, RECONNECT_MIN_S
        while rclpy.ok():
            if conn is None:
                try:
                    conn = psycopg2.connect(self.dsn)
                    conn.autocommit = True          # 행 하나가 곧 하나의 사실이다
                    self._load_kinds(conn)
                    self._drain_spool(conn)
                    backoff = RECONNECT_MIN_S
                    self.get_logger().info("DB 연결됨")
                except Exception as exc:
                    self.get_logger().warning(
                        f"DB 연결 실패({exc}) — {backoff:.0f}초 뒤 재시도. "
                        f"그 사이 들어오는 건 스풀로 간다")
                    time.sleep(backoff)
                    backoff = min(backoff * 2, RECONNECT_MAX_S)
                    self._flush_queue_to_spool("DB 연결 없음")
                    continue
            try:
                row = self.q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._insert(conn, row)
            except ValueError as exc:
                # 미등록 페이로드. 재시도해도 같으니 스풀에 남겨 사람이 보게 한다.
                self._to_spool(row, str(exc))
            except Exception as exc:
                self.get_logger().warning(f"INSERT 실패({exc}) — 재연결한다")
                self._to_spool(row, "INSERT 실패")
                try:
                    conn.close()
                except Exception:
                    pass
                conn = None

    def _flush_queue_to_spool(self, why):
        """DB 가 없는 동안 큐에 쌓인 것을 파일로 옮긴다 — 큐가 넘치기 전에."""
        moved = 0
        while True:
            try:
                self._to_spool(self.q.get_nowait(), why)
                moved += 1
            except queue.Empty:
                break
        if moved:
            self.get_logger().info(f"큐 {moved}건을 스풀로 옮김")


def main(args=None):
    rclpy.init(args=args)
    node = EventLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

-- ══════════════════════════════════════════════════════════════════════
--  001_schema.sql — 트레이스 4표 + 뷰 4개
--  원본: docs/DB구성.md §2 ~ §6.  그림: docs/ERD.md
--  ★ attempt · plant_code · run_outcome 은 a5c8661(표 1~4 미정 확정)에서 왔다.
--     event_logger 의 INSERT 가 이 셋을 전제한다 — 빼면 모든 기록이 실패한다.
--
--  ⚠️ 이 파일은 DB구성.md 의 DDL 을 그대로 옮긴 것이다. 스키마를 고칠 때는
--     문서를 먼저 고친다 — 같은 스키마가 두 곳에 있으면 §5-2 대로 갈라진다.
--
--  실행:  psql -d cobot3 -f sql/001_schema.sql
--  재실행 가능(IF NOT EXISTS / DO 블록).
-- ══════════════════════════════════════════════════════════════════════

-- ── ENUM 2개 (§2) ────────────────────────────────────────────────────
-- fail_reason 을 ENUM 으로 못 박은 이유: 다섯 번째 값을 DB 가 거부한다.
DO $$ BEGIN
    CREATE TYPE run_status AS ENUM ('IN_TRANSIT', 'COMPLETED', 'FAILED');
EXCEPTION WHEN duplicate_object THEN NULL; END $$;

DO $$ BEGIN
    CREATE TYPE fail_reason AS ENUM ('pick_error', 'nav_error', 'place_error', 'dock_error');
EXCEPTION WHEN duplicate_object THEN NULL; END $$;


-- ── 표 1 — carrier_pair (8행, 정적) (§3) ─────────────────────────────
-- 양쪽 UNIQUE 로 1:1 이 DB 레벨에서 보장된다.
CREATE TABLE IF NOT EXISTS carrier_pair (
    magazine_payload TEXT PRIMARY KEY,        -- 'F1-MGZB-1'  이 매거진이
    stack_payload    TEXT NOT NULL UNIQUE     -- 'F3-STKB-1'  이 스택이 된다
);


-- ── 표 4 — carrier_kind (4행, 정적) (§5) ─────────────────────────────
-- carrier_type 이 생성열이라 grasp/place.yaml 키와 family/color 가 어긋날 수 없다.
CREATE TABLE IF NOT EXISTS carrier_kind (
    kind_code    TEXT PRIMARY KEY,            -- 'MGZB'  QR 원문의 품목 코드
    family       TEXT NOT NULL,               -- 'magazine' | 'stack'
    color        TEXT NOT NULL,               -- 'orange'   | 'blue'
    carrier_type TEXT GENERATED ALWAYS AS (family || '_' || color) STORED UNIQUE,

    length_mm    NUMERIC(6,1) NOT NULL,       -- 긴 변
    width_mm     NUMERIC(6,1) NOT NULL,       -- 짧은 변
    height_mm    NUMERIC(6,1) NOT NULL,       -- 전체 높이 = 플랜지 판 윗면
    mass_kg      NUMERIC(5,2) NOT NULL,

    spec_extra   JSONB NOT NULL DEFAULT '{}'::jsonb
);


-- ── 표 2 — magazine_log (append-only) (§4) ───────────────────────────
-- 미션 1회당 최대 4행. UPDATE 도 DELETE 도 하지 않는다.
CREATE TABLE IF NOT EXISTS magazine_log (
    log_id       BIGSERIAL PRIMARY KEY,
    run_id       UUID        NOT NULL,        -- 미션 1회를 묶는 끈
    qr_payload   TEXT        NOT NULL,        -- 'F1-MGZB-1' 원문 그대로
    kind_code    TEXT        NOT NULL,        -- 'MGZB' → carrier_kind 조인
    plant_code   TEXT                         -- 'F1' 공장. 첫 토큰은 항상 첫 자리다(§8-3)
                 GENERATED ALWAYS AS (split_part(qr_payload, '-', 1)) STORED,
    robot_id     TEXT        NOT NULL,        -- 'robot1'

    stage        TEXT        NOT NULL,        -- pick | nav | place | return  (소문자)
    attempt      INT         NOT NULL DEFAULT 1,   -- 같은 단계의 몇 번째 시도인가 (§4-7)
    started_at   TIMESTAMPTZ NOT NULL,        -- 단계 시작 (벽시계)
    ended_at     TIMESTAMPTZ NOT NULL,        -- 단계 끝  (벽시계) ← 표시하는 값
    started_sim  DOUBLE PRECISION,            -- 같은 순간의 시뮬 시각
    ended_sim    DOUBLE PRECISION,
    duration_sec DOUBLE PRECISION
                 GENERATED ALWAYS AS (ended_sim - started_sim) STORED,

    succeeded    BOOLEAN     NOT NULL,        -- 그 '단계' 의 성패
    status       run_status  NOT NULL,        -- 그 시점 '물체' 의 상태
    fail_reason  fail_reason,                 -- 성공 행은 NULL
    fail_detail  TEXT,                        -- 'PORT_OCCUPIED(3)'
    port         TEXT,                        -- 'test_loader' 어디서

    CONSTRAINT mag_reason_iff_failed
        CHECK (succeeded = (fail_reason IS NULL)),
    CONSTRAINT mag_detail_needs_reason
        CHECK (fail_detail IS NULL OR fail_reason IS NOT NULL),
    CONSTRAINT mag_run_stage_once
        -- ★ attempt 가 키에 들어간다. 웹 복구로 같은 단계를 다시 돌리면
        --   run_id 를 이어가고 attempt 만 올리기 때문에(§4-7), 이게 없으면
        --   재시도 행이 제약에 막혀 기록을 통째로 잃는다.
        --   event_logger 의 ON CONFLICT (run_id, stage, attempt) 가 이 인덱스를 쓴다.
        UNIQUE (run_id, stage, attempt)
);

CREATE INDEX IF NOT EXISTS idx_mag_carrier ON magazine_log (qr_payload, ended_at DESC);
CREATE INDEX IF NOT EXISTS idx_mag_robot   ON magazine_log (robot_id,   ended_at DESC);
CREATE INDEX IF NOT EXISTS idx_mag_open    ON magazine_log (status) WHERE status = 'IN_TRANSIT';
-- 로그 화면 keyset 페이징 정렬 기준 (WBS 2.6)
CREATE INDEX IF NOT EXISTS idx_mag_page    ON magazine_log (ended_at DESC, log_id DESC);


-- ── 표 3 — stack_log (표 2와 컬럼이 완전히 같다) (§4) ────────────────
CREATE TABLE IF NOT EXISTS stack_log (
    log_id       BIGSERIAL PRIMARY KEY,
    run_id       UUID        NOT NULL,
    qr_payload   TEXT        NOT NULL,
    kind_code    TEXT        NOT NULL,
    plant_code   TEXT
                 GENERATED ALWAYS AS (split_part(qr_payload, '-', 1)) STORED,
    robot_id     TEXT        NOT NULL,

    stage        TEXT        NOT NULL,
    attempt      INT         NOT NULL DEFAULT 1,
    started_at   TIMESTAMPTZ NOT NULL,
    ended_at     TIMESTAMPTZ NOT NULL,
    started_sim  DOUBLE PRECISION,
    ended_sim    DOUBLE PRECISION,
    duration_sec DOUBLE PRECISION
                 GENERATED ALWAYS AS (ended_sim - started_sim) STORED,

    succeeded    BOOLEAN     NOT NULL,
    status       run_status  NOT NULL,
    fail_reason  fail_reason,
    fail_detail  TEXT,
    port         TEXT,

    CONSTRAINT stk_reason_iff_failed
        CHECK (succeeded = (fail_reason IS NULL)),
    CONSTRAINT stk_detail_needs_reason
        CHECK (fail_detail IS NULL OR fail_reason IS NOT NULL),
    CONSTRAINT stk_run_stage_once
        -- ★ attempt 가 키에 들어간다. 웹 복구로 같은 단계를 다시 돌리면
        --   run_id 를 이어가고 attempt 만 올리기 때문에(§4-7), 이게 없으면
        --   재시도 행이 제약에 막혀 기록을 통째로 잃는다.
        --   event_logger 의 ON CONFLICT (run_id, stage, attempt) 가 이 인덱스를 쓴다.
        UNIQUE (run_id, stage, attempt)
);

CREATE INDEX IF NOT EXISTS idx_stk_carrier ON stack_log (qr_payload, ended_at DESC);
CREATE INDEX IF NOT EXISTS idx_stk_robot   ON stack_log (robot_id,   ended_at DESC);
CREATE INDEX IF NOT EXISTS idx_stk_open    ON stack_log (status) WHERE status = 'IN_TRANSIT';
CREATE INDEX IF NOT EXISTS idx_stk_page    ON stack_log (ended_at DESC, log_id DESC);


-- ── 뷰 4개 (§6) ──────────────────────────────────────────────────────
-- append-only 의 유일한 비용이 "지금 상태가 뭐냐" 를 못 읽는 것이고, 뷰가 그걸 갚는다.

CREATE OR REPLACE VIEW magazine_latest AS
SELECT DISTINCT ON (qr_payload) *
FROM magazine_log ORDER BY qr_payload, log_id DESC;

CREATE OR REPLACE VIEW stack_latest AS
SELECT DISTINCT ON (qr_payload) *
FROM stack_log ORDER BY qr_payload, log_id DESC;

-- 매거진 · 스택을 한 줄로 (표 1이 잇는다)
CREATE OR REPLACE VIEW production_tracking AS
SELECT p.magazine_payload,  km.carrier_type AS magazine_type,
       m.robot_id AS magazine_robot,  m.status AS magazine_status,
       m.fail_reason AS magazine_fail, m.fail_detail AS magazine_fail_detail,
       m.ended_at AS magazine_at,
       p.stack_payload,     ks.carrier_type AS stack_type,
       s.robot_id AS stack_robot,     s.status AS stack_status,
       s.fail_reason AS stack_fail,   s.fail_detail AS stack_fail_detail,
       s.ended_at AS stack_at
FROM carrier_pair p
LEFT JOIN magazine_latest m ON m.qr_payload = p.magazine_payload
LEFT JOIN stack_latest    s ON s.qr_payload = p.stack_payload
LEFT JOIN carrier_kind   km ON km.kind_code = m.kind_code
LEFT JOIN carrier_kind   ks ON ks.kind_code = s.kind_code;

-- 전체 집계용 — 표 2 UNION ALL 표 3
CREATE OR REPLACE VIEW carrier_log AS
SELECT 'MAGAZINE' AS role, log_id, run_id, qr_payload, kind_code, plant_code, robot_id,
       stage, attempt, started_at, ended_at, duration_sec, succeeded, status,
       fail_reason, fail_detail, port
FROM magazine_log
UNION ALL
SELECT 'STACK',           log_id, run_id, qr_payload, kind_code, plant_code, robot_id,
       stage, attempt, started_at, ended_at, duration_sec, succeeded, status,
       fail_reason, fail_detail, port
FROM stack_log;

-- 미션별 최종 결과 = 그 run_id 의 마지막 행.
--
-- ★ 미션 단위 집계는 **반드시 이걸로** 한다(§4-7 · §6). 재시도한 미션에는
--   FAILED 행과 COMPLETED 행이 같은 run_id 안에 같이 있어서, carrier_log 를
--   그대로 세면 한 미션이 성공에도 실패에도 잡힌다.
CREATE OR REPLACE VIEW run_outcome AS
SELECT DISTINCT ON (run_id)
       run_id, role, qr_payload, kind_code, plant_code, robot_id,
       status AS final_status, fail_reason, fail_detail,
       (SELECT max(attempt) FROM carrier_log c2 WHERE c2.run_id = c.run_id) AS max_attempt
FROM carrier_log c
ORDER BY run_id, log_id DESC;

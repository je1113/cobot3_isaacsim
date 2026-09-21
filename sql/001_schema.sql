-- ═══════════════════════════════════════════════════════════════════════
--  생산 트래킹 DB — 스키마 (ENUM 2 · 표 4 · 인덱스 6 · 뷰 5)
--
--  설계 원본: docs/DB구성.md  ← feature/grasp 브랜치. "왜 이 모양인가" 는 전부 거기 있다
--  그림:      docs/ERD.md
--
--      psql "$COBOT3_DB_DSN" -v ON_ERROR_STOP=1 -1 -f sql/001_schema.sql
--
--      -v ON_ERROR_STOP=1   없으면 에러가 나도 계속 진행해 절반짜리 DB 가 남는다
--      -1                   파일 전체를 트랜잭션 하나로. 실패하면 아무것도 안 남는다
--
--  ── 한 줄 요약 ────────────────────────────────────────────────────────
--  로봇이 캐리어를 한 단계 다룰 때마다 행을 하나 append 한다. 고치지 않는다.
--  한 행에 QR 원문 · 종류 · 로봇 · 단계와 시각 · 물체 상태 · 실패 사유가 들어간다.
--  매거진과 스택은 표를 나누고, 둘의 1:1 대응은 정적인 표 하나가 갖는다.
--
--  ── 순서가 중요하다 ──────────────────────────────────────────────────
--    ① ENUM        표가 이걸 참조한다
--    ② carrier_pair / carrier_kind
--    ③ magazine_log / stack_log
--    ④ 인덱스
--    ⑤ 뷰          carrier_log 가 표 2·3 을, run_outcome 이 carrier_log 를 참조한다
-- ═══════════════════════════════════════════════════════════════════════


-- ═══════════════════════════════════════════════════════════════════════
--  ① ENUM 2개                                          (docs/DB구성.md §2)
-- ═══════════════════════════════════════════════════════════════════════

-- 물체의 상태. 세 값이 늘 일이 없어 ENUM 이다 — 오타('COMPLETE')를 DB 가 거부한다.
CREATE TYPE run_status AS ENUM ('IN_TRANSIT', 'COMPLETED', 'FAILED');

-- 실패 사유. 넷으로 못 박은 이유: 다섯 번째 값을 DB 가 거부한다.
-- 참조 테이블로 두면 새 사유가 조용히 늘어난다.
--   pick_error   PICK 실패
--   nav_error    NAV 실패 · RETURN 실패(같은 NavigateTo 액션이다)
--   place_error  PLACE 실패
--   dock_error   DOCK 실패 (미구현)
-- SCAN 은 없다 — 판독 실패는 미션이 시작되지도 않은 것이라 행 자체가 없다(§4-6).
CREATE TYPE fail_reason AS ENUM ('pick_error', 'nav_error', 'place_error', 'dock_error');


-- ═══════════════════════════════════════════════════════════════════════
--  ② 표 1 — carrier_pair (8행, 정적)                    (docs/DB구성.md §3)
--
--  매거진 1개 ↔ 스택 1개. 양쪽에 UNIQUE 가 걸려 1:1 이 DB 레벨에서 보장된다 —
--  한 매거진이 두 스택이 되거나 두 매거진이 같은 스택이 되는 행을 거부한다.
-- ═══════════════════════════════════════════════════════════════════════

CREATE TABLE carrier_pair (
    magazine_payload TEXT PRIMARY KEY,        -- 'F1-MGZB-1'  이 매거진이
    stack_payload    TEXT NOT NULL UNIQUE     -- 'F3-STKB-1'  이 스택이 된다
);


-- ═══════════════════════════════════════════════════════════════════════
--  ② 표 4 — carrier_kind (4행, 정적)                    (docs/DB구성.md §5)
--
--  규격은 개체가 아니라 '종류' 의 속성이다. F1-MGZB-1 과 F2-MGZB-2 는 규격이
--  같으므로 개체 표에 넣으면 같은 숫자가 16번 반복된다.
--
--  수치 출처: isaacpjt/assets/gen_carrier_assets.py 의 규격 요약
--  여기 없는 것과 그 주인
--    flange_* → grasp.yaml 의 flange_size   (로봇이 파지에 쓰는 값. 파일이 주인)
--    qr_size  → frames.yaml 의 label_side_m (생성물. 파일이 주인)
--    variant  → carrier_type 과 같은 값이다
-- ═══════════════════════════════════════════════════════════════════════

CREATE TABLE carrier_kind (
    kind_code    TEXT PRIMARY KEY,            -- 'MGZB'  QR 원문의 품목 코드
    family       TEXT NOT NULL,               -- 'magazine' | 'stack'  어느 표에 쓸지도 이걸로 갈림
    color        TEXT NOT NULL,               -- 'orange'   | 'blue'

    -- 'magazine_blue'. 「어떤 물체인지」이자 grasp.yaml · place.yaml 의 키다.
    -- 생성열로 둬서 family/color 와 yaml 키가 절대 어긋날 수 없게 한다.
    carrier_type TEXT GENERATED ALWAYS AS (family || '_' || color) STORED,

    length_mm    NUMERIC(6,1) NOT NULL,       -- 긴 변
    width_mm     NUMERIC(6,1) NOT NULL,       -- 짧은 변. 그리퍼 개폐량 조회
    height_mm    NUMERIC(6,1) NOT NULL,       -- 전체 높이 = 플랜지 판 윗면
    mass_kg      NUMERIC(5,2) NOT NULL,

    -- 종류별로 축이 다른 것만. 매거진은 slots, 스택은 trays 라 컬럼이 안 겹친다.
    spec_extra   JSONB NOT NULL DEFAULT '{}'::jsonb,

    CONSTRAINT carrier_kind_type_uniq UNIQUE (carrier_type)
);


-- ═══════════════════════════════════════════════════════════════════════
--  ③ 표 2 — magazine_log (append-only)                  (docs/DB구성.md §4)
--
--  미션 1회당 최대 4행 (pick · nav · place · return). UPDATE 도 DELETE 도 하지 않는다.
--  추적성 데이터를 나중에 고치면 그 순간 증거 능력이 사라진다(§8-1).
--  정정은 삭제가 아니라 '정정 행 추가' 다.
-- ═══════════════════════════════════════════════════════════════════════

CREATE TABLE magazine_log (
    log_id       BIGSERIAL PRIMARY KEY,       -- 삽입 순번. 같은 초에 두 행이 와도 순서가 갈린다
    run_id       UUID        NOT NULL,        -- 미션 1회를 묶는 끈. ScanLeaf 성공 순간 uuid4()
    qr_payload   TEXT        NOT NULL,        -- ① 'F1-MGZB-1'  CarrierScan 응답 원문 그대로
    kind_code    TEXT        NOT NULL,        -- ② 'MGZB'  carrier_code.parse_code() 결과

    -- 'F1' 공장. 첫 토큰은 날짜가 끼어도(F1-260921-MGZO-1) 항상 첫 자리라
    -- 생성열로 둬도 안전하다. 두 번째 토큰(kind_code)은 밀리므로 파이썬이 푼다(§8-3).
    plant_code   TEXT        GENERATED ALWAYS AS (split_part(qr_payload, '-', 1)) STORED,

    robot_id     TEXT        NOT NULL,        -- ③ 'robot1'  노드 네임스페이스에서

    stage        TEXT        NOT NULL,        -- ④ 'pick' | 'nav' | 'place' | 'return'
                                              --    BT 잎 상수 그대로라 소문자다
    attempt      INT         NOT NULL DEFAULT 1,   -- ④ 같은 단계의 몇 번째 시도인가.
                                              --    웹 복구(/orchestrator/resume)로 재시도하면 2, 3... (§4-7)

    started_at   TIMESTAMPTZ NOT NULL,        -- ④ 그 단계 시작 (벽시계). task_manager 가 찍는다 —
    ended_at     TIMESTAMPTZ NOT NULL,        -- ④ 그 단계 끝 (벽시계). ← 표시하는 값
                                              --    로거의 now() 를 쓰면 큐 지연·스풀 재적재분이
                                              --    전부 그때로 찍힌다
    started_sim  DOUBLE PRECISION,            --    같은 순간의 시뮬 시각 (use_sim_time)
    ended_sim    DOUBLE PRECISION,            --    Isaac 을 배속/일시정지하면 벽시계와 어긋난다

    -- 사이클 타임은 시뮬 시각이 물리적으로 옳다(§4-2). 벽시계 소요가 필요하면
    -- 쿼리에서 ended_at - started_at 을 빼면 된다.
    duration_sec DOUBLE PRECISION GENERATED ALWAYS AS (ended_sim - started_sim) STORED,

    succeeded    BOOLEAN     NOT NULL,        --    그 '단계' 의 성패
    status       run_status  NOT NULL,        -- ⑤ 그 시점 '물체' 의 상태
                                              --    둘을 따로 두는 이유: RETURN 실패는
                                              --    succeeded=false 인데 status=COMPLETED 다.
                                              --    캐리어는 이미 배달됐다 (§4-3)
    fail_reason  fail_reason,                 -- ⑤ 넷 중 하나. 성공 행은 NULL
    fail_detail  TEXT,                        -- ⑤ 'PORT_OCCUPIED(3)' — _reason_name() 이 만든 문자열.
                                              --    이게 없으면 pick_error 12건을 봐도 고칠 게 뭔지 모른다
    port         TEXT,                        --    'test_loader' 어디서. 당분간 pick 행은 NULL

    CONSTRAINT mag_reason_iff_failed
        CHECK (succeeded = (fail_reason IS NULL)),
    CONSTRAINT mag_detail_needs_reason
        CHECK (fail_detail IS NULL OR fail_reason IS NOT NULL),
    -- 3열인 것이 중요하다 — resume 재시도 행이 여기 걸려 기록을 잃으면 안 된다.
    -- 동시에 메시지 재전송에는 멱등이다 (로거가 ON CONFLICT DO NOTHING 으로 받는다).
    CONSTRAINT mag_run_stage_once
        UNIQUE (run_id, stage, attempt)
);


-- ═══════════════════════════════════════════════════════════════════════
--  ③ 표 3 — stack_log
--
--  magazine_log 와 컬럼이 '완전히' 같다. 제약·인덱스 이름만 stk_ 로 바꾼 것이다
--  (인덱스·제약 이름은 스키마 전역이라 같은 이름을 두 번 못 쓴다).
--  컬럼 설명은 위 magazine_log 를 본다.
-- ═══════════════════════════════════════════════════════════════════════

CREATE TABLE stack_log (
    log_id       BIGSERIAL PRIMARY KEY,
    run_id       UUID        NOT NULL,
    qr_payload   TEXT        NOT NULL,        -- 'F3-STKB-1'
    kind_code    TEXT        NOT NULL,        -- 'STKB'
    plant_code   TEXT        GENERATED ALWAYS AS (split_part(qr_payload, '-', 1)) STORED,
    robot_id     TEXT        NOT NULL,

    stage        TEXT        NOT NULL,
    attempt      INT         NOT NULL DEFAULT 1,
    started_at   TIMESTAMPTZ NOT NULL,
    ended_at     TIMESTAMPTZ NOT NULL,
    started_sim  DOUBLE PRECISION,
    ended_sim    DOUBLE PRECISION,
    duration_sec DOUBLE PRECISION GENERATED ALWAYS AS (ended_sim - started_sim) STORED,

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
        UNIQUE (run_id, stage, attempt)
);


-- ═══════════════════════════════════════════════════════════════════════
--  ④ 인덱스
-- ═══════════════════════════════════════════════════════════════════════

CREATE INDEX idx_mag_carrier ON magazine_log (qr_payload, ended_at DESC);
CREATE INDEX idx_mag_robot   ON magazine_log (robot_id,   ended_at DESC);
CREATE INDEX idx_mag_open    ON magazine_log (status) WHERE status = 'IN_TRANSIT';

CREATE INDEX idx_stk_carrier ON stack_log    (qr_payload, ended_at DESC);
CREATE INDEX idx_stk_robot   ON stack_log    (robot_id,   ended_at DESC);
CREATE INDEX idx_stk_open    ON stack_log    (status) WHERE status = 'IN_TRANSIT';


-- ═══════════════════════════════════════════════════════════════════════
--  ⑤ 뷰 5개                                             (docs/DB구성.md §6)
--
--  append-only 의 유일한 비용이 "지금 상태가 뭐냐" 를 바로 못 읽는 것이고,
--  뷰가 그것을 갚는다.
--
--  의존 순서 (이 순서를 지켜야 한다)
--      magazine_latest / stack_latest  ←  표 2·3
--      carrier_log                     ←  표 2·3
--      production_tracking             ←  latest 둘 + carrier_pair + carrier_kind
--      run_outcome                     ←  carrier_log
-- ═══════════════════════════════════════════════════════════════════════

-- 캐리어별 최신 행 = 그 캐리어의 현재 상태.
-- ⚠️ SELECT * 라서 뷰를 만드는 순간 컬럼 목록이 고정된다. 나중에 표에
--    컬럼을 추가하면 이 뷰 둘을 CREATE OR REPLACE VIEW 로 다시 만들어야 한다.
CREATE VIEW magazine_latest AS
SELECT DISTINCT ON (qr_payload) *
FROM magazine_log
ORDER BY qr_payload, log_id DESC;

CREATE VIEW stack_latest AS
SELECT DISTINCT ON (qr_payload) *
FROM stack_log
ORDER BY qr_payload, log_id DESC;


-- 전체 집계용 — 표 2 UNION ALL 표 3.
-- 표를 둘로 나눈 대가(§8-2)를 이 뷰가 갚는다. 로봇별·사유별 집계는 전부 여기서.
CREATE VIEW carrier_log AS
SELECT 'MAGAZINE' AS role, log_id, run_id, qr_payload, kind_code, plant_code, robot_id,
       stage, attempt, started_at, ended_at, duration_sec, succeeded, status,
       fail_reason, fail_detail, port
FROM magazine_log
UNION ALL
SELECT 'STACK',           log_id, run_id, qr_payload, kind_code, plant_code, robot_id,
       stage, attempt, started_at, ended_at, duration_sec, succeeded, status,
       fail_reason, fail_detail, port
FROM stack_log;


-- 로트 한 줄 — 매거진과 스택을 carrier_pair 가 잇는다.
-- 한 번도 이송하지 않은 개체는 종류가 NULL 이다. 종류는 로그 행이 들고 오기 때문이고,
-- carrier_pair 는 페이로드 두 개만 갖는다.
CREATE VIEW production_tracking AS
SELECT p.magazine_payload,
       km.carrier_type  AS magazine_type,
       m.robot_id       AS magazine_robot,
       m.status         AS magazine_status,
       m.fail_reason    AS magazine_fail,
       m.fail_detail    AS magazine_fail_detail,
       m.ended_at       AS magazine_at,
       p.stack_payload,
       ks.carrier_type  AS stack_type,
       s.robot_id       AS stack_robot,
       s.status         AS stack_status,
       s.fail_reason    AS stack_fail,
       s.fail_detail    AS stack_fail_detail,
       s.ended_at       AS stack_at
FROM carrier_pair p
LEFT JOIN magazine_latest m ON m.qr_payload = p.magazine_payload
LEFT JOIN stack_latest    s ON s.qr_payload = p.stack_payload
LEFT JOIN carrier_kind   km ON km.kind_code = m.kind_code
LEFT JOIN carrier_kind   ks ON ks.kind_code = s.kind_code;


-- ★ 미션별 최종 결과 = 그 run_id 의 마지막 행.
--
--   미션 단위 집계는 '반드시' 이걸 쓴다. 재시도한 미션에는 FAILED 행과
--   COMPLETED 행이 같이 들어 있어서(§4-7), magazine_log 를 직접 세면
--   한 미션이 실패로도 성공으로도 잡힌다.
--
--   max_attempt > 1 이 "사람이 개입해 살린 미션" 이라 그 자체가 지표가 된다.
CREATE VIEW run_outcome AS
SELECT DISTINCT ON (run_id)
       run_id,
       log_id,                                -- DISTINCT ON 의 정렬 키. 어느 행이 뽑혔는지도 보인다
       role,
       qr_payload,
       kind_code,
       plant_code,
       robot_id,
       status       AS final_status,
       fail_reason,
       fail_detail,
       ended_at     AS finished_at,
       (SELECT max(attempt) FROM carrier_log c2 WHERE c2.run_id = c.run_id) AS max_attempt
FROM carrier_log c
ORDER BY run_id, log_id DESC;

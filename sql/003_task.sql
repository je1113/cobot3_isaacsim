-- ══════════════════════════════════════════════════════════════════════
--  003_task.sql — 관제 2표 (표 5 task · 표 6 pending_pickup)
--  원본: docs/DB구성.md §10
--
--  ★ WBS 0단계 확정사항:
--    0.1  선반 '층' 개념 삭제 → task.resume_pass 컬럼 없음. resume_progress 만.
--    0.3  task/pending_pickup 의 유일한 writer 는 웹 백엔드.
--         task_manager 는 action feedback 으로만 보고한다.
--
--  ★ 0.2(중복 배정 인덱스)는 **적용하지 않았다.** 전제가 둘 다 바뀌었다:
--      · "task 1건 = 캐리어 1개" 라 같은 선반에 QUEUED 가 여럿인 것이 정상이다
--      · 두 로봇이 같은 선반에서 같이 일한다 — 선반을 잠그지 않는다
--    그래서 target_ref 에 걸리는 유니크 인덱스가 아예 없다(아래 주석 참고).
-- ══════════════════════════════════════════════════════════════════════

DO $$ BEGIN
    CREATE TYPE task_kind AS ENUM ('SCAN', 'RECOVER');
EXCEPTION WHEN duplicate_object THEN NULL; END $$;

DO $$ BEGIN
    CREATE TYPE task_status AS ENUM ('QUEUED', 'RUNNING', 'DONE', 'FAILED');
EXCEPTION WHEN duplicate_object THEN NULL; END $$;

DO $$ BEGIN
    CREATE TYPE pickup_status AS ENUM ('WAITING', 'QUEUED', 'DONE', 'GAVE_UP');
EXCEPTION WHEN duplicate_object THEN NULL; END $$;


-- ── 표 5 — task (작업 큐) (§10-2) ────────────────────────────────────
CREATE TABLE IF NOT EXISTS task (
    task_id         BIGSERIAL PRIMARY KEY,
    robot_id        TEXT        NOT NULL,      -- 'robot1' · 표 2·3과 같은 값
    kind            task_kind   NOT NULL,
    target_ref      TEXT        NOT NULL,      -- 'shelf_01' | 'test_loader' — yaml 키 (FK 아님)
    queue_order     INT         NOT NULL,      -- 화면 드래그 재정렬. RECOVER 는 음수로 선점
    status          task_status NOT NULL DEFAULT 'QUEUED',

    run_id          UUID,                      -- ScanLeaf 성공 순간 stamp → 표 2·3과 공유
    resume_progress DOUBLE PRECISION,          -- 0.0~1.0. 웹 복구용 (0.1: pass 개념 삭제)

    created_at      TIMESTAMPTZ NOT NULL DEFAULT now(),
    started_at      TIMESTAMPTZ,
    ended_at        TIMESTAMPTZ
);

-- ★ 선반을 잠그는 인덱스는 **두지 않는다.**
--   두 로봇이 같은 선반에 동시에 붙어 매거진을 나눠 빼는 것이 정상 운용이다.
--   선반에 3개가 있으면 두 대가 같이 비우는 편이 빠르다.
--
--   docs/DB구성.md §10-4 는 task_one_scan_per_ref 로 선반 lock 을 파생시키자고
--   적었는데, 그건 "한 선반에 한 대" 를 전제한 것이었다. 그 전제가 틀렸으므로
--   인덱스도 없다. 남은 배타 제약은 아래 '로봇당 하나' 하나뿐이다.
--
--   ⚠️ 대신 **두 로봇이 같은 캐리어를 노릴 수 있다.** task 는 선반까지만
--      가리키고 그 선반에 무엇이 있는지는 scan 해야 알기 때문에, 배정 시점에는
--      막을 방법이 없다. 필요해지면 배타는 선반이 아니라 **캐리어(qr_payload)**
--      수준에 걸어야 한다 — feedback 의 carrier_id 로 이미 올라오므로
--      "그 payload 로 다른 로봇이 RUNNING 중" 을 웹이 감지할 수는 있다.

-- 한 로봇이 동시에 두 작업을 RUNNING 할 수 없다 (§8-2 의 '대가 2' 를 부분적으로 갚는다)
CREATE UNIQUE INDEX IF NOT EXISTS task_one_running_per_robot
    ON task (robot_id) WHERE status = 'RUNNING';

-- 같은 로봇 큐 안에서 순서가 겹치지 않는다
CREATE UNIQUE INDEX IF NOT EXISTS task_queue_order
    ON task (robot_id, queue_order) WHERE status = 'QUEUED';

CREATE INDEX IF NOT EXISTS idx_task_run   ON task (run_id);
CREATE INDEX IF NOT EXISTS idx_task_queue ON task (robot_id, queue_order) WHERE status = 'QUEUED';
CREATE INDEX IF NOT EXISTS idx_task_hist  ON task (created_at DESC);


-- ── 표 6 — pending_pickup (회수 대기) (§10-3) ────────────────────────
-- expected_payload 를 두지 않는다 — source_run_id → magazine_log.qr_payload
-- → carrier_pair.stack_payload 조인으로 나오는 값을 복사할 이유가 없다(§5-2).
CREATE TABLE IF NOT EXISTS pending_pickup (
    pending_id    BIGSERIAL PRIMARY KEY,
    station_ref   TEXT          NOT NULL,      -- 'test_loader' — yaml 키 (FK 아님)
    source_run_id UUID          NOT NULL,      -- 이 산출물을 만든 미션 (표 2의 run_id)
    ready_at      TIMESTAMPTZ   NOT NULL,      -- place 완료 + stations.yaml 의 process_sec
    retry_count   INT           NOT NULL DEFAULT 0,   -- 가 보니 없더라 → 재예약
    status        pickup_status NOT NULL DEFAULT 'WAITING',
    task_id       BIGINT REFERENCES task(task_id) ON DELETE SET NULL,
    created_at    TIMESTAMPTZ   NOT NULL DEFAULT now(),

    CONSTRAINT pickup_task_when_queued
        CHECK (status IN ('WAITING', 'GAVE_UP') OR task_id IS NOT NULL)
);

-- 스케줄러가 매번 훑는 단 하나의 질의
CREATE INDEX IF NOT EXISTS idx_pickup_due
    ON pending_pickup (ready_at) WHERE status = 'WAITING';

-- 한 미션의 산출물에 대한 회수 대기는 하나뿐이다 (중복 생성 방지)
CREATE UNIQUE INDEX IF NOT EXISTS pickup_one_per_run
    ON pending_pickup (source_run_id)
    WHERE status IN ('WAITING', 'QUEUED');

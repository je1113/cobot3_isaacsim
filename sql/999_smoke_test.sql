-- ═══════════════════════════════════════════════════════════════════════
--  생산 트래킹 DB — 자가 점검
--
--      psql "$COBOT3_DB_DSN" -f sql/999_smoke_test.sql
--
--  ⚠️ -1 을 붙이지 말 것. 아래 ②가 BEGIN/ROLLBACK 을 직접 쓴다.
--  ⚠️ -v ON_ERROR_STOP=1 도 붙이지 말 것. ①은 '에러가 나는 것' 을 확인하는 검사다.
--
--  두 가지를 본다
--    ① 제약이 진짜 막는지  — 일부러 틀린 INSERT 4개. 전부 '막힘 OK' 가 나와야 한다
--    ② 뷰가 제대로 답하는지 — docs/DB구성.md §4-7 의 시나리오 5행을 넣어 보고 되돌린다
--
--  ②는 트랜잭션 안에서 돌고 ROLLBACK 으로 끝나므로 데이터가 남지 않는다.
--  스키마를 고칠 때마다 한 번씩 돌리면 뷰가 깨졌는지 바로 안다.
-- ═══════════════════════════════════════════════════════════════════════

\set ON_ERROR_STOP off
\timing off

\echo ''
\echo '════════════════════════════════════════════════════════════════'
\echo ' 0. 환경'
\echo '════════════════════════════════════════════════════════════════'

-- Asia/Seoul 이 아니면 아래 ②의 시각이 9시간 어긋나 보인다.
--   고치는 법:  ALTER DATABASE cobot3 SET timezone = 'Asia/Seoul';  후 재접속
SHOW timezone;

SELECT count(*) AS carrier_kind_행수 FROM carrier_kind;   -- 4 여야 한다
SELECT count(*) AS carrier_pair_행수 FROM carrier_pair;   -- 8 여야 한다


\echo ''
\echo '════════════════════════════════════════════════════════════════'
\echo ' 1. 제약 검사 — 네 개 다 "막힘 OK" 가 나와야 한다'
\echo '════════════════════════════════════════════════════════════════'

-- ① 성공한 단계에 실패 사유가 붙으면 안 된다
DO $$
BEGIN
    INSERT INTO magazine_log (run_id, qr_payload, kind_code, robot_id, stage,
                              started_at, ended_at, succeeded, status, fail_reason)
    VALUES (gen_random_uuid(), 'F1-MGZB-1', 'MGZB', 'robot1', 'pick',
            now(), now(), true, 'IN_TRANSIT', 'pick_error');
    RAISE WARNING '① 성공한 단계 + 실패 사유  →  !! 통과해버림 (mag_reason_iff_failed 가 없다)';
EXCEPTION WHEN check_violation THEN
    RAISE NOTICE  '① 성공한 단계 + 실패 사유  →  막힘 OK';
END $$;

-- ② ENUM 에 없는 사유 (scan_error 는 일부러 안 넣은 값이다)
DO $$
BEGIN
    INSERT INTO magazine_log (run_id, qr_payload, kind_code, robot_id, stage,
                              started_at, ended_at, succeeded, status, fail_reason)
    VALUES (gen_random_uuid(), 'F1-MGZB-1', 'MGZB', 'robot1', 'pick',
            now(), now(), false, 'FAILED', 'scan_error');
    RAISE WARNING '② ENUM 에 없는 fail_reason  →  !! 통과해버림';
EXCEPTION WHEN invalid_text_representation THEN
    RAISE NOTICE  '② ENUM 에 없는 fail_reason  →  막힘 OK';
END $$;

-- ③ 같은 (run_id, stage, attempt) 가 두 번 들어오면 안 된다 (메시지 재전송 멱등)
DO $$
DECLARE r UUID := gen_random_uuid();
BEGIN
    INSERT INTO magazine_log (run_id, qr_payload, kind_code, robot_id, stage,
                              started_at, ended_at, succeeded, status)
    VALUES (r, 'F1-MGZB-1', 'MGZB', 'robot1', 'pick', now(), now(), true, 'IN_TRANSIT');
    INSERT INTO magazine_log (run_id, qr_payload, kind_code, robot_id, stage,
                              started_at, ended_at, succeeded, status)
    VALUES (r, 'F1-MGZB-1', 'MGZB', 'robot1', 'pick', now(), now(), true, 'IN_TRANSIT');
    RAISE WARNING '③ (run_id, stage, attempt) 중복  →  !! 통과해버림';
EXCEPTION WHEN unique_violation THEN
    RAISE NOTICE  '③ (run_id, stage, attempt) 중복  →  막힘 OK';
END $$;

-- ③-b 같은 (run_id, stage) 라도 attempt 가 다르면 들어가야 한다 (웹 복구 재시도)
DO $$
DECLARE r UUID := gen_random_uuid();
BEGIN
    INSERT INTO magazine_log (run_id, qr_payload, kind_code, robot_id, stage, attempt,
                              started_at, ended_at, succeeded, status, fail_reason, fail_detail)
    VALUES (r, 'F1-MGZB-1', 'MGZB', 'robot1', 'place', 1,
            now(), now(), false, 'FAILED', 'place_error', 'PORT_OCCUPIED(3)');
    INSERT INTO magazine_log (run_id, qr_payload, kind_code, robot_id, stage, attempt,
                              started_at, ended_at, succeeded, status)
    VALUES (r, 'F1-MGZB-1', 'MGZB', 'robot1', 'place', 2,
            now(), now(), true, 'COMPLETED');
    RAISE NOTICE  '③-b attempt 1,2 재시도        →  들어감 OK';
    DELETE FROM magazine_log WHERE run_id = r;      -- 흔적 지우기
EXCEPTION WHEN unique_violation THEN
    RAISE WARNING '③-b attempt 1,2 재시도        →  !! 막혀버림 (제약이 2열이다)';
END $$;

-- ④ 한 스택이 두 매거진에서 나올 수 없다
DO $$
BEGIN
    INSERT INTO carrier_pair VALUES ('F9-MGZB-9', 'F3-STKB-1');
    RAISE WARNING '④ 스택 하나에 매거진 둘      →  !! 통과해버림';
EXCEPTION WHEN unique_violation THEN
    RAISE NOTICE  '④ 스택 하나에 매거진 둘      →  막힘 OK';
END $$;


\echo ''
\echo '════════════════════════════════════════════════════════════════'
\echo ' 2. 시나리오 리허설 — docs/DB구성.md §4-7'
\echo '   robot1 이 F1-MGZB-1 을 집어 옮기다 PLACE 에서 PORT_OCCUPIED 로'
\echo '   실패하고, 웹 복구로 재시도해 성공하는 한 미션'
\echo '════════════════════════════════════════════════════════════════'

BEGIN;

INSERT INTO magazine_log
    (run_id, qr_payload, kind_code, robot_id, stage, attempt,
     started_at, ended_at, started_sim, ended_sim,
     succeeded, status, fail_reason, fail_detail, port)
VALUES
 ('7c3f1a8e-9b24-4d51-a0f7-2e6b5c19d833', 'F1-MGZB-1', 'MGZB', 'robot1', 'pick',   1,
  '2026-09-21 12:16:12.3+09', '2026-09-21 12:17:24.5+09', 1893.40, 1936.70,
  true,  'IN_TRANSIT', NULL, NULL, NULL),

 ('7c3f1a8e-9b24-4d51-a0f7-2e6b5c19d833', 'F1-MGZB-1', 'MGZB', 'robot1', 'nav',    1,
  '2026-09-21 12:17:24.6+09', '2026-09-21 12:20:51.8+09', 1936.80, 2061.10,
  true,  'IN_TRANSIT', NULL, NULL, NULL),

 -- 여기서 얼어붙는다. 매거진은 아직 흡착에 붙어 있다.
 ('7c3f1a8e-9b24-4d51-a0f7-2e6b5c19d833', 'F1-MGZB-1', 'MGZB', 'robot1', 'place',  1,
  '2026-09-21 12:20:51.9+09', '2026-09-21 12:22:33.4+09', 2061.20, 2122.10,
  false, 'FAILED', 'place_error', 'PORT_OCCUPIED(3)', 'test_loader'),

 -- 화면에서 포트를 비우고 /orchestrator/resume → 같은 run_id, attempt 만 2 로
 ('7c3f1a8e-9b24-4d51-a0f7-2e6b5c19d833', 'F1-MGZB-1', 'MGZB', 'robot1', 'place',  2,
  '2026-09-21 12:25:00.0+09', '2026-09-21 12:26:10.0+09', 2200.00, 2242.00,
  true,  'COMPLETED', NULL, NULL, 'test_loader'),

 -- 복귀 주행. 물체는 이미 배달됐으므로 status 는 COMPLETED 를 유지한다
 ('7c3f1a8e-9b24-4d51-a0f7-2e6b5c19d833', 'F1-MGZB-1', 'MGZB', 'robot1', 'return', 1,
  '2026-09-21 12:26:10.1+09', '2026-09-21 12:29:30.1+09', 2242.10, 2362.10,
  true,  'COMPLETED', NULL, NULL, NULL);

\echo ''
\echo '── ① 미션 이력 (5행) · 시각이 12:17 대로 보여야 한다 ──'
SELECT upper(stage)                              AS 단계,
       attempt                                   AS 시도,
       to_char(ended_at, 'YYYY.MM.DD HH24:MI')   AS 시각,
       round(duration_sec::numeric, 1)           AS 소요초_시뮬,
       round(EXTRACT(EPOCH FROM (ended_at - started_at))::numeric, 1) AS 소요초_벽시계,
       succeeded, status, fail_reason, fail_detail
FROM magazine_log
WHERE run_id = '7c3f1a8e-9b24-4d51-a0f7-2e6b5c19d833'
ORDER BY log_id;

\echo ''
\echo '── ② run_outcome — final_status=COMPLETED, max_attempt=2 여야 한다 ★ ──'
SELECT run_id, role, qr_payload, robot_id, final_status, max_attempt,
       to_char(finished_at, 'HH24:MI') AS 끝난시각
FROM run_outcome;

\echo ''
\echo '── ③ magazine_latest — 마지막 return 행이 나와야 한다 ──'
SELECT qr_payload, stage, attempt, status, to_char(ended_at, 'HH24:MI') AS 시각
FROM magazine_latest WHERE qr_payload = 'F1-MGZB-1';

\echo ''
\echo '── ④ production_tracking — 매거진은 채워지고 스택은 NULL (아직 안 만들어짐) ──'
SELECT magazine_payload, magazine_type, magazine_robot, magazine_status,
       stack_payload, stack_type, stack_status
FROM production_tracking WHERE magazine_payload = 'F1-MGZB-1';

\echo ''
\echo '── ⑤ 종류별 집계 — run_outcome 을 쓴다. 미션 1건, 사람이 살린 것 1건 ──'
SELECT k.carrier_type,
       count(*)                                            AS 미션,
       count(*) FILTER (WHERE r.final_status = 'COMPLETED') AS 성공,
       count(*) FILTER (WHERE r.max_attempt > 1)            AS 사람이_살린것
FROM run_outcome r LEFT JOIN carrier_kind k USING (kind_code)
GROUP BY k.carrier_type;

ROLLBACK;

\echo ''
\echo '── 되돌림 확인 — 아래가 0 이어야 한다 ──'
SELECT count(*) AS magazine_log_남은행 FROM magazine_log;

\echo ''
\echo '════════════════════════════════════════════════════════════════'
\echo ' 끝. 확인할 것'
\echo '   · 1장의 네 줄이 전부 "막힘 OK" / "들어감 OK" 인가'
\echo '   · 2장 ②의 final_status 가 COMPLETED, max_attempt 가 2 인가'
\echo '   · 시각이 12:17 대인가 (03:17 이면 타임존이 UTC 다)'
\echo '   · magazine_log_남은행 이 0 인가'
\echo ''
\echo ' ★ 이 검사로는 못 잡는 것'
\echo '   started_sim/ended_sim 은 여기서 손으로 넣은 값이다. 실제 미션을'
\echo '   한 바퀴 돌린 뒤 아래가 0 이 아닌지 반드시 확인할 것 —'
\echo '   0 이면 GPU 머신의 /clock 이 ROS 머신까지 안 오는 것이고,'
\echo '   그러면 사이클 타임 분석이 통째로 죽는다 (DB 는 에러를 안 낸다).'
\echo ''
\echo '     SELECT stage, started_sim, ended_sim, duration_sec FROM magazine_log;'
\echo '════════════════════════════════════════════════════════════════'

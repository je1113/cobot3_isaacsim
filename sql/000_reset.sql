-- ═══════════════════════════════════════════════════════════════════════
--  생산 트래킹 DB — 전부 지운다 (개발 중 다시 만들기용)
--
--  설계 원본: docs/DB구성.md  ← feature/grasp 브랜치에 있다
--
--  왜 필요한가
--    CREATE TYPE 에는 IF NOT EXISTS 가 없다. 001_schema.sql 을 두 번 돌리면
--    "type run_status already exists" 로 막힌다. 스키마를 고칠 때마다 이
--    파일부터 돌리는 것이 정석이다.
--
--        psql "$COBOT3_DB_DSN" -v ON_ERROR_STOP=1 -f sql/000_reset.sql
--
--  ⚠️ 데이터가 전부 사라진다. 남겨야 하면 먼저 백업:
--        docker compose exec -T postgres pg_dump -U cobot3 cobot3 > backup/x.sql
--
--  지우는 순서는 만드는 순서의 역순 — 뷰 → 표 → 타입.
--  (CASCADE 가 붙어 있어 순서가 틀려도 지워지기는 한다)
-- ═══════════════════════════════════════════════════════════════════════

DROP VIEW  IF EXISTS run_outcome         CASCADE;
DROP VIEW  IF EXISTS production_tracking CASCADE;
DROP VIEW  IF EXISTS carrier_log         CASCADE;
DROP VIEW  IF EXISTS magazine_latest     CASCADE;
DROP VIEW  IF EXISTS stack_latest        CASCADE;

DROP TABLE IF EXISTS magazine_log        CASCADE;
DROP TABLE IF EXISTS stack_log           CASCADE;
DROP TABLE IF EXISTS carrier_pair        CASCADE;
DROP TABLE IF EXISTS carrier_kind        CASCADE;

DROP TYPE  IF EXISTS run_status          CASCADE;
DROP TYPE  IF EXISTS fail_reason         CASCADE;

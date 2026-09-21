-- ═══════════════════════════════════════════════════════════════════════
--  생산 트래킹 DB — 시드 (carrier_kind 4행 · carrier_pair 8행)
--
--  설계 원본: docs/DB구성.md §3 · §5   ← feature/grasp 브랜치
--
--      psql "$COBOT3_DB_DSN" -v ON_ERROR_STOP=1 -1 -f sql/002_seed.sql
--
--  001_schema.sql 을 먼저 돌려야 한다.
-- ═══════════════════════════════════════════════════════════════════════


-- ═══════════════════════════════════════════════════════════════════════
--  표 4 — carrier_kind (종류 4개)
--
--  수치는 isaacpjt/assets/gen_carrier_assets.py 의 규격 요약에서 옮긴 것이다.
--  edcdf14 에서 네 에셋 플랜지 판이 80x80 으로 통일되고 두께만 6/10 으로
--  갈린 것까지 반영했다 (플랜지 치수 자체는 grasp.yaml 이 주인이라 여기 없다).
--
--  ⚠️ carrier_type 은 생성열(family || '_' || color)이다. 값을 주면 에러난다.
-- ═══════════════════════════════════════════════════════════════════════

INSERT INTO carrier_kind
    (kind_code, family,     color,    length_mm, width_mm, height_mm, mass_kg, spec_extra)
VALUES
    -- MGZO → carrier_type 'magazine_orange'.  리드프레임 매거진 250x140x110 + 플랜지
    ('MGZO', 'magazine', 'orange',      250.0,    140.0,     142.0,    1.00,
     '{"slots": 20, "slot_pitch_mm": 5}'),

    -- MGZB → 'magazine_blue'.  1번보다 넓고 낮다
    ('MGZB', 'magazine', 'blue',        300.0,    200.0,     106.0,    1.00,
     '{"slots": 12, "slot_pitch_mm": 5}'),

    -- STKO → 'stack_orange'.  JEDEC 2인치 트레이 6장 (칩 5 + 뚜껑 1)
    ('STKO', 'stack',    'orange',      328.6,    135.9,      77.7,    1.00,
     '{"trays": 6, "tray_pitch_mm": 7.62}'),

    -- STKB → 'stack_blue'.  같은 트레이 8장
    ('STKB', 'stack',    'blue',        328.6,    135.9,      97.0,    1.00,
     '{"trays": 8, "tray_pitch_mm": 7.62}');


-- ═══════════════════════════════════════════════════════════════════════
--  표 1 — carrier_pair (8쌍)
--
--  어느 매거진이 어느 스택이 되는가. 씬이 안 바뀌면 이 표도 안 바뀐다.
--
--  isaacpjt/assets/carrier_code.py 의 CODES 16개와 대조해 확인한 것 —
--  매거진 8개·스택 8개가 각각 한 번씩만 나오고, 모든 쌍에서 매거진 색 == 스택 색이다.
--
--  저장하는 값은 '페이로드 형태'(F1-MGZB-1)다. 파일명(F1_MGZB_1.usda)은
--  carrier_code.usd_name() 이 '-' 를 '_' 로 바꾼 것이고, QR 에 구워진 문자열은
--  페이로드 쪽이다.
-- ═══════════════════════════════════════════════════════════════════════

INSERT INTO carrier_pair (magazine_payload, stack_payload) VALUES
    -- 파랑 4쌍
    ('F1-MGZB-1', 'F3-STKB-1'),
    ('F1-MGZB-2', 'F3-STKB-2'),
    ('F2-MGZB-1', 'F3-STKB-3'),
    ('F2-MGZB-2', 'F3-STKB-4'),
    -- 주황 4쌍
    ('F1-MGZO-1', 'F3-STKO-1'),
    ('F1-MGZO-2', 'F3-STKO-2'),
    ('F2-MGZO-1', 'F3-STKO-3'),
    ('F2-MGZO-2', 'F3-STKO-4');


-- ── 확인 ────────────────────────────────────────────────────────────────
\echo ''
\echo '── carrier_kind (carrier_type 이 생성열로 채워졌는지) ──'
SELECT kind_code, carrier_type, family, color, length_mm, width_mm, height_mm, spec_extra
FROM carrier_kind ORDER BY kind_code;

\echo ''
\echo '── carrier_pair (8행) ──'
SELECT magazine_payload, stack_payload FROM carrier_pair ORDER BY magazine_payload;

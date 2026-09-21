-- ══════════════════════════════════════════════════════════════════════
--  002_seed.sql — 정적 2표의 시드
--  원본: docs/DB구성.md §3(carrier_pair 8행) · §5(carrier_kind 4행)
--
--  수치는 전부 isaacpjt/assets/gen_carrier_assets.py 의 규격 요약에서 옮긴 것이다.
--  재실행 가능(ON CONFLICT DO NOTHING).
-- ══════════════════════════════════════════════════════════════════════

-- ── 표 4 — carrier_kind (§5) ─────────────────────────────────────────
INSERT INTO carrier_kind
    (kind_code, family, color, length_mm, width_mm, height_mm, mass_kg, spec_extra) VALUES
 ('MGZO', 'magazine', 'orange', 250.0, 140.0, 142.0, 1.00, '{"slots":20,"slot_pitch_mm":5}'),
 ('MGZB', 'magazine', 'blue',   300.0, 200.0, 106.0, 1.00, '{"slots":12,"slot_pitch_mm":5}'),
 ('STKO', 'stack',    'orange', 328.6, 135.9,  77.7, 1.00, '{"trays":6,"tray_pitch_mm":7.62}'),
 ('STKB', 'stack',    'blue',   328.6, 135.9,  97.0, 1.00, '{"trays":8,"tray_pitch_mm":7.62}')
ON CONFLICT (kind_code) DO NOTHING;


-- ── 표 1 — carrier_pair (§3, 확정) ───────────────────────────────────
-- carrier_code.py 의 CODES 16개와 대조: 매거진 8 · 스택 8 이 각각 한 번씩,
-- 모든 쌍에서 매거진 색 == 스택 색 (파랑 4쌍 · 주황 4쌍).
INSERT INTO carrier_pair (magazine_payload, stack_payload) VALUES
  ('F1-MGZB-1', 'F3-STKB-1'),   -- 파랑
  ('F1-MGZB-2', 'F3-STKB-2'),
  ('F2-MGZB-1', 'F3-STKB-3'),
  ('F2-MGZB-2', 'F3-STKB-4'),
  ('F1-MGZO-1', 'F3-STKO-1'),   -- 주황
  ('F1-MGZO-2', 'F3-STKO-2'),
  ('F2-MGZO-1', 'F3-STKO-3'),
  ('F2-MGZO-2', 'F3-STKO-4')
ON CONFLICT (magazine_payload) DO NOTHING;

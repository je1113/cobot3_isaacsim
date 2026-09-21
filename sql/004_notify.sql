-- ══════════════════════════════════════════════════════════════════════
--  004_notify.sql — trace.appended 실시간 채널 (WBS 6.4)
--
--  magazine_log / stack_log INSERT → pg_notify → 웹 백엔드 LISTEN → WebSocket.
--  event_logger 는 이 트리거의 존재를 몰라도 된다 — 평소대로 INSERT 만 하면 된다.
--
--  ★ 이 트리거는 웹 백엔드가 5.1(place 완료 → pending_pickup 생성)을 거는
--    지점이기도 하다. ROS 를 거치지 않고 DB 만으로 고리가 닫힌다.
--
--  payload 는 8000 바이트 제한이 있어 행 전체가 아니라 키만 싣는다.
--  받는 쪽이 필요하면 log_id 로 다시 읽는다.
-- ══════════════════════════════════════════════════════════════════════

CREATE OR REPLACE FUNCTION notify_trace_appended() RETURNS trigger AS $$
BEGIN
    PERFORM pg_notify('trace_appended', json_build_object(
        'role',       TG_ARGV[0],
        'log_id',     NEW.log_id,
        'run_id',     NEW.run_id,
        'qr_payload', NEW.qr_payload,
        'robot_id',   NEW.robot_id,
        'stage',      NEW.stage,
        'succeeded',  NEW.succeeded,
        'status',     NEW.status,
        'port',       NEW.port,
        'ended_at',   NEW.ended_at
    )::text);
    RETURN NULL;   -- AFTER 트리거라 반환값은 무시된다
END;
$$ LANGUAGE plpgsql;

DROP TRIGGER IF EXISTS trg_mag_notify ON magazine_log;
CREATE TRIGGER trg_mag_notify
    AFTER INSERT ON magazine_log
    FOR EACH ROW EXECUTE FUNCTION notify_trace_appended('MAGAZINE');

DROP TRIGGER IF EXISTS trg_stk_notify ON stack_log;
CREATE TRIGGER trg_stk_notify
    AFTER INSERT ON stack_log
    FOR EACH ROW EXECUTE FUNCTION notify_trace_appended('STACK');

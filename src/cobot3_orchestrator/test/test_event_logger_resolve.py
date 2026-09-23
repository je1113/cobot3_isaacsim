"""event_logger._resolve — 품목 코드 해석과 옛 숫자 QR 폴백."""

from types import SimpleNamespace

from cobot3_orchestrator.event_logger import EventLogger

KINDS = [('MGZO', 'magazine'), ('MGZB', 'magazine'),
         ('STKO', 'stack'), ('STKB', 'stack')]


def _resolve(payload, kinds=KINDS):
    return EventLogger._resolve(SimpleNamespace(_kinds=kinds), payload)


def test_new_payloads():
    assert _resolve('F1-MGZB-1') == ('MGZB', 'magazine_log')
    assert _resolve('F3-STKO-2') == ('STKO', 'stack_log')


def test_legacy_numeric_payloads():
    assert _resolve('1') == ('MGZO', 'magazine_log')
    assert _resolve(' 2 ') == ('MGZB', 'magazine_log')


def test_unknown_payloads_stay_unresolved():
    assert _resolve('3') == (None, None)
    assert _resolve('') == (None, None)
    assert _resolve('12') == (None, None)


def test_legacy_needs_code_in_db():
    assert _resolve('1', kinds=[('MGZB', 'magazine')]) == (None, None)

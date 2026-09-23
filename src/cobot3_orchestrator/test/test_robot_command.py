"""RobotCommand(PAUSE · RESUME · SKIP · STOP) 와 PauseGate 단위 테스트.

ROS 그래프 없이 돈다 — 액션 클라이언트·goal handle·future 를 가짜로 둔다.
"""

from types import SimpleNamespace

from cobot3_interfaces.action import NavigateTo, PickCarrier
from cobot3_interfaces.srv import RobotCommand
from py_trees.common import Status

from cobot3_orchestrator import task_manager as tm


class FakeFuture:
    def __init__(self, value=None, done=False):
        self._value = value
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._value

    def finish(self, value):
        self._value = value
        self._done = True


class FakeHandle:
    def __init__(self):
        self.accepted = True
        self.result_future = FakeFuture()
        self.cancels = 0

    def get_result_async(self):
        return self.result_future


class FakeClient:
    def __init__(self):
        self.sent = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        handle = FakeHandle()
        self.sent.append((goal, handle))
        return FakeFuture(handle, done=True)


class FakeLogger:
    def info(self, *_):
        pass

    warning = error = debug = info


class FakeNode:
    def __init__(self):
        self.paused = False
        self.cancelled = []
        self._driving = None

    def get_logger(self):
        return FakeLogger()

    def now_pair(self):
        return (None, None)

    def begin_cancel(self, handle, who):
        handle.cancels += 1
        self.cancelled.append(who)

    def set_driving(self, fut):
        self._driving = fut


def _leaf(node, client, moves_base, goals):
    it = iter(goals)
    result_cls = NavigateTo.Result if moves_base else PickCarrier.Result
    leaf = tm.ActionLeaf('leaf', node, client, 'srv', result_cls,
                         make_goal=lambda: next(it), timeout_s=60.0,
                         moves_base=moves_base)
    leaf.initialise()
    return leaf


def _result(success, fail_reason=0):
    return SimpleNamespace(result=SimpleNamespace(success=success, fail_reason=fail_reason))


def test_pause_cancels_drive_and_resend_same_goal():
    node, client = FakeNode(), FakeClient()
    leaf = _leaf(node, client, True, ['wp1', 'wp2'])
    assert leaf.update() == Status.RUNNING
    assert len(client.sent) == 1
    first = client.sent[0][1]

    node.paused = True
    assert leaf.update() == Status.RUNNING
    assert first.cancels == 1
    assert leaf.update() == Status.RUNNING
    assert first.cancels == 1          # 한 번만 거둔다

    # 취소 결과(CANCELED)는 실패로 읽지 않는다
    first.result_future.finish(_result(False, NavigateTo.Result.CANCELED))
    assert leaf.update() == Status.RUNNING
    assert len(client.sent) == 1       # 쉬는 동안 다시 보내지 않는다

    node.paused = False
    assert leaf.update() == Status.RUNNING
    assert len(client.sent) == 2
    assert client.sent[1][0] == 'wp1'  # 같은 정차점을 다시 — 건너뛰지 않는다

    client.sent[1][1].result_future.finish(_result(True))
    assert leaf.update() == Status.SUCCESS


def test_resume_before_cancel_result_waits():
    node, client = FakeNode(), FakeClient()
    leaf = _leaf(node, client, True, ['wp1'])
    leaf.update()
    first = client.sent[0][1]
    node.paused = True
    leaf.update()
    node.paused = False
    assert leaf.update() == Status.RUNNING
    assert len(client.sent) == 1       # 취소 응답 전에는 새로 안 보낸다
    first.result_future.finish(_result(False, NavigateTo.Result.CANCELED))
    leaf.update()
    assert len(client.sent) == 2


def test_pause_before_send_holds_goal():
    node, client = FakeNode(), FakeClient()
    node.paused = True
    leaf = _leaf(node, client, True, ['wp1'])
    for _ in range(3):
        assert leaf.update() == Status.RUNNING
    assert client.sent == []
    node.paused = False
    leaf.update()
    assert len(client.sent) == 1


def test_arm_goal_runs_to_completion_while_paused():
    node, client = FakeNode(), FakeClient()
    leaf = _leaf(node, client, False, ['pick'])
    leaf.update()
    arm = client.sent[0][1]
    node.paused = True
    assert leaf.update() == Status.RUNNING
    assert arm.cancels == 0            # 팔은 거두지 않는다
    arm.result_future.finish(_result(True))
    assert leaf.update() == Status.SUCCESS


def test_deadline_shifted_by_pause():
    node, client = FakeNode(), FakeClient()
    leaf = _leaf(node, client, False, ['pick'])
    before = leaf.deadline
    node.paused = True
    leaf.update()
    leaf._paused_at -= 30.0            # 30초 쉰 것으로 친다
    node.paused = False
    leaf.update()
    assert leaf.deadline - before >= 30.0


def _cmd_node(**kw):
    n = SimpleNamespace(paused=False, failed=False, _task=None, robot_id='robot1',
                        bb=SimpleNamespace(fail_stage=''), published=0)
    n.__dict__.update(kw)
    n.get_logger = FakeLogger
    n._state_string = lambda: 'state=patrol' + (' | PAUSED' if n.paused else '')

    def pub():
        n.published += 1
    n._publish_state = pub
    return n


def _call(node, command):
    req = RobotCommand.Request(command=command, reason='')
    return tm.TaskManager._on_command(node, req, RobotCommand.Response())


def test_pause_resume_cycle():
    n = _cmd_node()
    res = _call(n, 'PAUSE')
    assert res.accepted and n.paused and 'PAUSED' in res.state
    assert not _call(n, 'PAUSE').accepted
    res = _call(n, 'RESUME')
    assert res.accepted and not n.paused
    res = _call(n, 'RESUME')
    assert not res.accepted and 'RESUME 할 게 없다' in res.rejected_because


def test_pause_rejected_when_failed():
    n = _cmd_node(failed=True, bb=SimpleNamespace(fail_stage='nav'))
    res = _call(n, 'PAUSE')
    assert not res.accepted and 'orchestrator/resume' in res.rejected_because
    res = _call(n, 'RESUME')
    assert not res.accepted and 'orchestrator/resume' in res.rejected_because


def test_skip_stop_need_task():
    n = _cmd_node()
    assert not _call(n, 'SKIP').accepted
    assert not _call(n, 'STOP').accepted
    n._task = object()
    assert _call(n, 'SKIP').accepted
    assert _call(n, 'stop').accepted   # 대소문자 무관


def test_unknown_command():
    assert not _call(_cmd_node(), 'JUMP').accepted


def test_detection_ignored_while_paused():
    n = SimpleNamespace(failed=False, paused=True, patrolling=True,
                        _scan_cooldown_until=0.0, bb=SimpleNamespace(detected=False))
    n.get_logger = FakeLogger
    tm.TaskManager._on_carrier_detected(n, SimpleNamespace(data=True))
    assert n.bb.detected is False
    n.paused = False
    tm.TaskManager._on_carrier_detected(n, SimpleNamespace(data=True))
    assert n.bb.detected is True

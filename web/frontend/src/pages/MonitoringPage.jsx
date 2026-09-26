import {
  useEffect,
  useRef,
  useState,
} from 'react'

import {
  requestRobotPause,
  requestRobotResume,
} from '../api/robots'
import {
  cctvStreamUrl,
  fetchCctvStatus,
} from '../api/cctv'
import {
  deleteTask,
  fetchRobotQueue,
  resetSimQueue,
} from '../api/tasks'
import { ApiError } from '../api/client'

import useWebSocket from '../hooks/useWebSocket'
import { useRobots } from '../contexts/metaHooks'

// 상태를 다시 묻는 간격. 영상 자체는 스트림으로 오고, 이건 "끊겼나" 만 본다.
const CCTV_STATUS_POLL_MS = 3000

/**
 * CCTV — 씬 천장 카메라(/World/Environment/CCTV_Camera)의 실시간 영상.
 *
 * 영상은 백엔드의 MJPEG 스트림(/api/cctv.mjpeg)을 <img> 로 바로 본다.
 * 상태(/api/cctv/status)를 따로 물어 "영상 없음" 을 가른다 — 스트림은 새 장이
 * 안 오면 마지막 장에서 멈출 뿐 에러를 내지 않아서, <img> 만으로는 끊긴 것을
 * 알 수 없다.
 *
 * 카메라는 Top View 와 같은 방향으로 돌려 두었다(화면 오른쪽 = 월드 +y).
 */
function CctvCard() {
  const [status, setStatus] =
    useState(null)
  // 끊겼다 살아나면 스트림을 새로 열어야 한다 — 주소를 바꿔 <img> 를 다시 붙인다.
  const [streamKey, setStreamKey] =
    useState(0)

  useEffect(() => {
    let alive = true
    let wasLive = false

    async function poll() {
      try {
        const next =
          await fetchCctvStatus()

        if (!alive) {
          return
        }

        if (next.live && !wasLive) {
          setStreamKey((k) => k + 1)
        }

        wasLive = Boolean(next.live)
        setStatus(next)
      } catch {
        if (alive) {
          wasLive = false
          setStatus(null)
        }
      }
    }

    poll()

    const timer = setInterval(
      poll,
      CCTV_STATUS_POLL_MS,
    )

    return () => {
      alive = false
      clearInterval(timer)
    }
  }, [])

  const live = Boolean(status?.live)

  let emptyMessage =
    '백엔드에 연결할 수 없습니다.'

  if (status && !status.enabled) {
    emptyMessage =
      'ROS 브리지가 꺼져 있습니다 (COBOT3_ROS=1).'
  } else if (status && !live) {
    emptyMessage =
      '영상 신호 없음 — Isaac Sim 이 Play 중인지 확인하세요.'
  }

  return (
    <section className="monitor-map-card monitor-cctv-card">
      <div className="monitor-map-header">
        <div>
          <h2>
            CCTV
          </h2>

          <p>
            천장 카메라 · 공장 전체
          </p>
        </div>

        <span
          className={
            live
              ? 'monitor-connection-badge connected'
              : 'monitor-connection-badge'
          }
        >
          <i />

          {live
            ? '실시간'
            : '영상 없음'}
        </span>
      </div>

      <div className="monitor-cctv-surface">
        {live ? (
          <img
            key={streamKey}
            src={cctvStreamUrl(
              streamKey,
            )}
            alt="공장 천장 CCTV 실시간 영상"
          />
        ) : (
          <div className="monitor-cctv-empty">
            {emptyMessage}
          </div>
        )}
      </div>
    </section>
  )
}

function formatPosition(position) {
  if (!position) {
    return '-'
  }

  const {
    x,
    y,
    theta,
  } = position

  if (
    x === undefined ||
    y === undefined
  ) {
    return '-'
  }

  return [
    `x: ${x}`,
    `y: ${y}`,
    theta === undefined
      ? null
      : `θ: ${theta}`,
  ]
    .filter(Boolean)
    .join(' · ')
}

function formatTask(task) {
  if (
    task === null ||
    task === undefined
  ) {
    return '-'
  }

  if (
    typeof task === 'string' ||
    typeof task === 'number'
  ) {
    return String(task)
  }

  return (
    task.target_id ??
    task.task_id ??
    '-'
  )
}

const TASK_KIND_LABEL = {
  SCAN: 'SCAN',
  RECOVER: '회수',
}

const TASK_STATUS_LABEL = {
  RUNNING: '진행 중',
  QUEUED: '대기',
}

function getStateClass(state) {
  const normalized =
    String(state ?? '')
      .toUpperCase()

  if (
    normalized.includes('PAUSE')
  ) {
    return 'paused'
  }

  if (
    normalized.includes('NAVIGAT') ||
    normalized.includes('MOVING') ||
    normalized.includes('RUNNING')
  ) {
    return 'moving'
  }

  if (
    normalized.includes('FAIL') ||
    normalized.includes('ERROR')
  ) {
    return 'error'
  }

  return 'idle'
}

function MonitoringPage({
  robotStates = {},
  setRobotStates = () => {},
}) {
  // 로봇 목록은 서버가 준다. 예전에는 이 파일 맨 위에 ROBOT_IDS 상수가 있었고,
  // 그것과 백엔드 설정이 어긋나도 아무도 알아채지 못했다.
  const robots = useRobots()

  const {
    connectionStatus,
    lastMessage,
    error: socketError,
  } = useWebSocket({
    // ★ 2026-09-25: onReconnect 를 안 넘기고 있었다 — 훅은 "끊겼다 붙으면
    //   REST 로 다시 읽어라" 는 신호로 이걸 부르게 설계돼 있는데
    //   (useWebSocket.js 독스트링), 여기서 안 받으면 백엔드가 재시작될
    //   때마다(오늘 여러 번 있었다) 소켓만 재연결되고 화면 상태는 마운트
    //   시점 스냅샷에 멈춘 채다 — 그 사이 놓친 task_changed 는 영원히
    //   못 받는다. 실측: robot2 에 RUNNING task 가 있는데 화면 "작업 큐"
    //   패널은 0건으로 보였다. 로봇별 큐를 다시 받아 스냅샷을 맞춘다.
    onReconnect: () => {
      robots.forEach((robotId) => loadRobotQueue(robotId))
    },
  })

  const [
    pendingControl,
    setPendingControl,
  ] = useState({})

  const [
    controlError,
    setControlError,
  ] = useState('')

  const [
    events,
    setEvents,
  ] = useState([])

  // ── 작업 큐(표 5) — 서버가 실제로 들고 있는 상태 ────────────────────────
  // TaskAssignmentPage 의 queues 는 '작업 시작' 을 누르기 전까지 화면
  // 로컬에만 있는 계획안이다. RECOVER 작업은 사람이 그 화면에서 만들지
  // 않고 pickup 스케줄러(웹 backend)가 자동으로 큐에 넣으므로, 로컬 상태만
  // 봐서는 그게 실제로 생겼는지 알 수 없다 — 그래서 여기서는 서버를 직접 본다.
  const [taskQueues, setTaskQueues] =
    useState({})

  // ★ 2026-09-25: loadRobotQueue 는 여러 군데서 겹쳐 불린다 — 마운트,
  //   task_changed(로봇 하나에 몇 번씩), removeTask 자신의 후속 새로고침,
  //   onReconnect. 네트워크 응답은 보낸 순서대로 안 돌아올 수 있어서,
  //   "삭제 → 새로고침" 요청보다 그 *직전에* 보낸(아직 삭제 전 상태를 담은)
  //   요청의 응답이 나중에 도착하면 방금 지운 항목이 화면에 되살아난다 —
  //   실측: 삭제는 서버에서 매번 204 로 성공했는데(uvicorn 로그 확인),
  //   화면에서는 항목이 다시 보여 "삭제가 자꾸 실패하는" 것처럼 보였다.
  //   로봇별로 마지막에 보낸 요청 번호만 기억해 두고, 응답이 왔을 때 그
  //   번호가 아니면(더 최근 요청이 이미 나간 뒤라면) 버린다.
  const queueRequestSeqRef = useRef({})

  function loadRobotQueue(robotId) {
    const seq = (queueRequestSeqRef.current[robotId] ?? 0) + 1
    queueRequestSeqRef.current[robotId] = seq

    fetchRobotQueue(robotId)
      .then((rows) => {
        if (queueRequestSeqRef.current[robotId] !== seq) {
          return // 더 최근 요청이 이미 나갔다 — 이 응답은 낡았다
        }
        setTaskQueues((prev) => ({
          ...prev,
          [robotId]: Array.isArray(rows)
            ? rows
            : [],
        }))
      })
      .catch(() => {
        // 못 받아도 화면은 그대로 쓴다 — 아래 렌더링이 빈 큐로 폴백한다.
      })
  }

  // 큐 항목 제거 — QUEUED · RUNNING 둘 다 이 버튼 하나로 된다. 서버가
  // RUNNING 이면 STOP 과 같은 경로로 로봇에 정중히 요청해 본 뒤, 응답을
  // 기다리지 않고 표를 강제로 닫는다(queue.cancel() 독스트링 참고 —
  // goal_handle 이 죽은 프로세스 것으로 남아 STOP 만으로는 안 지워지던
  // 버그를 여기서 고쳤다). 그래서 화면도 상태로 버튼을 가르지 않는다.
  const [removingTaskId, setRemovingTaskId] =
    useState(null)

  const [taskActionError, setTaskActionError] =
    useState('')

  function removeTask(taskId, robotId) {
    setTaskActionError('')
    setRemovingTaskId(taskId)

    deleteTask(taskId)
      .then(() => {
        loadRobotQueue(robotId)
      })
      .catch((error) => {
        setTaskActionError(
          error instanceof ApiError
            ? error.message
            : '작업 제거 실패',
        )
      })
      .finally(() => {
        setRemovingTaskId(null)
      })
  }

  // ── 시뮬레이션 초기화 (개발용) ────────────────────────────────────────
  // Isaac Sim 을 껐다 켜면 물리 세계는 리셋되는데 DB 큐는 "이전 세계"를
  // 가리킨 채 남아, 화면에서 하나씩 강제삭제해야 했다(사용자 지적) — 버튼
  // 하나로 task/pending_pickup 을 통째로 비운다.
  const [resettingSim, setResettingSim] =
    useState(false)

  function resetSim() {
    const confirmed = window.confirm(
      '모든 로봇의 작업 큐를 통째로 비웁니다. Isaac Sim 을 새로 시작했을 ' +
        '때만 쓰세요. 되돌릴 수 없습니다 — 계속할까요?',
    )

    if (!confirmed) {
      return
    }

    setTaskActionError('')
    setResettingSim(true)

    resetSimQueue()
      .then(() => {
        robots.forEach((robotId) =>
          loadRobotQueue(robotId),
        )
      })
      .catch((error) => {
        setTaskActionError(
          error instanceof ApiError
            ? error.message
            : '시뮬레이션 초기화 실패',
        )
      })
      .finally(() => {
        setResettingSim(false)
      })
  }

  useEffect(() => {
    robots.forEach((robotId) =>
      loadRobotQueue(robotId),
    )
  }, [robots])

  // task_changed 는 바뀐 행 하나만 싣고 온다 — 그 로봇의 큐를 통째로 다시
  // 받는 편이 부분 병합보다 간단하고, 순서(queue_order)·상태 전이를 놓칠
  // 일이 없다. 폭주 걱정은 없다 — 작업 1건당 몇 번뿐이다.
  useEffect(() => {
    if (!lastMessage) {
      return
    }

    try {
      const raw =
        typeof lastMessage === 'string'
          ? lastMessage
          : lastMessage.data

      if (typeof raw !== 'string') {
        return
      }

      const message = JSON.parse(raw)

      if (message.type === 'task_changed') {
        const robotId = message.robot_id

        if (robotId && robots.includes(robotId)) {
          loadRobotQueue(robotId)
        }
      }
    } catch {
      // 다른 메시지는 무시
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [lastMessage])

  function addEvent(message) {
    const now =
      new Date().toLocaleTimeString(
        'ko-KR',
        {
          hour12: false,
          hour: '2-digit',
          minute: '2-digit',
          second: '2-digit',
        },
      )

    setEvents((prev) => [
      {
        id: `${Date.now()}-${Math.random()}`,
        time: now,
        message,
      },
      ...prev,
    ].slice(0, 8))
  }

  useEffect(() => {
    if (!lastMessage) {
      return
    }

    try {
      const raw =
        typeof lastMessage === 'string'
          ? lastMessage
          : lastMessage.data

      if (
        typeof raw !== 'string'
      ) {
        return
      }

      const message =
        JSON.parse(raw)

      if (
        message.type !==
        'robot_state'
      ) {
        return
      }

      const incomingRobots =
        Array.isArray(message.robots)
          ? message.robots
          : message.robot
            ? [message.robot]
            : message.robot_id
              ? [message]
              : []

      if (
        incomingRobots.length === 0
      ) {
        return
      }

      setRobotStates((prev) => {
        const next = {
          ...prev,
        }

        incomingRobots.forEach(
          (robot) => {
            if (
              !robots.includes(
                robot.robot_id,
              )
            ) {
              return
            }

            const current =
              next[
                robot.robot_id
              ] ?? {
                robot_id:
                  robot.robot_id,
                current_state:
                  null,
                current_task:
                  null,
                position: null,
              }

            next[
              robot.robot_id
            ] = {
              ...current,

              current_state:
                robot.current_state ??
                current.current_state,

              current_task:
                robot.current_task ??
                current.current_task,

              position:
                robot.position ??
                current.position,
            }
          },
        )

        return next
      })
    } catch {
      // WebSocket의 다른 메시지는 무시
    }
  }, [
    lastMessage,
    robots,
    setRobotStates,
  ])

  function getQueue(robotId) {
    return taskQueues[robotId] ?? []
  }

  function getRobot(robotId) {
    return (
      robotStates[robotId] ?? {
        robot_id: robotId,
        current_state: null,
        current_task: null,
        position: null,
      }
    )
  }


  async function pauseRobot(
    robotId,
  ) {
    setControlError('')

    setPendingControl(
      (prev) => ({
        ...prev,
        [robotId]: 'pause',
      }),
    )

    addEvent(
      `${robotId} 일시정지 요청`,
    )

    try {
      await requestRobotPause(
        robotId,
      )

      addEvent(
        `${robotId} 일시정지 명령 전송`,
      )
    } catch (error) {
      setControlError(
        error instanceof Error
          ? error.message
          : '일시정지 명령 전송 실패',
      )

      addEvent(
        `${robotId} 일시정지 명령 전송 실패`,
      )
    } finally {
      setPendingControl(
        (prev) => ({
          ...prev,
          [robotId]: null,
        }),
      )
    }
  }

  async function resumeRobot(
    robotId,
  ) {
    setControlError('')

    setPendingControl(
      (prev) => ({
        ...prev,
        [robotId]: 'resume',
      }),
    )

    addEvent(
      `${robotId} 재개 요청`,
    )

    try {
      await requestRobotResume(
        robotId,
      )

      addEvent(
        `${robotId} 현재 작업 재개 명령 전송`,
      )
    } catch (error) {
      setControlError(
        error instanceof Error
          ? error.message
          : '재개 명령 전송 실패',
      )

      addEvent(
        `${robotId} 재개 명령 전송 실패`,
      )
    } finally {
      setPendingControl(
        (prev) => ({
          ...prev,
          [robotId]: null,
        }),
      )
    }
  }

  const socketConnected =
    connectionStatus ===
    'CONNECTED'

  return (
    <section className="monitoring-page monitoring-control-page">
      <header className="monitor-control-page-header">
        <div>
          <h1>
            실시간 모니터링
          </h1>

          <p>
            AMR을 개별적으로
            일시정지·재개합니다.
          </p>
        </div>

        <div className="monitor-connection-area">
          <button
            type="button"
            className="monitor-reset-sim-button"
            disabled={resettingSim}
            onClick={resetSim}
            title="Isaac Sim 을 새로 시작했을 때 눌러 작업 큐를 통째로 비운다"
          >
            {resettingSim
              ? '초기화 중…'
              : '시뮬레이션 새로 시작'}
          </button>

          <span
            className={
              socketConnected
                ? 'monitor-connection-badge connected'
                : 'monitor-connection-badge'
            }
          >
            <i />

            {socketConnected
              ? '실시간 연결됨'
              : '실시간 연동 전'}
          </span>
        </div>
      </header>

      {(socketError ||
        controlError) && (
        <div className="monitor-control-error">
          {controlError ||
            socketError}
        </div>
      )}

      <div className="monitor-control-layout">
        <div className="monitor-main-column">
        <CctvCard />
        </div>

        <aside className="monitor-robot-column">
          {robots.map(
            (robotId) => {
              const robot =
                getRobot(robotId)

              const queue =
                getQueue(robotId)

              const pending =
                pendingControl[
                  robotId
                ]

              const stateLabel =
                pending === 'pause'
                  ? '일시정지 요청 중'
                  : pending ===
                      'resume'
                    ? '재개 요청 중'
                    : robot
                        .current_state ??
                      '연동 전'

              return (
                <article
                  key={
                    robotId
                  }
                  className="monitor-robot-control-card"
                >
                  <div className="monitor-robot-control-header">
                    <div>
                      <span
                        className={`monitor-robot-dot ${
                          robots.indexOf(
                            robotId,
                          ) === 0
                            ? 'one'
                            : 'two'
                        }`}
                      />

                      <h3>
                        {robotId}
                      </h3>
                    </div>

                    <span
                      className={
                        `monitor-state-badge ${
                          getStateClass(
                            robot
                              .current_state,
                          )
                        }`
                      }
                    >
                      {stateLabel}
                    </span>
                  </div>

                  <div className="monitor-robot-details">
                    <div>
                      <span>
                        현재 작업
                      </span>

                      <strong>
                        {formatTask(
                          robot
                            .current_task,
                        )}
                      </strong>
                    </div>

                    <div>
                      <span>
                        현재 위치
                      </span>

                      <strong>
                        {formatPosition(
                          robot.position,
                        )}
                      </strong>
                    </div>

                  </div>

                  <div className="monitor-task-queue">
                    <div className="monitor-task-queue-header">
                      <span>
                        작업 큐
                      </span>

                      <strong>
                        {queue.length}건
                      </strong>
                    </div>

                    {queue.length === 0 ? (
                      <div className="monitor-task-queue-empty">
                        대기 중인 작업이 없습니다.
                      </div>
                    ) : (
                      <div className="monitor-task-queue-list">
                        {queue.map((task) => (
                          <div
                            className="monitor-task-queue-item"
                            key={task.task_id}
                          >
                            <span
                              className={`monitor-task-kind ${
                                task.kind === 'RECOVER'
                                  ? 'recover'
                                  : 'scan'
                              }`}
                            >
                              {TASK_KIND_LABEL[task.kind] ?? task.kind}
                            </span>

                            <strong>
                              {task.target_ref}
                            </strong>

                            <span
                              className={`monitor-task-status ${
                                task.status === 'RUNNING'
                                  ? 'running'
                                  : 'queued'
                              }`}
                            >
                              {TASK_STATUS_LABEL[task.status] ?? task.status}
                            </span>

                            <button
                              type="button"
                              className="monitor-task-remove-button"
                              disabled={
                                removingTaskId === task.task_id
                              }
                              onClick={() =>
                                removeTask(
                                  task.task_id,
                                  robotId,
                                )
                              }
                              title={
                                task.status === 'RUNNING'
                                  ? '진행 중인 이 작업을 정지하고 큐에서 제거'
                                  : '이 작업을 큐에서 제거'
                              }
                            >
                              {removingTaskId === task.task_id
                                ? '...'
                                : '✕'}
                            </button>
                          </div>
                        ))}
                      </div>
                    )}

                    {taskActionError && (
                      <div className="monitor-task-queue-error">
                        {taskActionError}
                      </div>
                    )}
                  </div>

                  <div className="monitor-control-buttons">
                    <button
                      type="button"
                      className="monitor-pause-button"
                      disabled={
                        Boolean(
                          pending,
                        )
                      }
                      onClick={() =>
                        pauseRobot(
                          robotId,
                        )
                      }
                    >
                      ■ 일시정지
                    </button>

                    <button
                      type="button"
                      className="monitor-resume-button"
                      disabled={
                        Boolean(
                          pending,
                        )
                      }
                      onClick={() =>
                        resumeRobot(
                          robotId,
                        )
                      }
                    >
                      ▶ 재개
                    </button>
                  </div>
                </article>
              )
            },
          )}
        </aside>
      </div>

      <section className="monitor-event-card">
        <div className="monitor-event-header">
          <div>
            <h2>
              Live Event
            </h2>

            <p>
              지도 선택과 제어 명령
              전송 기록
            </p>
          </div>
        </div>

        {events.length === 0 ? (
          <div className="monitor-event-empty">
            아직 명령 기록이 없습니다.
          </div>
        ) : (
          <div className="monitor-event-list">
            {events.map(
              (event) => (
                <div
                  className="monitor-event-row"
                  key={
                    event.id
                  }
                >
                  <time>
                    {event.time}
                  </time>

                  <span>
                    {
                      event.message
                    }
                  </span>
                </div>
              ),
            )}
          </div>
        )}
      </section>
    </section>
  )
}

export default MonitoringPage

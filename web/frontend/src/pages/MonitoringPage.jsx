import {
  useEffect,
  useState,
} from 'react'

import {
  requestRobotGoal,
  requestRobotPause,
  requestRobotResume,
} from '../api/robots'

import useWebSocket from '../hooks/useWebSocket'

const ROBOT_IDS = [
  'AMR-01',
  'AMR-02',
]

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
  robot1Queue = [],
  robot2Queue = [],
}) {
  const {
    connectionStatus,
    lastMessage,
    error: socketError,
  } = useWebSocket()

  const [
    selectedRobotId,
    setSelectedRobotId,
  ] = useState('AMR-01')

  const [
    lastGoals,
    setLastGoals,
  ] = useState({
    'AMR-01': null,
    'AMR-02': null,
  })

  const [
    pendingControl,
    setPendingControl,
  ] = useState({
    'AMR-01': null,
    'AMR-02': null,
  })

  const [
    controlError,
    setControlError,
  ] = useState('')

  const [
    events,
    setEvents,
  ] = useState([])

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
              !ROBOT_IDS.includes(
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
    setRobotStates,
  ])

  function getQueue(robotId) {
    return robotId === 'AMR-01'
      ? robot1Queue
      : robot2Queue
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

  async function handleMapClick(
    event,
  ) {
    const rect =
      event.currentTarget
        .getBoundingClientRect()

    const xRatio =
      (event.clientX - rect.left) /
      rect.width

    const yRatio =
      (event.clientY - rect.top) /
      rect.height

    const goal = {
      x_ratio:
        Number(
          xRatio.toFixed(5),
        ),

      y_ratio:
        Number(
          yRatio.toFixed(5),
        ),
    }

    setLastGoals((prev) => ({
      ...prev,
      [selectedRobotId]: goal,
    }))

    setControlError('')

    addEvent(
      `${selectedRobotId} 지도 목표 지점 선택`,
    )

    setPendingControl(
      (prev) => ({
        ...prev,
        [selectedRobotId]:
          'goal',
      }),
    )

    try {
      await requestRobotGoal(
        selectedRobotId,
        goal,
      )

      addEvent(
        `${selectedRobotId} 이동 명령 전송`,
      )
    } catch (error) {
      setControlError(
        error instanceof Error
          ? error.message
          : '이동 명령 전송 실패',
      )

      addEvent(
        `${selectedRobotId} 이동 명령 전송 실패`,
      )
    } finally {
      setPendingControl(
        (prev) => ({
          ...prev,
          [selectedRobotId]:
            null,
        }),
      )
    }
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

    const savedGoal =
      lastGoals[robotId]

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
        savedGoal,
      )

      addEvent(
        savedGoal
          ? `${robotId} 저장된 목표 지점으로 재개 명령 전송`
          : `${robotId} 현재 작업 재개 명령 전송`,
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
            지도에서 목표 지점을 지정하고
            AMR을 개별적으로
            일시정지·재개합니다.
          </p>
        </div>

        <div className="monitor-connection-area">
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
        <section className="monitor-map-card">
          <div className="monitor-map-header">
            <div>
              <h2>
                Top View
              </h2>

              <p>
                이동시킬 AMR을 선택한 뒤
                지도에서 목표 지점을
                클릭합니다.
              </p>
            </div>

            <div className="monitor-robot-selector">
              {ROBOT_IDS.map(
                (robotId) => (
                  <button
                    key={
                      robotId
                    }
                    type="button"
                    className={
                      selectedRobotId ===
                      robotId
                        ? 'active'
                        : ''
                    }
                    onClick={() =>
                      setSelectedRobotId(
                        robotId,
                      )
                    }
                  >
                    {robotId}
                  </button>
                ),
              )}
            </div>
          </div>

          <div
            className="monitor-map-surface"
            onClick={
              handleMapClick
            }
            role="presentation"
          >
            <div className="monitor-map-guide">
              <strong>
                실제 ROS Map 표시 영역
              </strong>

              <span>
                지도 클릭 →
                {selectedRobotId}
                목표 위치 전송
              </span>
            </div>

            {ROBOT_IDS.map(
              (robotId) => {
                const goal =
                  lastGoals[
                    robotId
                  ]

                if (!goal) {
                  return null
                }

                return (
                  <div
                    key={
                      robotId
                    }
                    className={
                      robotId ===
                      'AMR-01'
                        ? 'monitor-goal-marker robot-one'
                        : 'monitor-goal-marker robot-two'
                    }
                    style={{
                      left:
                        `${goal.x_ratio * 100}%`,
                      top:
                        `${goal.y_ratio * 100}%`,
                    }}
                  >
                    <span />

                    <strong>
                      {robotId}
                      {' '}
                      GOAL
                    </strong>
                  </div>
                )
              },
            )}
          </div>

          <div className="monitor-map-footer">
            <span>
              선택 로봇
            </span>

            <strong>
              {selectedRobotId}
            </strong>

            <span>
              · 지도 클릭 위치는
              재개를 위해 프론트에서
              유지됩니다.
            </span>
          </div>
        </section>

        <aside className="monitor-robot-column">
          {ROBOT_IDS.map(
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
                    : pending ===
                        'goal'
                      ? '이동 명령 전송 중'
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
                        className={
                          robotId ===
                          'AMR-01'
                            ? 'monitor-robot-dot one'
                            : 'monitor-robot-dot two'
                        }
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

                    <div>
                      <span>
                        Queue
                      </span>

                      <strong>
                        {queue.length}
                        개
                      </strong>
                    </div>

                    <div>
                      <span>
                        마지막 Goal
                      </span>

                      <strong>
                        {lastGoals[
                          robotId
                        ]
                          ? '지도 선택 위치 저장됨'
                          : '-'}
                      </strong>
                    </div>
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

          <article className="monitor-camera-card-light">
            <div className="monitor-camera-header">
              <div>
                <h3>
                  AMR Camera
                </h3>

                <span>
                  ROS2 Image 연동 영역
                </span>
              </div>

              <span className="monitor-camera-status">
                연동 전
              </span>
            </div>

            <div className="monitor-camera-placeholder-light">
              Camera
            </div>
          </article>
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

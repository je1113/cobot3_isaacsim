import {
  useEffect,
  useState,
} from 'react'

import {
  requestRobotPause,
  requestRobotResume,
} from '../api/robots'
import { fetchMapInfo } from '../api/map'
import {
  fetchShelves,
  fetchStations,
} from '../api/config'

import useWebSocket from '../hooks/useWebSocket'
import { useRobots } from '../contexts/MetaContext'

function toNumber(value) {
  if (
    value === null ||
    value === undefined ||
    value === ''
  ) {
    return null
  }

  const n = Number(value)
  return Number.isFinite(n)
    ? n
    : null
}

function toPoint(pose) {
  if (!pose) {
    return null
  }

  const x = toNumber(pose.x)
  const y = toNumber(pose.y)

  if (x === null || y === null) {
    return null
  }

  return { x, y }
}

function midpoint(a, b) {
  if (!a || !b) {
    return a || b
  }

  return {
    x: (a.x + b.x) / 2,
    y: (a.y + b.y) / 2,
  }
}

// 선반 실제 크기(m) — simple_factory_layout.usda Shelf_01/02 의 Level_2
// 큐브 스케일(2.5 x 1 x 0.05)을 그대로 쓴다.
const SHELF_WIDTH_M = 2.5
const SHELF_DEPTH_M = 1.0
// 선반 슬롯 구분선 — 매거진 4개 자리를 3개 선으로 나눈다(중앙 기준 오프셋).
const SHELF_SLOT_OFFSETS = [
  -SHELF_WIDTH_M / 4,
  0,
  SHELF_WIDTH_M / 4,
]

// 컨베이어(스테이션) 실제 크기(m) — PackagingZone/ConveyorFrame 큐브
// 스케일(4.4 x 1.35 x 0.45)을 그대로 쓴다. 세 구역(PKG-01 계열) 모두 같은
// 스케일이라 station_type 과 무관하게 동일 크기를 쓴다.
const STATION_LENGTH_M = 4.4
const STATION_WIDTH_M = 1.35
// 롤러 표시선 — 길이 방향으로 고르게 나눈 7개.
const STATION_ROLLER_OFFSETS = [-1.8, -1.2, -0.6, 0, 0.6, 1.2, 1.8]

// shelves.yaml 에 선반으로 들어 있지만 씬에서는 컨베이어인 것.
// PKG-OUT 은 포장 출력(PackagingUnloaderZone) 벨트 앞의 스택 관측 자리다 —
// shelves.yaml 에 있는 이유는 carrier_code_reader 가 그 목록으로 관측 자세를
// 고르기 때문이다(그 파일의 PKG-OUT 주석). 벨트는 PackagingZone · TestingZone
// 과 크기가 같으므로 스테이션과 같은 컨베이어 아이콘으로 그린다.
const CONVEYOR_SHELF_IDS = new Set(['PKG-OUT'])

/**
 * 컨베이어 아이콘 하나. 스테이션(PKG-01 · TEST-01)과 PKG-OUT 이 같이 쓴다 —
 * 세 벨트가 씬에서 같은 크기라 화면에서도 한 모양이어야 한다.
 *
 * ★ 벨트는 월드 x 축을 따라 흐른다. 화면은 x/y 를 맞바꿔 그리므로(toSvg)
 *   긴 변이 세로로 선다.
 */
function ConveyorIcon({ p, label, variant }) {
  return (
    <g
      className={`monitor-topview-station ${variant}`}
    >
      {/* 컨베이어 프레임 — 실제 크기(4.4 x 1.35 m) */}
      <rect
        className="conveyor-frame"
        x={p.x - STATION_WIDTH_M / 2}
        y={p.y - STATION_LENGTH_M / 2}
        width={STATION_WIDTH_M}
        height={STATION_LENGTH_M}
        rx={0.1}
      />

      {/* 벨트 중심선 — 흐름 방향(세로) */}
      <line
        className="conveyor-belt-line"
        x1={p.x}
        y1={p.y - STATION_LENGTH_M / 2 + 0.15}
        x2={p.x}
        y2={p.y + STATION_LENGTH_M / 2 - 0.15}
      />

      {/* 롤러 표시 — 흐름에 직각(가로) */}
      {STATION_ROLLER_OFFSETS.map((dy) => (
        <line
          key={dy}
          className="conveyor-roller"
          x1={p.x - STATION_WIDTH_M / 2 + 0.1}
          y1={p.y + dy}
          x2={p.x + STATION_WIDTH_M / 2 - 0.1}
          y2={p.y + dy}
        />
      ))}

      <text
        x={p.x}
        y={p.y + STATION_LENGTH_M / 2 + 0.32}
      >
        {label}
      </text>
    </g>
  )
}

/**
 * Top View — 실제 지도 이미지 대신 선반·스테이션·로봇 위치로 그리는
 * 개략도. 격자 점유 지도(simple_factory_layout.png)는 흑백회색 3색뿐인
 * 단순 도형이라 그대로 키워 보여줘도 알아보기 어려웠다(더 정밀한 지도도
 * 지금은 필요 없다 — 지도 클릭으로 이동시키는 기능이 없다).
 *
 * worldBounds(=/api/map 의 origin/resolution/크기)로 화면 좌표계만
 * map 프레임에 맞춘다 — 실제 지도 픽셀은 안 쓴다.
 *
 * map 프레임은 y 가 위로 증가하는데 SVG 화면좌표는 y 가 아래로 증가하므로
 * toSvg() 에서 y 를 뒤집는다. 로봇 방향(theta, rad, 반시계)도 화면에서는
 * 시계 방향 회전이 되므로 같이 뒤집는다.
 *
 * 지도가 세로로 길어 화면에서 좁게 보이므로, 위 변환 결과를 화면 전체
 * 기준으로 시계 방향 90도 더 돌려서(가로로 길게) 보여준다 — toSvg() 에서
 * x/y 를 맞바꾸고, 로봇 회전각에 90도를 더한다.
 */
function FactoryTopView({
  worldBounds,
  shelves,
  stations,
  robots,
  robotStates,
}) {
  if (!worldBounds) {
    return (
      <div className="monitor-topview-empty">
        지도 정보를 불러오는 중…
      </div>
    )
  }

  const minX = worldBounds.origin[0]
  const minY = worldBounds.origin[1]
  const width =
    worldBounds.width_px *
    worldBounds.resolution
  const height =
    worldBounds.height_px *
    worldBounds.resolution
  // 화면을 가로로 길게 쓰기 위해 x/y 축을 맞바꿔 90도 회전시켜 그린다.
  function toSvg(point) {
    return {
      x: point.y - minY,
      y: point.x - minX,
    }
  }

  return (
    <svg
      className="monitor-topview-svg"
      viewBox={`0 0 ${height} ${width}`}
      preserveAspectRatio="xMidYMid meet"
    >
      <rect
        className="monitor-topview-floor"
        x={0}
        y={0}
        width={height}
        height={width}
      />

      {shelves.map((shelf) => {
        const center = midpoint(
          toPoint(
            shelf.waypoint_start,
          ),
          toPoint(shelf.waypoint_end),
        )

        if (!center) {
          return null
        }

        const p = toSvg(center)

        if (
          CONVEYOR_SHELF_IDS.has(
            shelf.shelf_id,
          )
        ) {
          return (
            <ConveyorIcon
              key={shelf.shelf_id}
              p={p}
              label={shelf.shelf_id}
              variant="output"
            />
          )
        }

        return (
          <g
            key={shelf.shelf_id}
            className="monitor-topview-shelf"
          >
            {/* 선반 프레임 — 실제 크기(2.5 x 1.0 m).
                ★ 선반의 긴 변은 월드 x 축을 따른다. 화면은 x/y 를 맞바꿔
                  그리므로(toSvg) 긴 변이 세로로 선다. */}
            <rect
              className="shelf-frame"
              x={p.x - SHELF_DEPTH_M / 2}
              y={p.y - SHELF_WIDTH_M / 2}
              width={SHELF_DEPTH_M}
              height={SHELF_WIDTH_M}
              rx={0.05}
            />

            {/* 매거진 슬롯 구분선 — 긴 변(세로)을 나누므로 가로선이다 */}
            {SHELF_SLOT_OFFSETS.map((dy) => (
              <line
                key={dy}
                className="shelf-slot"
                x1={p.x - SHELF_DEPTH_M / 2 + 0.08}
                y1={p.y + dy}
                x2={p.x + SHELF_DEPTH_M / 2 - 0.08}
                y2={p.y + dy}
              />
            ))}

            <text
              x={p.x}
              y={p.y + SHELF_WIDTH_M / 2 + 0.32}
            >
              {shelf.shelf_id}
            </text>
          </g>
        )
      })}

      {stations.map((station) => {
        const point = toPoint(
          station.place_pose,
        )

        if (!point) {
          return null
        }

        const p = toSvg(point)

        return (
          <ConveyorIcon
            key={
              station.station_id
            }
            p={p}
            label={
              station.station_id
            }
            variant={
              station.station_type ===
              'PACKAGING'
                ? 'packaging'
                : 'test'
            }
          />
        )
      })}

      {robots.map(
        (robotId, index) => {
          const robot =
            robotStates[robotId]

          const point = toPoint(
            robot?.position,
          )

          if (!point) {
            return null
          }

          const p = toSvg(point)

          const theta =
            toNumber(
              robot?.position
                ?.theta,
            ) ?? 0

          // rad(반시계) → svg 화면 회전(시계) — y 를 뒤집은 것과 같은 이유.
          // +90 은 화면 전체를 가로로 돌린 것(toSvg 의 x/y 맞바꿈)에 맞춘 보정.
          const deg =
            90 -
            (theta * 180) /
              Math.PI

          return (
            <g
              key={robotId}
              className={
                `monitor-topview-robot ${
                  index === 0
                    ? 'robot-one'
                    : 'robot-two'
                }`
              }
              transform={
                `translate(${p.x}, ${p.y})`
              }
            >
              <g
                transform={
                  `rotate(${deg})`
                }
              >
                {/* 차체 — 위에서 본 AMR */}
                <rect
                  className="robot-body"
                  x={-0.22}
                  y={-0.3}
                  width={0.44}
                  height={0.55}
                  rx={0.12}
                />

                {/* 좌우 바퀴 */}
                <rect
                  className="robot-wheel"
                  x={-0.32}
                  y={-0.09}
                  width={0.1}
                  height={0.24}
                  rx={0.03}
                />

                <rect
                  className="robot-wheel"
                  x={0.22}
                  y={-0.09}
                  width={0.1}
                  height={0.24}
                  rx={0.03}
                />

                {/* 정면 표시(진행 방향) */}
                <path
                  className="robot-heading"
                  d="M 0 -0.42 L 0.13 -0.2 L -0.13 -0.2 Z"
                />
              </g>

              <text y={-0.5}>
                {robotId}
              </text>
            </g>
          )
        },
      )}
    </svg>
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
  queues = {},
}) {
  // 로봇 목록은 서버가 준다. 예전에는 이 파일 맨 위에 ROBOT_IDS 상수가 있었고,
  // 그것과 백엔드 설정이 어긋나도 아무도 알아채지 못했다.
  const robots = useRobots()

  const {
    connectionStatus,
    lastMessage,
    error: socketError,
  } = useWebSocket()

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

  // Top View 는 실제 지도 이미지 대신 선반·스테이션·로봇 위치로 그린
  // 개략도다(격자 지도는 3색뿐인 단순 도형이라 그대로 보여줘도 알아보기
  // 어려웠다). world 범위(origin/resolution/크기)만 /api/map 에서 받아
  // 화면 좌표계를 map 프레임에 맞춘다 — 세션 내내 안 바뀌니 한 번만 받는다.
  const [worldBounds, setWorldBounds] =
    useState(null)

  // 선반·스테이션 위치 — 「설정」 화면이 편집하는 그 파일들을 그대로 읽는다.
  // 좌표 자체가 바뀌는 일은 드물어서(설정 화면에서 저장할 때뿐) 마운트 시
  // 한 번만 받는다 — 실시간으로 다시 받을 이유가 없다.
  const [shelves, setShelves] =
    useState([])

  const [stations, setStations] =
    useState([])

  useEffect(() => {
    let cancelled = false

    fetchMapInfo()
      .then((info) => {
        if (!cancelled) {
          setWorldBounds(info)
        }
      })
      .catch(() => {
        // 못 받아도 화면은 그대로 쓸 수 있어야 한다 — 아래 렌더링이
        // worldBounds 없으면 빈 캔버스로 폴백한다.
      })

    fetchShelves()
      .then((data) => {
        if (!cancelled) {
          setShelves(data.shelves ?? [])
        }
      })
      .catch(() => {})

    fetchStations()
      .then((data) => {
        if (!cancelled) {
          setStations(
            data.stations ?? [],
          )
        }
      })
      .catch(() => {})

    return () => {
      cancelled = true
    }
  }, [])

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
    return queues[robotId] ?? []
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
                공장 배치도입니다.
              </p>
            </div>
          </div>

          <div className="monitor-map-surface">
            <FactoryTopView
              worldBounds={
                worldBounds
              }
              shelves={shelves}
              stations={stations}
              robots={robots}
              robotStates={
                robotStates
              }
            />
          </div>
        </section>

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

                    <div>
                      <span>
                        Queue
                      </span>

                      <strong>
                        {queue.length}
                        개
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

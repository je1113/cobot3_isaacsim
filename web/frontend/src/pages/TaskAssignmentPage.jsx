import {
  requestTaskStart,
} from '../api/tasks'

import useApiRequest from '../hooks/useApiRequest'
import {
  useMeta,
  useRobots,
} from '../contexts/metaHooks'

function isTeachingComplete(shelf) {
  return (
    shelf.scan_passes.length > 0 &&
    shelf.scan_passes.every((pass) =>
      pass.arm_teach_pose.every(
        (joint) => joint.trim() !== '',
      ),
    )
  )
}

function TaskAssignmentPage({
  shelves,
  stations = [],
  queues,
  setQueues,
}) {
  // ★ 로봇 목록은 서버가 준다(GET /api/meta ← COBOT3_ROBOTS).
  //   예전에는 'AMR-01' / 'AMR-02' 가 이 파일에만 12곳 박혀 있었고,
  //   큐도 robot1Queue / robot2Queue 두 개의 useState 였다. 그러면 로봇이
  //   3대가 되는 날 화면 전체를 다시 훑어야 한다.
  const robots = useRobots()

  const {
    scan_direction_arrows:
      directionArrows,
  } = useMeta()

  const {
    loading: taskStartLoading,
    error: taskStartError,
    execute,
  } = useApiRequest()

  const queueOf = (robotId) =>
    queues[robotId] ?? []

  const updateQueue = (
    robotId,
    updater,
  ) =>
    setQueues((prev) => ({
      ...prev,
      [robotId]: updater(
        prev[robotId] ?? [],
      ),
    }))

  function getAssignedRobot(shelfId) {
    return (
      robots.find((robotId) =>
        queueOf(robotId).some(
          (shelf) =>
            shelf.shelf_id === shelfId,
        ),
      ) ?? null
    )
  }

  function assignShelf(shelf, robotId) {
    if (!isTeachingComplete(shelf)) {
      window.alert(
        `${shelf.shelf_id}은 티칭 미완료 상태라 배정할 수 없습니다.`,
      )
      return
    }

    const assignedRobot =
      getAssignedRobot(shelf.shelf_id)

    if (assignedRobot) {
      window.alert(
        `${shelf.shelf_id}은 이미 ${assignedRobot}에 배정되어 있습니다.`,
      )
      return
    }

    updateQueue(robotId, (prev) => [
      ...prev,
      shelf,
    ])
  }

  function removeShelf(
    robotId,
    shelfId,
  ) {
    updateQueue(robotId, (prev) =>
      prev.filter(
        (shelf) =>
          shelf.shelf_id !== shelfId,
      ),
    )
  }

  function moveQueueItem(
    robotId,
    index,
    direction,
  ) {
    updateQueue(robotId, (prev) => {
      const nextIndex =
        index + direction

      if (
        nextIndex < 0 ||
        nextIndex >= prev.length
      ) {
        return prev
      }

      const nextQueue = [...prev]

      const currentItem =
        nextQueue[index]

      nextQueue[index] =
        nextQueue[nextIndex]

      nextQueue[nextIndex] =
        currentItem

      return nextQueue
    })
  }

  /** 가장 적게 맡은 로봇에게 하나씩 — 대수와 무관하게 같은 규칙이다. */
  function autoDistribute() {
    const unassignedShelves =
      shelves.filter(
        (shelf) =>
          isTeachingComplete(shelf) &&
          !getAssignedRobot(
            shelf.shelf_id,
          ),
      )

    const next = Object.fromEntries(
      robots.map((robotId) => [
        robotId,
        [...queueOf(robotId)],
      ]),
    )

    unassignedShelves.forEach(
      (shelf) => {
        // 동점이면 robots 순서가 앞선 쪽. 결과가 매번 같아야 사람이 예측한다.
        const target = robots.reduce(
          (best, robotId) =>
            next[robotId].length <
            next[best].length
              ? robotId
              : best,
          robots[0],
        )

        next[target].push(shelf)
      },
    )

    setQueues((prev) => ({
      ...prev,
      ...next,
    }))
  }

  async function startTasks() {
    const isEmpty = robots.every(
      (robotId) =>
        queueOf(robotId).length === 0,
    )

    if (isEmpty) {
      window.alert(
        '작업 Queue가 비어 있습니다.',
      )
      return
    }

    // 큐가 빈 로봇도 실어 보낸다 — 서버는 이 목록으로 대기 큐를 **교체**하므로,
    // 빠뜨리면 그 로봇의 기존 대기 작업이 그대로 남는다.
    const taskRequest = {
      robots: robots.map(
        (robotId) => ({
          robot_id: robotId,
          queue: queueOf(robotId).map(
            (shelf) => shelf.shelf_id,
          ),
        }),
      ),
    }

    try {
      await execute(() =>
        requestTaskStart(
          taskRequest,
        ),
      )

      window.alert(
        '작업 시작 요청을 Backend에 전달했습니다.',
      )
    } catch {
      return
    }
  }

  return (
    <section className="task-assignment-page">
      <header className="assignment-header">
        <div>
          <h1>작업 할당</h1>

          <p>
            선반을 {robots.join(' / ')} 작업 큐에 배정합니다.
          </p>
        </div>

        <div className="assignment-header-actions">
          <button
            type="button"
            className="auto-distribute-button"
            onClick={autoDistribute}
          >
            자동 분배
          </button>

          <button
            type="button"
            className="start-task-button"
            disabled={taskStartLoading}
            onClick={startTasks}
          >
            {taskStartLoading
              ? '작업 시작 요청 중...'
              : '작업 시작'}
          </button>
        </div>
      </header>

      {taskStartError && (
        <div className="assignment-api-error">
          {taskStartError}
        </div>
      )}

      <div className="assignment-layout">
        <div className="assignment-left">
          <div className="assignment-map-card">
            <div className="assignment-card-header">
              <h2>Top View</h2>
              <span>개념 배치 · 좌표 미사용</span>
            </div>

                        <div className="assignment-map-schematic">
              <div className="assignment-shelf-lanes">
                {shelves.map((shelf) => {
                  const passes =
                    shelf.scan_passes ?? []

                  const assignedRobot =
                    getAssignedRobot(
                      shelf.shelf_id,
                    )

                  return (
                    <article
                      className="assignment-shelf-lane"
                      key={shelf.shelf_id}
                    >
                      <div className="assignment-lane-title">
                        <strong>
                          {shelf.shelf_id}
                        </strong>

                        <span>
                          {passes.length} pass
                        </span>
                      </div>

                      <div className="assignment-pass-track">
                        {passes.length === 0 ? (
                          <span className="assignment-no-pass">
                            스캔 구간 없음
                          </span>
                        ) : (
                          passes.map((pass) => (
                            <span
                              className="assignment-pass-chip"
                              key={pass.pass_id}
                            >
                              Level {pass.level}

                              <b>
                                {directionArrows[
                                  pass.direction
                                ] ?? '→'}
                              </b>
                            </span>
                          ))
                        )}
                      </div>

                      <div className="assignment-lane-state">
                        <span
                          className={
                            isTeachingComplete(
                              shelf,
                            )
                              ? 'complete'
                              : 'incomplete'
                          }
                        >
                          {isTeachingComplete(
                            shelf,
                          )
                            ? '티칭 완료'
                            : '티칭 미완료'}
                        </span>

                        {assignedRobot && (
                          <strong>
                            {assignedRobot}
                          </strong>
                        )}
                      </div>
                    </article>
                  )
                })}
              </div>

              <aside className="assignment-station-overview">
                <div className="assignment-station-overview-header">
                  <strong>
                    등록 스테이션
                  </strong>

                  <span>
                    {stations.length}개
                  </span>
                </div>

                <div className="assignment-station-chip-list">
                  {stations.length === 0 ? (
                    <span className="assignment-no-station">
                      등록된 스테이션이 없습니다.
                    </span>
                  ) : (
                    stations.map((station) => (
                      <div
                        className="assignment-station-chip"
                        key={
                          station.station_id
                        }
                      >
                        <strong>
                          {station.station_id}
                        </strong>

                        <span>
                          {station.station_type ||
                            '유형 미설정'}
                        </span>
                      </div>
                    ))
                  )}
                </div>
              </aside>
            </div>
          </div>

          <div className="available-shelf-card">
            <div className="assignment-card-header">
              <h2>선반</h2>

              <span>
                등록 {shelves.length}개
              </span>
            </div>

            <div className="assignable-shelf-list">
              {shelves.map((shelf) => {
                const assignedRobot =
                  getAssignedRobot(
                    shelf.shelf_id,
                  )

                return (
                  <div
                    className="assignable-shelf-item"
                    key={shelf.shelf_id}
                  >
                    <div>
                      <strong>
                        {shelf.shelf_id}
                      </strong>

                      <span
                        className={
                          isTeachingComplete(
                            shelf,
                          )
                            ? 'assignment-teach-status complete'
                            : 'assignment-teach-status incomplete'
                        }
                      >
                        {isTeachingComplete(
                          shelf,
                        )
                          ? '티칭 완료'
                          : '티칭 미완료'}
                      </span>
                    </div>

                    {assignedRobot ? (
                      <span className="assigned-badge">
                        {assignedRobot} 배정됨
                      </span>
                    ) : (
                      <div className="assign-buttons">
                        {robots.map(
                          (robotId) => (
                            <button
                              key={robotId}
                              type="button"
                              onClick={() =>
                                assignShelf(
                                  shelf,
                                  robotId,
                                )
                              }
                            >
                              {robotId}
                            </button>
                          ),
                        )}
                      </div>
                    )}
                  </div>
                )
              })}
            </div>
          </div>
        </div>

        <div className="robot-queue-column">
          {robots.map((robotId) => (
            <RobotQueue
              key={robotId}
              robotId={robotId}
              queue={queueOf(robotId)}
              onRemove={removeShelf}
              onMove={moveQueueItem}
            />
          ))}
        </div>
      </div>
    </section>
  )
}

function RobotQueue({
  robotId,
  queue,
  onRemove,
  onMove,
}) {
  return (
    <article className="robot-queue-card">
      <div className="robot-queue-header">
        <div>
          <h2>{robotId}</h2>

          <span>
            {queue.length}개 작업
          </span>
        </div>

        <span className="queue-time">
          예상 소요시간 -
        </span>
      </div>

      {queue.length === 0 ? (
        <div className="empty-queue">
          배정된 선반이 없습니다.
        </div>
      ) : (
        <div className="queue-list">
          {queue.map(
            (shelf, index) => (
              <div
                className="queue-item"
                key={shelf.shelf_id}
              >
                <span className="queue-index">
                  {index + 1}
                </span>

                <strong>
                  {shelf.shelf_id}
                </strong>

                <div className="queue-actions">
                  <button
                    type="button"
                    disabled={
                      index === 0
                    }
                    onClick={() =>
                      onMove(
                        robotId,
                        index,
                        -1,
                      )
                    }
                  >
                    ↑
                  </button>

                  <button
                    type="button"
                    disabled={
                      index ===
                      queue.length - 1
                    }
                    onClick={() =>
                      onMove(
                        robotId,
                        index,
                        1,
                      )
                    }
                  >
                    ↓
                  </button>

                  <button
                    type="button"
                    onClick={() =>
                      onRemove(
                        robotId,
                        shelf.shelf_id,
                      )
                    }
                  >
                    제거
                  </button>
                </div>
              </div>
            ),
          )}
        </div>
      )}
    </article>
  )
}

export default TaskAssignmentPage

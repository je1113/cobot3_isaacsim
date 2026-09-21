import {
  requestTaskStart,
} from '../api/tasks'

import useApiRequest from '../hooks/useApiRequest'

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
  robot1Queue,
  setRobot1Queue,
  robot2Queue,
  setRobot2Queue,
}) {
  const {
    loading: taskStartLoading,
    error: taskStartError,
    execute,
  } = useApiRequest()
  function getAssignedRobot(shelfId) {
    if (
      robot1Queue.some(
        (shelf) =>
          shelf.shelf_id === shelfId,
      )
    ) {
      return 'AMR-01'
    }

    if (
      robot2Queue.some(
        (shelf) =>
          shelf.shelf_id === shelfId,
      )
    ) {
      return 'AMR-02'
    }

    return null
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

    if (robotId === 'AMR-01') {
      setRobot1Queue((prev) => [
        ...prev,
        shelf,
      ])
    }

    if (robotId === 'AMR-02') {
      setRobot2Queue((prev) => [
        ...prev,
        shelf,
      ])
    }
  }

  function removeShelf(
    robotId,
    shelfId,
  ) {
    if (robotId === 'AMR-01') {
      setRobot1Queue((prev) =>
        prev.filter(
          (shelf) =>
            shelf.shelf_id !== shelfId,
        ),
      )
    }

    if (robotId === 'AMR-02') {
      setRobot2Queue((prev) =>
        prev.filter(
          (shelf) =>
            shelf.shelf_id !== shelfId,
        ),
      )
    }
  }

  function moveQueueItem(
    robotId,
    index,
    direction,
  ) {
    const setQueue =
      robotId === 'AMR-01'
        ? setRobot1Queue
        : setRobot2Queue

    setQueue((prev) => {
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

  function autoDistribute() {
    const unassignedShelves =
      shelves.filter(
        (shelf) =>
          isTeachingComplete(shelf) &&
          !getAssignedRobot(
            shelf.shelf_id,
          ),
      )

    let nextRobot1Queue = [
      ...robot1Queue,
    ]

    let nextRobot2Queue = [
      ...robot2Queue,
    ]

    unassignedShelves.forEach(
      (shelf) => {
        if (
          nextRobot1Queue.length <=
          nextRobot2Queue.length
        ) {
          nextRobot1Queue = [
            ...nextRobot1Queue,
            shelf,
          ]
        } else {
          nextRobot2Queue = [
            ...nextRobot2Queue,
            shelf,
          ]
        }
      },
    )

    setRobot1Queue(
      nextRobot1Queue,
    )

    setRobot2Queue(
      nextRobot2Queue,
    )
  }

  async function startTasks() {
    if (
      robot1Queue.length === 0 &&
      robot2Queue.length === 0
    ) {
      window.alert(
        '작업 Queue가 비어 있습니다.',
      )
      return
    }

    const taskRequest = {
      robots: [
        {
          robot_id: 'AMR-01',
          queue: robot1Queue.map(
            (shelf) =>
              shelf.shelf_id,
          ),
        },
        {
          robot_id: 'AMR-02',
          queue: robot2Queue.map(
            (shelf) =>
              shelf.shelf_id,
          ),
        },
      ],
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
            선반을 AMR-01 / AMR-02 작업 큐에 배정합니다.
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
                                {pass.direction ===
                                'BACKWARD'
                                  ? '←'
                                  : '→'}
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
                        <button
                          type="button"
                          onClick={() =>
                            assignShelf(
                              shelf,
                              'AMR-01',
                            )
                          }
                        >
                          AMR-01
                        </button>

                        <button
                          type="button"
                          onClick={() =>
                            assignShelf(
                              shelf,
                              'AMR-02',
                            )
                          }
                        >
                          AMR-02
                        </button>
                      </div>
                    )}
                  </div>
                )
              })}
            </div>
          </div>
        </div>

        <div className="robot-queue-column">
          <RobotQueue
            robotId="AMR-01"
            queue={robot1Queue}
            onRemove={removeShelf}
            onMove={moveQueueItem}
          />

          <RobotQueue
            robotId="AMR-02"
            queue={robot2Queue}
            onRemove={removeShelf}
            onMove={moveQueueItem}
          />
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

import { useState } from 'react'

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

/**
 * 작업 할당 — 로봇마다 담당 선반을 **하나씩** 저장한다.
 *
 * ★ 2026-09-25 이전에는 이 화면이 "선반을 로봇 작업 큐에 넣고 작업 시작을
 *   누르는" 화면이었다. 그런데 로봇을 실제로 순찰 돌게 만드는 건 그 큐가
 *   아니라 shelves.yaml 의 assigned_robot 이다(task_manager._resolve_patrol_
 *   route) — mission_nodes.launch.py 를 띄우는 순간 이미 그 값을 보고
 *   움직이기 시작한다. "작업 시작" 버튼은 그 사실과 안 맞았고, assigned_
 *   robot 을 고칠 화면은 어디에도 없었다(ShelfSettingsPage 에도 없다).
 *   그래서 이 화면을 그 자리로 바꿨다 — shelves 리소스(App.jsx 의
 *   shelvesRes, ShelfSettingsPage 와 공유)에 직접 쓰고 저장한다.
 *
 * ★ 로봇 하나에 선반 하나뿐이다 — _resolve_patrol_route 가 assigned_robot
 *   이 여럿이면 "첫 번째만 쓰고 경고"로 처리해서, 여러 개를 넣어 봐야
 *   나머지는 조용히 무시된다. 그래서 화면도 다중 큐가 아니라 1:1 배정으로
 *   맞춘다.
 */
function TaskAssignmentPage({
  shelves,
  setShelves,
  resource,
  stations = [],
}) {
  // ★ 로봇 목록은 서버가 준다(GET /api/meta ← COBOT3_ROBOTS).
  const robots = useRobots()

  const {
    scan_direction_arrows:
      directionArrows,
  } = useMeta()

  const [saving, setSaving] =
    useState(false)

  const [saveError, setSaveError] =
    useState('')

  function getAssignedRobot(shelfId) {
    return (
      shelves.find(
        (shelf) =>
          shelf.shelf_id === shelfId,
      )?.assigned_robot || null
    )
  }

  function getAssignedShelfId(robotId) {
    return (
      shelves.find(
        (shelf) =>
          shelf.assigned_robot ===
          robotId,
      )?.shelf_id ?? null
    )
  }

  function assignShelf(shelf, robotId) {
    if (!isTeachingComplete(shelf)) {
      window.alert(
        `${shelf.shelf_id}은 티칭 미완료 상태라 배정할 수 없습니다.`,
      )
      return
    }

    setShelves((prev) =>
      prev.map((s) => {
        if (s.shelf_id === shelf.shelf_id) {
          return {
            ...s,
            assigned_robot: robotId,
          }
        }

        // 그 로봇이 맡고 있던 다른 선반은 배정을 뗀다 — 로봇 하나엔
        // 담당 선반이 하나뿐이다(위 헤더 주석).
        if (s.assigned_robot === robotId) {
          return {
            ...s,
            assigned_robot: null,
          }
        }

        return s
      }),
    )
  }

  function unassignShelf(shelfId) {
    setShelves((prev) =>
      prev.map((s) =>
        s.shelf_id === shelfId
          ? { ...s, assigned_robot: null }
          : s,
      ),
    )
  }

  /** 담당 선반이 없는 로봇에게, 배정 안 된 티칭완료 선반을 하나씩 짝지어준다. */
  function autoDistribute() {
    const idleRobots = robots.filter(
      (robotId) =>
        !getAssignedShelfId(robotId),
    )

    const unassignedShelves =
      shelves.filter(
        (shelf) =>
          isTeachingComplete(shelf) &&
          !shelf.assigned_robot,
      )

    const pairs = new Map(
      idleRobots.map((robotId, i) => [
        unassignedShelves[i]?.shelf_id,
        robotId,
      ]),
    )

    if (pairs.size === 0) {
      return
    }

    setShelves((prev) =>
      prev.map((s) =>
        pairs.has(s.shelf_id)
          ? {
              ...s,
              assigned_robot: pairs.get(
                s.shelf_id,
              ),
            }
          : s,
      ),
    )
  }

  async function saveAssignments() {
    setSaving(true)
    setSaveError('')

    try {
      await resource.save()

      window.alert(
        '담당 선반을 저장했습니다.',
      )
    } catch (err) {
      // 백엔드 메시지를 그대로 보여준다(ShelfSettingsPage 와 같은 이유).
      setSaveError(
        err instanceof Error
          ? err.message
          : '저장 실패',
      )
    } finally {
      setSaving(false)
    }
  }

  return (
    <section className="task-assignment-page">
      <header className="assignment-header">
        <div>
          <h1>작업 할당</h1>

          <p>
            로봇마다 담당 선반을 하나씩 지정합니다. 지정한 선반을 순찰하며
            매거진을 처리합니다 — 저장 즉시가 아니라 그 로봇이 다음 순찰
            사이클에 들어갈 때 반영됩니다.
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
            disabled={
              saving || !resource.dirty
            }
            onClick={saveAssignments}
          >
            {saving ? '저장 중...' : '저장'}
          </button>
        </div>
      </header>

      {saveError && (
        <div className="assignment-api-error">
          {saveError}
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
                      <div className="assign-buttons">
                        <span className="assigned-badge">
                          {assignedRobot} 배정됨
                        </span>

                        <button
                          type="button"
                          onClick={() =>
                            unassignShelf(
                              shelf.shelf_id,
                            )
                          }
                        >
                          해제
                        </button>
                      </div>
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
          {robots.map((robotId) => {
            const shelfId =
              getAssignedShelfId(robotId)

            return (
              <RobotAssignment
                key={robotId}
                robotId={robotId}
                shelfId={shelfId}
                onUnassign={unassignShelf}
              />
            )
          })}
        </div>
      </div>
    </section>
  )
}

function RobotAssignment({
  robotId,
  shelfId,
  onUnassign,
}) {
  return (
    <article className="robot-queue-card">
      <div className="robot-queue-header">
        <div>
          <h2>{robotId}</h2>

          <span>
            {shelfId
              ? '담당 선반 1개'
              : '담당 선반 없음'}
          </span>
        </div>
      </div>

      {shelfId ? (
        <div className="queue-list">
          <div className="queue-item">
            <span className="queue-index">
              1
            </span>

            <strong>{shelfId}</strong>

            <div className="queue-actions">
              <button
                type="button"
                onClick={() =>
                  onUnassign(shelfId)
                }
              >
                해제
              </button>
            </div>
          </div>
        </div>
      ) : (
        <div className="empty-queue">
          담당 선반이 없습니다.
        </div>
      )}
    </article>
  )
}

export default TaskAssignmentPage

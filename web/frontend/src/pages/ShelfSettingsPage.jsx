import { useState } from 'react'
import { NavLink } from 'react-router-dom'

function createScanPass(level) {
  return {
    pass_id: level,
    level,
    direction:
      level % 2 === 1
        ? 'FORWARD'
        : 'BACKWARD',
    arm_teach_pose: ['', '', '', '', '', ''],
  }
}

function createShelf(shelfId) {
  return {
    shelf_id: shelfId,
    waypoint_start: {
      x: '',
      y: '',
      theta: '',
    },
    waypoint_end: {
      x: '',
      y: '',
      theta: '',
    },
    standoff_distance: '',
    scan_passes: [],
    first_taught_at: null,
  }
}

function isPoseComplete(pass) {
  return pass.arm_teach_pose.every(
    (joint) => joint.trim() !== '',
  )
}

function isTeachingComplete(shelf) {
  return (
    shelf.scan_passes.length > 0 &&
    shelf.scan_passes.every(
      (pass) => isPoseComplete(pass),
    )
  )
}

function formatTeachingTime(value) {
  if (!value) {
    return '-'
  }

  const date = new Date(value)

  if (Number.isNaN(date.getTime())) {
    return '-'
  }

  const pad = (number) =>
    String(number).padStart(2, '0')

  return [
    `${date.getFullYear()}-${pad(
      date.getMonth() + 1,
    )}-${pad(date.getDate())}`,
    `${pad(date.getHours())}:${pad(
      date.getMinutes(),
    )}`,
  ].join(' ')
}

function ShelfSettingsPage({
  shelves,
  setShelves,
}) {
  const [
    selectedShelfId,
    setSelectedShelfId,
  ] = useState('SHELF-A')

  const selectedShelf =
    shelves.find(
      (shelf) =>
        shelf.shelf_id ===
        selectedShelfId,
    )

  function updateSelectedShelf(
    updater,
  ) {
    setShelves((prev) =>
      prev.map((shelf) =>
        shelf.shelf_id ===
        selectedShelfId
          ? updater(shelf)
          : shelf,
      ),
    )
  }

  function updateShelfField(
    field,
    value,
  ) {
    updateSelectedShelf(
      (shelf) => ({
        ...shelf,
        [field]: value,
      }),
    )
  }

  function updateWaypoint(
    group,
    field,
    value,
  ) {
    updateSelectedShelf(
      (shelf) => ({
        ...shelf,
        [group]: {
          ...shelf[group],
          [field]: value,
        },
      }),
    )
  }

  function updateJoint(
    passId,
    jointIndex,
    value,
  ) {
    updateSelectedShelf(
      (shelf) => {
        const nextPasses =
          shelf.scan_passes.map(
            (pass) => {
              if (
                pass.pass_id !==
                passId
              ) {
                return pass
              }

              const nextPose = [
                ...pass.arm_teach_pose,
              ]

              nextPose[jointIndex] =
                value

              return {
                ...pass,
                arm_teach_pose:
                  nextPose,
              }
            },
          )

        const nextShelf = {
          ...shelf,
          scan_passes: nextPasses,
        }

        if (
          !nextShelf.first_taught_at &&
          isTeachingComplete(
            nextShelf,
          )
        ) {
          return {
            ...nextShelf,
            first_taught_at:
              new Date().toISOString(),
          }
        }

        return nextShelf
      },
    )
  }

  function addLevel() {
    updateSelectedShelf(
      (shelf) => {
        const levels =
          shelf.scan_passes.map(
            (pass) => pass.level,
          )

        const nextLevel =
          levels.length === 0
            ? 1
            : Math.max(
                ...levels,
              ) + 1

        return {
          ...shelf,
          scan_passes: [
            ...shelf.scan_passes,
            createScanPass(
              nextLevel,
            ),
          ],
        }
      },
    )
  }

  function removeLevel(passId) {
    updateSelectedShelf(
      (shelf) => ({
        ...shelf,
        scan_passes:
          shelf.scan_passes.filter(
            (pass) =>
              pass.pass_id !==
              passId,
          ),
      }),
    )
  }

  function addShelf() {
    const shelfId =
      window.prompt(
        '새 선반 ID를 입력하세요.',
      )

    if (!shelfId) {
      return
    }

    const trimmedShelfId =
      shelfId.trim()

    if (trimmedShelfId === '') {
      return
    }

    const alreadyExists =
      shelves.some(
        (shelf) =>
          shelf.shelf_id ===
          trimmedShelfId,
      )

    if (alreadyExists) {
      window.alert(
        '이미 등록된 선반 ID입니다.',
      )
      return
    }

    setShelves((prev) => [
      ...prev,
      createShelf(
        trimmedShelfId,
      ),
    ])

    setSelectedShelfId(
      trimmedShelfId,
    )
  }

  function deleteShelf() {
    if (!selectedShelf) {
      return
    }

    const confirmed =
      window.confirm(
        `${selectedShelf.shelf_id} 선반을 삭제하시겠습니까?`,
      )

    if (!confirmed) {
      return
    }

    const remainingShelves =
      shelves.filter(
        (shelf) =>
          shelf.shelf_id !==
          selectedShelf.shelf_id,
      )

    setShelves(
      remainingShelves,
    )

    setSelectedShelfId(
      remainingShelves[0]
        ?.shelf_id ?? '',
    )
  }

  function saveCurrentPose(level) {
    window.alert(
      `Level ${level} 현재 자세 저장은 ROS2 연동 단계에서 연결합니다.`,
    )
  }

  function testDrive() {
    window.alert(
      `${selectedShelfId} 시험 주행은 ROS2 연동 단계에서 연결합니다.`,
    )
  }

  function saveShelf() {
    window.alert(
      `${selectedShelfId} 저장 기능은 Backend 연동 단계에서 연결합니다.`,
    )
  }

  if (!selectedShelf) {
    return null
  }

  const teachingComplete =
    isTeachingComplete(
      selectedShelf,
    )

  return (
    <section className="shelf-settings-page">
      <header className="settings-page-header shelf-page-header">
        <div>
          <h1>설정</h1>
          <p>
            선반의 스캔 구간과 층별 팔 자세를 설정합니다.
          </p>
        </div>

        <div className="shelf-header-actions">
          <button
            type="button"
            className="delete-shelf-button"
            onClick={deleteShelf}
          >
            선반 삭제
          </button>

          <button
            type="button"
            className="add-shelf-button"
            onClick={addShelf}
          >
            + 선반 추가
          </button>
        </div>
      </header>

      <nav className="settings-tabs">
        <NavLink
          to="/settings/shelf"
          className={({ isActive }) =>
            isActive
              ? 'settings-tab active'
              : 'settings-tab'
          }
        >
          선반
        </NavLink>

        <NavLink
          to="/settings/station"
          className={({ isActive }) =>
            isActive
              ? 'settings-tab active'
              : 'settings-tab'
          }
        >
          스테이션
        </NavLink>

        <NavLink
          to="/settings/process"
          className={({ isActive }) =>
            isActive
              ? 'settings-tab active'
              : 'settings-tab'
          }
        >
          공정 흐름
        </NavLink>
      </nav>

      <div className="shelf-workspace">
        <aside className="shelf-list-panel">
          <p className="shelf-list-count">
            등록된 선반 {shelves.length}개
          </p>

          <div className="shelf-list">
            {shelves.map(
              (shelf) => {
                const complete =
                  isTeachingComplete(
                    shelf,
                  )

                const passCount =
                  shelf.scan_passes
                    .length

                return (
                  <button
                    key={
                      shelf.shelf_id
                    }
                    type="button"
                    className={
                      shelf.shelf_id ===
                      selectedShelfId
                        ? 'shelf-list-item active'
                        : 'shelf-list-item'
                    }
                    onClick={() =>
                      setSelectedShelfId(
                        shelf.shelf_id,
                      )
                    }
                  >
                    <strong>
                      {shelf.shelf_id}
                    </strong>

                    <span>
                      {complete
                        ? '티칭 완료'
                        : '티칭 미완료'}

                      {passCount > 0
                        ? ` · ${passCount} pass`
                        : ''}
                    </span>
                  </button>
                )
              },
            )}
          </div>
        </aside>

        <div className="shelf-detail-panel">
          <div className="selected-shelf-header">
            <h2>
              {selectedShelf.shelf_id}
            </h2>

            <div className="shelf-header-meta">
              <span className="shelf-first-taught">
                최초 티칭{' '}
                {formatTeachingTime(
                  selectedShelf
                    .first_taught_at,
                )}
              </span>

              <span
                className={
                  teachingComplete
                    ? 'teaching-status complete'
                    : 'teaching-status incomplete'
                }
              >
                {teachingComplete
                  ? '티칭 완료 · 작업 할당 가능'
                  : '티칭 미완료 · 작업 할당 불가'}
              </span>
            </div>
          </div>

          <div className="shelf-coordinate-row">
            <div className="shelf-coordinate-section">
              <h3>스캔 시작 좌표</h3>

              <div className="coordinate-grid">
                {[
                  'x',
                  'y',
                  'theta',
                ].map((field) => (
                  <label key={field}>
                    <span>
                      {field}
                    </span>

                    <input
                      type="number"
                      value={
                        selectedShelf
                          .waypoint_start[
                            field
                          ]
                      }
                      onChange={(
                        event,
                      ) =>
                        updateWaypoint(
                          'waypoint_start',
                          field,
                          event.target
                            .value,
                        )
                      }
                    />
                  </label>
                ))}
              </div>
            </div>

            <div className="shelf-coordinate-section">
              <h3>스캔 종료 좌표</h3>

              <div className="coordinate-grid">
                {[
                  'x',
                  'y',
                  'theta',
                ].map((field) => (
                  <label key={field}>
                    <span>
                      {field}
                    </span>

                    <input
                      type="number"
                      value={
                        selectedShelf
                          .waypoint_end[
                            field
                          ]
                      }
                      onChange={(
                        event,
                      ) =>
                        updateWaypoint(
                          'waypoint_end',
                          field,
                          event.target
                            .value,
                        )
                      }
                    />
                  </label>
                ))}
              </div>
            </div>

            <div className="shelf-standoff-section">
              <h3>
                이격 거리 (standoff)
              </h3>

              <label>
                <span>distance</span>

                <input
                  type="number"
                  value={
                    selectedShelf
                      .standoff_distance
                  }
                  onChange={(
                    event,
                  ) =>
                    updateShelfField(
                      'standoff_distance',
                      event.target.value,
                    )
                  }
                />
              </label>
            </div>
          </div>

          <div className="scan-pass-section-header">
            <div>
              <h3>
                스캔 패스 · 층별 티칭 자세
              </h3>

              <p>
                주행 방향은 층 순서에 따라 자동 교대됩니다.
              </p>
            </div>
          </div>

          <div className="scan-pass-list">
            {selectedShelf
              .scan_passes
              .map((pass) => {
                const poseComplete =
                  isPoseComplete(pass)

                return (
                  <article
                    className="scan-pass-card"
                    key={
                      pass.pass_id
                    }
                  >
                    <div className="scan-pass-level">
                      <span className="scan-pass-number">
                        {pass.level}
                      </span>

                      <div>
                        <strong>
                          {pass.level}층
                        </strong>

                        <span className="scan-pass-direction">
                          {pass.direction}
                        </span>
                      </div>
                    </div>

                    <div className="scan-pass-joints">
                      <span className="scan-pass-joint-title">
                        팔 티칭 자세 (J1-J6)
                      </span>

                      <div className="joint-grid">
                        {pass.arm_teach_pose.map(
                          (
                            joint,
                            index,
                          ) => (
                            <label
                              key={
                                index
                              }
                            >
                              <span>
                                J
                                {index +
                                  1}
                              </span>

                              <input
                                type="number"
                                value={
                                  joint
                                }
                                onChange={(
                                  event,
                                ) =>
                                  updateJoint(
                                    pass.pass_id,
                                    index,
                                    event
                                      .target
                                      .value,
                                  )
                                }
                              />
                            </label>
                          ),
                        )}
                      </div>
                    </div>

                    <div className="scan-pass-actions">
                      <span
                        className={
                          poseComplete
                            ? 'pose-status complete'
                            : 'pose-status incomplete'
                        }
                      >
                        {poseComplete
                          ? '티칭 완료'
                          : '티칭 필요'}
                      </span>

                      <button
                        type="button"
                        className="save-pose-button"
                        onClick={() =>
                          saveCurrentPose(
                            pass.level,
                          )
                        }
                      >
                        현재 자세로 저장
                      </button>

                      <button
                        type="button"
                        className="delete-button"
                        onClick={() =>
                          removeLevel(
                            pass.pass_id,
                          )
                        }
                      >
                        삭제
                      </button>
                    </div>
                  </article>
                )
              })}
          </div>

          <button
            type="button"
            className="shelf-add-level-button"
            onClick={addLevel}
          >
            + 층 추가
          </button>

          <div className="shelf-footer-actions">
            <button
              type="button"
              className="shelf-test-button"
              onClick={testDrive}
            >
              시험 주행
            </button>

            <button
              type="button"
              className="save-shelf-button"
              onClick={saveShelf}
            >
              저장
            </button>
          </div>
        </div>
      </div>
    </section>
  )
}

export default ShelfSettingsPage

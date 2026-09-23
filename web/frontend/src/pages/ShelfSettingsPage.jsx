import { useState } from 'react'
import { NavLink } from 'react-router-dom'

import { captureShelfPose } from '../api/config'
import {
  useEnum,
  useMeta,
} from '../contexts/metaHooks'

/**
 * 새 층 하나.
 *
 * ★ `directions` 와 `joints` 는 서버가 준다(GET /api/meta).
 *   방향 목록은 **홀수 층부터** 순서대로다 — 1층이 FORWARD 다.
 *   서버의 shapes.DIRECTIONS 와 같은 순서여야 한다.
 */
function createScanPass(
  level,
  directions,
  joints,
) {
  return {
    pass_id: level,
    level,
    direction:
      directions[
        (level - 1) % directions.length
      ],
    arm_teach_pose: Array.from(
      { length: joints },
      () => '',
    ),
  }
}

// 티칭 자세는 1층 하나로 고정한다. 층 추가·삭제는 화면에서 뺐다.
const FIXED_LEVEL = 1

/**
 * 선반의 스캔 패스를 1층 하나로 맞춘다.
 *
 * ★ 1층이 없으면 새로 만든다. '현재 자세로 저장'(POST .../capture)은 그 층이
 *   파일에 있어야 동작하므로(없으면 404), 1층은 늘 있어야 한다.
 * ★ 예전에 저장된 2층 이상은 다음 편집 때 떨어져 나가고, 저장하면 파일에서도 빠진다.
 */
function withFixedLevel(
  shelf,
  directions,
  joints,
) {
  const existing =
    shelf.scan_passes.find(
      (pass) =>
        Number(pass.level) ===
        FIXED_LEVEL,
    )

  if (
    existing &&
    shelf.scan_passes.length === 1
  ) {
    return shelf
  }

  return {
    ...shelf,
    scan_passes: [
      existing ??
        createScanPass(
          FIXED_LEVEL,
          directions,
          joints,
        ),
    ],
  }
}

function createShelf(
  shelfId,
  directions,
  joints,
) {
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
    scan_passes: [
      createScanPass(
        FIXED_LEVEL,
        directions,
        joints,
      ),
    ],
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
  resource,
}) {
  const [
    selectedShelfId,
    setSelectedShelfId,
  ] = useState('')

  const [busy, setBusy] =
    useState(false)

  const directions = useEnum(
    'scan_directions',
  )

  const { joints } = useMeta()

  // ★ 선반 목록이 서버에서 오므로 특정 id('SHELF-A')를 기본값으로 박아 둘 수
  //   없다. 그 선반이 없으면 페이지가 통째로 빈 화면이 된다.
  //   고른 것이 없거나 사라졌으면 첫 번째로 떨어진다.
  const selectedShelf =
    shelves.find(
      (shelf) =>
        shelf.shelf_id ===
        selectedShelfId,
    ) ?? shelves[0]

  // 실제로 화면에 떠 있는 선반의 id. selectedShelfId 는 사용자가 '누른' 값이고,
  // 이쪽은 '지금 보고 있는' 값이다 — 처음 열었을 때나 방금 지운 뒤에는 둘이 다르다.
  // 편집·강조는 전부 이 값을 기준으로 해야 한다.
  const activeShelfId =
    selectedShelf?.shelf_id ?? ''

  function updateSelectedShelf(
    updater,
  ) {
    setShelves((prev) =>
      prev.map((shelf) =>
        shelf.shelf_id ===
        activeShelfId
          ? updater(
              withFixedLevel(
                shelf,
                directions,
                joints,
              ),
            )
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
        directions,
        joints,
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

  /**
   * 로봇의 지금 관절값을 그 층에 채운다.
   *
   * ★ 이건 서버가 파일에 직접 쓴다 — 관절값의 주인은 로봇이지 화면이 아니다.
   *   그래서 저장 안 한 다른 편집이 있으면 먼저 막는다. 서버가 파일을 쓰면
   *   revision 이 바뀌고, 그 편집은 다음 저장에서 412 로 거절되기 때문이다.
   */
  async function saveCurrentPose(level) {
    if (resource?.dirty) {
      window.alert(
        '저장하지 않은 변경이 있습니다. 먼저 저장한 뒤 현재 자세를 가져오세요.',
      )

      return
    }

    setBusy(true)

    try {
      await captureShelfPose(
        selectedShelf.shelf_id,
        level,
      )

      await resource?.reload()
    } catch (err) {
      window.alert(err.message)
    } finally {
      setBusy(false)
    }
  }

  function testDrive() {
    window.alert(
      `${selectedShelf.shelf_id} 시험 주행은 ROS2 연동 단계에서 연결합니다.`,
    )
  }

  async function saveShelf() {
    setBusy(true)

    try {
      await resource.save()

      window.alert('선반 설정을 저장했습니다.')
    } catch (err) {
      // 백엔드 메시지를 그대로 보여준다. 서버가 아는 맥락(어느 선반, 어느
      // revision)이 화면이 지어낼 수 있는 어떤 문장보다 구체적이다.
      window.alert(err.message)
    } finally {
      setBusy(false)
    }
  }

  if (!selectedShelf) {
    return null
  }

  // 화면은 1층 하나만 보여준다 — 파일에 다른 층이 남아 있어도 마찬가지다.
  const viewShelf =
    withFixedLevel(
      selectedShelf,
      directions,
      joints,
    )

  const teachingComplete =
    isTeachingComplete(
      viewShelf,
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
                      activeShelfId
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
                스캔 패스 · 1층 티칭 자세
              </h3>

              <p>
                팔 티칭 자세는 1층 하나로 고정됩니다.
              </p>
            </div>
          </div>

          <div className="scan-pass-list">
            {viewShelf
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
                        disabled={busy}
                        onClick={() =>
                          saveCurrentPose(
                            pass.level,
                          )
                        }
                      >
                        현재 자세로 저장
                      </button>
                    </div>
                  </article>
                )
              })}
          </div>

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
              disabled={busy}
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

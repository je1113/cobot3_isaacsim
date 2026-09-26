import {
  useEffect,
  useState,
} from 'react'
import {
  NavLink,
  useNavigate,
} from 'react-router-dom'

import { fetchRouting } from '../api/config'
import {
  useEnum,
  useMeta,
} from '../contexts/metaHooks'

/** 새 스테이션. */
function createStation(stationId) {
  return {
    station_id: stationId,
    station_type: '',
    place_pose: {
      x: '',
      y: '',
      theta: '',
    },
    process_time: '',
    output_type: '',
    completion_signal: '',
    capacity: '',
    updated_at: null,
  }
}

function formatDateTime(value) {
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

/**
 * 완료 신호가 뜻하는 바. 설명은 서버가 준다(GET /api/meta).
 *
 * ★ 어휘와 설명을 한곳에 두기 위해서다. 값만 서버가 주고 설명을 화면이 들고
 *   있으면, 신호를 하나 더할 때 두 곳을 고쳐야 하고 한쪽을 잊으면 빈 설명이 뜬다.
 */
function getCompletionDescription(
  signal,
  help,
) {
  return (
    help[signal] ??
    '완료 신호를 선택하면 공정 완료 판단 방식이 여기에 표시됩니다.'
  )
}

function StationSettingsPage({
  stations,
  setStations,
  resource,
}) {
  const navigate =
    useNavigate()

  // ★ 목록이 서버에서 오므로 특정 id('PKG-01')를 기본값으로 박아 둘 수 없다.
  //   그 스테이션이 없으면 페이지가 통째로 빈 화면이 된다.
  const [
    selectedStationId,
    setSelectedStationId,
  ] = useState('')

  const [busy, setBusy] =
    useState(false)

  const {
    completion_signal_help:
      completionHelp,
  } = useMeta()

  const stationTypes = useEnum(
    'station_types',
  )

  const completionSignals = useEnum(
    'completion_signals',
  )

  // 공정 흐름에서 "이 스테이션 다음" 을 찾는다.
  // 읽기만 하므로 useConfigResource(저장·revision·dirty) 는 과하다.
  const [routingRules, setRoutingRules] =
    useState([])

  useEffect(() => {
    let alive = true

    fetchRouting()
      .then((res) => {
        if (alive) {
          setRoutingRules(
            res.rules ?? [],
          )
        }
      })
      .catch(() => {
        // 못 읽으면 '미설정' 으로 보인다. 이 패널 하나 때문에
        // 스테이션 편집 전체를 막을 이유는 없다.
      })

    return () => {
      alive = false
    }
  }, [])

  // 고른 것이 없거나 방금 지웠으면 첫 번째로 떨어진다.
  const selectedStation =
    stations.find(
      (station) =>
        station.station_id ===
        selectedStationId,
    ) ?? stations[0]

  // 실제로 화면에 떠 있는 스테이션의 id. 편집·강조는 이 값을 기준으로 한다.
  const activeStationId =
    selectedStation?.station_id ?? ''

  // 이 스테이션에서 나가는 규칙의 도착지. 선형 체인이라 최대 하나다.
  const nextStationId =
    routingRules.find(
      (rule) =>
        rule.from === activeStationId,
    )?.to || null

  function updateSelectedStation(
    updater,
  ) {
    setStations((prev) =>
      prev.map((station) =>
        station.station_id ===
        activeStationId
          ? updater(station)
          : station,
      ),
    )
  }

  function updateStationField(
    field,
    value,
  ) {
    updateSelectedStation(
      (station) => ({
        ...station,
        [field]: value,
        updated_at:
          new Date().toISOString(),
      }),
    )
  }

  function updatePlacePose(
    field,
    value,
  ) {
    updateSelectedStation(
      (station) => ({
        ...station,
        place_pose: {
          ...station.place_pose,
          [field]: value,
        },
        updated_at:
          new Date().toISOString(),
      }),
    )
  }

  function addStation() {
    const stationId =
      window.prompt(
        '새 스테이션 ID를 입력하세요.',
      )

    if (!stationId) {
      return
    }

    const trimmedStationId =
      stationId.trim()

    if (trimmedStationId === '') {
      return
    }

    const alreadyExists =
      stations.some(
        (station) =>
          station.station_id ===
          trimmedStationId,
      )

    if (alreadyExists) {
      window.alert(
        '이미 등록된 스테이션 ID입니다.',
      )
      return
    }

    setStations((prev) => [
      ...prev,
      createStation(
        trimmedStationId,
      ),
    ])

    setSelectedStationId(
      trimmedStationId,
    )
  }

  function deleteStation() {
    if (!selectedStation) {
      return
    }

    const confirmed =
      window.confirm(
        `${selectedStation.station_id} 스테이션을 삭제하시겠습니까?`,
      )

    if (!confirmed) {
      return
    }

    const remainingStations =
      stations.filter(
        (station) =>
          station.station_id !==
          selectedStation.station_id,
      )

    setStations(
      remainingStations,
    )

    setSelectedStationId(
      remainingStations[0]
        ?.station_id ?? '',
    )
  }

  function redefinePlacePosition() {
    window.alert(
      `${activeStationId} Place 위치 재지정은 ROS2 위치 연동 단계에서 연결합니다.`,
    )
  }

  function testPlace() {
    window.alert(
      `${activeStationId} 시험 배치는 ROS2 연동 단계에서 연결합니다.`,
    )
  }

  async function saveStation() {
    setBusy(true)

    try {
      await resource.save()

      window.alert('스테이션 설정을 저장했습니다.')
    } catch (err) {
      // 백엔드 메시지를 그대로 — 어떤 값이 왜 거절됐는지가 거기 적혀 있다
      // (예: 완료 신호는 'TIMER + VISION' 처럼 공백까지 맞아야 한다).
      window.alert(err.message)
    } finally {
      setBusy(false)
    }
  }

  function goToProcessFlow() {
    navigate(
      '/settings/process',
    )
  }

  if (!selectedStation) {
    return null
  }

  return (
    <section className="station-settings-page">
      <header className="settings-page-header station-page-header">
        <div>
          <h1>설정</h1>
          <p>
            스테이션 위치와 처리 정보를 설정합니다.
          </p>
        </div>

        <div className="station-page-actions">
          <button
            type="button"
            className="station-delete-button"
            onClick={deleteStation}
          >
            스테이션 삭제
          </button>

          <button
            type="button"
            className="station-add-button"
            onClick={addStation}
          >
            스테이션 추가
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

      <div className="station-workspace">
        <aside className="station-list-panel">
          <p className="station-list-count">
            등록된 스테이션 {stations.length}개
          </p>

          <div className="station-list">
            {stations.map(
              (station) => (
                <button
                  key={station.station_id}
                  type="button"
                  className={
                    station.station_id ===
                    activeStationId
                      ? 'station-list-item active'
                      : 'station-list-item'
                  }
                  onClick={() =>
                    setSelectedStationId(
                      station.station_id,
                    )
                  }
                >
                  <strong>
                    {station.station_id}
                  </strong>

                  <span>
                    {station.station_type ||
                      '유형 미설정'}
                  </span>
                </button>
              ),
            )}
          </div>
        </aside>

        <div className="station-detail-panel">
          <div className="selected-station-header">
            <div className="selected-station-title">
              <h2>
                {selectedStation.station_id}
              </h2>

              <span className="station-type-badge">
                {selectedStation.station_type ||
                  '유형 미설정'}
              </span>
            </div>

            <span className="station-modified-time">
              최종 수정{' '}
              {formatDateTime(
                selectedStation.updated_at,
              )}
            </span>
          </div>

          <section className="station-section station-basic-card">
            <h3>
              스테이션 기본 정보
            </h3>

            <div className="station-overview-grid">
              <label>
                <span>유형</span>

                <select
                  value={
                    selectedStation.station_type
                  }
                  onChange={(event) =>
                    updateStationField(
                      'station_type',
                      event.target.value,
                    )
                  }
                >
                  {/* 선택지는 서버가 준다(GET /api/meta ← shapes.STATION_TYPES).
                      화면에만 적어 두면 서버가 모르는 값을 고를 수 있게 되고,
                      저장할 때 422 를 받는다. */}
                  {stationTypes.map(
                    (type) => (
                      <option
                        key={type}
                        value={type}
                      >
                        {type || '선택'}
                      </option>
                    ),
                  )}
                </select>
              </label>

              <label>
                <span>
                  동시 처리 수
                </span>

                <input
                  type="number"
                  value={
                    selectedStation.capacity
                  }
                  onChange={(event) =>
                    updateStationField(
                      'capacity',
                      event.target.value,
                    )
                  }
                />
              </label>
            </div>
          </section>

          <section className="station-section station-place-card">
            <div className="station-section-heading">
              <h3>
                Place 위치
              </h3>

              <button
                type="button"
                className="station-secondary-button"
                onClick={
                  redefinePlacePosition
                }
              >
                Place 위치 재지정
              </button>
            </div>

            <div className="station-pose-grid">
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
                      selectedStation
                        .place_pose[field]
                    }
                    onChange={(event) =>
                      updatePlacePose(
                        field,
                        event.target.value,
                      )
                    }
                  />
                </label>
              ))}
            </div>
          </section>

          <section className="station-section station-output-card">
            <h3>
              산출물 처리
            </h3>

            <div className="station-output-grid">
              <label>
                <span>산출물</span>

                <input
                  type="text"
                  value={
                    selectedStation.output_type
                  }
                  onChange={(event) =>
                    updateStationField(
                      'output_type',
                      event.target.value,
                    )
                  }
                />
              </label>

              <label>
                <span>
                  처리 시간 (초)
                </span>

                <input
                  type="number"
                  value={
                    selectedStation.process_time
                  }
                  onChange={(event) =>
                    updateStationField(
                      'process_time',
                      event.target.value,
                    )
                  }
                />
              </label>

              <label>
                <span>
                  완료 신호
                </span>

                <select
                  value={
                    selectedStation
                      .completion_signal
                  }
                  onChange={(event) =>
                    updateStationField(
                      'completion_signal',
                      event.target.value,
                    )
                  }
                >
                  {/* 'TIMER + VISION' 은 공백까지 서버 값과 같아야 한다.
                      손으로 적으면 언젠가 'TIMER+VISION' 이 된다. */}
                  {completionSignals.map(
                    (signal) => (
                      <option
                        key={signal}
                        value={signal}
                      >
                        {signal || '선택'}
                      </option>
                    ),
                  )}
                </select>
              </label>
            </div>

            <div className="completion-signal-help">
              <strong>
                {selectedStation
                  .completion_signal ||
                  '완료 신호'}
              </strong>

              <span>
                {getCompletionDescription(
                  selectedStation
                    .completion_signal,
                  completionHelp,
                )}
              </span>
            </div>
          </section>

          <section className="station-next-process">
            <div>
              <span>
                다음 공정
              </span>

              {/* 라우팅에서 유도한다. 예전에는 'TEST-01' 이 적혀 있어서,
                  어느 스테이션을 보고 있든 항상 같은 이름이 나왔다. */}
              <strong>
                {nextStationId ?? '미설정'}
              </strong>
            </div>

            <button
              type="button"
              onClick={
                goToProcessFlow
              }
            >
              공정 흐름에서 편집 →
            </button>
          </section>

          <div className="station-footer-actions">
            <button
              type="button"
              className="station-test-button"
              onClick={testPlace}
            >
              시험 배치
            </button>

            <button
              type="button"
              className="station-save-button"
              disabled={busy}
              onClick={saveStation}
            >
              저장
            </button>
          </div>
        </div>
      </div>
    </section>
  )
}

export default StationSettingsPage

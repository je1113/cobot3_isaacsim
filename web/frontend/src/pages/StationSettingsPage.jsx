import { useState } from 'react'
import {
  NavLink,
  useNavigate,
} from 'react-router-dom'

function createStation(stationId) {
  return {
    station_id: stationId,
    station_type: '',
    place_pose: {
      x: '',
      y: '',
      theta: '',
    },
    place_arm_pose: ['', '', '', '', '', ''],
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

function getCompletionDescription(signal) {
  if (signal === 'TIMER') {
    return '설정한 처리 시간이 지나면 공정 완료로 판단합니다.'
  }

  if (signal === 'VISION') {
    return '비전 인식 결과를 기준으로 공정 완료를 판단합니다.'
  }

  if (signal === 'TIMER + VISION') {
    return '처리 시간과 비전 확인 조건을 함께 사용해 공정 완료를 판단합니다.'
  }

  if (signal === 'EXTERNAL') {
    return '외부 완료 신호를 수신하면 공정 완료로 판단합니다.'
  }

  return '완료 신호를 선택하면 공정 완료 판단 방식이 여기에 표시됩니다.'
}

function StationSettingsPage({
  stations,
  setStations,
}) {
  const navigate =
    useNavigate()

  const [
    selectedStationId,
    setSelectedStationId,
  ] = useState('PKG-01')

  const selectedStation =
    stations.find(
      (station) =>
        station.station_id ===
        selectedStationId,
    )

  function updateSelectedStation(
    updater,
  ) {
    setStations((prev) =>
      prev.map((station) =>
        station.station_id ===
        selectedStationId
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

  function updateArmJoint(
    index,
    value,
  ) {
    updateSelectedStation(
      (station) => {
        const nextPose = [
          ...station.place_arm_pose,
        ]

        nextPose[index] = value

        return {
          ...station,
          place_arm_pose:
            nextPose,
          updated_at:
            new Date().toISOString(),
        }
      },
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
      `${selectedStationId} Place 위치 재지정은 ROS2 위치 연동 단계에서 연결합니다.`,
    )
  }

  function saveCurrentArmPose() {
    window.alert(
      `${selectedStationId} 현재 팔 자세 지정은 ROS2 연동 단계에서 연결합니다.`,
    )
  }

  function testPlace() {
    window.alert(
      `${selectedStationId} 시험 배치는 ROS2 연동 단계에서 연결합니다.`,
    )
  }

  function saveStation() {
    window.alert(
      `${selectedStationId} 저장 기능은 Backend 연동 단계에서 연결합니다.`,
    )
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
                    selectedStationId
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
                  <option value="">
                    선택
                  </option>
                  <option value="PACKAGING">
                    PACKAGING
                  </option>
                  <option value="TEST">
                    TEST
                  </option>
                  <option value="STORAGE">
                    STORAGE
                  </option>
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

          <section className="station-section station-arm-card">
            <div className="station-arm-header">
              <div>
                <h3>
                  Place 팔 자세 (J1-J6)
                </h3>

                <p>
                  배치 시 사용할 로봇 팔 관절 자세입니다.
                </p>
              </div>

              <button
                type="button"
                className="save-pose-button"
                onClick={
                  saveCurrentArmPose
                }
              >
                현재 자세 지정
              </button>
            </div>

            <div className="station-joint-grid">
              {selectedStation
                .place_arm_pose
                .map(
                  (
                    joint,
                    index,
                  ) => (
                    <label
                      key={index}
                    >
                      <span>
                        J{index + 1}
                      </span>

                      <input
                        type="number"
                        value={joint}
                        onChange={(event) =>
                          updateArmJoint(
                            index,
                            event.target.value,
                          )
                        }
                      />
                    </label>
                  ),
                )}
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
                  <option value="">
                    선택
                  </option>
                  <option value="TIMER">
                    TIMER
                  </option>
                  <option value="VISION">
                    VISION
                  </option>
                  <option value="TIMER + VISION">
                    TIMER + VISION
                  </option>
                  <option value="EXTERNAL">
                    EXTERNAL
                  </option>
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
                )}
              </span>
            </div>
          </section>

          <section className="station-next-process">
            <div>
              <span>
                다음 공정
              </span>

              <strong>
                TEST-01
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

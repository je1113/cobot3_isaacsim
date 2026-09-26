// Top View — 원래 MonitoringPage 에 있었다. 2026-09-26: 사용자 지시로
// 실시간 모니터링에서 빼고 작업 할당(TaskAssignmentPage)으로 옮겼다 —
// 로봇↔선반 배정을 할 때 실제 배치를 보는 게 더 맞는 자리라서. 두 화면이
// 다시 같이 쓸 수도 있어 공유 컴포넌트로 뺐다.

import {
  CONVEYOR_CENTERS_M,
  CONVEYOR_SHELF_IDS,
  SHELF_DEPTH_M,
  SHELF_SLOT_OFFSETS,
  SHELF_WIDTH_M,
  STATION_LENGTH_M,
  STATION_ROLLER_OFFSETS,
  STATION_WIDTH_M,
  computeTopViewViewBox,
  midpoint,
  toNumber,
  toPoint,
} from './topViewLayout'

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

      {/* 이름표 — 스테이션이 다 Top View 아래쪽에 몰려 있어(같은 월드 x 줄),
          아래에 적으면 화면/뷰박스 가장자리와 겹치기 쉽다. 위에 적는다. */}
      <text
        x={p.x}
        y={p.y - STATION_LENGTH_M / 2 - 0.32}
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

export function FactoryTopView({
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
  // 화면을 가로로 길게 쓰기 위해 x/y 축을 맞바꿔 90도 회전시켜 그린다.
  function toSvg(point) {
    return {
      x: point.y - minY,
      y: point.x - minX,
    }
  }

  const viewBox = computeTopViewViewBox(
    worldBounds,
    stations,
  )

  return (
    <svg
      className="monitor-topview-svg"
      viewBox={`${viewBox.x} ${viewBox.y} ${viewBox.width} ${viewBox.height}`}
      preserveAspectRatio="xMidYMid meet"
    >
      <rect
        className="monitor-topview-floor"
        x={0}
        y={0}
        width={viewBox.mapSvgWidth}
        height={viewBox.mapSvgHeight}
      />

      {shelves.map((shelf) => {
        if (
          CONVEYOR_SHELF_IDS.has(
            shelf.shelf_id,
          )
        ) {
          // 컨베이어 프레임 실제 중심을 쓴다 — waypoint 는 로봇이 관측하려고
          // 서는 자리(벨트 앞)라 중심이 아니다(위 CONVEYOR_CENTERS_M 주석).
          const center =
            CONVEYOR_CENTERS_M[
              shelf.shelf_id
            ] ??
            midpoint(
              toPoint(
                shelf.waypoint_start,
              ),
              toPoint(
                shelf.waypoint_end,
              ),
            )

          if (!center) {
            return null
          }

          return (
            <ConveyorIcon
              key={shelf.shelf_id}
              p={toSvg(center)}
              label={shelf.shelf_id}
              variant="output"
            />
          )
        }

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
        // 컨베이어 프레임 실제 중심을 쓴다 — place_pose 는 로봇이 놓으려고
        // 서는 자리(벨트 앞)라 중심이 아니다(위 CONVEYOR_CENTERS_M 주석).
        const point =
          CONVEYOR_CENTERS_M[
            station.station_id
          ] ??
          toPoint(
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

      {(robots ?? []).map(
        (robotId, index) => {
          const robot =
            (robotStates ?? {})[robotId]

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

// FactoryTopView 가 쓰는 좌표/치수 상수와 순수 함수 — 컴포넌트가 아닌
// 것들을 여기 따로 둔다(react-refresh 규칙: 컴포넌트 파일은 컴포넌트만
// export 해야 Fast Refresh 가 깨지지 않는다).

export function toNumber(value) {
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

export function toPoint(pose) {
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

export function midpoint(a, b) {
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
export const SHELF_WIDTH_M = 2.5
export const SHELF_DEPTH_M = 1.0
// 선반 슬롯 구분선 — 매거진 4개 자리를 3개 선으로 나눈다(중앙 기준 오프셋).
export const SHELF_SLOT_OFFSETS = [
  -SHELF_WIDTH_M / 4,
  0,
  SHELF_WIDTH_M / 4,
]

// 컨베이어(스테이션) 실제 크기(m) — PackagingZone/ConveyorFrame 큐브
// 스케일(4.4 x 1.35 x 0.45)을 그대로 쓴다. 세 구역(PKG-01 계열) 모두 같은
// 스케일이라 station_type 과 무관하게 동일 크기를 쓴다.
export const STATION_LENGTH_M = 4.4
export const STATION_WIDTH_M = 1.35
// 롤러 표시선 — 길이 방향으로 고르게 나눈 7개.
export const STATION_ROLLER_OFFSETS = [-1.8, -1.2, -0.6, 0, 0.6, 1.2, 1.8]

// shelves.yaml 에 선반으로 들어 있지만 씬에서는 컨베이어인 것.
// PKG-OUT 은 포장 출력(PackagingUnloaderZone) 벨트 앞의 스택 관측 자리다 —
// shelves.yaml 에 있는 이유는 carrier_code_reader 가 그 목록으로 관측 자세를
// 고르기 때문이다(그 파일의 PKG-OUT 주석). 벨트는 PackagingZone · TestingZone
// 과 크기가 같으므로 스테이션과 같은 컨베이어 아이콘으로 그린다.
export const CONVEYOR_SHELF_IDS = new Set(['PKG-OUT'])

// 컨베이어 프레임의 실제 중심(map=world 좌표, m) — simple_factory_layout.usda
// PackagingZone/TestingZone/PackagingUnloaderZone 의 ConveyorFrame
// xformOp:translate 를 그대로 쓴다(부모 Xform translate 포함).
//   PackagingZone   : 부모(0,4.6,0) + Cube(6.4,0)      -> (6.4, 4.6)
//   TestingZone     : 부모(0,4.6,0) + Cube(6.4,-7.3053) -> (6.4, -2.705315)
//   PackagingUnloaderZone(PKG-OUT) : 부모 identity + Cube(6.4,0.971365)
// station.place_pose / shelf.waypoint 는 로봇이 place·관측하려고 서는
// 자리(벨트 앞 0.35~2.7m)라 벨트 중심이 아니다 — 그걸 그대로 그리면
// 아이콘이 실제 컨베이어보다 로봇 주차 지점(벽 쪽)으로 당겨져 보인다.
export const CONVEYOR_CENTERS_M = {
  'PKG-01': { x: 6.4, y: 4.6 },
  'TEST-01': { x: 6.4, y: -2.705315 },
  'PKG-OUT': { x: 6.4, y: 0.971365 },
}

/**
 * SVG 뷰박스 — 지도(occupancy grid) 경계만 쓰면 스테이션이 잘릴 수 있다.
 * 예) PKG-01 은 place_pose 대신 실제 컨베이어 중심(CONVEYOR_CENTERS_M)을
 * 쓰는데, 그 풋프린트(4.4 x 1.35m)가 지도 x 최댓값보다 밖으로 뻗어서
 * 그대로 그리면 아래가 잘린다. 지도 경계와 모든 스테이션 풋프린트를
 * 함께 담는 크기로 뷰박스를 잡는다 — 컨테이너 aspect-ratio 도 이 함수로
 * 똑같이 계산해야 레터박스 없이 맞는다.
 */
export function computeTopViewViewBox(worldBounds, stations) {
  if (!worldBounds) {
    return null
  }

  const minX = worldBounds.origin[0]
  const minY = worldBounds.origin[1]
  const mapSvgWidth =
    worldBounds.height_px *
    worldBounds.resolution
  const mapSvgHeight =
    worldBounds.width_px *
    worldBounds.resolution

  let x0 = 0
  let y0 = 0
  let x1 = mapSvgWidth
  let y1 = mapSvgHeight

  for (const station of stations ?? []) {
    const point =
      CONVEYOR_CENTERS_M[
        station.station_id
      ] ??
      toPoint(station.place_pose)

    if (!point) {
      continue
    }

    // toSvg() 와 같은 축 맞바꿈.
    const svgX = point.y - minY
    const svgY = point.x - minX

    x0 = Math.min(
      x0,
      svgX - STATION_WIDTH_M / 2,
    )
    x1 = Math.max(
      x1,
      svgX + STATION_WIDTH_M / 2,
    )
    y0 = Math.min(
      y0,
      svgY - STATION_LENGTH_M / 2,
    )
    y1 = Math.max(
      y1,
      svgY + STATION_LENGTH_M / 2,
    )
  }

  return {
    x: x0,
    y: y0,
    width: x1 - x0,
    height: y1 - y0,
    mapSvgWidth,
    mapSvgHeight,
  }
}

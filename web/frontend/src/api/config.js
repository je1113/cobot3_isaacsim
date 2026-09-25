import {
  apiGet,
  apiPut,
} from './client'

/**
 * 설정 — 선반 / 스테이션 / 공정 흐름.
 *
 * ★ 이 값들의 주인은 DB 가 아니라 **yaml 파일**이다(docs/DB구성.md §10-1).
 *   그래서 DB 가 안 붙은 상태에서도 이 화면들은 정상 동작한다.
 *
 * ★ 저장은 낙관적 잠금이다. GET 이 준 `revision` 을 PUT 이 If-Match 로
 *   되돌려줘야 하고, 그 사이 파일이 바뀌었으면 412 가 온다. 사람이 둘이서
 *   같은 파일을 고칠 수 있고(에디터로 직접 고치는 경우 포함), 그때 조용히
 *   덮어쓰는 것이 가장 나쁘다.
 *
 * ★ 좌표·관절값은 **문자열**로 주고받는다. 화면의 <input type="number"> 가
 *   값을 문자열로 들고 있고 미입력이 '' 이기 때문이다. 파일에는 숫자로
 *   저장되고, 변환은 백엔드(app/shapes.py) 한 곳에서만 한다.
 */

// ── 선반 ─────────────────────────────────────────────────────────────
export function fetchShelves() {
  return apiGet('/api/shelves')
}

export function saveShelves(
  shelves,
  revision,
) {
  return apiPut(
    '/api/shelves',
    { shelves },
    revision,
  )
}

// ── 스테이션 ─────────────────────────────────────────────────────────
export function fetchStations() {
  return apiGet('/api/stations')
}

export function saveStations(
  stations,
  revision,
) {
  return apiPut(
    '/api/stations',
    { stations },
    revision,
  )
}

// ── 공정 흐름 ────────────────────────────────────────────────────────
export function fetchRouting() {
  return apiGet('/api/routing')
}

/**
 * 규칙과 최종 목적지를 **함께** 저장한다.
 * 화면에서는 둘이 다른 곳에 살지만(rules 는 페이지, finalDestination 은 App),
 * "완성됐는가" 판정이 둘을 같이 보므로 서버에서는 한 리소스다.
 */
export function saveRouting(
  rules,
  finalDestination,
  revision,
) {
  return apiPut(
    '/api/routing',
    {
      rules,
      final_destination:
        finalDestination,
    },
    revision,
  )
}

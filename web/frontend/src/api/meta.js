import { apiGet } from './client'

/**
 * 화면이 하드코딩하던 값들의 단일 출처.
 *
 * 로봇 이름·열거값·팔 축 수가 여기서 온다. 예전에는 같은 목록이 두 곳에
 * 있었다 — 로봇 이름은 백엔드 설정과 `MonitoringPage.ROBOT_IDS` 에,
 * 완료 신호는 `shapes.py` 와 <select> 에. 한쪽만 고치면 조용히 갈라지고,
 * 화면에는 선택지가 있는데 서버가 422 로 거절하는 상태가 된다.
 */
export function fetchMeta() {
  return apiGet('/api/meta')
}

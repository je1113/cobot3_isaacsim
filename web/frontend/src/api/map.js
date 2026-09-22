import { apiGet } from './client'

/**
 * MonitoringPage Top View 의 좌표계 범위(origin/resolution/크기).
 *
 * Nav2 map_server 가 쓰는 것과 같은 파일에서 온다(backend compat.py GET
 * /api/map 주석 참고) — 로봇이 실제로 그 지도로 로컬라이즈하므로, 화면이
 * 다른 범위를 쓰면 선반·스테이션·로봇 위치가 실제 배치와 어긋난다.
 *
 * 지도 이미지 자체는 안 낸다 — Top View 는 이 범위 안에 선반·스테이션·
 * 로봇 위치를 직접 그린다(MonitoringPage.jsx FactoryTopView).
 */
export function fetchMapInfo() {
  return apiGet('/api/map')
}

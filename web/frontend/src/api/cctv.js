import { apiGet } from './client'

// client.js 의 API_BASE_URL 과 같은 규칙 — 비어 있으면 같은 출처(/api 는 vite 가 넘긴다).
const API_BASE_URL =
  import.meta.env.VITE_API_BASE_URL ?? ''

/** 영상이 오고 있는지. { enabled, live, age_sec, width, height } */
export function fetchCctvStatus() {
  return apiGet('/api/cctv/status')
}

/**
 * MJPEG 스트림 주소. <img src> 에 그대로 넣는다.
 * key 를 바꾸면 브라우저가 스트림을 새로 연다(끊겼다 살아났을 때).
 */
export function cctvStreamUrl(key) {
  return `${API_BASE_URL}/api/cctv.mjpeg?k=${key}`
}

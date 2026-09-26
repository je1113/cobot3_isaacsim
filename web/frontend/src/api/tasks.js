import {
  apiDelete,
  apiGet,
  apiPost,
} from './client'

// ── 실제 서버 큐(표 5) 조회 ──────────────────────────────────────────
// 서버(dispatcher)가 실제로 들고 있는 작업이다. RECOVER 는 사람이 화면에서
// 만들지 않고 pickup 스케줄러가 자동으로 큐에 넣으므로, 여기서 직접 봐야만
// 그게 생겼는지 알 수 있다(MonitoringPage 참고).
export function fetchRobotQueue(
  robotId,
) {
  return apiGet(
    `/api/robots/${encodeURIComponent(robotId)}/queue`,
  )
}

// ── 큐 항목 제거 ──────────────────────────────────────────────────────
// 상태와 무관하게 무조건 지운다(서버 queue.cancel() 이 QUEUED/RUNNING/
// 그 외 전부 받아준다 — 2026-09-25). RUNNING 이면 서버가 먼저 정중하게
// STOP 과 같은 경로로도 요청해본다.
export function deleteTask(taskId) {
  return apiDelete(`/api/tasks/${encodeURIComponent(taskId)}`)
}

// ── 시뮬레이션 초기화 (개발용) ────────────────────────────────────────
// Isaac Sim 을 껐다 켤 때 쓴다 — task/pending_pickup 을 통째로 비운다.
// run 기록(magazine_log/stack_log)은 안 건드린다.
export function resetSimQueue() {
  return apiPost('/api/tasks/reset-sim')
}

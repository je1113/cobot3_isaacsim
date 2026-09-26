import { apiPost } from './client'

function robotPath(robotId) {
  return encodeURIComponent(robotId)
}

export function requestRobotPause(robotId) {
  return apiPost(
    `/api/robots/${robotPath(robotId)}/pause`,
    {},
  )
}

export function requestRobotResume(robotId) {
  return apiPost(
    `/api/robots/${robotPath(robotId)}/resume`,
    {},
  )
}

// ── 일반 명령(PAUSE/RESUME/SKIP/STOP) ────────────────────────────────
// pause/resume 과 달리 이건 202 로 바로 답하고 실제 결과는 command_result
// 웹소켓으로 온다(commands.py 주석) — 여기서는 "받아줬는지"만 본다. RUNNING
// 인 작업을 화면 큐에서 세우는 데 쓴다(QUEUED 는 DELETE /api/tasks/{id}).
export function requestRobotCommand(robotId, command, reason = '') {
  return apiPost(
    `/api/robots/${robotPath(robotId)}/commands`,
    { command, reason },
  )
}

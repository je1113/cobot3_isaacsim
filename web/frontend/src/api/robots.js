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

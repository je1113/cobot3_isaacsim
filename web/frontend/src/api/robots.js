import { apiPost } from './client'

function robotPath(robotId) {
  return encodeURIComponent(robotId)
}

export function requestRobotGoal(
  robotId,
  goal,
) {
  return apiPost(
    `/api/robots/${robotPath(robotId)}/goal`,
    {
      goal,
    },
  )
}

export function requestRobotPause(robotId) {
  return apiPost(
    `/api/robots/${robotPath(robotId)}/pause`,
    {},
  )
}

export function requestRobotResume(
  robotId,
  goal = null,
) {
  return apiPost(
    `/api/robots/${robotPath(robotId)}/resume`,
    {
      goal,
    },
  )
}

import { apiPost } from './client'

const TASK_START_PATH =
  import.meta.env.VITE_TASK_START_PATH ?? ''

export function requestTaskStart(
  payload,
) {
  if (!TASK_START_PATH) {
    throw new Error(
      'VITE_TASK_START_PATH가 설정되지 않았습니다.',
    )
  }

  return apiPost(
    TASK_START_PATH,
    payload,
  )
}

import { apiGet } from './client'

const LOGS_PATH =
  import.meta.env.VITE_LOGS_PATH ?? ''

export function requestLogs() {
  if (!LOGS_PATH) {
    throw new Error(
      'VITE_LOGS_PATH가 설정되지 않았습니다.',
    )
  }

  return apiGet(LOGS_PATH)
}

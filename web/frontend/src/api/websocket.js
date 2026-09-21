const WS_BASE_URL =
  import.meta.env.VITE_WS_BASE_URL ?? ''

/**
 * 비어 있으면 **같은 출처**의 /ws 로 간다 — client.js 의 API_BASE_URL 과 같은 규칙이다.
 *
 * ★ 예전 기본값은 'ws://localhost:5173/ws' 였다. dev 서버 포트가 박혀 있어서
 *   포트가 밀리거나(5174) 배포판(백엔드가 dist 를 8000 에서 서빙)에서는
 *   엉뚱한 곳을 가리켰다. vite.config.js 가 /ws 를 프록시하므로, 자기 출처로
 *   보내면 개발이든 배포든 같은 코드로 맞는 곳에 닿는다.
 */
function resolveUrl() {
  if (WS_BASE_URL) {
    return WS_BASE_URL
  }

  const protocol =
    window.location.protocol === 'https:'
      ? 'wss:'
      : 'ws:'

  return `${protocol}//${window.location.host}/ws`
}

export function createWebSocketClient({
  onOpen,
  onMessage,
  onClose,
  onError,
} = {}) {
  const socket =
    new WebSocket(resolveUrl())

  socket.addEventListener(
    'open',
    (event) => {
      onOpen?.(event)
    },
  )

  socket.addEventListener(
    'message',
    (event) => {
      onMessage?.(event)
    },
  )

  socket.addEventListener(
    'close',
    (event) => {
      onClose?.(event)
    },
  )

  socket.addEventListener(
    'error',
    (event) => {
      onError?.(event)
    },
  )

  return socket
}

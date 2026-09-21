const WS_BASE_URL =
  import.meta.env.VITE_WS_BASE_URL ?? ''

export function createWebSocketClient({
  onOpen,
  onMessage,
  onClose,
  onError,
} = {}) {
  if (!WS_BASE_URL) {
    throw new Error(
      'VITE_WS_BASE_URL이 설정되지 않았습니다.',
    )
  }

  const socket =
    new WebSocket(WS_BASE_URL)

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

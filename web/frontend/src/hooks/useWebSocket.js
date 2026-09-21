import {
  useCallback,
  useEffect,
  useRef,
  useState,
} from 'react'

import {
  createWebSocketClient,
} from '../api/websocket'

// 재연결 간격. 곧바로 다시 붙되, 서버가 죽어 있으면 점점 뜸해진다.
const BACKOFF_MS = [500, 1000, 2000, 5000, 10000, 15000]

/**
 * 관제 WebSocket 한 줄.
 *
 * ★ 예전 판은 `connect` 를 돌려주기만 하고 **아무도 부르지 않았다.**
 *   그래서 소켓이 한 번도 열리지 않았고, 실시간 화면은 영원히
 *   DISCONNECTED 인 채 빈 값을 그렸다. 이제 마운트되면 스스로 붙는다.
 *
 * ★ 재연결도 여기서 한다. 서버(ws_routes.py)는 놓친 메시지를 재전송하지
 *   않는다 — 버퍼를 두면 "실시간 상태를 서버가 들고 있다" 는 뜻이 되어
 *   설계가 어긋난다. 그래서 끊겼다 붙으면 **REST 로 다시 읽는 것**이
 *   맞고, 그 신호로 onReconnect 를 부른다.
 *
 * @param onMessage   봉투를 파싱한 객체를 받는다. types 로 거른 뒤에 온다.
 * @param types       듣고 싶은 message.type 목록. 비우면 전부.
 * @param onReconnect 끊겼다 다시 붙은 순간. 스냅샷을 다시 읽으라는 뜻.
 */
function useWebSocket({
  onMessage,
  types,
  onReconnect,
} = {}) {
  const socketRef = useRef(null)
  const timerRef = useRef(null)
  const attemptRef = useRef(0)
  const closingRef = useRef(false)
  const everConnectedRef = useRef(false)

  // 콜백은 ref 로 들고 있는다 — 페이지가 리렌더될 때마다 새 함수가 와도
  // 소켓을 다시 열지 않기 위해서다. 커밋 뒤에 넣는다(렌더 중 ref 쓰기 금지).
  const handlersRef = useRef({})

  useEffect(() => {
    handlersRef.current = {
      onMessage,
      onReconnect,
    }
  })

  const typeKey = Array.isArray(types)
    ? [...types].sort().join(',')
    : ''

  const [
    connectionStatus,
    setConnectionStatus,
  ] = useState('CONNECTING')

  const [lastMessage, setLastMessage] =
    useState(null)

  const [error, setError] =
    useState(null)

  const disconnect =
    useCallback(() => {
      closingRef.current = true

      if (timerRef.current) {
        clearTimeout(timerRef.current)
        timerRef.current = null
      }

      if (socketRef.current) {
        socketRef.current.close()
        socketRef.current = null
      }

      setConnectionStatus(
        'DISCONNECTED',
      )
    }, [])

  useEffect(() => {
    closingRef.current = false
    attemptRef.current = 0

    const wanted = typeKey
      ? new Set(typeKey.split(','))
      : null

    function open() {
      if (closingRef.current) {
        return
      }

      setConnectionStatus(
        'CONNECTING',
      )

      let socket

      try {
        socket =
          createWebSocketClient({
            onOpen: () => {
              attemptRef.current = 0
              setError(null)
              setConnectionStatus(
                'CONNECTED',
              )

              // 끊겼다 붙은 것이면 그 사이를 놓쳤다 — 다시 읽으라고 알린다.
              if (
                everConnectedRef.current
              ) {
                handlersRef.current.onReconnect?.()
              }

              everConnectedRef.current = true
            },

            onMessage: (event) => {
              // MonitoringPage 가 원문 문자열을 직접 파싱한다 — 그대로 둔다.
              setLastMessage(event.data)

              let parsed

              try {
                parsed = JSON.parse(
                  event.data,
                )
              } catch {
                return
              }

              if (
                wanted &&
                !wanted.has(parsed.type)
              ) {
                return
              }

              handlersRef.current.onMessage?.(
                parsed,
              )
            },

            onClose: () => {
              socketRef.current = null

              if (closingRef.current) {
                return
              }

              setConnectionStatus(
                'DISCONNECTED',
              )

              const delay =
                BACKOFF_MS[
                  Math.min(
                    attemptRef.current,
                    BACKOFF_MS.length - 1,
                  )
                ]

              attemptRef.current += 1

              timerRef.current =
                setTimeout(open, delay)
            },

            onError: () => {
              // 여기서 재연결을 걸지 않는다 — error 뒤에는 close 가 반드시 온다.
              setError(
                'WebSocket 연결 중 오류가 발생했습니다.',
              )
            },
          })
      } catch (connectError) {
        setConnectionStatus(
          'DISCONNECTED',
        )

        setError(
          connectError instanceof Error
            ? connectError.message
            : 'WebSocket 연결 중 오류가 발생했습니다.',
        )

        return
      }

      socketRef.current = socket
    }

    open()

    // StrictMode 는 개발에서 이 효과를 두 번 돌린다. 여기서 확실히 닫아
    // 두지 않으면 소켓이 두 개 열린 채로 남는다.
    return () => {
      closingRef.current = true

      if (timerRef.current) {
        clearTimeout(timerRef.current)
        timerRef.current = null
      }

      if (socketRef.current) {
        socketRef.current.close()
        socketRef.current = null
      }
    }
  }, [typeKey])

  const connect =
    useCallback(() => {
      // 자동으로 붙으므로 보통은 쓸 일이 없다. 수동으로 끊은 뒤 되살릴 때만.
      closingRef.current = false
    }, [])

  return {
    connectionStatus,
    lastMessage,
    error,
    connect,
    disconnect,
  }
}

export default useWebSocket

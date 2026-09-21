import {
  useCallback,
  useEffect,
  useRef,
  useState,
} from 'react'

import {
  createWebSocketClient,
} from '../api/websocket'

function useWebSocket() {
  const socketRef = useRef(null)

  const [
    connectionStatus,
    setConnectionStatus,
  ] = useState('DISCONNECTED')

  const [
    lastMessage,
    setLastMessage,
  ] = useState(null)

  const [error, setError] =
    useState(null)

  const disconnect =
    useCallback(() => {
      if (socketRef.current) {
        socketRef.current.close()
        socketRef.current = null
      }

      setConnectionStatus(
        'DISCONNECTED',
      )
    }, [])

  const connect =
    useCallback(() => {
      if (socketRef.current) {
        return
      }

      setConnectionStatus(
        'CONNECTING',
      )

      setError(null)

      try {
        socketRef.current =
          createWebSocketClient({
            onOpen: () => {
              setConnectionStatus(
                'CONNECTED',
              )
            },

            onMessage: (event) => {
              setLastMessage(
                event.data,
              )
            },

            onClose: () => {
              socketRef.current =
                null

              setConnectionStatus(
                'DISCONNECTED',
              )
            },

            onError: () => {
              setError(
                'WebSocket 연결 중 오류가 발생했습니다.',
              )
            },
          })
      } catch (connectError) {
        setConnectionStatus(
          'DISCONNECTED',
        )

        const message =
          connectError instanceof Error
            ? connectError.message
            : 'WebSocket 연결 중 오류가 발생했습니다.'

        setError(message)
      }
    }, [])

  useEffect(() => {
    return () => {
      if (socketRef.current) {
        socketRef.current.close()
      }
    }
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

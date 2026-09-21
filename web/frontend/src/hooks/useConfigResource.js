import {
  useCallback,
  useEffect,
  useRef,
  useState,
} from 'react'

import { ApiError } from '../api/client'

/**
 * 서버에 사는 설정 한 덩어리를 들고 있는 훅.
 *
 * 화면은 지금까지 localStorage 를 진실로 삼았다. 그걸 서버로 옮기면서도
 * **편집 경험은 그대로** 두려고 이 모양을 골랐다:
 *
 *   · `value` / `setValue` 는 예전 useState 와 똑같이 쓴다. 타이핑할 때마다
 *     서버를 부르지 않는다 — 좌표 한 칸 고칠 때마다 PUT 을 날리면 느리고,
 *     반쯤 입력된 값이 파일에 들어간다.
 *   · 명시적으로 `save()` 를 불러야 서버에 쓴다. '저장' 버튼이 그 자리다.
 *
 * ★ 낙관적 잠금: GET 이 준 revision 을 save 가 If-Match 로 되돌려준다.
 *   그 사이 남이 먼저 저장했으면 412 가 오고, 그때는 **덮어쓰지 않는다** —
 *   `stale` 을 세워 화면이 "다시 불러오기" 를 권하게 한다.
 *
 * ★ `dirty` 는 마지막으로 서버와 맞춘 값과 지금 값이 다른지다. 저장 안 한
 *   변경을 사람이 알아야 페이지를 떠나기 전에 누를 수 있다.
 */
export function useConfigResource(
  fetcher,
  saver,
  pick,
  emptyValue,
) {
  const [value, setValue] =
    useState(null)
  const [revision, setRevision] =
    useState(null)
  const [status, setStatus] =
    useState('loading') // loading | ready | saving | error
  const [error, setError] =
    useState(null)
  const [stale, setStale] =
    useState(false)

  // 마지막으로 서버와 맞춘 스냅샷. dirty 판정에만 쓴다.
  const syncedRef = useRef(null)

  const load = useCallback(
    async () => {
      setStatus('loading')
      setError(null)

      try {
        const res = await fetcher()
        const next = pick(res)

        setValue(next)
        setRevision(res.revision)
        syncedRef.current =
          JSON.stringify(next)
        setStale(false)
        setStatus('ready')
      } catch (err) {
        setError(err)
        setStatus('error')
      }
    },
    [fetcher, pick],
  )

  useEffect(() => {
    load()
  }, [load])

  const save = useCallback(
    async (override) => {
      const payload =
        override ?? value

      if (payload === null) {
        return null
      }

      setStatus('saving')
      setError(null)

      try {
        const res = await saver(
          payload,
          revision,
        )

        const next = pick(res)

        setValue(next)
        setRevision(res.revision)
        syncedRef.current =
          JSON.stringify(next)
        setStale(false)
        setStatus('ready')

        return res
      } catch (err) {
        setError(err)
        setStatus('ready')

        // 412 는 '고장' 이 아니라 '당신 화면이 낡았다' 이다.
        if (
          err instanceof ApiError &&
          err.stale
        ) {
          setStale(true)
        }

        throw err
      }
    },
    [saver, value, revision, pick],
  )

  /**
   * setValue 의 null 안전판.
   *
   * ★ 로딩이 끝나기 전에도 화면은 이미 그려져 있다(빈 목록으로). 그 상태에서
   *   '선반 추가' 를 누르면 `setValue(prev => [...prev, x])` 의 prev 가 null 이라
   *   터진다. 로딩이 0.2초면 잘 안 밟히고, 느릴 때만 밟히는 종류의 버그다.
   *   그래서 훅이 빈 값으로 바꿔서 넘긴다.
   */
  const update = useCallback(
    (updater) => {
      setValue((prev) =>
        typeof updater === 'function'
          ? updater(
              prev ?? emptyValue,
            )
          : updater,
      )
    },
    [emptyValue],
  )

  const dirty =
    value !== null &&
    syncedRef.current !==
      JSON.stringify(value)

  return {
    value,
    setValue: update,
    revision,
    status,
    error,
    stale,
    dirty,
    save,
    reload: load,
  }
}

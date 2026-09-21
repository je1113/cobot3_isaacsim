import {
  createContext,
  useContext,
  useEffect,
  useState,
} from 'react'

import { fetchMeta } from '../api/meta'

const MetaContext =
  createContext(null)

/**
 * 로봇 목록과 열거값을 앱 전체에 공급한다.
 *
 * ★ **기본값(fallback)을 두지 않는다.** 서버가 못 오면 화면을 그리지 않는다.
 *   "일단 AMR-01/AMR-02 로 그려 두자" 는 하드코딩을 코드에서 지우고
 *   폴백에 옮겨 적는 것일 뿐이고, 로봇이 3대가 되는 날 바로 그 폴백이
 *   조용히 틀린 화면을 그린다. 못 받으면 못 받았다고 말하는 편이 낫다.
 *
 * ★ 이 값들은 파일도 DB 도 아니다. 전부 서버 설정에서 유도되므로 편집 대상이
 *   아니고, 그래서 revision/저장 개념이 없다.
 */
export function MetaProvider({
  children,
}) {
  const [meta, setMeta] =
    useState(null)
  const [error, setError] =
    useState(null)

  useEffect(() => {
    let alive = true

    fetchMeta()
      .then((value) => {
        if (alive) setMeta(value)
      })
      .catch((err) => {
        if (alive) setError(err)
      })

    return () => {
      alive = false
    }
  }, [])

  if (error) {
    return (
      <div className="boot-error">
        <h1>백엔드에 연결할 수 없습니다</h1>

        <p>{error.message}</p>

        <p className="boot-hint">
          백엔드가 떠 있는지 확인하세요.
          개발 중이라면 web/backend 에서{' '}
          <code>
            uvicorn app.main:app --port 8000
          </code>{' '}
          입니다.
        </p>

        <button
          type="button"
          onClick={() =>
            window.location.reload()
          }
        >
          다시 시도
        </button>
      </div>
    )
  }

  if (!meta) {
    return (
      <div className="boot-loading">
        불러오는 중…
      </div>
    )
  }

  return (
    <MetaContext.Provider
      value={meta}
    >
      {children}
    </MetaContext.Provider>
  )
}

export function useMeta() {
  const meta =
    useContext(MetaContext)

  if (!meta) {
    throw new Error(
      'useMeta 는 MetaProvider 안에서만 쓸 수 있습니다.',
    )
  }

  return meta
}

/** 로봇 목록. 화면 어디서나 이 순서를 그대로 쓴다. */
export function useRobots() {
  return useMeta().robots
}

/** 열거값 하나. 이름이 틀리면 조용히 빈 목록이 되지 않도록 던진다. */
export function useEnum(name) {
  const { enums } = useMeta()

  if (!enums[name]) {
    throw new Error(
      `모르는 열거값: ${name}`,
    )
  }

  return enums[name]
}

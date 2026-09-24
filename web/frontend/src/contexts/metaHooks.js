import { useContext } from 'react'

import { MetaContext } from './metaContext'

export function useMeta() {
  const meta = useContext(MetaContext)

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

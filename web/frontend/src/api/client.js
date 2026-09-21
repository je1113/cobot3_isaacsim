const API_BASE_URL =
  import.meta.env.VITE_API_BASE_URL ?? ''

/**
 * 백엔드가 주는 실패 정보를 그대로 들고 다니는 에러.
 *
 * ★ 백엔드는 실패마다 {code, message} 를 주고, message 는 "무엇이 잘못됐고
 *   어떻게 고치는지" 가 적힌 사람이 읽는 문장이다. 예:
 *     "SHELF-B 는 티칭이 끝나지 않아 배정할 수 없다. 설정 > 선반에서…"
 *     "다른 사람이 먼저 저장했다. 화면을 새로 고친 뒤 다시 시도할 것."
 *   이전 판은 이걸 버리고 `API 요청 실패: 422` 만 보여줬다. 상태 코드만으로는
 *   사용자가 할 수 있는 일이 없다.
 */
export class ApiError extends Error {
  constructor(status, body) {
    super(
      body?.message ||
        `API 요청 실패: ${status}`,
    )

    this.name = 'ApiError'
    this.status = status
    this.code =
      body?.code || 'http_error'
    this.body = body || {}
  }

  /** DB·ROS 가 아직 안 붙었다. '고장' 과 구분해서 보여줄 것. */
  get unavailable() {
    return this.status === 503
  }

  /** 그 사이 남이 먼저 저장했다 → 다시 읽어야 한다. */
  get stale() {
    return this.status === 412
  }
}

export async function apiRequest(
  path,
  options = {},
) {
  const response = await fetch(
    `${API_BASE_URL}${path}`,
    {
      ...options,

      headers: {
        'Content-Type':
          'application/json',

        ...options.headers,
      },
    },
  )

  if (!response.ok) {
    let body = null

    try {
      body = await response.json()
    } catch {
      // 본문이 JSON 이 아닐 수 있다 (프록시 오류 등)
    }

    throw new ApiError(
      response.status,
      body,
    )
  }

  const contentType =
    response.headers.get(
      'content-type',
    )

  const payload =
    contentType?.includes(
      'application/json',
    )
      ? await response.json()
      : await response.text()

  // 설정 화면은 저장할 때 revision 을 If-Match 로 되돌려줘야 한다.
  // 본문에도 들어 있지만, 헤더로도 오므로 둘 다 쓸 수 있게 붙여 둔다.
  const etag =
    response.headers.get('etag')

  if (
    etag &&
    payload &&
    typeof payload === 'object' &&
    !Array.isArray(payload) &&
    !payload.revision
  ) {
    payload.revision = etag
  }

  return payload
}

export function apiGet(path) {
  return apiRequest(path, {
    method: 'GET',
  })
}

export function apiPost(
  path,
  data,
) {
  return apiRequest(path, {
    method: 'POST',
    body: JSON.stringify(data),
  })
}

export function apiPut(
  path,
  data,
  revision,
) {
  return apiRequest(path, {
    method: 'PUT',
    body: JSON.stringify(data),

    // ★ 없으면 백엔드가 428 로 거절한다. 남의 편집을 덮어쓰지 않기 위한 것이라
    //   "일단 저장되게" 빼면 안 된다.
    headers: revision
      ? { 'If-Match': revision }
      : {},
  })
}

export function apiDelete(path) {
  return apiRequest(path, {
    method: 'DELETE',
  })
}

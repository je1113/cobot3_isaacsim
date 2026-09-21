const API_BASE_URL =
  import.meta.env.VITE_API_BASE_URL ?? ''

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
    throw new Error(
      `API 요청 실패: ${response.status}`,
    )
  }

  const contentType =
    response.headers.get(
      'content-type',
    )

  if (
    contentType?.includes(
      'application/json',
    )
  ) {
    return response.json()
  }

  return response.text()
}

export function apiGet(path) {
  return apiRequest(
    path,
    {
      method: 'GET',
    },
  )
}

export function apiPost(
  path,
  data,
) {
  return apiRequest(
    path,
    {
      method: 'POST',
      body: JSON.stringify(data),
    },
  )
}

export function apiPut(
  path,
  data,
) {
  return apiRequest(
    path,
    {
      method: 'PUT',
      body: JSON.stringify(data),
    },
  )
}

export function apiDelete(path) {
  return apiRequest(
    path,
    {
      method: 'DELETE',
    },
  )
}

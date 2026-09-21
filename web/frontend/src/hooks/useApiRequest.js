import { useState } from 'react'

function useApiRequest() {
  const [loading, setLoading] =
    useState(false)

  const [error, setError] =
    useState(null)

  async function execute(request) {
    setLoading(true)
    setError(null)

    try {
      return await request()
    } catch (requestError) {
      const message =
        requestError instanceof Error
          ? requestError.message
          : 'API 요청 중 오류가 발생했습니다.'

      setError(message)

      throw requestError
    } finally {
      setLoading(false)
    }
  }

  function clearError() {
    setError(null)
  }

  return {
    loading,
    error,
    execute,
    clearError,
  }
}

export default useApiRequest

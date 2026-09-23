import {
  useCallback,
  useEffect,
  useRef,
  useState,
} from 'react'

import {
  requestLogs,
} from '../api/logs'

import useApiRequest from '../hooks/useApiRequest'
import useWebSocket from '../hooks/useWebSocket'
import {
  useEnum,
  useRobots,
} from '../contexts/metaHooks'

function getLogSummary(
  logs,
  successValue,
  failedValue,
) {
  const total = logs.length

  const completed =
    logs.filter(
      (log) =>
        log.result === successValue,
    ).length

  const failed =
    logs.filter(
      (log) =>
        log.result === failedValue,
    ).length

  const durations =
    logs
      .map((log) =>
        Number(log.duration),
      )
      .filter((duration) =>
        Number.isFinite(duration),
      )

  const averageDuration =
    durations.length > 0
      ? durations.reduce(
          (sum, duration) =>
            sum + duration,
          0,
        ) / durations.length
      : null

  return {
    total,
    completed,
    failed,
    averageDuration,
  }
}

function LogsPage() {
  const robots = useRobots()

  // 'SUCCESS' / 'FAILED' 는 백엔드가 만드는 값이다(shapes.log_row_out).
  // 이름을 바꾸려면 한 곳만 고치면 되도록 여기서도 서버 것을 쓴다.
  const [successValue, failedValue] =
    useEnum('task_results')
  const [logs, setLogs] =
    useState([])

  const [logsLoaded, setLogsLoaded] =
    useState(false)

  const {
    loading: logsLoading,
    error: logsError,
    execute,
  } = useApiRequest()

  const [robotFilter, setRobotFilter] =
    useState('ALL')

  const [resultFilter, setResultFilter] =
    useState('ALL')

  const [searchQuery, setSearchQuery] =
    useState('')

  const loadLogsRef = useRef(null)
  const reloadTimerRef = useRef(null)

  // 6.4 — event_logger 가 로그를 INSERT 하면 DB 트리거가 pg_notify 를 쏘고,
  // 백엔드가 그걸 trace_appended 로 흘려 준다. 그 신호에 다시 읽는다.
  //
  // ★ 봉투를 그대로 목록에 붙이지 않고 통째로 다시 읽는 이유:
  //   봉투는 8000바이트 제한 때문에 키만 싣는다(004_notify.sql). 이 화면이
  //   쓰는 duration·task_type 이 없고, 통계도 배열 전체로 계산한다.
  // ★ 한 작업이 단계마다 여러 줄을 쌓으므로 신호가 연달아 온다. 400ms 모아서
  //   한 번만 읽는다.
  const scheduleReload =
    useCallback(() => {
      if (reloadTimerRef.current) {
        return
      }

      reloadTimerRef.current =
        setTimeout(() => {
          reloadTimerRef.current = null
          loadLogsRef.current?.()
        }, 400)
    }, [])

  useEffect(() => {
    return () => {
      if (reloadTimerRef.current) {
        clearTimeout(
          reloadTimerRef.current,
        )
      }
    }
  }, [])

  const {
    connectionStatus: liveStatus,
  } = useWebSocket({
    types: ['trace_appended'],
    onMessage: scheduleReload,
    // 끊긴 사이에 쌓인 줄은 서버가 재전송하지 않는다 — 다시 붙으면 다시 읽는다.
    onReconnect: () =>
      loadLogsRef.current?.(),
  })

  async function loadLogs() {
    try {
      const data =
        await execute(async () => {
          const response =
            await requestLogs()

          if (!Array.isArray(response)) {
            throw new Error(
              '로그 응답 형식이 배열이 아닙니다.',
            )
          }

          return response
        })

      setLogs(data)
      setLogsLoaded(true)
    } catch {
      setLogsLoaded(false)
    }
  }

  // ★ execute 는 렌더마다 새 함수라 의존성에 넣을 수 없다. 최신 loadLogs 를
  //   ref 로 들고 효과와 소켓 콜백은 그걸 부른다. 커밋 뒤에 넣는다.
  useEffect(() => {
    loadLogsRef.current = loadLogs
  })

  // 들어오면 바로 한 번 읽는다 — 예전에는 '로그 불러오기' 를 누르기 전까지
  // 빈 화면이었고, 그래서 화면이 백엔드와 안 붙은 것처럼 보였다.
  useEffect(() => {
    loadLogsRef.current?.()
  }, [])

  const summary =
    getLogSummary(
      logs,
      successValue,
      failedValue,
    )

  const averageDurationLabel =
    summary.averageDuration === null
      ? '-'
      : `${summary.averageDuration.toFixed(1)}초`

  const filteredLogs = logs.filter((log) => {
    const robotMatch =
      robotFilter === 'ALL' ||
      log.robot_id === robotFilter

    const resultMatch =
      resultFilter === 'ALL' ||
      log.result === resultFilter

    const normalizedSearch =
      searchQuery.trim().toLowerCase()

    const searchMatch =
      normalizedSearch === '' ||
      [
        log.robot_id,
        log.task_type,
        log.target_id,
        log.result,
      ].some((value) =>
        String(value ?? '')
          .toLowerCase()
          .includes(normalizedSearch),
      )

    return (
      robotMatch &&
      resultMatch &&
      searchMatch
    )
  })

  function exportCsv() {
    if (logs.length === 0) {
      return
    }

    const escapeCsv = (value) =>
      `"${String(value ?? '')
        .replace(/"/g, '""')}"`

    const rows = [
      [
        'timestamp',
        'robot_id',
        'task_type',
        'target_id',
        'result',
        'duration',
      ],
      ...logs.map((log) => [
        log.timestamp,
        log.robot_id,
        log.task_type,
        log.target_id,
        log.result,
        log.duration,
      ]),
    ]

    const csv = rows
      .map((row) =>
        row
          .map(escapeCsv)
          .join(','),
      )
      .join('\n')

    const blob = new Blob(
      [`\uFEFF${csv}`],
      {
        type:
          'text/csv;charset=utf-8;',
      },
    )

    const url =
      URL.createObjectURL(blob)

    const anchor =
      document.createElement('a')

    anchor.href = url
    anchor.download =
      'magazine-ops-logs.csv'

    document.body.appendChild(anchor)
    anchor.click()
    anchor.remove()

    URL.revokeObjectURL(url)
  }

  return (
    <section className="logs-page">
      <header className="logs-page-header">
        <div>
          <h1>로그 · 통계</h1>

          <p>
            AMR 작업 이력과 작업 결과를 확인합니다.
          </p>
        </div>

        <div className="logs-header-actions">
          <span className="logs-connection-badge">
            {logsLoading
              ? '불러오는 중'
              : logsError
                ? 'Backend 오류'
                : logsLoaded
                  ? liveStatus === 'CONNECTED'
                    ? '실시간 연결됨'
                    : 'Backend 연결됨 · 실시간 끊김'
                  : 'Backend 연동 전'}
          </span>

          <button
            type="button"
            className="logs-refresh-button"
            disabled={logsLoading}
            onClick={loadLogs}
          >
            {logsLoading
              ? '불러오는 중...'
              : '로그 불러오기'}
          </button>

          <button
            type="button"
            className="logs-export-button"
            disabled={logs.length === 0}
            onClick={exportCsv}
          >
            CSV 내보내기
          </button>
        </div>
      </header>

      {logsError && (
        <div className="logs-api-error">
          {logsError}
        </div>
      )}

      <section className="logs-summary-grid">
        <SummaryCard
          label="총 작업"
          value={summary.total}
        />

        <SummaryCard
          label="완료"
          value={summary.completed}
        />

        <SummaryCard
          label="실패"
          value={summary.failed}
        />

        <SummaryCard
          label="평균 소요시간"
          value={averageDurationLabel}
        />
      </section>

      <section className="logs-table-card">
        <div className="logs-table-header">
          <div>
            <h2>작업 로그</h2>

            <p>
              작업 수행 결과를 시간 순서로 확인합니다.
            </p>
          </div>
        </div>

        <div className="logs-toolbar">
          <input
            className="logs-search-input"
            type="search"
            value={searchQuery}
            placeholder="작업, 대상, AMR 검색"
            onChange={(event) =>
              setSearchQuery(
                event.target.value,
              )
            }
          />

          <div className="logs-filter-pills">
            <button
              type="button"
              className={
                robotFilter === 'ALL' &&
                resultFilter === 'ALL'
                  ? 'active'
                  : ''
              }
              onClick={() => {
                setRobotFilter('ALL')
                setResultFilter('ALL')
              }}
            >
              전체
            </button>

            {/* 로봇 버튼은 서버가 준 목록에서 만든다.
                예전에는 AMR-01 / AMR-02 두 개가 통째로 적혀 있어서,
                로봇이 늘어도 그 로그를 걸러 볼 방법이 없었다. */}
            {robots.map((robotId) => (
              <button
                key={robotId}
                type="button"
                className={
                  robotFilter === robotId
                    ? 'active'
                    : ''
                }
                onClick={() =>
                  setRobotFilter(robotId)
                }
              >
                {robotId}
              </button>
            ))}

            <button
              type="button"
              className={
                resultFilter === failedValue
                  ? 'failed active'
                  : 'failed'
              }
              onClick={() =>
                setResultFilter(
                  resultFilter === failedValue
                    ? 'ALL'
                    : failedValue,
                )
              }
            >
              실패만
            </button>
          </div>
        </div>

        <div className="logs-table-wrapper">
          <table className="logs-table">
            <thead>
              <tr>
                <th>시간</th>
                <th>AMR</th>
                <th>작업</th>
                <th>대상</th>
                <th>결과</th>
                <th>소요시간</th>
              </tr>
            </thead>

            <tbody>
              {filteredLogs.length === 0 ? (
                <tr>
                  <td
                    className="logs-empty-cell"
                    colSpan="6"
                  >
                    표시할 로그가 없습니다.
                  </td>
                </tr>
              ) : (
                filteredLogs.map((log) => (
                  <tr
                    key={log.id}
                    className={
                      log.result === failedValue
                        ? 'failed'
                        : ''
                    }
                  >
                    <td>{log.timestamp}</td>
                    <td>{log.robot_id}</td>
                    <td>{log.task_type}</td>
                    <td>{log.target_id}</td>
                    <td>
                      <span
                        className={
                          log.result === successValue
                            ? 'logs-result-badge success'
                            : log.result === failedValue
                              ? 'logs-result-badge failed'
                              : 'logs-result-badge neutral'
                        }
                      >
                        {log.result}
                      </span>
                    </td>
                    <td>{log.duration}</td>
                  </tr>
                ))
              )}
            </tbody>
          </table>
        </div>

        <div className="logs-table-footer">
          총 {logs.length}건 중{' '}
          {filteredLogs.length}건 표시
        </div>
      </section>
    </section>
  )
}

function SummaryCard({
  label,
  value,
}) {
  return (
    <article className="logs-summary-card">
      <span>{label}</span>
      <strong>{value}</strong>
    </article>
  )
}

export default LogsPage

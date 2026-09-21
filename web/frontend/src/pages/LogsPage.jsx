import { useState } from 'react'

import {
  requestLogs,
} from '../api/logs'

import useApiRequest from '../hooks/useApiRequest'

function getLogSummary(logs) {
  const total = logs.length

  const completed =
    logs.filter(
      (log) =>
        log.result === 'SUCCESS',
    ).length

  const failed =
    logs.filter(
      (log) =>
        log.result === 'FAILED',
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

  const summary =
    getLogSummary(logs)

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
                  ? 'Backend 연결됨'
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

            <button
              type="button"
              className={
                robotFilter === 'AMR-01'
                  ? 'active'
                  : ''
              }
              onClick={() =>
                setRobotFilter('AMR-01')
              }
            >
              AMR-01
            </button>

            <button
              type="button"
              className={
                robotFilter === 'AMR-02'
                  ? 'active'
                  : ''
              }
              onClick={() =>
                setRobotFilter('AMR-02')
              }
            >
              AMR-02
            </button>

            <button
              type="button"
              className={
                resultFilter === 'FAILED'
                  ? 'failed active'
                  : 'failed'
              }
              onClick={() =>
                setResultFilter(
                  resultFilter === 'FAILED'
                    ? 'ALL'
                    : 'FAILED',
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
                      log.result === 'FAILED'
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
                          log.result === 'SUCCESS'
                            ? 'logs-result-badge success'
                            : log.result === 'FAILED'
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

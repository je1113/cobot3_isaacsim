import {
  Fragment,
  useEffect,
  useState,
} from 'react'
import { NavLink } from 'react-router-dom'

const ROUTING_RULES_STORAGE_KEY =
  'magazine-ops:routingRules'

const LEGACY_ROUTING_STEPS_KEY =
  'magazine-ops:routingSteps'

const DEFAULT_ROUTING_RULES = [
  {
    rule_id: 'rule-1',
    from: 'SHELF',
    to: 'PKG-01',
    description:
      '매거진 스캔 후 이동',
  },
  {
    rule_id: 'rule-2',
    from: 'PKG-01',
    to: 'TEST-01',
    description:
      '패키징 완료 후 검사 이동',
  },
]

function createRuleId() {
  return [
    'rule',
    Date.now(),
    Math.random()
      .toString(36)
      .slice(2, 8),
  ].join('-')
}

function migrateLegacySteps(
  steps,
) {
  if (
    !Array.isArray(steps) ||
    steps.length < 2
  ) {
    return null
  }

  const rules = []

  for (
    let index = 0;
    index < steps.length - 1;
    index += 1
  ) {
    const current =
      steps[index]

    const next =
      steps[index + 1]

    if (!current || !next) {
      continue
    }

    const legacyDescription =
      typeof current
        .rule_description ===
        'string'
        ? current
            .rule_description
            .trim()
        : ''

    const label =
      typeof current.label ===
      'string'
        ? current.label.trim()
        : ''

    rules.push({
      rule_id:
        `legacy-rule-${index + 1}`,
      from:
        current.station_id ?? '',
      to:
        next.station_id ?? '',
      description:
        legacyDescription ||
        label,
    })
  }

  return rules.length > 0
    ? rules
    : null
}

function loadRoutingRules() {
  try {
    const storedRules =
      localStorage.getItem(
        ROUTING_RULES_STORAGE_KEY,
      )

    if (storedRules) {
      const parsed =
        JSON.parse(
          storedRules,
        )

      if (Array.isArray(parsed)) {
        return parsed
      }
    }

    const legacySteps =
      localStorage.getItem(
        LEGACY_ROUTING_STEPS_KEY,
      )

    if (legacySteps) {
      const parsedLegacy =
        JSON.parse(
          legacySteps,
        )

      const migrated =
        migrateLegacySteps(
          parsedLegacy,
        )

      if (migrated) {
        return migrated
      }
    }
  } catch {
    return DEFAULT_ROUTING_RULES
  }

  return DEFAULT_ROUTING_RULES
}

function ProcessFlowPage({
  stations = [],
  finalDestination,
  setFinalDestination,
}) {
  const [
    routingRules,
    setRoutingRules,
  ] = useState(
    loadRoutingRules,
  )

  useEffect(() => {
    localStorage.setItem(
      ROUTING_RULES_STORAGE_KEY,
      JSON.stringify(
        routingRules,
      ),
    )
  }, [routingRules])

  const registeredStationIds =
    new Set(
      stations.map(
        (station) =>
          station.station_id,
      ),
    )

  function isRegisteredStation(
    stationId,
  ) {
    return registeredStationIds.has(
      stationId,
    )
  }

  function isValidFrom(
    stationId,
  ) {
    return (
      stationId === 'SHELF' ||
      isRegisteredStation(
        stationId,
      )
    )
  }

  function isValidTo(
    stationId,
  ) {
    return isRegisteredStation(
      stationId,
    )
  }

  const rulesExist =
    routingRules.length > 0

  const validStart =
    rulesExist &&
    routingRules[0].from ===
      'SHELF'

  const stationLinksValid =
    rulesExist &&
    routingRules.every(
      (rule) =>
        rule.from.trim() !== '' &&
        rule.to.trim() !== '' &&
        isValidFrom(
          rule.from,
        ) &&
        isValidTo(
          rule.to,
        ),
    )

  const continuityValid =
    rulesExist &&
    routingRules.every(
      (rule, index) => {
        if (index === 0) {
          return (
            rule.from ===
            'SHELF'
          )
        }

        return (
          rule.from ===
          routingRules[
            index - 1
          ].to
        )
      },
    )

  const descriptionsComplete =
    rulesExist &&
    routingRules.every(
      (rule) =>
        rule.description
          .trim() !== '',
    )

  const destinationComplete =
    finalDestination !== ''

  const routingComplete =
    rulesExist &&
    validStart &&
    stationLinksValid &&
    continuityValid &&
    descriptionsComplete &&
    destinationComplete

  const chainNodes = []

  if (rulesExist) {
    chainNodes.push({
      node_id: 'start-node',
      station_id:
        routingRules[0].from,
      description: '시작점',
      incoming_rule_id: null,
    })

    routingRules.forEach(
      (rule, index) => {
        chainNodes.push({
          node_id:
            `${rule.rule_id}-target`,
          station_id:
            rule.to ||
            '미설정',
          description:
            rule.description ||
            '규칙 설명 미입력',
          incoming_rule_id:
            rule.rule_id,
          rule_index:
            index,
        })
      },
    )
  }

  function getStation(
    stationId,
  ) {
    return stations.find(
      (station) =>
        station.station_id ===
        stationId,
    )
  }

  function updateRuleTo(
    ruleId,
    value,
  ) {
    setRoutingRules(
      (prev) => {
        const index =
          prev.findIndex(
            (rule) =>
              rule.rule_id ===
              ruleId,
          )

        if (index === -1) {
          return prev
        }

        return prev.map(
          (rule, ruleIndex) => {
            if (
              ruleIndex ===
              index
            ) {
              return {
                ...rule,
                to: value,
              }
            }

            if (
              ruleIndex ===
              index + 1
            ) {
              return {
                ...rule,
                from: value,
              }
            }

            return rule
          },
        )
      },
    )
  }

  function updateRuleDescription(
    ruleId,
    value,
  ) {
    setRoutingRules(
      (prev) =>
        prev.map((rule) =>
          rule.rule_id ===
          ruleId
            ? {
                ...rule,
                description:
                  value,
              }
            : rule,
        ),
    )
  }

  function addRule() {
    setRoutingRules(
      (prev) => {
        const lastRule =
          prev[
            prev.length - 1
          ]

        const from =
          lastRule
            ? lastRule.to
            : 'SHELF'

        return [
          ...prev,
          {
            rule_id:
              createRuleId(),
            from,
            to: '',
            description: '',
          },
        ]
      },
    )
  }

  function deleteRule(
    ruleId,
  ) {
    setRoutingRules(
      (prev) => {
        const removeIndex =
          prev.findIndex(
            (rule) =>
              rule.rule_id ===
              ruleId,
          )

        if (
          removeIndex === -1
        ) {
          return prev
        }

        const next =
          prev.filter(
            (rule) =>
              rule.rule_id !==
              ruleId,
          )

        if (
          removeIndex <
          next.length
        ) {
          const newFrom =
            removeIndex === 0
              ? 'SHELF'
              : next[
                  removeIndex -
                    1
                ].to

          next[
            removeIndex
          ] = {
            ...next[
              removeIndex
            ],
            from: newFrom,
          }
        }

        return next
      },
    )
  }

  function saveRouting() {
    if (!routingComplete) {
      window.alert(
        '라우팅 규칙의 연결 상태, 규칙 설명, 최종 목적지를 확인하세요.',
      )
      return
    }

    window.alert(
      '라우팅 저장은 Backend 연동 단계에서 연결합니다.',
    )
  }

  const lastProcess =
    routingRules.length > 0
      ? routingRules[
          routingRules.length -
            1
        ].to
      : '-'

  return (
    <section className="process-flow-page">
      <header className="settings-page-header process-page-header">
        <div>
          <h1>설정</h1>

          <p>
            스테이션 간 공정 라우팅 규칙을 설정합니다.
          </p>
        </div>
      </header>

      <nav className="settings-tabs">
        <NavLink
          to="/settings/shelf"
          className={({ isActive }) =>
            isActive
              ? 'settings-tab active'
              : 'settings-tab'
          }
        >
          선반
        </NavLink>

        <NavLink
          to="/settings/station"
          className={({ isActive }) =>
            isActive
              ? 'settings-tab active'
              : 'settings-tab'
          }
        >
          스테이션
        </NavLink>

        <NavLink
          to="/settings/process"
          className={({ isActive }) =>
            isActive
              ? 'settings-tab active'
              : 'settings-tab'
          }
        >
          공정 흐름
        </NavLink>
      </nav>

      <div className="process-flow-panel process-flow-quality-panel">
        <div className="process-flow-top">
          <div>
            <h2>
              라우팅 규칙
            </h2>

            <p>
              공정 순서와 이동 규칙의 의미를 설정합니다.
            </p>
          </div>

          <span
            className={
              routingComplete
                ? 'routing-status complete'
                : 'routing-status incomplete'
            }
          >
            {routingComplete
              ? '라우팅 설정 완료'
              : '라우팅 설정 미완료'}
          </span>
        </div>

        <section className="routing-chain-section">
          <div className="routing-chain-heading">
            <h3>
              현재 공정 체인
            </h3>

            <span>
              등록된 규칙{' '}
              {routingRules.length}개
            </span>
          </div>

          {routingRules.length ===
          0 ? (
            <div className="routing-empty-chain">
              등록된 라우팅 규칙이 없습니다.
            </div>
          ) : (
            <div className="dynamic-process-chain routing-rules-chain">
              {chainNodes.map(
                (
                  node,
                  index,
                ) => {
                  const station =
                    getStation(
                      node.station_id,
                    )

                  return (
                    <Fragment
                      key={
                        node.node_id
                      }
                    >
                      <article className="process-chain-node routing-chain-node">
                        <span className="process-step-number">
                          {index + 1}
                        </span>

                        <div className="routing-chain-node-content">
                          <strong>
                            {
                              node.station_id
                            }
                          </strong>

                          {station && (
                            <small>
                              {
                                station.station_type ||
                                '유형 미설정'
                              }
                            </small>
                          )}

                          <span>
                            {
                              node.description
                            }
                          </span>
                        </div>
                      </article>

                      {index <
                        chainNodes.length -
                          1 && (
                        <span className="process-chain-arrow">
                          →
                        </span>
                      )}
                    </Fragment>
                  )
                },
              )}

              <span className="process-chain-arrow">
                →
              </span>

              <article
                className={
                  finalDestination
                    ? 'process-chain-node destination-node complete routing-chain-node'
                    : 'process-chain-node destination-node routing-chain-node'
                }
              >
                <span className="process-step-number">
                  {chainNodes.length +
                    1}
                </span>

                <div className="process-final-node">
                  <small>
                    최종 목적지
                  </small>

                  <select
                    value={
                      finalDestination
                    }
                    onChange={(
                      event,
                    ) =>
                      setFinalDestination(
                        event.target
                          .value,
                      )
                    }
                  >
                    <option value="">
                      선택
                    </option>

                    <option value="SHELF">
                      SHELF
                    </option>

                    <option value="출하">
                      출하
                    </option>
                  </select>
                </div>
              </article>
            </div>
          )}
        </section>

        <section className="routing-rule-section">
          <div className="routing-rule-section-header">
            <div>
              <h3>
                라우팅 규칙
              </h3>

              <p>
                출발지와 도착지 사이의 이동 규칙을 관리합니다.
              </p>
            </div>
          </div>

          <div className="routing-rule-list">
            {routingRules.map(
              (
                rule,
                index,
              ) => (
                <article
                  className="routing-rule-row routing-rule-record"
                  key={
                    rule.rule_id
                  }
                >
                  <span className="routing-rule-number">
                    {String(
                      index + 1,
                    ).padStart(
                      2,
                      '0',
                    )}
                  </span>

                  <div className="routing-rule-node routing-rule-readonly">
                    <small>
                      출발
                    </small>

                    <strong>
                      {rule.from ||
                        '미설정'}
                    </strong>
                  </div>

                  <span className="routing-rule-arrow">
                    →
                  </span>

                  <label className="routing-rule-destination-select">
                    <span>
                      도착
                    </span>

                    <select
                      value={
                        rule.to
                      }
                      onChange={(
                        event,
                      ) =>
                        updateRuleTo(
                          rule.rule_id,
                          event.target
                            .value,
                        )
                      }
                    >
                      <option value="">
                        스테이션 선택
                      </option>

                      {rule.to &&
                        !isRegisteredStation(
                          rule.to,
                        ) && (
                          <option
                            value={
                              rule.to
                            }
                            disabled
                          >
                            미등록 ·{' '}
                            {rule.to}
                          </option>
                        )}

                      {stations.map(
                        (
                          station,
                        ) => (
                          <option
                            key={
                              station.station_id
                            }
                            value={
                              station.station_id
                            }
                          >
                            {
                              station.station_id
                            }
                          </option>
                        ),
                      )}
                    </select>
                  </label>

                  <label className="routing-description-field">
                    <span>
                      규칙 설명
                    </span>

                    <input
                      type="text"
                      value={
                        rule.description
                      }
                      placeholder="이 이동 규칙이 하는 일을 입력하세요."
                      onChange={(
                        event,
                      ) =>
                        updateRuleDescription(
                          rule.rule_id,
                          event.target
                            .value,
                        )
                      }
                    />
                  </label>

                  <button
                    type="button"
                    className="routing-rule-delete-button"
                    onClick={() =>
                      deleteRule(
                        rule.rule_id,
                      )
                    }
                  >
                    삭제
                  </button>
                </article>
              ),
            )}
          </div>

          <button
            type="button"
            className="routing-add-rule-button"
            onClick={addRule}
          >
            + 규칙 추가
          </button>

          <div className="routing-guide-box">
            라우팅 규칙은 실제 등록된 스테이션 사이의 이동 관계입니다.
            {' '}
            규칙 설명을 수정하면 상단 공정 체인에도 즉시 반영됩니다.
            {' '}
            규칙을 삭제해도 설정에 등록된 스테이션 자체는 삭제되지 않습니다.
          </div>

          <div className="routing-validation-box routing-validation-five">
            <div>
              <span>
                규칙
              </span>

              <strong
                className={
                  rulesExist
                    ? 'valid'
                    : 'invalid'
                }
              >
                {rulesExist
                  ? `${routingRules.length}개`
                  : '추가 필요'}
              </strong>
            </div>

            <div>
              <span>
                시작점
              </span>

              <strong
                className={
                  validStart
                    ? 'valid'
                    : 'invalid'
                }
              >
                {validStart
                  ? 'SHELF 확인'
                  : '확인 필요'}
              </strong>
            </div>

            <div>
              <span>
                스테이션 연결
              </span>

              <strong
                className={
                  stationLinksValid &&
                  continuityValid
                    ? 'valid'
                    : 'invalid'
                }
              >
                {stationLinksValid &&
                continuityValid
                  ? '연결 정상'
                  : '확인 필요'}
              </strong>
            </div>

            <div>
              <span>
                규칙 설명
              </span>

              <strong
                className={
                  descriptionsComplete
                    ? 'valid'
                    : 'invalid'
                }
              >
                {descriptionsComplete
                  ? '입력 완료'
                  : '입력 필요'}
              </strong>
            </div>

            <div>
              <span>
                최종 목적지
              </span>

              <strong
                className={
                  destinationComplete
                    ? 'valid'
                    : 'invalid'
                }
              >
                {destinationComplete
                  ? finalDestination
                  : '선택 필요'}
              </strong>
            </div>
          </div>
        </section>

        <section className="routing-final-row">
          <div>
            <span>
              마지막 공정
            </span>

            <strong>
              {lastProcess || '-'}
            </strong>
          </div>

          <span className="routing-final-arrow">
            →
          </span>

          <label>
            <span>
              최종 목적지
            </span>

            <select
              value={
                finalDestination
              }
              onChange={(
                event,
              ) =>
                setFinalDestination(
                  event.target.value,
                )
              }
            >
              <option value="">
                선택
              </option>

              <option value="SHELF">
                SHELF
              </option>

              <option value="출하">
                출하
              </option>
            </select>
          </label>
        </section>

        <div className="process-flow-footer">
          <button
            type="button"
            className="process-save-button"
            onClick={
              saveRouting
            }
          >
            저장
          </button>
        </div>
      </div>
    </section>
  )
}

export default ProcessFlowPage

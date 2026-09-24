import {
  useEffect,
  useState,
} from 'react'
import {
  Navigate,
  NavLink,
  Route,
  Routes,
  useLocation,
} from 'react-router-dom'

import {
  fetchShelves,
  saveShelves,
  fetchStations,
  saveStations,
} from './api/config'
import { useConfigResource } from './hooks/useConfigResource'
import { useRobots } from './contexts/metaHooks'

import ShelfSettingsPage from './pages/ShelfSettingsPage'
import StationSettingsPage from './pages/StationSettingsPage'
import ProcessFlowPage from './pages/ProcessFlowPage'
import TaskAssignmentPage from './pages/TaskAssignmentPage'
import MonitoringPage from './pages/MonitoringPage'
import LogsPage from './pages/LogsPage'
import './App.css'

// ★ createInitialShelf / createInitialStation / createInitialScanPass 를 지웠다.
//   선반·스테이션의 초기값은 이제 서버(src/cobot3_bringup/config/*.yaml)가 준다.
//   빈 화면을 씨앗으로 채워 두면, 서버가 느리거나 실패했을 때 사람이
//   "설정이 지워졌다" 고 오해하고 그 위에 덮어 저장한다.
//   새 항목을 만드는 것은 각 설정 페이지가 한다(createShelf / createStation).

// useConfigResource 가 응답에서 편집 대상만 꺼내는 셀렉터.
// 모듈 스코프에 두는 이유: 컴포넌트 안에서 만들면 매 렌더마다 새 함수가 되어
// 훅의 useCallback 이 다시 만들어지고, 그러면 load 가 무한히 다시 돈다.
// 로딩 중에 사용자가 '추가' 를 눌렀을 때 setValue 가 넘길 기본값.
// 모듈 스코프 상수여야 한다 — 렌더마다 새 배열을 만들면 useCallback 이 깨진다.
const EMPTY_LIST = []

function pickShelves(res) {
  return res.shelves ?? []
}

function pickStations(res) {
  return res.stations ?? []
}

function loadStoredState(
  key,
  initialValue,
) {
  const storedValue =
    localStorage.getItem(key)

  if (storedValue === null) {
    return initialValue
  }

  try {
    return JSON.parse(storedValue)
  } catch {
    return initialValue
  }
}

/**
 * 설정이 서버에서 오는지, 못 오고 있는지를 한 줄로 보여준다.
 *
 * ★ 실패를 조용히 삼키면 빈 목록이 그려지고, 사람은 "설정이 지워졌다" 고
 *   오해한 뒤 그 위에 덮어 저장한다. 그게 가장 비싼 실패다.
 * ★ 503(DB·ROS 미연결)은 '고장' 이 아니라 '아직 안 붙음' 이라 따로 말한다 —
 *   설정 화면 자체는 yaml 만 쓰므로 그 상태에서도 멀쩡히 동작한다.
 */
function ConfigBanner({
  shelves,
  stations,
}) {
  const failing = [shelves, stations].find(
    (r) => r.status === 'error',
  )

  if (failing) {
    return (
      <div className="config-banner error">
        <span>
          설정을 불러오지 못했다 —{' '}
          {failing.error?.message ??
            '알 수 없는 오류'}
        </span>

        <button
          type="button"
          onClick={() => {
            shelves.reload()
            stations.reload()
          }}
        >
          다시 시도
        </button>
      </div>
    )
  }

  if (
    shelves.status === 'loading' ||
    stations.status === 'loading'
  ) {
    return (
      <div className="config-banner">
        설정을 불러오는 중…
      </div>
    )
  }

  if (shelves.stale || stations.stale) {
    return (
      <div className="config-banner error">
        <span>
          다른 곳에서 먼저 저장됐다. 덮어쓰지 않았으니 다시 불러온 뒤 편집할 것.
        </span>

        <button
          type="button"
          onClick={() => {
            shelves.reload()
            stations.reload()
          }}
        >
          다시 불러오기
        </button>
      </div>
    )
  }

  if (shelves.dirty || stations.dirty) {
    return (
      <div className="config-banner warn">
        저장하지 않은 변경이 있다. 각 화면의 저장 버튼을 누를 것.
      </div>
    )
  }

  return null
}

// 나브바가 접혔을 때(아이콘만 보임) 메뉴를 구분할 아이콘.
// 별도 아이콘 라이브러리를 안 쓰므로 직접 그린 최소한의 SVG다.
function IconSettings() {
  return (
    <svg
      viewBox="0 0 24 24"
      width="18"
      height="18"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
    >
      <line x1="4" y1="6" x2="20" y2="6" />
      <line x1="4" y1="12" x2="20" y2="12" />
      <line x1="4" y1="18" x2="20" y2="18" />
      <circle cx="9" cy="6" r="2" fill="currentColor" stroke="none" />
      <circle cx="15" cy="12" r="2" fill="currentColor" stroke="none" />
      <circle cx="9" cy="18" r="2" fill="currentColor" stroke="none" />
    </svg>
  )
}

function IconTasks() {
  return (
    <svg
      viewBox="0 0 24 24"
      width="18"
      height="18"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect x="5" y="4" width="14" height="17" rx="2" />
      <line x1="9" y1="9" x2="15" y2="9" />
      <line x1="9" y1="13" x2="15" y2="13" />
      <line x1="9" y1="17" x2="13" y2="17" />
    </svg>
  )
}

function IconMonitor() {
  return (
    <svg
      viewBox="0 0 24 24"
      width="18"
      height="18"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect x="3" y="4" width="18" height="12" rx="2" />
      <line x1="8" y1="20" x2="16" y2="20" />
      <line x1="12" y1="16" x2="12" y2="20" />
    </svg>
  )
}

function IconLogs() {
  return (
    <svg
      viewBox="0 0 24 24"
      width="18"
      height="18"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <line x1="5" y1="20" x2="5" y2="12" />
      <line x1="12" y1="20" x2="12" y2="8" />
      <line x1="19" y1="20" x2="19" y2="14" />
    </svg>
  )
}

function App() {
  const location = useLocation()

  // 설정의 주인은 서버의 yaml 파일이다(docs/DB구성.md §10-1).
  // 타이핑할 때마다 저장하지 않는다 — 각 페이지의 '저장' 버튼이 save() 를 부른다.
  const shelvesRes = useConfigResource(
    fetchShelves,
    saveShelves,
    pickShelves,
    EMPTY_LIST,
  )

  const stationsRes = useConfigResource(
    fetchStations,
    saveStations,
    pickStations,
    EMPTY_LIST,
  )

  const robots = useRobots()

  // 나브바 접힘 — 화면 로컬이라 새로고침해도 유지되게 localStorage 에 둔다.
  const [sidebarCollapsed, setSidebarCollapsed] =
    useState(() =>
      loadStoredState(
        'magazine-ops:sidebar-collapsed',
        false,
      ),
    )

  useEffect(() => {
    localStorage.setItem(
      'magazine-ops:sidebar-collapsed',
      JSON.stringify(sidebarCollapsed),
    )
  }, [sidebarCollapsed])

  const shelves = shelvesRes.value ?? []
  const setShelves = shelvesRes.setValue
  const stations = stationsRes.value ?? []
  const setStations = stationsRes.setValue

  // finalDestination 은 routing 리소스의 일부라 ProcessFlowPage 가 직접 갖는다.
  // 규칙과 최종 목적지는 "완성됐는가" 판정을 같이 받으므로 한 리소스다.

  // ★ 큐를 로봇별 useState 두 개로 두면 대수가 코드 구조에 박힌다.
  //   robots 를 키로 하는 map 하나면 3대가 돼도 이 파일은 안 바뀐다.
  const [queues, setQueues] =
    useState(() =>
      loadStoredState(
        'magazine-ops:queues',
        {},
      ),
    )

  // 로봇 큐는 아직 화면 로컬이다. '작업 시작' 을 누르는 순간에만 서버로 간다
  // (POST /api/tasks/start) — 그때 서버의 대기 큐가 이 목록으로 교체된다.
  useEffect(() => {
    localStorage.setItem(
      'magazine-ops:queues',
      JSON.stringify(queues),
    )
  }, [queues])

  // 실시간 상태는 어디에도 저장하지 않는다 — WebSocket 이 밀어 주는 대로만 산다.
  // 시드도 로봇 목록에서 만든다(예전에는 'AMR-01'/'AMR-02' 를 적어 뒀다).
  const [robotStates, setRobotStates] =
    useState(() =>
      Object.fromEntries(
        robots.map((robotId) => [
          robotId,
          {
            robot_id: robotId,
            current_state: null,
            current_task: null,
            position: null,
          },
        ]),
      ),
    )

  const settingsActive =
    location.pathname.startsWith('/settings')

  return (
    <div
      className={
        sidebarCollapsed
          ? 'app sidebar-collapsed'
          : 'app'
      }
    >
      <aside className="sidebar">
        <div className="sidebar-header">
          {!sidebarCollapsed && (
            <h2 className="sidebar-brand">
              MAGAZINE OPS
            </h2>
          )}

          <button
            type="button"
            className="sidebar-toggle"
            onClick={() =>
              setSidebarCollapsed(
                (collapsed) => !collapsed,
              )
            }
            aria-label={
              sidebarCollapsed
                ? '나브바 펼치기'
                : '나브바 접기'
            }
            title={
              sidebarCollapsed
                ? '나브바 펼치기'
                : '나브바 접기'
            }
          >
            {sidebarCollapsed ? '›' : '‹'}
          </button>
        </div>

        <nav>
          <NavLink
            to="/settings/shelf"
            className={
              settingsActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
            title="설정"
          >
            <span className="sidebar-link-icon">
              <IconSettings />
            </span>
            <span className="sidebar-link-text">
              설정
            </span>
          </NavLink>

          <NavLink
            to="/assignment"
            className={({ isActive }) =>
              isActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
            title="작업 할당"
          >
            <span className="sidebar-link-icon">
              <IconTasks />
            </span>
            <span className="sidebar-link-text">
              작업 할당
            </span>
          </NavLink>

          <NavLink
            to="/monitoring"
            className={({ isActive }) =>
              isActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
            title="실시간 모니터링"
          >
            <span className="sidebar-link-icon">
              <IconMonitor />
            </span>
            <span className="sidebar-link-text">
              실시간 모니터링
            </span>
          </NavLink>

          <NavLink
            to="/logs"
            className={({ isActive }) =>
              isActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
            title="로그 · 통계"
          >
            <span className="sidebar-link-icon">
              <IconLogs />
            </span>
            <span className="sidebar-link-text">
              로그 · 통계
            </span>
          </NavLink>
        </nav>
      </aside>

      <main className="content">
        <ConfigBanner
          shelves={shelvesRes}
          stations={stationsRes}
        />

        <Routes>
          <Route
            path="/"
            element={
              <Navigate
                to="/settings/shelf"
                replace
              />
            }
          />

          <Route
            path="/settings/shelf"
            element={
              <ShelfSettingsPage
                shelves={shelves}
                setShelves={setShelves}
                resource={shelvesRes}
              />
            }
          />

          <Route
            path="/settings/station"
            element={
              <StationSettingsPage
                stations={stations}
                setStations={setStations}
                resource={stationsRes}
              />
            }
          />

          <Route
            path="/settings/process"
            element={
              <ProcessFlowPage
                stations={stations}
              />
            }
          />

          <Route
            path="/assignment"
            element={
              <TaskAssignmentPage
                shelves={shelves}
                stations={stations}
                queues={queues}
                setQueues={setQueues}
              />
            }
          />

          <Route
            path="/monitoring"
            element={
              <MonitoringPage
                robotStates={robotStates}
                setRobotStates={setRobotStates}
                queues={queues}
              />
            }
          />

          <Route
            path="/logs"
            element={<LogsPage />}
          />
        </Routes>
      </main>
    </div>
  )
}

export default App

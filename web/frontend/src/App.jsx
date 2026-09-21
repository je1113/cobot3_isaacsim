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

import ShelfSettingsPage from './pages/ShelfSettingsPage'
import StationSettingsPage from './pages/StationSettingsPage'
import ProcessFlowPage from './pages/ProcessFlowPage'
import TaskAssignmentPage from './pages/TaskAssignmentPage'
import MonitoringPage from './pages/MonitoringPage'
import LogsPage from './pages/LogsPage'
import './App.css'

function createInitialScanPass(level) {
  return {
    pass_id: level,
    level,
    direction: level % 2 === 1 ? 'FORWARD' : 'BACKWARD',
    arm_teach_pose: ['', '', '', '', '', ''],
  }
}

function createInitialShelf(shelfId, passCount) {
  return {
    shelf_id: shelfId,
    waypoint_start: {
      x: '',
      y: '',
      theta: '',
    },
    waypoint_end: {
      x: '',
      y: '',
      theta: '',
    },
    standoff_distance: '',
    scan_passes: Array.from(
      { length: passCount },
      (_, index) => createInitialScanPass(index + 1),
    ),
  }
}

function createInitialStation(
  stationId,
  stationType = '',
  processTime = '',
) {
  return {
    station_id: stationId,
    station_type: stationType,
    place_pose: {
      x: '',
      y: '',
      theta: '',
    },
    place_arm_pose: ['', '', '', '', '', ''],
    process_time: processTime,
    output_type: '',
    completion_signal: '',
    capacity: '',
  }
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

function App() {
  const location = useLocation()

  const [shelves, setShelves] =
    useState(() =>
      loadStoredState(
        'magazine-ops:shelves',
        [
          createInitialShelf(
            'SHELF-A',
            3,
          ),
          createInitialShelf(
            'SHELF-B',
            2,
          ),
          createInitialShelf(
            'SHELF-C',
            3,
          ),
          createInitialShelf(
            'SHELF-G',
            0,
          ),
        ],
      ),
    )

  const [stations, setStations] =
    useState(() =>
      loadStoredState(
        'magazine-ops:stations',
        [
          createInitialStation(
            'PKG-01',
            'PACKAGING',
            '60',
          ),
          createInitialStation(
            'TEST-01',
            'TEST',
          ),
        ],
      ),
    )

  const [
    finalDestination,
    setFinalDestination,
  ] = useState(() =>
    loadStoredState(
      'magazine-ops:finalDestination',
      '',
    ),
  )

  const [
    robot1Queue,
    setRobot1Queue,
  ] = useState(() =>
    loadStoredState(
      'magazine-ops:robot1Queue',
      [],
    ),
  )

  const [
    robot2Queue,
    setRobot2Queue,
  ] = useState(() =>
    loadStoredState(
      'magazine-ops:robot2Queue',
      [],
    ),
  )

  useEffect(() => {
    localStorage.setItem(
      'magazine-ops:shelves',
      JSON.stringify(shelves),
    )

    localStorage.setItem(
      'magazine-ops:stations',
      JSON.stringify(stations),
    )

    localStorage.setItem(
      'magazine-ops:finalDestination',
      JSON.stringify(
        finalDestination,
      ),
    )

    localStorage.setItem(
      'magazine-ops:robot1Queue',
      JSON.stringify(robot1Queue),
    )

    localStorage.setItem(
      'magazine-ops:robot2Queue',
      JSON.stringify(robot2Queue),
    )
  }, [
    shelves,
    stations,
    finalDestination,
    robot1Queue,
    robot2Queue,
  ])

  const [
    robotStates,
    setRobotStates,
  ] = useState({
    'AMR-01': {
      robot_id: 'AMR-01',
      current_state: null,
      current_task: null,
      position: null,
    },

    'AMR-02': {
      robot_id: 'AMR-02',
      current_state: null,
      current_task: null,
      position: null,
    },
  })

  const settingsActive =
    location.pathname.startsWith('/settings')

  return (
    <div className="app">
      <aside className="sidebar">
        <h2 className="sidebar-brand">
          MAGAZINE OPS
        </h2>

        <nav>
          <NavLink
            to="/settings/shelf"
            className={
              settingsActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
          >
            설정
          </NavLink>

          <NavLink
            to="/assignment"
            className={({ isActive }) =>
              isActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
          >
            작업 할당
          </NavLink>

          <NavLink
            to="/monitoring"
            className={({ isActive }) =>
              isActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
          >
            실시간 모니터링
          </NavLink>

          <NavLink
            to="/logs"
            className={({ isActive }) =>
              isActive
                ? 'sidebar-link active'
                : 'sidebar-link'
            }
          >
            로그 · 통계
          </NavLink>
        </nav>
      </aside>

      <main className="content">
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
              />
            }
          />

          <Route
            path="/settings/station"
            element={
              <StationSettingsPage
                stations={stations}
                setStations={setStations}
              />
            }
          />

          <Route
            path="/settings/process"
            element={
              <ProcessFlowPage
                stations={stations}
                finalDestination={
                  finalDestination
                }
                setFinalDestination={
                  setFinalDestination
                }
              />
            }
          />

          <Route
            path="/assignment"
            element={
              <TaskAssignmentPage
                shelves={shelves}
                stations={stations}
                robot1Queue={robot1Queue}
                setRobot1Queue={setRobot1Queue}
                robot2Queue={robot2Queue}
                setRobot2Queue={setRobot2Queue}
              />
            }
          />

          <Route
            path="/monitoring"
            element={
              <MonitoringPage
                shelves={shelves}
                stations={stations}
                robotStates={robotStates}
                setRobotStates={setRobotStates}
                robot1Queue={robot1Queue}
                robot2Queue={robot2Queue}
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

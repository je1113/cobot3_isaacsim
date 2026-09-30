# rokey_cobot3 — AMR 매거진 이송 관제 (Isaac Sim)

Isaac Sim 위의 가상 공장에서 **모바일 매니퓰레이터(Nova Carter + Doosan M0609) 2대**가
선반을 순찰하며 QR 로 매거진을 찾아 집고, 공정 스테이션으로 옮겨 놓은 뒤,
산출물(스택)을 회수해 검사 스테이션까지 나르는 시스템이다.
웹 화면에서 작업을 배정하고 진행 상황과 이력을 본다.

## 1. 시스템 설계

### 1-1. 구성도

```mermaid
flowchart LR
    subgraph WEB["웹 관제"]
        FE["React 프론트<br/>(Vite :5173)"]
        BE["FastAPI 백엔드<br/>(:8000)"]
        DB[("PostgreSQL")]
    end

    subgraph ROS["ROS 2 Jazzy — 로봇 네임스페이스마다 한 벌 (/robot1, /robot2)"]
        TM["task_manager<br/>(행동트리)"]
        NAV["nav_server"]
        PER["carrier_code_reader"]
        MAN["pick_place_server"]
        NAV2["Nav2 / AMCL"]
    end

    EL["event_logger<br/>(전역 1개)"]

    subgraph SIM["Isaac Sim 5.1"]
        SB["sim_backend.py<br/>(JSON-RPC :8765)"]
        BR["ROS2 Bridge<br/>(OmniGraph)"]
    end

    FE -- "REST /api · WebSocket /ws" --> BE
    BE -- "읽기/쓰기" --> DB
    DB -- "NOTIFY trace_appended" --> BE
    BE -- "ExecuteTask 액션<br/>RobotCommand · ReloadConfig 서비스" --> TM

    TM -- "NavigateTo" --> NAV
    TM -- "CarrierScan" --> PER
    TM -- "PickCarrier / PlaceCarrier" --> MAN
    TM -- "/trace/event" --> EL
    EL -- "INSERT" --> DB

    NAV -- "cmd_vel" --> BR
    NAV2 <-- "/clock · odom · lidar · TF" --> BR
    PER -- "JSON-RPC" --> SB
    MAN -- "JSON-RPC" --> SB
```

- **Isaac Sim 과 ROS 는 프로세스가 분리돼 있다.** Isaac 의 kit 파이썬(3.11)에서는 시스템
  ROS 2 Jazzy(3.12)의 `rclpy` 를 쓸 수 없어서, 팔 동작·카메라 캡처는 TCP JSON-RPC 로,
  주행 관련 토픽(`/clock`·odom·lidar·TF)은 Isaac 의 ROS2 Bridge 로 주고받는다.
- **네임스페이스가 로봇 식별자다.** `/robot1` 의 `task_manager` 에게는 `/robot1` 의 서버만 보인다.
- **설정은 파일(yaml)이, 이력은 DB 가 주인이다.** 실시간 상태(위치 등)는 저장하지 않고
  WebSocket 으로 흘려보낸다. 자세한 내용은 [`web/backend/README.md`](web/backend/README.md).

### 1-2. 미션 플로우 차트

```mermaid
flowchart TD
    S([도크에서 출발]) --> P["선반 순찰<br/>(팔을 관측 자세로)"]
    P --> D{QR 감지?}
    D -- 아니오 --> P
    D -- 예 --> H["정지 (HOLD)"]
    H --> SC["QR 판독 (CarrierScan)"]
    SC --> F{판독 성공?}
    F -- 아니오 --> P
    F -- 예 --> PK["손목캠 보정 후 흡착 pick"]
    PK --> N["공정 스테이션으로 주행"]
    N --> L{로더 차선이<br/>비었나?}
    L -- 아니오 --> W["대기 자리에서 양보"]
    W --> L
    L -- 예 --> PL["매거진 place"]
    PL --> ST{산출물 스택이<br/>있나?}
    ST -- 예 --> SP["스택 pick"]
    SP --> T["검사 스테이션에 place"]
    T --> R["선반으로 복귀"]
    ST -- 아니오 --> R
    R --> P
```

각 단계(`pick`·`nav`·`place`·`return`)는 `/trace/event` 로 발행되고 `event_logger` 가 DB 에 기록한다.

### 1-3. 디렉터리

| 경로 | 내용 |
|---|---|
| `src/cobot3_bringup` | launch 파일, 설정 yaml (`shelves`·`stations`·`grasp`·`place`·`frames` 등) |
| `src/cobot3_interfaces` | 액션·서비스·메시지 정의 |
| `src/cobot3_orchestrator` | `task_manager`(행동트리), `event_logger`(DB 기록) |
| `src/cobot3_navigation` | `nav_server`, Nav2 다중 로봇 launch·파라미터·지도 |
| `src/cobot3_perception` | `carrier_code_reader` (QR 판독·자세 추정) |
| `src/cobot3_manipulation` | `pick_place_server` |
| `isaacpjt/ros_bridge` | `sim_backend.py` 와 실행 스크립트 |
| `isaacpjt/worlds` | 공장 씬 (USD) |
| `isaacpjt/tools` | 씬 실측·티칭·QR 평가 도구 ([README](isaacpjt/tools/README.md)) |
| `web/backend`, `web/frontend` | FastAPI 백엔드, React 프론트 |
| `sql` | DB 스키마 (001 → 004 순서로 적용) |
| `docs` | 설계 문서 (`DB구성.md`, `ERD.md` 등) |

## 2. 운영체제 환경

| 항목 | 버전 |
|---|---|
| OS | Ubuntu 24.04 LTS (커널 6.14) |
| ROS | ROS 2 Jazzy |
| 시뮬레이터 | NVIDIA Isaac Sim 5.1.0 |
| Python | 3.12 (시스템·ROS) / 3.11 (Isaac Sim 내장) |
| Node.js | 22 |
| NVIDIA 드라이버 | 580 |
| DDS | `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=102` |

## 3. 사용한 장비 목록

### 개발 PC

| 항목 | 사양 |
|---|---|
| CPU | Intel Core Ultra 9 275HX |
| GPU | NVIDIA GeForce RTX 5080 Laptop (16 GB) |
| RAM | 64 GB |

### 시뮬레이션 장비 (Isaac Sim 내 가상 모델)

| 장비 | 용도 |
|---|---|
| NVIDIA Nova Carter | 자율주행 베이스 (AMR), 2대 |
| Doosan Robotics M0609 | 6축 협동로봇 팔, 베이스에 탑재 |
| 흡착 그리퍼 (Isaac Surface Gripper) | 매거진·스택 파지 |
| 손목 RGB-D 카메라 (1280×720) | QR 판독, 파지 자세 보정 |
| 2D 라이다 | Nav2 위치 추정·장애물 감지 |
| 선반 · 매거진 로더 · 검사 스테이션 | 작업 대상 설비 |

## 4. 의존성

### ROS 2 (apt)

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-py-trees ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros \
    python3-yaml python3-psycopg2 python3-opencv python3-numpy python3.12-venv
```

### Isaac Sim

- Isaac Sim 5.1.0 을 `~/isaacsim` 에 설치한다. 다른 경로면 `ISAAC_SIM_ROOT` 로 알려준다.
- 사용하는 확장: `isaacsim.ros2.bridge`, `isaacsim.robot.surface_gripper`

### 웹 백엔드 (`web/backend/requirements.txt`)

| 패키지 | 버전 |
|---|---|
| fastapi | 0.115.6 |
| uvicorn[standard] | 0.34.0 |
| psycopg[binary,pool] | 3.2.3 |
| pydantic | 2.10.4 |
| ruamel.yaml | 0.18.6 |

PostgreSQL 서버가 필요하다 (로컬·컨테이너 무관, DSN 만 맞으면 된다).

### 웹 프론트 (`web/frontend/package.json`)

React 19 · React Router 7 · Vite 8

## 5. 간단한 사용설명

### 5-1. 빌드

```bash
cd ~/cobot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 5-2. DB 준비

```bash
export COBOT3_DSN=postgresql://cobot3:cobot3@localhost:5432/cobot3
for f in sql/00*.sql; do psql "$COBOT3_DSN" -f "$f"; done
```

### 5-3. 실행 (터미널마다 하나씩, 이 순서로)

```bash
# 1) Isaac Sim + RPC 서버 — 반드시 이 스크립트로 띄운다
./isaacpjt/ros_bridge/run_sim_backend.sh
#    환경만 검사:   ./isaacpjt/ros_bridge/run_sim_backend.sh --check
#    시험용 씬:     SIM_WORLD_USD=isaacpjt/worlds/simple_factory_layout_test.usda ./isaacpjt/ros_bridge/run_sim_backend.sh

# 2) Nav2 (robot1 / robot2)
ros2 launch cobot3_navigation multi_navigation.launch.py

# 3) 팔 TF
ros2 launch cobot3_bringup tf.launch.py

# 4) 미션 노드
ros2 launch cobot3_bringup mission_nodes.launch.py robots:=robot1,robot2 \
    db_dsn:=$COBOT3_DSN 2>&1 | tee ~/mission.log
```

```bash
# 5) 웹 백엔드
cd web/backend
python3 -m venv --system-site-packages .venv && . .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env && set -a && . ./.env && set +a
uvicorn app.main:app --reload --port 8000

# 6) 웹 프론트
cd web/frontend
npm install
npm run dev
```

- 관제 화면: <http://localhost:5173>
- API 문서: <http://localhost:8000/docs> · 상태 확인: `GET /health`


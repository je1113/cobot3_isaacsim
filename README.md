# rokey_cobot3 — AMR 매거진 이송 관제 (Isaac Sim)
<img width="1209" height="666" alt="반도체 후공정 라인의 부품 캐리어 이송·피킹 시스템" src="https://github.com/user-attachments/assets/58a5e47f-88f0-45fe-ab8d-0c1ce779a04b" />


Isaac Sim 위의 가상 공장에서 **모바일 매니퓰레이터(Nova Carter + Doosan M0609) 2대**가
선반을 순찰하며 QR 로 매거진을 찾아 집고, 공정 스테이션으로 옮겨 놓은 뒤,
산출물(스택)을 회수해 검사 스테이션까지 나르는 시스템이다.
웹 화면에서 작업을 배정하고 진행 상황과 이력을 본다.

## 1. 시스템 설계

### 1-1. 구성도
<img width="1103" height="500" alt="image" src="https://github.com/user-attachments/assets/84bc4fe8-2564-49b6-9521-d6243079ccc5" />

<img width="1238" height="573" alt="image" src="https://github.com/user-attachments/assets/c9d4598c-c242-419c-a9c6-8dd42218aa64" />


- **Isaac Sim 과 ROS 는 프로세스가 분리돼 있다.** Isaac 의 kit 파이썬(3.11)에서는 시스템
  ROS 2 Jazzy(3.12)의 `rclpy` 를 쓸 수 없어서, 팔 동작·카메라 캡처는 TCP JSON-RPC 로,
  주행 관련 토픽(`/clock`·odom·lidar·TF)은 Isaac 의 ROS2 Bridge 로 주고받는다.
- **네임스페이스가 로봇 식별자다.** `/robot1` 의 `task_manager` 에게는 `/robot1` 의 서버만 보인다.
- **설정은 파일(yaml)이, 이력은 DB 가 주인이다.** 실시간 상태(위치 등)는 저장하지 않고
  WebSocket 으로 흘려보낸다. 자세한 내용은 [`web/backend/README.md`](web/backend/README.md).

### 1-2. 미션 플로우 차트
<img width="483" height="440" alt="image" src="https://github.com/user-attachments/assets/a5301487-a8fc-4a83-b2ed-b1d67250949d" />


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

## 6.핵심 모션
### 6-1. 주행 시 장애물 회피
<img width="720" height="385" alt="노드별기능및통신3_1_왼쪽_장애물회피" src="https://github.com/user-attachments/assets/a53878e0-475a-401a-aa7a-d044ba829e10" />


### 6-2. Patrol중 QR인식 후 정차
<img width="640" height="358" alt="노드별기능및통신2_패트롤중qr인식돼서멈춤_로봇팔시점" src="https://github.com/user-attachments/assets/4bd3f3fd-adbe-4d63-8258-0c24afbacaa7" />


### 6-3. QR인식 후 PICK
<img width="720" height="433" alt="3_노드별기능및통신3_2_왼쪽_매거진pick" src="https://github.com/user-attachments/assets/f10dc156-fdaf-474a-8182-e13995995ee0" />


### 6-4. place
<img width="720" height="385" alt="6_노드별기능및통신3_2_오른쪽_place" src="https://github.com/user-attachments/assets/5e114f2c-a84c-479e-93f8-f58d1f320805" />


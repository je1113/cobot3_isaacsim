cmake_minimum_required(VERSION 3.8)
project(cobot3_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build tools
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message dependencies
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 여기 없는 파일은 만들어도 생성되지 않는다. 에러도 안 난다. ──
  # /perception/carrier_detected 는 std_msgs/Bool 이라 여기에 없다.
  # 노드 그래프의 인터페이스 6개 중 5개가 이 패키지에 있다.
  # Messages
  "msg/TraceEvent.msg"        # task_manager → event_logger  (DB구성.md §9 반영판)
  "msg/RobotAlert.msg"        # task_manager → web           (payload 없는 사건 · §4-6 구멍)
  "msg/CycleLog.msg"          # 보류 — event 테이블에서 집계 가능
  # msg/TaskOrder.msg 는 삭제했다 — ExecuteTask.action 의 goal 에 인라인했다.
  # 소비자가 그 액션 하나뿐이라 중간 타입이 깊이만 더했다.

  # Services
  "srv/CarrierScan.srv"       # task_manager → carrier_code_reader (멈춘 뒤 판독 요청)
  "srv/RobotCommand.srv"      # web → task_manager            (PAUSE/RESUME/SKIP/STOP)
  "srv/ReloadConfig.srv"      # web → 설정 yaml 을 읽는 노드 전부
  # srv/RegisterTask.srv 는 삭제했다 — 08 문서가 예고한 그대로다.
  # 방향이 반대였다: 큐가 로봇에 있을 때의 '입구' 였는데, 큐가 웹으로
  # 올라가면서 필요한 것이 '출구'(ExecuteTask.action)로 바뀌었다.

  # Actions
  "action/NavigateTo.action"  # task_manager → nav_server
  "action/PickCarrier.action" # task_manager → pick_place_server
  "action/PlaceCarrier.action" # task_manager → pick_place_server
  "action/ExecuteTask.action" # web → task_manager            (WBS 0.3 · 4.7 · 4.8)

  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
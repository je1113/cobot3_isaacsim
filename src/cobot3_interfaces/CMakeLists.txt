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
  # Messages
  "msg/CarrierScan.msg"       # carrier_code_reader → task_manager
  "msg/TraceEvent.msg"        # task_manager → event_logger
  "msg/TaskOrder.msg"         # 보류 — 웹/MES 연동 시
  "msg/CycleLog.msg"          # 보류 — event 테이블에서 집계 가능

  # Services
  "srv/RegisterTask.srv"      # 보류 — 웹/MES 연동 시

  # Actions
  "action/NavigateTo.action"  # task_manager → nav_server
  "action/PickCarrier.action" # task_manager → pick_place_server
  "action/PlaceCarrier.action" # task_manager → pick_place_server
  "action/Dock.action"        # task_manager → docking_server

  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
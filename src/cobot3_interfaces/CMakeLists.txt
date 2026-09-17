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
  "msg/CarrierScan.msg"       # QR 판독 결과          05 §2-6
  "msg/TraceEvent.msg"        # 추적 이벤트 1건       05 §2-6
  "msg/BatteryState.msg"      # 배터리 방송           07 §6
  "msg/TaskOrder.msg"         # 보류 — 웹/MES 연동 시
  "msg/CycleLog.msg"          # 보류 — event 테이블에서 집계 가능

  # Services
  "srv/VerifyCarrier.srv"     # 파지·배치 인터록      05 §2-6
  "srv/RegisterTask.srv"      # 보류 — 웹/MES 연동 시

  # Actions
  "action/NavigateTo.action"  # 주행                  06 §6 개명
  "action/PickCarrier.action" # 흡착 파지             06 §6 개명 (흡착 모델)
  "action/PlaceCarrier.action" # 흡착 해제 배치       06 §6 개명 (흡착 모델)
  "action/Dock.action"        # 도킹                  06 §6

  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
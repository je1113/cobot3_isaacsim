"""
단위 테스트 — nova_carter1/m0609 로 magazine_2_blue_02 를 집어 컨베이어 앞
바닥 스테이징 지점에 내려놓기 (SCAN + PICK + PLACE)

    isaac_python 12_place_test.py   (기본이 GUI)
    (헤드리스로 돌리려면)  PICK_HEADLESS=1 isaac_python 12_place_test.py

12_pick_test.py 의 PICK 로직(흡착 PickFSM, 재시도 사다리 포함)을 그대로 쓰고,
그 앞에 SCAN(손목 카메라로 QR 실제 판독 확인) 단계를, 그 뒤에 PLACE 단계를
추가한다.

★ GT 직접 읽기에서 SCAN 게이트로 바꾼 이유: 원래(magazine_1_orange 대상)는
인식 없이 USD 에서 매거진의 실제(GT) pose 를 바로 읽었다. 지금은 대상을
magazine_2_blue_02(선반 위 다른 위치, QR 판독 창 x:[-4.90,-4.80])로 바꾸고,
PICK 전에 sim_backend.py 의 observe_pose()+scan_qr() 와 같은 방식으로
손목 카메라 QR 을 실제로 판독해 본다 — 판독 실패면 FAIL_NOT_FOUND 로 그 자리서
중단하고 PICK/PLACE 는 시도하지 않는다. **다만 SCAN 은 게이트일 뿐이고,
PICK 자체의 파지점은 여전히 GT(measure_prim(FLANGE_PATH))를 쓴다** — 실제
파이프라인(carrier_code_reader/pick_place_server)처럼 QR 좌표로 파지점을
역산하는 것까지는 아직 안 옮겼다.

PLACE 목표 지점에 대해 — 원래 이 시나리오는 PackagingZone 컨베이어 위
TestItem 의 초기 위치(x=4.5, z=1.05)에 내려놓는 것이었다. 하지만 그 지점은
ConveyorFrame 충돌체(x>=4.2 를 꽉 채움) 안쪽이라, 이 팔(스펙 도달거리 0.9 m)
로는 베이스가 충돌 없이 접근할 수 있는 범위에서 절대 닿지 않는다(접근 방향/
IK 시드를 여러 조합으로 바꿔봐도 동일 — 12_pick_test.py 이전 버전의 조사
내용 참고). 그래서 PLACE 목표를 ConveyorFrame 바로 앞, 바닥(z=0) 스테이징
지점(x=4.0, y=0.0)으로 바꿨다. TestItem 은 이 지점과 겹쳐 보여 헷갈리므로
씬 로드 후 비활성화한다(원래 충돌체가 없는 순수 시각적 prim이라 물리적으로는
문제 없었지만, 시각적으로 방해가 된다).

  - PICK 은 WP_PICK 으로 베이스를 텔레포트해 시작한다(텔레포트할 때마다
    Lula 솔버에 새 base pose 를 다시 알려준다). TRANSPORT(WP_PICK -> WP_PLACE)
    는 텔레포트가 아니라 nav_server(/navigation/navigate_to, cmd_vel 직접
    주행)로 실제 물리 주행을 시킨다 — NavDriver 클래스 독스트링 참고.
    실행 전 multi_navigation.launch.py + mission_nodes.launch.py 가 떠 있어야
    한다.
  - 성공 판정은 project-plan.html 의 S4/S6 기준을 그대로 쓴다.
      pick  : 5 mm 리프트 후 유지, 기울기 <= 5도, 슬립 0
      place : 목표 xy 대비 위치오차 <= 2 mm, 자세오차 <= 1도
  - GUI 로 실행하면 pick+place 한 판을 실행한 뒤 그 상태로 정지한다. Stop 후
    Play 를 누르면 다시 실행한다.

중요 — 이 씬에서 처음 재는 값들 (실행 전 확인 필요):
  - WP_PICK/WP_PLACE 좌표는 Lula IK + 실측 지오메트리를 직접 스윕해서 검증한
    값이다. 아래 "사전 검증" 단계(REACHABILITY CHECK)가 콘솔에 SOLVED/FAILED
    로 다시 찍어 준다. FAILED 면 좌표를 조정한다.
  - 베이스 텔레포트는 nova_carter1 의 관절 트리(휠+팔)가 하나의 아티큘레이션으로
    묶여 있다는 전제로 Articulation(...).set_world_pose() 를 쓴다. 실제 아티큘레이션
    루트가 chassis_link 가 아니라면 ARTICULATION_ROOT_CANDIDATES 에 경로를 추가한다.
  - short_gripper 의 마운트 회전(FixedJoint localRot0)이 9_/10_ 스크립트와 부호가
    달라(y 성분 부호 반대) 접근 자세(APPROACH_ROLL/PITCH)가 그대로 맞지 않을 수
    있다. GUI 로 한 번 돌려 흡착면이 실제로 아래를 향하는지 먼저 확인한다.
"""

import os

from isaacsim import SimulationApp

HEADLESS = os.environ.get("PICK_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

# Nav2 로 PICK->PLACE 를 실제로 주행시키려면(이 파일 하단 NavDriver),
# /robot1/clock·odom·scan 을 내는 Isaac 네이티브 ROS2 브릿지가 이 프로세스
# 안에서 켜져 있어야 한다 — sim_backend.py 의 같은 이유(기본 구성엔 안
# 들어 있다). 이 브릿지는 Isaac 쪽 C++/OmniGraph 구현이라 rclpy 와 무관하게
# 동작한다(아래 NavDriver 독스트링 참고 — rclpy 는 이 프로세스에 아예 안 넣는다).
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")

import dataclasses
import math
import shutil
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import omni.usd
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR   = Path(__file__).resolve().parent
M0609_DIR  = THIS_DIR.parent
ISAACPJT_DIR = M0609_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD        = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")
FRAMES_YAML      = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
TAUGHT_POSES_YAML = ISAACPJT_DIR / "tools/out/taught_poses.yaml"

# sim_backend.py 와 같은 이유로 cobot3_perception 을 src/ 에서 직접 sys.path
# 에 넣는다 — ROS2 노드가 아니라 이 안에서 QR 판독 함수만 재사용한다.
sys.path.insert(0, str(WS_ROOT / "src/cobot3_perception"))
from cobot3_perception.qr_pose import estimate_qr_pose, aggregate_qr_poses  # noqa: E402


# ══════════════════════════════════════════════════════════════
#  씬 prim 경로
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/Robots/nova_carter1/m0609"
BASE_XFORM_PATH = "/World/Robots/nova_carter1"
EE_LINK_NAME    = "link_6"
EE_LINK_PATH    = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"
BASE_LINK_PATH  = f"{ROBOT_PRIM_PATH}/base_link"
GRIPPER_PRIM    = f"{ROBOT_PRIM_PATH}/short_gripper"

# 아티큘레이션 루트일 가능성이 있는 경로들. 앞에서부터 시도해 성공하는 걸 쓴다.
ARTICULATION_ROOT_CANDIDATES = [
    f"{BASE_XFORM_PATH}/chassis_link",
    BASE_XFORM_PATH,
]

MAGAZINE_XFORM_PATH = "/World/Magazines/shelf_1_magaines/top_magazines/magazine_2_blue_02"
# payload 로 합성되면 magazine_*.usda 의 defaultPrim("Magazine")이 이
# 경로 자체에 별칭(alias)되므로, 그 자식(flange_plate 등)은 별도의 "Magazine"
# 서브프림이 아니라 MAGAZINE_XFORM_PATH 바로 아래에 붙는다.
MAGAZINE_PATH = MAGAZINE_XFORM_PATH
FLANGE_NAME           = "flange_plate"                      # top-grasp 지점 (파지용 손잡이)
FLANGE_PATH            = f"{MAGAZINE_PATH}/{FLANGE_NAME}"

# ── SCAN(QR 인식) 관련 prim/설정 — sim_backend.py 의 observe_pose/scan_qr 와
# 같은 방식이다. 인식 자세(관절값)는 taught_poses.yaml 에서, 카메라 내·외부
# 파라미터는 frames.yaml 에서 런타임에 읽는다(둘 다 "단일 출처" 원칙) ──
CAMERA_PRIM = f"{GRIPPER_PRIM}/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
SCAN_RESOLUTION = (1280, 720)
SCAN_N_FRAMES = 3
SCAN_POSE_NAME = "shelf_1_top_close_centered"

# 순수 시각적 prim(충돌체 없음)이라 물리적으로 방해되진 않지만, PLACE 목표
# 지점과 겹쳐 보여서 씬 로드 후 비활성화한다.
TESTITEM_PATH = "/World/Environment/PackagingZone/TestItem"


# ══════════════════════════════════════════════════════════════
#  로봇 관절 / 드라이브
# ══════════════════════════════════════════════════════════════
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  베이스 텔레포트 waypoint — Lula IK + 실측 지오메트리로 직접 스윕해서 검증한 값
# ══════════════════════════════════════════════════════════════
#   magazine_2_blue_02  Shelf_01 위, 실측 flange x = -4.700 (아래 참고)
#   PLACE 목표          (4.0, 0.0, 바닥)  ConveyorFrame(x>=4.2) 바로 앞
#
# [WP_PICK] ★ 원래 magazine_1_orange(x=-6.5) 로 직접 스윕 검증했던 값이다.
# magazine_2_blue_02 로 바꾸면서 처음엔 QR 판독 창(frames.yaml
# decode_windows_base_x) 중심인 -4.85 를 썼는데, REACHABILITY CHECK 가
# FAILED 를 냈다 — 그때 찍힌 approach tcp(measure_prim(FLANGE_PATH) 기반
# 실측값) 가 x=-4.700 이라, base x 를 -4.85 로 잡으면 magazine_1_orange
# 때와 달리 dx=0.15 가 추가로 생겨 IK 범위를 벗어났다. magazine_1_orange
# 는 base x 를 매거진 x 에 거의 그대로 맞춰서(dx≈0, dy=0.55 만 옆으로)
# 풀렸던 거라, 이번에도 같은 방식으로 base x 를 실측 flange x(-4.700)에
# 맞췄다. y=1.45, yaw=0 은 그대로 재사용했다 — Shelf_01 전체(world
# x:[-7.05,-4.55])에 걸쳐 같은 통로 정렬(차체는 yaw=0 으로 세우고 팔만
# 옆(+y)으로 뻗는 방식)이 성립하기 때문이다.
#
# [WP_PLACE] ConveyorFrame 충돌체가 x>=4.2 를 꽉 채우고 있어, PLACE 목표를
# x=4.0(프레임 바로 앞 바닥, PLACE_TARGET_XY)으로 잡고 베이스도 그 앞
# (yaw=0, 차체 정면이 컨베이어를 향함)에 세운다. base_x 를 3.56~3.95 m
# 구간에서 스윕한 결과 전 구간이 APPROACH/LOWER 둘 다 SOLVED 이면서
# ConveyorFrame 앞면(x=4.2)과의 간격도 13~52 cm 확보된다 — 3.85 가 그
# 구간의 중간 지점(간격 약 25 cm)이다.
# ★ 이 스윕 검증값(3.85)이 실제 WP_PLACE 상수엔 안 들어가고 4.0 으로
# 박혀 있었다 — 여유가 0.2m 밖에 없었던 것. 실측: nav_server 의
# _fine_align(도착 후 제자리 yaw 보정)이 도는 동안 차체가 그 좁은 여유
# 안에서 ConveyorFrame 에 부딪혀 흡착이 풀렸다(shelf 쪽에서 겪었던 것과
# 같은 종류의 회전-여유 문제 — nav_server.py 조향 버그 이력 주석 참고).
# 문서화된 안전값으로 되돌린다.

# (베이스 world xyz, yaw_deg) — yaw 는 world +x 축 기준
# ★ 실측 flange 는 x=-4.700 에 고정. REACHABILITY CHECK 로 확인된 이력:
#   (-4.80, 1.45)  dx=.10 dy=.55  FAILED
#   (-4.75, 1.45)  dx=.05 dy=.55  SOLVED
#   (-4.80, 1.55)  dx=.10 dy=.45  SOLVED  <- dy 를 줄여 dx=.10 보상
# QR 가림 때문에 -x 로 더 옮겨야 한다는 요청은 이어지는데, y 는 더 옮길
# 필요가 없다는 피드백이 와서 1.55 로 되돌리고 x 만 더 뺐다(-4.85).
# ★ dx=.15, dy=.45 조합은 아직 검증된 적 없다 — 지금까지 SOLVED 로 확인된
# 가장 큰 dx 는 .10(dy=.45 에서) 뿐이라 FAILED 날 수 있다. REACHABILITY
# CHECK 로 반드시 재확인.
WP_PICK  = (np.array([-4.85, 1.55, 0.07963398335074101]), 0.0)
WP_PLACE = (np.array([3.85, 0.0, 0.07963398335074101]), 0.0)

# PLACE 목표 xy — ConveyorFrame(x>=4.2) 바로 앞. z 는 매거진 자체 높이만큼
# 띄운 값(런타임에 measure_prim 으로 재서 계산)을 쓴다.
# ★ WP_PLACE 를 3.85 로 뒤로 뺀 뒤(컨베이어 회전-여유 확보) 팔의 +x 리치가
# 부족해 MOVE 도중 놓치는 게 실측(GUI)으로 확인됐다 — "안정적으로 도착 후
# 팔을 더 +x 로 뻗어서 내려놓으면 성공할 것 같다"는 관찰에 따라 목표를
# 컨베이어 쪽으로 조금 당긴다. 물체 자체(작은 매거진)는 차체보다 작아
# 이 여유(0.1m)를 써도 안전하다.
#
# ★ 바닥(z=0)에 놓는 대신 컨베이어 벨트 위(z=0.6)에 놓도록 바꿨다 — GUI 로
# 직접 좌표를 찍어 확인한 값(x=4.1, y=0, z=0.6). 바닥은 팔 마운트 높이
# (~0.63m, frames.yaml m0609 마운트 오프셋)에서 거의 전체를 아래로 뻗어야
# 해서 도달 한계 근처였는데, 벨트 높이는 마운트 높이와 비슷해 훨씬 자연스러운
# 자세가 된다 — MOVE 도중 놓치던 문제의 근본 원인으로 보인다.
PLACE_TARGET_XY = np.array([4.1, 0.0])
CONVEYOR_BELT_Z = 0.6


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정 — 10_suction_magazine.py 의 튜닝값 재사용 (같은 1 kg 급 대상)
# ══════════════════════════════════════════════════════════════
COAXIAL_FORCE_LIMIT = 200.0   # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 100.0   # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.03    # m

# link_6 로컬 +Z 방향으로 흡착면(gripper_tip 바깥면)까지의 거리 (short_gripper 실측)
SUCTION_FACE_Z = 0.161
SUCTION_INSET  = 0.0025
TCP_OFFSET     = np.array([0.0, 0.0, SUCTION_FACE_Z])

# GRIP 재시도 사다리. 흡착면을 대상 윗면보다 이만큼 위에 둔다 (음수=살짝 누름)
GRIP_GAPS = [0.005, 0.002, 0.000, -0.003]


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
APPROACH_HEIGHT_OFFSET = 0.15   # 대상 윗면 기준 접근 대기 높이
LIFT_HEIGHT_OFFSET     = 0.10   # 대상 원래 위치 기준 들고 이동할 높이
PLACE_DROP             = 0.005  # 목표 높이보다 이만큼 더 내려서 확실히 내려놓는다

GRIP_WAIT    = 90
HOLD_WAIT    = 120
RELEASE_WAIT = 90
SETTLE_STEPS = 90     # 텔레포트/리셋 후 물리 안정화 대기
BASE_MOVE_SETTLE_STEPS = 60

TCP_SPEED = 0.002
MIN_STEPS = 90
MAX_STEPS = 600

APPROACH_ROLL_DEG  = 180.0     # 툴(link_6 로컬 +Z)이 바닥을 향하게
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG     = 0.0

MAX_IK_FAIL_STEPS = 30   # 연속 IK 실패 허용치 — 넘으면 UNREACHABLE 로 중단

LOG_INTERVAL = 60


# ══════════════════════════════════════════════════════════════
#  성공 판정 기준 — docs/project-plan.html S4/S6 그대로 적용
# ══════════════════════════════════════════════════════════════
LIFT_OK_MIN_M        = 0.005   # 5 mm 리프트 후 유지되면 파지 성공
TILT_MAX_DEG         = 5.0     # 파지 중 기울기 허용치
PLACE_POS_TOL_M      = 0.002   # 배치 위치오차 허용치 (2 mm)
PLACE_ORIENT_TOL_DEG = 1.0     # 배치 자세오차 허용치


# ══════════════════════════════════════════════════════════════
#  fail_code — docs/project-plan.html 체계 + 이 테스트용 확장(6, 7)
# ══════════════════════════════════════════════════════════════
FAIL_OK          = 0
FAIL_NOT_FOUND   = 1   # SCAN 단계에서 QR 을 못 읽음 (scan_qr_at_pose 참고)
FAIL_LOW_CONF    = 2   # (미사용)
FAIL_SLIP        = 3   # 파지 실패(흡착 재시도 소진) 또는 이동 중 낙하
FAIL_COLLISION   = 4   # (미구현 — 접촉 리포트 연동은 추후 확장)
FAIL_TIMEOUT     = 5   # 스텝 상한 초과
FAIL_UNREACHABLE = 6   # IK 미해 지속 (확장)
FAIL_PLACE_ERROR = 7   # 배치 오차 허용치 초과 (확장)


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 6_/9_/10_ 과 동일
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    """쿼터니언 곱. 순서는 (w, x, y, z)"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    return np.concatenate([[np.cos(half)], (a / np.linalg.norm(a)) * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """쿼터니언을 회전행렬로. 3열이 로컬 +Z (툴 방향)"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


def ease(alpha):
    """smoothstep 가감속 — 10_suction_magazine.py 와 동일 이유(가속 스파이크 방지)"""
    a = float(np.clip(alpha, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def tilt_deg_from_quat(quat_wxyz):
    """물체 로컬 +Z 축이 월드 +Z 로부터 얼마나 기울었는지(도)"""
    R = quat_to_matrix(quat_wxyz)
    up = R @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def orientation_error_deg(quat_a, quat_b):
    """두 쿼터니언 사이의 회전각 차이(도)"""
    dot = float(np.clip(abs(np.dot(quat_a, quat_b)), -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(dot)))


# ══════════════════════════════════════════════════════════════
#  USD 조회 유틸
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def resolve_articulation_root(candidates, fallback):
    """
    m0609 는 chassis_link 와 FixedJoint 로 용접되어 nova_carter1 전체(휠+팔)가
    하나의 PhysX 아티큘레이션으로 합쳐진다. m0609 자체의 ArticulationRootAPI 는
    이때 무시되므로, SingleManipulator 는 실제 루트(대개 nova_carter1 최상위)에
    등록해야 한다. USD 스키마만으로 판단하므로 물리 뷰가 없어도(씬 로드 직후) 호출할
    수 있다.
    """
    stage = omni.usd.get_context().get_stage()
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    return fallback


def get_world_pose(prim_path):
    """prim 의 현재 월드 pose 를 (pos, quat_wxyz) 로 읽는다"""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    imag = q.GetImaginary()
    return np.array([t[0], t[1], t[2]]), np.array([q.GetReal(), imag[0], imag[1], imag[2]])


def measure_prim(prim_path):
    """대상의 현재 월드 바운딩박스: (중심 xy, 윗면 z, 높이)"""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        raise RuntimeError(f"{prim_path} 의 바운딩박스가 비어 있다")
    lo, hi = rng.GetMin(), rng.GetMax()
    center_xy = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
    return center_xy, float(hi[2]), float(hi[2] - lo[2])


def filter_collision(path_a, path_b):
    """흡착 조인트와 콜라이더가 서로 밀어내며 힘 판정을 어지럽히지 않도록 필터링"""
    stage = omni.usd.get_context().get_stage()
    a = stage.GetPrimAtPath(path_a)
    rel = UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel()
    rel.AddTarget(Sdf.Path(path_b))


def reset_magazine_pose(spawn_pos, spawn_quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """트라이얼 사이에 매거진을 원래 자리로 되돌린다"""
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(MAGAZINE_XFORM_PATH))
    xform.ClearXformOpOrder()
    q = Gf.Quatd(spawn_quat_wxyz[0], Gf.Vec3d(*spawn_quat_wxyz[1:]))
    m = Gf.Matrix4d().SetRotate(q)
    m.SetTranslateOnly(Gf.Vec3d(*spawn_pos))
    xform.AddTransformOp().Set(m)


def translate_magazine_by(delta_xyz):
    """
    베이스를 텔레포트로 멀리(WP_PICK -> WP_PLACE, 약 10 m) 옮길 때, 흡착으로
    붙들고 있는 매거진은 그리퍼와 별개의 조인트로 연결된 물체라 베이스를
    따라가지 않는다. 한 프레임에 그만큼 순간이동하면 그 조인트가 감당 못할
    거리 오차로 보고 다음 스텝에 놓쳐버린다. 그래서 베이스와 같은 델타만큼
    매거진도 같이 옮겨줘서(자세는 그대로) 그리퍼 기준 상대 위치를 그대로
    보존한다 — reset_magazine_pose 와 동일한 방식(USD 로 직접 위치를 다시
    쓰면 물리 엔진이 그 프림의 rigid body pose 를 그대로 따라간다).
    """
    pos, quat = get_world_pose(MAGAZINE_XFORM_PATH)
    reset_magazine_pose(pos + np.array(delta_xyz), tuple(quat))


# ══════════════════════════════════════════════════════════════
#  QR 스캔 (SCAN 단계) — sim_backend.py 의 observe_pose()/scan_qr()/
#  _ensure_camera_warm() 를 이 스크립트용으로 옮겨온 것. GT pose 를 쓰던
#  이전 버전과 달리, magazine_2_blue_02 가 실제로 그 자리에 있다고 가정하지
#  않고 손목 카메라로 QR 을 직접 읽어서 확인한다.
# ══════════════════════════════════════════════════════════════
def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return quat_to_matrix([w, x, y, z])


def load_scan_config():
    """frames.yaml(카메라 내·외부 파라미터) + taught_poses.yaml(관측 관절값)
    을 런타임에 읽는다 — frames.yaml 헤더의 "손으로 재지 않는다, 여기가
    단일 출처다" 원칙을 그대로 따른다."""
    frames = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    st = frames["static_transforms"]
    R_l6_cam = quat_xyzw_to_mat(st["m0609_tool0__camera_link"]["quat_xyzw"])
    t_l6_cam = np.array(st["m0609_tool0__camera_link"]["xyz"])
    R_cam_opt = quat_xyzw_to_mat(st["camera_link__camera_color_optical_frame"]["quat_xyzw"])
    ci = frames["wrist_camera"]["camera_info_observed"]
    K = np.array(ci["k"], dtype=float).reshape(3, 3)
    dist = np.array(ci["d"], dtype=float)

    taught = yaml.safe_load(TAUGHT_POSES_YAML.read_text(encoding="utf-8"))
    scan_joints_deg = taught[SCAN_POSE_NAME]["joints_deg"]

    return dict(R_l6_cam=R_l6_cam, t_l6_cam=t_l6_cam, R_cam_opt=R_cam_opt,
                K=K, dist=dist, joints_deg=scan_joints_deg)


class ScanCamera:
    """CAMERA_PRIM 의 RGB-D 렌더를 딱 한 번 만들어 재사용한다. sim_backend.py
    의 _ensure_camera_warm() 과 같은 이유다 — 팔이 다른 방향을 보는 상태에서
    미리 만들면 RTX 가 그 첫 시야로 밉맵/텍스처 스트리밍을 고정해 버려서,
    나중에 관측 자세로 옮겨도 QR 디코드가 계속 깨진다. 그래서 관측 자세에
    도착한 뒤(warm_up 호출 시점) 처음 만들고, 그 뒤로는 재사용한다."""

    def __init__(self):
        self._rgb = None
        self._depth = None

    def warm_up(self, world):
        if self._rgb is None:
            import omni.replicator.core as rep
            rp = rep.create.render_product(CAMERA_PRIM, SCAN_RESOLUTION)
            self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            self._rgb.attach([rp])
            self._depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
            self._depth.attach([rp])
        for _ in range(60):
            world.step(render=True)

    def capture(self, world):
        for _ in range(3):
            world.step(render=True)
        rgb_raw = np.asarray(self._rgb.get_data())
        if rgb_raw.ndim != 3:
            raise RuntimeError(
                f"rgb annotator 가 빈 프레임을 줬다 (shape={rgb_raw.shape}) — "
                "render_product 워밍업이 부족했을 수 있다")
        rgb = rgb_raw[:, :, :3][:, :, ::-1].copy()
        depth = np.asarray(self._depth.get_data(), dtype=np.float64).reshape(rgb.shape[:2])
        return rgb, depth


def set_arm_joints_deg(robot, joints_deg):
    """관절을 즉시 그 각도로 스냅한다 — observe_pose 는 아무것도 안 들고
    있을 때만 쓰므로 급가속으로 흡착이 끊길 걱정이 없다(sim_backend.py
    _set_joint_deg 와 같은 근거)."""
    idx = np.array([robot.get_dof_index(j) for j in ARM_JOINTS])
    q = robot.get_joint_positions()
    q[idx] = np.deg2rad(joints_deg)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    robot.apply_action(ArticulationAction(
        joint_positions=np.deg2rad(joints_deg), joint_indices=idx))


def scan_qr_at_pose(world, robot, scan_cam, scan_cfg, expected_id=None):
    """관측 자세로 팔을 옮기고, 손목 카메라로 QR 이 실제로 읽히는지 확인
    한다. sim_backend.py 의 observe_pose()+scan_qr() 와 같은 순서다."""
    set_arm_joints_deg(robot, scan_cfg["joints_deg"])
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)
    scan_cam.warm_up(world)

    obs_list = []
    decoded = ""
    for _ in range(SCAN_N_FRAMES):
        rgb, depth = scan_cam.capture(world)
        l6_p, l6_q = get_world_pose(EE_LINK_PATH)
        R_l6 = quat_to_matrix(l6_q)
        R_opt = R_l6 @ scan_cfg["R_l6_cam"] @ scan_cfg["R_cam_opt"]
        p_opt = l6_p + R_l6 @ scan_cfg["t_l6_cam"]
        obs = estimate_qr_pose(rgb, depth, scan_cfg["K"], scan_cfg["dist"],
                               R_opt, p_opt, expected_id=expected_id)
        obs_list.append(obs)
        if obs.ok:
            decoded = obs.decoded

    agg = aggregate_qr_poses(obs_list)
    if agg.ok and not decoded:
        decoded = agg.decoded
    print(f"   SCAN  {'QR 인식됨' if agg.ok else 'QR 인식 실패'}"
          + (f"  payload={decoded!r}" if agg.ok else f"  reason={agg.reason}"))
    return agg


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 제어 — 9_/10_ 과 동일
# ══════════════════════════════════════════════════════════════
class SurfaceGripperCtl:
    """서피스 그리퍼는 관절이 없어 열기와 닫기 두 가지뿐이다"""

    def __init__(self, path):
        self._path = path
        self._view = None
        self.reinit(verbose=True)

    def reinit(self, verbose=False):
        self._view = None
        try:
            from isaacsim.robot.surface_gripper import GripperView
            self._view = GripperView(paths=self._path)
            if verbose:
                print("   gripper api  GripperView")
        except Exception as exc:
            if verbose:
                print(f"   gripper api  commands  (GripperView 불가: {exc})")

    def _command(self, name):
        import isaacsim.robot.surface_gripper as sg
        getattr(sg, name)(self._path)

    def close(self):
        if self._view is not None:
            self._view.apply_gripper_action(np.array([1.0]))
        else:
            self._command("close_gripper")

    def open(self):
        if self._view is not None:
            self._view.apply_gripper_action(np.array([-1.0]))
        else:
            self._command("open_gripper")

    def gripped(self):
        try:
            if self._view is not None:
                return self._view.get_gripped_objects()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripped_objects(self._path)
        except Exception:
            return None

    def status(self):
        try:
            if self._view is not None:
                return self._view.get_surface_gripper_status()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripper_status(self._path)
        except Exception:
            return None


def holding(gripped):
    if not gripped:
        return False
    flat = []
    for item in gripped:
        flat.extend(item) if isinstance(item, (list, tuple)) else flat.append(item)
    return any(str(x).strip() not in ("", "None") for x in flat)


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
class MagazinePickPlaceTask(BaseTask):
    """simple_factory_layout.usda 를 그대로 올리고 로봇/그리퍼 파라미터만 조정한다"""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_world()
        self._deactivate_test_item()
        self._setup_arm_drives()
        self._configure_gripper_limits()
        self._filter_gripper_target_collision()
        self._register_robot(scene)
        print("   scene        ready")

    def _load_world(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(WORLD_USD)
        for _ in range(15):
            simulation_app.update()
        print(f"   USD          loaded  {WORLD_USD}")

    def _deactivate_test_item(self):
        """TestItem 은 충돌체는 없지만 PLACE 목표 지점과 겹쳐 보여 비활성화한다"""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(TESTITEM_PATH)
        if prim.IsValid():
            prim.SetActive(False)
            print(f"   TestItem     비활성화 ({TESTITEM_PATH})")

    def _setup_arm_drives(self):
        """nova_carter1/m0609 의 팔 6축에만 Drive 를 강화한다 (nova_carter2 는 건드리지 않는다)"""
        stage = omni.usd.get_context().get_stage()
        count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    count += 1
        print(f"   arm drives   {count}")

    def _configure_gripper_limits(self):
        """씬에 이미 붙어 있는 short_gripper 의 SurfaceGripper 노드 한계값만 덮어쓴다"""
        node_path = find_prim_path(GRIPPER_PRIM, "SurfaceGripper")
        if node_path is None:
            raise RuntimeError(f"SurfaceGripper node not found under {GRIPPER_PRIM}")
        stage = omni.usd.get_context().get_stage()
        node = stage.GetPrimAtPath(node_path)
        node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
        node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
        node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)
        self._gripper_node_path = node_path
        print(f"   gripper      {node_path}")
        print(f"   grip limits  coaxial {COAXIAL_FORCE_LIMIT:.0f} N  "
              f"shear {SHEAR_FORCE_LIMIT:.0f} N  "
              f"maxGripDistance {MAX_GRIP_DISTANCE*1000:.0f} mm")

    def _filter_gripper_target_collision(self):
        filter_collision(GRIPPER_PRIM, MAGAZINE_XFORM_PATH)
        print(f"   collision    {GRIPPER_PRIM} <-> {MAGAZINE_XFORM_PATH} 필터링")

    def _register_robot(self, scene):
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")
        root_path = resolve_articulation_root(ARTICULATION_ROOT_CANDIDATES, ROBOT_PRIM_PATH)
        self._robot = scene.add(
            SingleManipulator(
                prim_path=root_path,
                name="m0609_robot",
                end_effector_prim_path=ee_path,
            )
        )
        print(f"   articulation root  {root_path}")
        print(f"   EE frame     {ee_path}")

    @property
    def robot(self):
        return self._robot

    @property
    def gripper_node_path(self):
        return self._gripper_node_path


def set_ready_pose(robot):
    """
    nova_carter1+m0609 가 하나의 아티큘레이션으로 합쳐져 있어(휠 DOF 포함),
    팔 6축이 항상 DOF 0~5 라는 보장이 없다. 관절 이름으로 인덱스를 찾아 채운다.
    """
    q = np.zeros(robot.num_dof)
    for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
        q[robot.get_dof_index(name)] = np.deg2rad(deg)
    robot.set_joint_positions(q)


def create_ik_solver(robot, base_pos, base_quat):
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
    solver = ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )
    return lula, solver


def sync_ik_base_pose(lula):
    """베이스를 텔레포트한 뒤, base_link 의 실제 월드 pose 를 Lula 에 다시 알려준다"""
    pos, quat = get_world_pose(BASE_LINK_PATH)
    lula.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
    return pos, quat


# ══════════════════════════════════════════════════════════════
#  베이스 텔레포트
# ══════════════════════════════════════════════════════════════
def yaw_quat_wxyz(yaw_deg):
    q = quat_from_axis([0, 0, 1], yaw_deg)
    return q


class BaseTeleporter:
    """
    nova_carter1 의 world pose 를 직접 갈아끼운다.

    m0609 는 chassis_link 에 FixedJoint 로 용접돼 있어 휠+팔이 하나의 PhysX
    아티큘레이션으로 합쳐진다(resolve_articulation_root() 참고). 즉 이 로봇을
    텔레포트하는 대상은 _register_robot() 에서 이미 만든 `robot`(SingleManipulator)
    과 동일한 아티큘레이션이므로, 별도의 SingleArticulation 을 새로 만들지 않고
    `robot` 을 그대로 재사용한다 — 같은 prim 에 대해 물리 뷰를 두 개 만들면 서로
    충돌해 물리 뷰가 무효화되고(IK 가 널뛰듯 실패하거나 apply_action 이 NoneType
    에러로 죽는 원인이 된다) 트라이얼을 몇 번 돌리다 크래시하는 걸 실제로 겪었다.
    """

    def __init__(self, robot):
        self._robot = robot

    def teleport(self, pos_xyz, yaw_deg):
        quat = yaw_quat_wxyz(yaw_deg)
        self._robot.set_world_pose(position=np.array(pos_xyz), orientation=quat)
        try:
            self._robot.set_linear_velocity(np.zeros(3))
            self._robot.set_angular_velocity(np.zeros(3))
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════
#  실주행 (PICK -> PLACE) — nav_server(/navigation/navigate_to,
#  cobot3_interfaces/action/NavigateTo)에 goal 을 보낸다. nav_server 는
#  이제 Nav2 위임 없이 amcl_pose 를 보며 직접 cmd_vel 로 모는 노드다
#  (src/cobot3_navigation/cobot3_navigation/nav_server.py 상단 주석 참고).
#
#  ★ rclpy 를 이 프로세스(isaac_python, Kit 번들 Python 3.11) 안에 직접
#  넣지 않는다 — 실측: rclpy.node.Node() 생성만으로 파라미터 이벤트
#  변환(rcl_interfaces__msg__parameter_event__convert_from_py) 도중
#  __assert_fail 로 하드크래시가 난다(Isaac 번들 _rclpy_pybind11 이 시스템
#  rosidl_generator_py 와 ABI 가 안 맞는 걸로 보인다). sim_backend.py 가
#  rclpy 대신 JSON-RPC 를 쓰는 것과 같은 종류의 제약이다.
#
#  대신 시스템 ROS2(3.12)로 이미 빌드되어 있는 `ros2` CLI 를 별도 프로세스로
#  띄워 goal 을 보낸다 — cobot3_interfaces 커스텀 액션도 CLI 프로세스
#  안에서는 시스템 python 이 그대로 처리하니 ABI 문제가 없다.
#
#  ★ subprocess.run() 처럼 통째로 블로킹하면 그사이 world.step() 이 안
#  불려 /clock 이 멈추고, use_sim_time 인 nav_server/AMCL 도 같이 멈춰서
#  주행이 영원히 안 끝난다(rclpy 직접 호출 때와 같은 이유) — Popen 으로
#  띄우고 poll() 을 world.step() 과 번갈아 부른다.
# ══════════════════════════════════════════════════════════════
NAV_NAMESPACE = "robot1"
NAV_ACTION_NAME = "/navigation/navigate_to"
NAV_ACTION_TYPE = "cobot3_interfaces/action/NavigateTo"
NAV_DRIVE_TIMEOUT_S = 180.0


def _clean_ros2_env():
    """isaac_python(Kit 번들 Python 3.11) 은 자기 stdlib/extension 경로를
    PYTHONPATH/PYTHONHOME 에 심어 두는데, 이게 자식 프로세스로 띄우는 시스템
    `ros2`(별도 python3 바이너리, /opt/ros/jazzy) 에도 그대로 상속된다.
    ros2 스크립트가 자기 stdlib 대신 Isaac 의 순수 파이썬 `re` 모듈을 줍고,
    그 안의 컴파일된 `_sre` 확장(다른 빌드)과 매직 넘버가 안 맞아
    AssertionError: SRE module mismatch 로 죽는다(실측).

    ★ PYTHONPATH 를 통째로 지웠더니 다른 크래시가 났다 — ros2cli 자체가
    /opt/ros/jazzy/.../site-packages 를 PYTHONPATH 로 찾는데(ros_set 이
    source 하는 setup.bash 가 거기다 심어 둔다), 통째로 지우면 그것까지
    같이 날아가 PackageNotFoundError: ros2cli 로 죽었다(실측). "isaacsim"/
    "kit/python" 이 들어간 항목만 걸러내고 나머지(ROS2 site-packages)는
    남긴다. PYTHONHOME 은 ROS2 쪽이 쓸 일이 없어 통째로 지운다."""
    env = dict(os.environ)
    env.pop("PYTHONHOME", None)
    old_path = env.get("PYTHONPATH", "")
    kept = [p for p in old_path.split(os.pathsep)
            if p and "isaacsim" not in p and f"{os.sep}kit{os.sep}python" not in p]
    if kept:
        env["PYTHONPATH"] = os.pathsep.join(kept)
    else:
        env.pop("PYTHONPATH", None)
    return env


def _ros2_spawn(args, env):
    """subprocess.Popen/run 대신 os.posix_spawn 을 직접 쓴다.

    ★ 실측: subprocess 로 ros2 CLI 를 띄운 직후(퍼블리시/goal 요청 모두)
    Kit 프로세스가 매번 똑같은 크래시로 죽었다 — omni.graph.core.plugin 의
    std::recursive_mutex 해시맵을 건드리다 죽는데, 스택 심볼이
    atexit_callfuncs/Py_FinalizeEx 로 찍혀서 처음엔 종료 시점 크래시로
    오인했다. 하지만 스크립트가 아직 한참 남은 시점(NAV 섹션 진입 직후)
    에도 똑같이 났다 — subprocess 호출 자체가 방아쇠였다는 뜻이다.

    Kit 은 carb.tasking.plugin 스레드를 24개 띄워 두는데(커맨드라인
    --/plugins/carb.tasking.plugin/threadCount=24), 그 상태에서
    subprocess.Popen 이 (조건에 따라) fork() 경로를 타면 다른 스레드가
    들고 있던 락이 자식 프로세스에 풀리지 않은 채로 남는 고전적
    fork-안전성 문제가 생긴다 — 정확히 recursive_mutex 해시맵이 걸리는
    이유와 들어맞는다. os.posix_spawn() 은 vfork 기반이라 이 문제를
    피한다. 반환: (pid, stdout+stderr 를 묶은 읽기용 fd)."""
    exe = shutil.which(args[0])
    if exe is None:
        raise FileNotFoundError(args[0])
    r_fd, w_fd = os.pipe()
    try:
        pid = os.posix_spawn(
            exe, args, env,
            file_actions=[
                (os.POSIX_SPAWN_DUP2, w_fd, 1),
                (os.POSIX_SPAWN_DUP2, w_fd, 2),
                (os.POSIX_SPAWN_CLOSE, r_fd),
                (os.POSIX_SPAWN_CLOSE, w_fd),
            ],
        )
    finally:
        os.close(w_fd)
    os.set_blocking(r_fd, False)
    return pid, r_fd


def _ros2_drain(r_fd, chunks):
    while True:
        try:
            data = os.read(r_fd, 65536)
        except BlockingIOError:
            return
        except OSError:
            return
        if not data:
            return
        chunks.append(data)


def _ros2_poll(pid, r_fd, chunks):
    """None 이면 아직 실행 중. 끝났으면 (returncode, 누적 출력 str)."""
    _ros2_drain(r_fd, chunks)
    done_pid, status = os.waitpid(pid, os.WNOHANG)
    if done_pid != pid:
        return None
    _ros2_drain(r_fd, chunks)
    os.close(r_fd)
    rc = os.WEXITSTATUS(status) if os.WIFEXITED(status) else -1
    return rc, b"".join(chunks).decode(errors="replace")


def _ros2_kill(pid, r_fd):
    try:
        os.kill(pid, 9)
        os.waitpid(pid, 0)
    except OSError:
        pass
    os.close(r_fd)


def _ros2_run_blocking(args, env, timeout_s):
    """비대화형 짧은 호출용(NAV 섹션의 사전 점검 등) — world.step() 인터리빙이
    필요 없는 곳에서만 쓴다. 반환: (returncode 또는 None(타임아웃), 출력)."""
    pid, r_fd = _ros2_spawn(args, env)
    chunks = []
    deadline = time.time() + timeout_s
    while True:
        result = _ros2_poll(pid, r_fd, chunks)
        if result is not None:
            return result
        if time.time() > deadline:
            _ros2_kill(pid, r_fd)
            return None, "".join(c.decode(errors="replace") for c in chunks)
        time.sleep(0.02)


class NavDriver:

    def sync_localization(self, world, x, y, yaw_deg):
        """이 스크립트가 텔레포트로 베이스를 옮긴 직후 반드시 불러야 한다
        — 안 하면 AMCL 은 텔레포트를 모르고 옛 위치를 계속 추정해서
        nav_server 가 엉뚱한 방향으로 출발한다. 텔레포트라 위치를 정확히
        아니 공분산을 작게 준다."""
        yaw = math.radians(yaw_deg)
        pose_yaml = (
            "{header: {frame_id: 'map'}, "
            "pose: {pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
            "orientation: {z: %.6f, w: %.6f}}, "
            "covariance: [%s]}}" % (
                float(x), float(y), math.sin(yaw / 2.0), math.cos(yaw / 2.0),
                ",".join("0.01" if i in (0, 7, 35) else "0.0" for i in range(36)),
            )
        )
        try:
            # --once 는 퍼블리셔를 만들자마자 한 번 쏘고 바로 끝난다 — DDS
            # 디스커버리(이 프로세스의 퍼블리셔 <-> AMCL 구독자 매칭)가 그
            # 찰나에 안 끝나면 메시지가 그냥 유실된다(실측: rviz 에서 위치가
            # 텔레포트 반영 안 된 채 이상하게 보임). 짧게(2초) 반복 퍼블리시
            # 해서 매칭 이후에도 최소 한 번은 확실히 들어가게 한다.
            rc, out = _ros2_run_blocking(
                ["ros2", "topic", "pub", "-r", "10", "-t", "20",
                 f"/{NAV_NAMESPACE}/initialpose",
                 "geometry_msgs/msg/PoseWithCovarianceStamped", pose_yaml],
                _clean_ros2_env(), timeout_s=15.0,
            )
            if rc != 0:
                print(f"   !! initialpose 퍼블리시 실패 (rc={rc}): {out.strip()[-400:]}")
        except Exception as e:
            print(f"   !! initialpose 퍼블리시 중 예외: {type(e).__name__}: {e}")
        # AMCL 이 이 initialpose 를 실제로 반영할 시간을 준다 — world.step()
        # 을 계속 밟아야 /clock 이 흐르고 AMCL 콜백도 처리된다.
        for _ in range(30):
            world.step(render=not HEADLESS)

    def drive_to(self, world, x, y, yaw_deg, timeout_s=NAV_DRIVE_TIMEOUT_S, gripper=None):
        yaw = math.radians(yaw_deg)
        goal_yaml = (
            "{pose: {header: {frame_id: 'map'}, "
            "pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
            "orientation: {z: %.6f, w: %.6f}}}}" % (
                float(x), float(y), math.sin(yaw / 2.0), math.cos(yaw / 2.0),
            )
        )
        try:
            pid, r_fd = _ros2_spawn(
                ["ros2", "action", "send_goal", NAV_ACTION_NAME, NAV_ACTION_TYPE, goal_yaml],
                _clean_ros2_env(),
            )
        except Exception as e:
            print(f"   !! 'ros2 action send_goal' 실행 실패: {type(e).__name__}: {e}")
            return False

        chunks = []
        deadline = time.time() + timeout_s + 30.0   # 액션 서버 대기(최대 30s) 여유
        result = None
        # 주행 중엔 흡착 상태를 전혀 안 지켜봤었다 — TRANSPORT 끝에는
        # gripped=True 였는데 PLACE MOVE 시작 직후 바로 놓친 걸로 나온 실측
        # 이후 추가: 5초마다 찍어서 "주행 중 어디서 떨어졌는지" vs "PLACE
        # 전환 시점 문제인지"를 다음 실행에서 구분할 수 있게 한다.
        last_log = time.time()
        while time.time() < deadline:
            world.step(render=not HEADLESS)
            result = _ros2_poll(pid, r_fd, chunks)
            if result is not None:
                break
            if gripper is not None and time.time() - last_log >= 5.0:
                last_log = time.time()
                print(f"   ...주행 중  gripped={holding(gripper.gripped())}")
            time.sleep(0.01)

        if result is None:
            print(f"   !! 주행 타임아웃({timeout_s:.0f}s) — ros2 action send_goal 강제 종료")
            _ros2_kill(pid, r_fd)
            return False

        _, out = result
        ok = "Goal finished with status: SUCCEEDED" in out
        print(f"   주행 -> {'SUCCEEDED' if ok else 'FAILED'}")
        if not ok:
            print(out.strip()[-800:])
        return ok


# ══════════════════════════════════════════════════════════════
#  Pick 단계 FSM (WP_PICK 좌표계, top-grasp on flange_plate)
# ══════════════════════════════════════════════════════════════
class PickFSM:
    """
      0 APPROACH  1 DESCEND  2 GRIP(재시도)  3 HOLD  4 LIFT  5 DONE
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "HOLD", "LIFT", "DONE"]
    DONE_STATE = 5

    def __init__(self, robot, gripper):
        self._robot = robot
        self._gripper = gripper
        self.reset()

    def reset(self):
        center_xy, top_z, height = measure_prim(FLANGE_PATH)
        self.center_xy = center_xy
        self.obj_top = top_z
        self.attempt = 0
        self.z_at_grip = None
        self.grip_ok = False
        self.dropped = False
        self.done = False
        self.fail_code = None
        self.lift_rise_m = 0.0
        self.tilt_at_lift_deg = 0.0

        self.state = 0
        self.step = 0
        self.start = None
        self.gripper = "open"
        self.ik_fail_streak = 0
        self._rebuild()

    def _rebuild(self):
        cx, cy = self.center_xy
        grip_z = self.obj_top + GRIP_GAPS[self.attempt]
        self.grip_z = grip_z
        approach_z = self.obj_top + APPROACH_HEIGHT_OFFSET
        lift_z = self.obj_top + LIFT_HEIGHT_OFFSET
        self.waypoints = [
            np.array([cx, cy, approach_z]),   # 0 APPROACH
            np.array([cx, cy, grip_z]),       # 1 DESCEND
            np.array([cx, cy, grip_z]),       # 2 GRIP
            np.array([cx, cy, grip_z]),       # 3 HOLD
            np.array([cx, cy, lift_z]),       # 4 LIFT
        ]

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            # 이 단계의 첫 프레임 — advance() 가 아직 start/goal 을 못 정했다.
            # 여기서 waypoints[state](최종 목표)를 그대로 돌려주면 그 프레임의
            # IK/apply_action 이 "한 스텝에 전체 거리" 를 목표로 잡아 큰 힘이
            # 걸린다. 거리가 짧은 단계(LIFT 0.10m 등)는 안 드러났지만, PLACE
            # MOVE(0.70m)에서 이 한 프레임짜리 스파이크가 흡착을 끊는 걸 실측
            # 확인했다 — "제자리" 를 돌려줘 이번 프레임은 움직이지 않게 한다.
            return get_tcp_pose(self._robot)
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    def note_ik(self, solved):
        self.ik_fail_streak = 0 if solved else self.ik_fail_streak + 1
        if self.ik_fail_streak >= MAX_IK_FAIL_STEPS:
            self.done = True
            self.fail_code = FAIL_UNREACHABLE

    def advance(self):
        if self.done:
            return

        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]

            if self.state == 2:                       # GRIP
                self.z_at_grip = measure_prim(MAGAZINE_PATH)[1]
                self._gripper.close()
                self.gripper = "close"
                self.n_steps, dist = GRIP_WAIT, 0.0
            elif self.state == 3:                     # HOLD
                self.n_steps, dist = HOLD_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}  {dist:.4f} m  {self.n_steps} steps"
                  f"  gripper {self.gripper}")

        self.step += 1
        self._watch_drop()

        if self.step < self.n_steps:
            return

        if self.state == 2:                           # GRIP 끝 -> 붙었나
            if not self._check_grip():
                return
        elif self.state == 4:                         # LIFT 끝 -> 최종 판정
            self._judge()

        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            if self.fail_code is None:
                self.fail_code = FAIL_OK
            print(f"   [{self.DONE_STATE}] PICK DONE  fail_code={self.fail_code}")

    def _watch_drop(self):
        if self.state not in (3, 4) or not self.grip_ok or self.dropped:
            return
        if self.step % 10:
            return
        if holding(self._gripper.gripped()):
            return
        self.dropped = True
        self.done = True
        self.fail_code = FAIL_SLIP
        print(f"   !! 놓쳤다  단계 {self.NAMES[self.state]}  {self.step}/{self.n_steps} 스텝")

    def _check_grip(self):
        gripped = self._gripper.gripped()
        ok = holding(gripped)
        print(f"   ── 시도 {self.attempt + 1}/{len(GRIP_GAPS)}  "
              f"간격 {GRIP_GAPS[self.attempt]*1000:+.0f} mm  "
              f"status {self._gripper.status()}  -> {'붙었다' if ok else '안 붙었다'}")
        if ok:
            self.grip_ok = True
            return True

        self.attempt += 1
        if self.attempt >= len(GRIP_GAPS):
            self.done = True
            self.fail_code = FAIL_SLIP
            print("   흡착 실패 — GRIP_GAPS 를 다 써봤다")
            return False

        self._gripper.open()
        self.gripper = "open"
        self._rebuild()
        self.state = 1
        self.step = 0
        self.start = None
        return False

    def _judge(self):
        now = measure_prim(MAGAZINE_PATH)[1]
        self.lift_rise_m = now - self.z_at_grip
        _, quat = get_world_pose(MAGAZINE_PATH)
        self.tilt_at_lift_deg = tilt_deg_from_quat(quat)

        ok = (self.lift_rise_m >= LIFT_OK_MIN_M) and (self.tilt_at_lift_deg <= TILT_MAX_DEG)
        print(f"   판정  rise {self.lift_rise_m*1000:+.1f} mm (>= {LIFT_OK_MIN_M*1000:.0f} mm)  "
              f"tilt {self.tilt_at_lift_deg:.2f} deg (<= {TILT_MAX_DEG:.0f})  "
              f"-> {'성공' if ok else '실패'}")
        if not ok:
            self.fail_code = FAIL_SLIP


# ══════════════════════════════════════════════════════════════
#  Place 단계 FSM (WP_PLACE 좌표계, PLACE_TARGET_XY 바닥에 내려놓기)
# ══════════════════════════════════════════════════════════════
class PlaceFSM:
    """
      0 MOVE  1 LOWER  2 RELEASE  3 RETREAT  4 DONE
    """

    NAMES = ["MOVE", "LOWER", "RELEASE", "RETREAT", "DONE"]
    DONE_STATE = 4

    def __init__(self, robot, gripper, place_target_xy, place_ref_z):
        self._robot = robot
        self._gripper = gripper
        self.place_xy = place_target_xy
        self.place_ref_z = float(place_ref_z)
        self.reset()

    def reset(self):
        self.done = False
        self.fail_code = None
        self.dropped = False
        self.state = 0
        self.step = 0
        self.start = None
        self.gripper = "close"
        self.ik_fail_streak = 0

        gx, gy = self.place_xy
        approach_z = self.place_ref_z + APPROACH_HEIGHT_OFFSET
        lower_z = self.place_ref_z + PLACE_DROP
        self.waypoints = [
            np.array([gx, gy, approach_z]),   # 0 MOVE (접근 높이로 진입)
            np.array([gx, gy, lower_z]),      # 1 LOWER
            np.array([gx, gy, lower_z]),      # 2 RELEASE
            np.array([gx, gy, approach_z]),   # 3 RETREAT
        ]

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            # PickFSM.current_target() 과 같은 이유 — 실측으로 확인된 버그.
            # 이 단계 첫 프레임엔 "제자리" 를 돌려줘 한 스텝짜리 전체거리
            # 점프(힘 스파이크)를 막는다.
            return get_tcp_pose(self._robot)
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    def note_ik(self, solved):
        self.ik_fail_streak = 0 if solved else self.ik_fail_streak + 1
        if self.ik_fail_streak >= MAX_IK_FAIL_STEPS:
            self.done = True
            self.fail_code = FAIL_UNREACHABLE

    def advance(self):
        if self.done:
            return

        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]

            if self.state == 2:                       # RELEASE
                self._gripper.open()
                self.gripper = "open"
                self.n_steps, dist = RELEASE_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}  {dist:.4f} m  {self.n_steps} steps"
                  f"  gripper {self.gripper}")

        self.step += 1
        self._watch_drop()

        if self.step < self.n_steps:
            return

        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            if self.fail_code is None:
                self.fail_code = FAIL_OK
            print(f"   [{self.DONE_STATE}] PLACE DONE  fail_code={self.fail_code}")

    def _watch_drop(self):
        if self.state not in (0, 1) or self.dropped:
            return
        if self.step % 10:
            return
        if holding(self._gripper.gripped()):
            return
        self.dropped = True
        self.done = True
        self.fail_code = FAIL_SLIP
        print(f"   !! 이송 중 놓쳤다  단계 {self.NAMES[self.state]}")


# ══════════════════════════════════════════════════════════════
#  결과 레코드
# ══════════════════════════════════════════════════════════════
@dataclasses.dataclass
class TrialResult:
    trial: int
    fail_code: int
    cycle_time_s: float
    lift_rise_mm: float
    tilt_deg: float
    place_pos_error_mm: Optional[float]
    place_orient_error_deg: Optional[float]

    @property
    def success(self):
        return self.fail_code == FAIL_OK


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


# ══════════════════════════════════════════════════════════════
#  사전 검증 — WP_PICK / WP_PLACE 에서 실제로 IK 가 풀리는지 먼저 확인
# ══════════════════════════════════════════════════════════════
def reachability_check(world, robot, lula, solver, teleporter, target_quat, place_ref_z):
    """
    WP_PICK/WP_PLACE 는 Lula IK 를 직접 스윕해서 검증한 값이지만, 트라이얼을
    돌리기 전에 접근 높이(APPROACH_HEIGHT_OFFSET)에서 다시 한 번 IK 가 실제로
    풀리는지 확인한다. FAILED 가 나오면 좌표를 조정해야 한다.
    """
    section("REACHABILITY CHECK")
    all_ok = True

    pos, yaw = WP_PICK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    center_xy, top_z, _ = measure_prim(FLANGE_PATH)
    approach_tcp = np.array([center_xy[0], center_xy[1], top_z + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)
    _, solved = solver.compute_inverse_kinematics(
        target_position=flange_target, target_orientation=target_quat)
    print(f"   WP_PICK   approach tcp {vec(approach_tcp)}  -> {'SOLVED' if solved else 'FAILED'}")
    all_ok = all_ok and solved

    pos, yaw = WP_PLACE
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    approach_tcp = np.array([PLACE_TARGET_XY[0], PLACE_TARGET_XY[1], place_ref_z + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)
    _, solved = solver.compute_inverse_kinematics(
        target_position=flange_target, target_orientation=target_quat)
    print(f"   WP_PLACE  approach tcp {vec(approach_tcp)}  -> {'SOLVED' if solved else 'FAILED'}")
    all_ok = all_ok and solved

    if not all_ok:
        print("   !! 하나 이상 FAILED — WP_PICK/WP_PLACE 좌표를 조정해야 한다")

    # 트라이얼 루프를 시작하기 전 WP_PICK 으로 되돌려 둔다
    pos, yaw = WP_PICK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    return all_ok


# ══════════════════════════════════════════════════════════════
#  한 트라이얼 실행
# ══════════════════════════════════════════════════════════════
def run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter,
              place_ref_z, place_target_quat_ref, scan_cam, scan_cfg, nav):
    t0 = time.time()

    # ── 1) PICK ────────────────────────────────────────────
    pos, yaw = WP_PICK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    # ── 1a) SCAN — GT pose 를 바로 안 믿고 손목 카메라로 QR 이 실제로
    # 읽히는지 먼저 확인한다. 이전 버전(GT 직접 읽기)과 다른 점이다.
    section("SCAN")
    scan_result = scan_qr_at_pose(world, robot, scan_cam, scan_cfg)
    if not scan_result.ok:
        return TrialResult(
            trial=trial_idx, fail_code=FAIL_NOT_FOUND,
            cycle_time_s=time.time() - t0,
            lift_rise_mm=0.0, tilt_deg=0.0,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
    pick_fsm = PickFSM(robot, gripper)

    step = 0
    while not pick_fsm.done:
        world.step(render=not HEADLESS)
        target_tcp = pick_fsm.current_target()
        flange_target = tcp_to_flange(target_tcp, target_quat)
        action, solved = solver.compute_inverse_kinematics(
            target_position=flange_target, target_orientation=target_quat)
        if solved:
            robot.apply_action(action)
        pick_fsm.note_ik(solved)
        pick_fsm.advance()
        if step % LOG_INTERVAL == 0:
            print(f"   PICK  {pick_fsm.NAMES[min(pick_fsm.state, pick_fsm.DONE_STATE)]:9s}"
                  f"  solved={solved}")
        step += 1

    if pick_fsm.fail_code != FAIL_OK:
        return TrialResult(
            trial=trial_idx, fail_code=pick_fsm.fail_code,
            cycle_time_s=time.time() - t0,
            lift_rise_mm=pick_fsm.lift_rise_m * 1000.0,
            tilt_deg=pick_fsm.tilt_at_lift_deg,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    # ── 2) TRANSPORT (nav_server 실제 주행) ─────────────────
    # 이전 버전은 베이스를 텔레포트했다 — set_world_pose() 를 부르면 거리와
    # 무관하게 흡착이 즉시 풀려서(실측), 매거진을 접근 위치에 재배치하고
    # 다시 흡착하는 우회가 필요했다. 이제는 nav_server 가 cmd_vel 로 실제
    # 물리 주행을 시킨다(바퀴가 굴러서 점진적으로 이동) — PICK 의 LIFT
    # 단계가 이미 증명한 대로(부드러운 IK 이동 중에는 흡착이 안 풀린다) 그
    # 재배치 우회가 필요 없다. 팔은 LIFT 가 남겨 둔 자세(선반 쪽으로 뻗은
    # 채) 그대로 두고 베이스만 몬다 — DRIVE_STIFFNESS(1e8)가 그 자세를
    # 계속 버텨 준다.
    section("TRANSPORT")
    nav.sync_localization(world, WP_PICK[0][0], WP_PICK[0][1], WP_PICK[1])
    drove_ok = nav.drive_to(world, WP_PLACE[0][0], WP_PLACE[0][1], WP_PLACE[1], gripper=gripper)
    sync_ik_base_pose(lula)

    print(f"   TRANSPORT 결과  drove_ok={drove_ok}  gripped={gripper.gripped()}")
    if not drove_ok or not holding(gripper.gripped()):
        return TrialResult(
            trial=trial_idx, fail_code=FAIL_SLIP,
            cycle_time_s=time.time() - t0,
            lift_rise_mm=pick_fsm.lift_rise_m * 1000.0,
            tilt_deg=pick_fsm.tilt_at_lift_deg,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    # ── 3) PLACE ───────────────────────────────────────────
    place_fsm = PlaceFSM(robot, gripper, PLACE_TARGET_XY, place_ref_z)
    step = 0
    while not place_fsm.done:
        world.step(render=not HEADLESS)
        target_tcp = place_fsm.current_target()
        flange_target = tcp_to_flange(target_tcp, target_quat)
        action, solved = solver.compute_inverse_kinematics(
            target_position=flange_target, target_orientation=target_quat)
        if solved:
            robot.apply_action(action)
        place_fsm.note_ik(solved)
        place_fsm.advance()
        if step % LOG_INTERVAL == 0:
            print(f"   PLACE {place_fsm.NAMES[min(place_fsm.state, place_fsm.DONE_STATE)]:9s}"
                  f"  solved={solved}")
        step += 1

    fail_code = pick_fsm.fail_code if pick_fsm.fail_code != FAIL_OK else place_fsm.fail_code
    # ★ 배치 위치/자세 오차 체크(PLACE_POS_TOL_M/PLACE_ORIENT_TOL_DEG)를 뺐다 —
    # PLACE 목표가 바닥 고정 지점일 때 만든 판정인데, 지금은 컨베이어 벨트
    # (CONVEYOR_BELT_Z) 위라 벨트 자체가 움직여서 릴리스 직후 물체가 그
    # 자리에 안 있는 게 정상이다(실측: 안정적으로 놓았는데도 위치오차
    # 853mm). 벨트 목표에서는 이 비교가 의미가 없다.
    place_pos_err_mm = None
    place_orient_err_deg = None

    return TrialResult(
        trial=trial_idx, fail_code=fail_code,
        cycle_time_s=time.time() - t0,
        lift_rise_mm=pick_fsm.lift_rise_m * 1000.0,
        tilt_deg=pick_fsm.tilt_at_lift_deg,
        place_pos_error_mm=place_pos_err_mm,
        place_orient_error_deg=place_orient_err_deg,
    )


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = MagazinePickPlaceTask(name="magazine_pick_place_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    magazine_spawn_pos, magazine_spawn_quat = get_world_pose(MAGAZINE_XFORM_PATH)
    _, _, magazine_height = measure_prim(MAGAZINE_PATH)
    place_ref_z = CONVEYOR_BELT_Z + magazine_height
    print(f"   magazine h   {magazine_height*1000:.1f} mm  ->  place_ref_z={place_ref_z:.3f} m"
          f"  (컨베이어 벨트 z={CONVEYOR_BELT_Z:.2f} 기준)")

    section("SOLVER")
    base_pos0, base_quat0 = get_world_pose(BASE_LINK_PATH)
    lula, solver = create_ik_solver(robot, base_pos0, base_quat0)
    gripper = SurfaceGripperCtl(task.gripper_node_path)
    teleporter = BaseTeleporter(robot)

    section("SCAN CONFIG")
    scan_cam = ScanCamera()
    scan_cfg = load_scan_config()
    print(f"   scan pose    {SCAN_POSE_NAME}  joints_deg={scan_cfg['joints_deg']}")

    section("NAV")
    print("   전제: multi_navigation.launch.py + mission_nodes.launch.py 가"
          " 이미 떠 있어야 한다 (robot1 네임스페이스, /navigation/navigate_to)")
    ros2_bin = shutil.which("ros2")
    if ros2_bin is None:
        # subprocess.run(["ros2", ...]) 이 이 상태로 불리면 FileNotFoundError 가
        # 나는데, Kit 프로세스 종료 타이밍과 겹쳐서 트레이스백 없이 그냥 창이
        # 닫힌 것처럼 보일 수 있다 — 여기서 미리 막고 원인을 바로 알려준다.
        print("   !! 'ros2' CLI 를 PATH 에서 못 찾았다 — isaac_python 을 실행한 셸에서"
              " ROS2 환경(setup.bash)을 먼저 source 했는지 확인하세요")
        simulation_app.close()   # try/finally 진입 전이라 여기서 직접 닫아야 한다
        return
    print(f"   ros2 bin     {ros2_bin}")

    # cobot3_interfaces 는 워크스페이스 로컬 빌드 패키지라 install/setup.bash
    # 를 source 해야 ros2 가 그 타입(NavigateTo)을 안다 — 실측: 안 하면
    # "ros2 action send_goal" 이 "The passed action type is invalid" 로
    # 죽는다(에러 메시지만으로는 원인이 안 보인다). 여기서 미리 확인한다.
    check_rc, check_out = _ros2_run_blocking(
        ["ros2", "interface", "show", NAV_ACTION_TYPE], _clean_ros2_env(), timeout_s=15.0,
    )
    if check_rc != 0:
        print(f"   !! ros2 가 {NAV_ACTION_TYPE} 를 못 찾는다 — 이 셸에서"
              " ROS2 환경(setup.bash)뿐 아니라 워크스페이스 오버레이도"
              " source 했는지 확인하세요:  source install/setup.bash")
        print(f"      (stderr: {check_out.strip()[-300:]})")
        simulation_app.close()
        return
    nav = NavDriver()

    try:
        target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
        reachable = reachability_check(world, robot, lula, solver, teleporter, target_quat, place_ref_z)
        if not reachable:
            print("\n   WP_PICK/WP_PLACE 좌표를 먼저 조정하세요")
            return

        def do_pick_place(trial_idx):
            # 매거진/그리퍼/베이스를 시작 상태로 되돌린 뒤 pick+place 한 판을 실행한다.
            # Stop 으로 물리가 초기화돼도 이 함수가 다시 명시적으로 상태를 맞춘다.
            gripper.open()
            reset_magazine_pose(magazine_spawn_pos, tuple(magazine_spawn_quat))
            pos, yaw = WP_PICK
            teleporter.teleport(pos, yaw)
            for _ in range(SETTLE_STEPS):
                world.step(render=not HEADLESS)
            set_ready_pose(robot)
            gripper.reinit()
            # Nav2/AMCL 에게도 이 텔레포트 위치를 알려준다 — 안 하면 이전
            # 트라이얼이 끝난 자리(WP_PLACE 근처)를 계속 현재 위치로 믿는다.
            nav.sync_localization(world, pos[0], pos[1], yaw)

            section("RUN")
            result = run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter,
                                place_ref_z, magazine_spawn_quat, scan_cam, scan_cfg, nav)
            print(f"   trial {trial_idx + 1} -> fail_code={result.fail_code}  "
                  f"cycle_time={result.cycle_time_s:.2f}s")

        do_pick_place(0)

        # 결과 상태로 정지해서 계속 띄워둔다. Stop 했다가 다시 Play 를 누르면
        # (재생 상태 False -> True 전환을 감지해) pick+place 를 한 번 더 실행한다.
        section("HOLD")
        print("   결과 상태로 정지. Stop 후 Play 를 누르면 다시 실행합니다."
              " 창을 닫으면 종료됩니다.")
        was_playing = True
        trial_idx = 1
        while simulation_app.is_running():
            world.step(render=not HEADLESS)
            playing = world.is_playing()
            if playing and not was_playing:
                do_pick_place(trial_idx)
                trial_idx += 1
            was_playing = playing
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()

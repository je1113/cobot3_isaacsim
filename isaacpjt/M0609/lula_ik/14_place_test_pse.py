"""
단위 테스트 — 매거진을 흡착한 채 컨베이어 앞으로 순간이동해서 내려놓기 (PICK + PLACE)

    isaac_python 14_place_test_pse.py   (기본이 GUI)
    (헤드리스로 돌리려면)  PICK_HEADLESS=1 isaac_python 14_place_test_pse.py

12_pick_test.py 가 성공시킨 PICK 을 그대로 쓰고, 그 뒤에 PLACE 를 붙인다.
12_ 에서 검증된 것은 한 줄도 바꾸지 않았다 — WP_PICK 좌표, GRIP_GAPS 재시도
사다리, ease() 가감속, 충돌 필터, 아티큘레이션 루트 해석, 관절 이름으로 DOF
찾기가 전부 그대로다. 새로 들어간 것은 아래 다섯 가지뿐이다.

  1. 페이로드를 같이 옮기는 순간이동 (MagazinePayload + BaseTeleporter.carry)
  2. 정거장마다 달라지는 툴 yaw      (tool_quat_for)
  3. 운반 자세 CARRY                 (CARRY_OFFSET)
  4. 벨트 위 배치 웨이포인트         (PLACE_XY / BELT_TOP_Z / PLACE_* )
  5. 전 웨이포인트 사전 IK 검증      (reachability_check)

시나리오
  ① 카터를 Shelf_01 앞(WP_PICK)으로 순간이동 — 12_ 와 동일
  ② 선반 2단 magazine_1_orange 의 flange_plate 를 흡착 (12_ 의 PickFSM 그대로)
  ③ 팔을 베이스 쪽으로 접는다 (CARRY)
  ④ 카터 + 매거진을 같은 강체 변환으로 컨베이어 앞(WP_PLACE)까지 함께 옮긴다
  ⑤ 벨트 위로 내려놓는다

순간이동이 이 스크립트의 핵심이다. 카터·팔·매거진은 조인트로 물려 있다
(카터-팔은 arm_mount_joint, 팔-매거진은 흡착 조인트). 카터만 옮기면 흡착
조인트가 거대한 구속 위반을 보고 매거진을 놓아 버린다. 그래서 매거진에
'같은' 강체 변환을 적용한다. 관절 각도는 건드리지 않으므로 팔 모양과 흡착
상대 자세가 그대로 유지된다.

  주의 — 강체 변환은 set_world_pose() 가 인자를 어떻게 해석하든 옳다.
  from/to 를 둘 다 우리가 명령하므로, 해석에 상수 오프셋이 끼어 있어도
  '이동량'은 같다. 그래서 명령값으로 계산하고, IK 베이스만 base_link 에서
  실측해 쓴다 (sync_ik_base_pose).


─── 툴 yaw 는 왜 카터 yaw 를 빼는가 ───────────────────────────────

compute_inverse_kinematics() 의 target_orientation 은 월드 기준이다. 베이스가
yaw 만큼 돌아간 정거장에서 픽업 때와 '같은 손목 자세'를 쓰려면

    R_base^T · R_tool_world = R_x(180)        (픽업 때의 손목 자세)
    ->  R_tool_world = R_z(yaw) · R_x(180)

한편 roll 180 도가 툴 Z 축을 뒤집기 때문에

    R_x(180) · R_z(t) · R_x(180)^-1 = R_z(-t)
    ->  R_x(180) · R_z(t) = R_z(-t) · R_x(180)

둘을 맞추면 -t = yaw, 즉 t = -yaw 다. make_target_quat 은
R_x(roll)·R_y(pitch)·R_z(yaw) 순서라 세 번째 인자에 -carter_yaw 를 넣으면 된다.
부호를 반대로 넣으면 손목이 베이스와 정반대로 돌아 두 배로 비틀린다.
WP_PICK 은 yaw 0 이므로 이 식이 12_pick_test.py 와 정확히 같은 값을 준다.


─── 씬에서 실측한 값 (전부 simple_factory_layout.usda / URDF 출처) ───

  BeltTop        translate z 0.72 + scale z 0.1/2  ->  상판 z = 0.770
                 x [4.05, 8.75]  y [-0.55, 0.55]   PhysicsCollisionAPI 있음
  ConveyorFrame  x [4.20, 8.60]  y [-0.675, 0.675]  z [0.225, 0.675]
                 -> 카터 차체는 x < 4.2 에 세워야 한다
  SideRail_A/B   y ±0.66,  z [0.72, 1.08]
                 -> 벨트 옆(±y)은 막혀 있다. x 낮은 쪽 열린 끝으로만 넣는다
  magazine_1_orange  250 x 140 x 142 mm, 1.0 kg, flange_plate 80x80x6 (윗면 z 0.142)
  carter footprint   [[0.14,0.25],[0.14,-0.25],[-0.607,-0.25],[-0.607,0.25]]
                 carter_navigation_params.yaml — 원점이 중심이 아니라 뒤로 0.607 m
  arm_mount_joint    카터 -> m0609 오프셋 (-0.20649, 0, 0.54920)

  참고: 벨트 위 x 4.5 근처에 TestItem 이 보이는데 콜라이더가 없는(시각 전용)
  애니메이션 프롭이다. 물리 간섭은 없지만 시야를 가리면 숨기고 보면 된다.


─── 왜 PLACE 는 12_ 가 포기했고, 여기서는 어떻게 풀었나 ──────────

벨트 상판(0.770)이 팔 베이스(0.629)보다 높다. 툴을 수직 하향으로 유지한 채
'위로' 뻗는 게 손목 한계에 걸린다. 도달반경(0.9 m)으로는 여유가 있어 보여도
IK 가 통째로 실패한다 — 12_ 가 LIFT_HEIGHT_OFFSET 0.23 에서 겪은 것과 같은
현상이다.

실패를 예측하는 값은 반경이 아니라 '어깨에서 손목중심까지의 거리 / 2링크
한계'다 (two_link_load). 12_ 실측 대조:

    LIFT +0.10 m  ->  88.5%   풀린다 (성공 사례)
    LIFT +0.23 m  ->  98.3%   안 풀린다

그래서 이 스크립트는 전 웨이포인트를 88% 이하로 맞췄다. 카터를 벨트 열린
끝에 최대한 붙이고(차체가 프레임에서 100 mm), 배치 지점을 벨트 안쪽 깊이
넣지 않는다(x 4.30). 아래 설정으로 계산한 부하는

    CARRY 75.7%   APPROACH2 88.3%   RELEASE 80.9%

실행하면 REACHABILITY CHECK 가 이 값과 Lula IK 의 SOLVED/FAILED 를 나란히
찍어 준다. FAILED 가 나오면 이 순서로 손본다.
  1) PLACE_APPROACH_CLEAR 를 줄인다 (부하가 가장 큰 구간이다)
  2) PLACE_XY 의 x 를 줄여 카터에 붙인다 (매거진이 벨트에 다 올라가는 한계는
     x 4.05 + 0.07 = 4.12)
  3) WP_PLACE 의 x 를 키워 카터를 벨트에 붙인다 (차체 x+0.25 < 4.2)


GUI 로 실행하면 한 사이클(pick+place)을 돌린 뒤 그 상태로 정지한다.
Stop 후 Play 를 누르면 매거진을 선반으로 되돌리고 다시 한 사이클 돌린다.
"""

import os

from isaacsim import SimulationApp

HEADLESS = os.environ.get("PICK_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import dataclasses
import time
from pathlib import Path

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleRigidPrim
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

WORLD_USD        = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ══════════════════════════════════════════════════════════════
#  씬 prim 경로 — 12_pick_test.py 와 동일
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/nova_carter1/m0609"
BASE_XFORM_PATH = "/World/nova_carter1"
EE_LINK_NAME    = "link_6"
EE_LINK_PATH    = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"
BASE_LINK_PATH  = f"{ROBOT_PRIM_PATH}/base_link"
GRIPPER_PRIM    = f"{ROBOT_PRIM_PATH}/short_gripper"

# 아티큘레이션 루트일 가능성이 있는 경로들. 앞에서부터 시도해 성공하는 걸 쓴다.
ARTICULATION_ROOT_CANDIDATES = [
    f"{BASE_XFORM_PATH}/chassis_link",
    BASE_XFORM_PATH,
]

MAGAZINE_XFORM_PATH = "/World/magazine_1_orange"
# payload 로 합성되면 magazine_1_orange.usda 의 defaultPrim("Magazine")이 이
# 경로 자체에 별칭(alias)되므로, 그 자식(flange_plate 등)은 별도의 "Magazine"
# 서브프림이 아니라 MAGAZINE_XFORM_PATH 바로 아래에 붙는다.
MAGAZINE_PATH = MAGAZINE_XFORM_PATH
FLANGE_NAME   = "flange_plate"                      # top-grasp 지점 (파지용 손잡이)
FLANGE_PATH   = f"{MAGAZINE_PATH}/{FLANGE_NAME}"


# ══════════════════════════════════════════════════════════════
#  로봇 관절 / 드라이브 — 12_pick_test.py 와 동일
# ══════════════════════════════════════════════════════════════
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# URDF 실측 — two_link_load() 와 도달 리포트에 쓴다
SHOULDER_Z = 0.1345           # base_link -> joint_1
LINK_2     = 0.411            # joint_1 -> joint_3
LINK_3     = 0.368            # joint_3 -> joint_4
WRIST_Z    = 0.121            # joint_5 -> joint_6
TWO_LINK   = LINK_2 + LINK_3  # 어깨 -> 손목중심 최대
SPEC_REACH = LINK_2 + LINK_3 + WRIST_Z   # 0.900, 카탈로그 도달반경


# ══════════════════════════════════════════════════════════════
#  정거장 1 — 선반 앞 (픽업).  12_pick_test.py 의 검증된 값 그대로
# ══════════════════════════════════════════════════════════════
#   magazine_1_orange  (-6.5, 2.0, 0.6)  Shelf_01 Level_2 위
#
# Shelf_01 은 Level_1/Level_2 두 단 선반판이 x 방향 2.5 m 전체를 덮고 있어
# 선반 옆(x축)으로 접근(yaw=180)하면 선반 몸체를 관통하는 경로가 된다.
# nova_carter1 은 선반과 나란한 통로(x 방향)를 주행하는 차체라 yaw=90 으로
# 돌려 세우면 차체 옆면이 선반에 닿는다. 그래서 차체는 통로 방향(yaw=0)으로
# 두고 팔만 옆(+y)으로 뻗어 집는다. base_x=-6.50, base_y=1.42~1.72 구간 전체가
# APPROACH/LIFT 양쪽 IK BOTH-OK 이고, 아래 1.45 는 선반 앞면(y=1.867)과
# 약 17.9 cm 간격이 남는 여유 지점이다.
WP_PICK = (np.array([-6.5, 1.45, 0.07963398335074101]), 0.0)


# ══════════════════════════════════════════════════════════════
#  정거장 2 — 컨베이어 앞 (배치).  이 스크립트에서 새로 잡은 값
# ══════════════════════════════════════════════════════════════
# yaw=-90 으로 세우면 차체 로컬 +y 가 월드 +x 를 향한다. 즉 픽업 때와 '같은
# 로컬 방향'으로 팔을 뻗게 되어 joint_1 스윙이 양쪽 정거장에서 0 이 된다.
# 1 kg 을 흡착으로 물고 팔을 180 도 휘두르지 않아도 된다.
#
# 차체 footprint 는 yaw=-90 에서 world x [cx-0.25, cx+0.25],
#                              y [cy-0.14, cy+0.607] 가 된다.
#   cx=3.85 -> 차체 x [3.60, 4.10],  컨베이어 프레임(x 4.2)까지 100 mm 여유
#   cy=-0.20649 -> 팔 베이스 y = cy + 0.20649 = 0.0  (벨트 중심선에 정확히 정렬)
# 카터 z 는 픽업과 같다 (바닥이 평평하다).
WP_PLACE = (np.array([3.85, -0.20649390288330727, 0.07963398335074101]), -90.0)

# 벨트 위 내려놓을 자리. BeltTop 은 x [4.05, 8.75] 이고, 매거진은 순간이동으로
# -90 도 돌아가 x 방향 폭이 140 mm 가 되므로 x 4.30 이면 [4.23, 4.37] 로 벨트에
# 완전히 올라간다 (프레임 끝 4.20 을 30 mm 넘어선다).
PLACE_XY = np.array([4.30, 0.0])

# BeltTop:  translate z 0.72 + scale z 0.1/2
BELT_TOP_Z = 0.770

# 운반 자세 — 팔 베이스 기준 상대 좌표(카터 로컬 프레임).
#   로컬 +y 0.30 : 양쪽 정거장 모두 팔이 뻗는 방향이라 joint_1 이 0 을 유지한다
#   +z 0.36      : 순간이동 도착 시 매거진 바닥이 z 0.85 로, 컨베이어 프레임
#                  윗면(0.675) 과 벨트 상판(0.770) 위로 충분히 뜬다
CARRY_OFFSET = np.array([0.0, 0.30, 0.36])


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정 — 12_pick_test.py 의 튜닝값 그대로
# ══════════════════════════════════════════════════════════════
COAXIAL_FORCE_LIMIT = 200.0   # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 100.0   # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.03    # m

# link_6 로컬 +Z 방향으로 흡착면(gripper_tip 바깥면)까지의 거리 (short_gripper 실측)
SUCTION_FACE_Z = 0.161
TCP_OFFSET     = np.array([0.0, 0.0, SUCTION_FACE_Z])

# GRIP 재시도 사다리. 흡착면을 대상 윗면보다 이만큼 위에 둔다 (음수=살짝 누름)
GRIP_GAPS = [0.005, 0.002, 0.000, -0.003]


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
# ── 픽업쪽 (12_pick_test.py 와 동일) ──────────────────────────
APPROACH_HEIGHT_OFFSET = 0.15   # 대상 윗면 기준 접근 대기 높이
LIFT_HEIGHT_OFFSET     = 0.10   # 대상 원래 위치 기준 들고 이동할 높이
                                #   0.23 이면 2링크 부하 98% 로 IK 가 안 풀린다

# ── 배치쪽 (새로 잡은 값) ─────────────────────────────────────
PLACE_DROP           = 0.005    # 놓을 때 이만큼 높게 둬서 매거진이 튀지 않게 한다
PLACE_APPROACH_CLEAR = 0.08     # 벨트 위 대기 높이. 전 구간 중 부하가 가장 크다
                                #   (88.3%). IK 가 실패하면 여기부터 줄인다

GRIP_WAIT    = 90
HOLD_WAIT    = 120
RELEASE_WAIT = 90
SETTLE_STEPS = 90     # 텔레포트/리셋 후 물리 안정화 대기
BASE_MOVE_SETTLE_STEPS = 60

# 순간이동 직후: USD 가 물리 결과를 받아쓸 시간을 줘야 base_link 실측이 맞는다.
# 이 스텝 동안은 apply_action 을 하지 않으므로 팔은 관절 드라이브 목표를
# 유지한 채 모양을 그대로 잡고 있는다.
TELEPORT_SYNC_STEPS = 10
TELEPORT_SETTLE     = 120   # 옮긴 뒤 제자리에서 버티며 흡착이 견디는지 본다

TCP_SPEED = 0.002
MIN_STEPS = 90
MAX_STEPS = 600

APPROACH_ROLL_DEG  = 180.0     # 툴(link_6 로컬 +Z)이 바닥을 향하게
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0       # 흡착컵은 축대칭이라 추가 회전이 필요 없다

# 12_ 는 30 이었다. 배치쪽은 2링크 부하가 커서 구간 진입 순간 몇 프레임 튈 수
# 있는데, 사전 검증(reachability_check)이 전 웨이포인트를 이미 확인해 두므로
# 조금 더 기다려 준다. 60 프레임 = 1 초 연속 실패면 그건 진짜 문제다.
MAX_IK_FAIL_STEPS = 60

# 12_ 가 선언만 해두고 안 쓰던 FAIL_TIMEOUT 을 실제로 걸었다.
# 최장 경로(재시도 4회 포함)가 약 2000 스텝이라 넉넉히 잡은 상한이다.
MAX_TRIAL_STEPS = 8000

LOG_INTERVAL = 60


# ══════════════════════════════════════════════════════════════
#  성공 판정 기준
# ══════════════════════════════════════════════════════════════
LIFT_OK_MIN_M = 0.005   # 5 mm 리프트 후 유지되면 파지 성공 (12_ 와 동일)
TILT_MAX_DEG  = 5.0     # 파지/배치 중 기울기 허용치

PLACE_XY_TOL_M = 0.05   # 벨트 위 목표 xy 로부터 이 안에 앉아야 한다
PLACE_Z_TOL_M  = 0.02   # 매거진 바닥이 벨트 상판에서 이 안에 있어야 한다


# ══════════════════════════════════════════════════════════════
#  fail_code — 12_pick_test.py 체계 + 배치용 확장(7)
# ══════════════════════════════════════════════════════════════
FAIL_OK          = 0
FAIL_NOT_FOUND   = 1   # (미사용 — GT pose 라 인식 실패 케이스 없음)
FAIL_LOW_CONF    = 2   # (미사용)
FAIL_SLIP        = 3   # 파지 실패(흡착 재시도 소진) 또는 이동/순간이동 중 낙하
FAIL_COLLISION   = 4   # (미구현 — 접촉 리포트 연동은 추후 확장)
FAIL_TIMEOUT     = 5   # 스텝 상한 초과
FAIL_UNREACHABLE = 6   # IK 미해 지속
FAIL_PLACE_POSE  = 7   # 놓긴 놓았는데 벨트 위 자세/위치가 허용치를 벗어남


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 12_pick_test.py 와 동일 (+ quat_conj)
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


def quat_conj(q):
    """켤레 = 역회전 (단위 쿼터니언이므로)"""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


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


def yaw_quat_wxyz(yaw_deg):
    """카터 자세 — world +z 축 회전만"""
    return quat_from_axis([0, 0, 1], yaw_deg)


def tool_quat_for(carter_yaw_deg):
    """
    이 정거장에서 쓸 툴 목표 자세 (월드 기준).

    파일 상단 "툴 yaw 는 왜 카터 yaw 를 빼는가" 에 유도가 있다. 요약하면
    roll 180 도가 툴 Z 축을 뒤집기 때문에 툴 프레임 yaw 가 월드에서 반대로
    돌고, 그래서 베이스 yaw 를 '더하지 말고 빼야' 손목이 픽업 때와 같은 해를
    쓴다. WP_PICK(yaw 0)에서는 12_pick_test.py 와 정확히 같은 값이 나온다.
    """
    return make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG,
                            GRIPPER_YAW_DEG - carter_yaw_deg)


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


def ease(alpha):
    """smoothstep 가감속 — 12_pick_test.py 와 동일 이유(가속 스파이크 방지)"""
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


def rigid_map(pos, quat, from_pos, from_quat, to_pos, to_quat):
    """
    (from_pos, from_quat) -> (to_pos, to_quat) 강체 변환을 pose 하나에 적용한다.

    순간이동 때 카터·매거진에 '같은' 변환을 먹이는 데 쓴다. from/to 를 둘 다
    우리가 명령하므로, set_world_pose() 가 인자를 어떻게 해석하든(상수
    오프셋이 끼어 있든) 이동량 자체는 이 식과 같다.
    """
    R_from = quat_to_matrix(from_quat)
    R_to   = quat_to_matrix(to_quat)
    rel = R_from.T @ (np.array(pos) - np.array(from_pos))
    dq  = quat_mul(to_quat, quat_conj(from_quat))
    return np.array(to_pos) + R_to @ rel, quat_mul(dq, quat)


def two_link_load(tcp, base_pos):
    """
    어깨(joint_1)에서 손목중심까지의 거리를 2링크 한계로 나눈 값.

    툴을 수직 하향으로 유지하는 이 작업에서는 도달반경(SPEC_REACH 0.9)보다
    이 값이 IK 실패를 훨씬 잘 예측한다. 12_pick_test.py 실측 대조:

        LIFT +0.10 m -> 88.5%  풀린다 (성공 사례)
        LIFT +0.23 m -> 98.3%  안 풀린다 (반경으로는 0.62 m 로 여유가 있는데도)

    툴이 아래를 보므로 플랜지는 TCP 보다 SUCTION_FACE_Z 위, 손목중심은
    거기서 WRIST_Z 만큼 더 위에 있다.
    """
    shoulder = np.array(base_pos, dtype=float) + np.array([0.0, 0.0, SHOULDER_Z])
    wrist = np.array(tcp, dtype=float) + np.array([0.0, 0.0, SUCTION_FACE_Z + WRIST_Z])
    d = float(np.linalg.norm(wrist - shoulder))
    return d, d / TWO_LINK


# ══════════════════════════════════════════════════════════════
#  USD 조회 유틸 — 12_pick_test.py 와 동일
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


def write_usd_pose(prim_path, pos, quat_wxyz):
    """USD xform 을 직접 갈아끼운다 (물리 핸들을 못 잡았을 때의 대비책)"""
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xform.ClearXformOpOrder()
    q = Gf.Quatd(float(quat_wxyz[0]), Gf.Vec3d(*[float(v) for v in quat_wxyz[1:]]))
    m = Gf.Matrix4d().SetRotate(q)
    m.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in pos]))
    xform.AddTransformOp().Set(m)


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 제어 — 12_pick_test.py 와 동일
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
#  페이로드 (매거진) — 물리 API 로 옮긴다
# ══════════════════════════════════════════════════════════════
class MagazinePayload:
    """
    순간이동 때 매거진을 카터와 같이 옮기기 위한 핸들.

    시뮬이 도는 중에는 USD xform 을 고쳐도 물리 물체가 안 움직인다 (다음
    스텝에 물리 값으로 덮인다). SingleRigidPrim 만 물리 텐서 API 로 실제
    텔레포트한다. 핸들을 못 잡으면 USD 쓰기로 떨어지되, 그때는 순간이동이
    제대로 안 될 수 있으니 콘솔에 남긴다.
    """

    def __init__(self, path):
        self._path = path
        self._prim = None
        try:
            self._prim = SingleRigidPrim(prim_path=path, name="magazine_payload")
            print(f"   payload      {path}  (SingleRigidPrim)")
        except Exception as exc:
            print(f"   !! payload   {path} 물리 핸들 실패, USD 쓰기로 대체: {exc}")

    @property
    def physical(self):
        return self._prim is not None

    def initialize(self):
        """
        물리 뷰가 준비된 뒤(world.reset 이후) 한 번 불러야 텔레포트가 먹는다.

        Isaac 소스(isaacsim.core.prims RigidPrim.initialize) 기준: 핸들이 이미
        유효하면 아무것도 안 하고, Stop 으로 핸들이 무효화됐으면 다시 만든다.
        Play 시에는 PHYSICS_READY 이벤트로 자동 재생성되기도 하므로, 여기서
        다시 부르는 것은 그 자동 경로가 늦거나 빠졌을 때의 안전장치다.
        """
        if self._prim is None:
            return
        try:
            self._prim.initialize()
        except Exception as exc:
            print(f"   !! payload   initialize 실패: {exc}")

    def get_world_pose(self):
        if self._prim is not None:
            try:
                pos, quat = self._prim.get_world_pose()
                return np.array(pos, dtype=float), np.array(quat, dtype=float)
            except Exception:
                pass
        return get_world_pose(self._path)

    def set_world_pose(self, pos, quat):
        if self._prim is not None:
            try:
                self._prim.set_world_pose(position=np.array(pos, dtype=float),
                                          orientation=np.array(quat, dtype=float))
                self.zero_velocity()
                return True
            except Exception as exc:
                print(f"   !! payload   set_world_pose 실패: {exc}")
        write_usd_pose(self._path, pos, quat)
        return False

    def zero_velocity(self):
        """순간이동 직후 남은 속도가 흡착 조인트를 때리지 않게 눌러 준다"""
        if self._prim is None:
            return
        for fn in ("set_linear_velocity", "set_angular_velocity"):
            try:
                getattr(self._prim, fn)(np.zeros(3))
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task.  12_pick_test.py 와 동일
# ══════════════════════════════════════════════════════════════
class MagazinePnPTask(BaseTask):
    """simple_factory_layout.usda 를 그대로 올리고 로봇/그리퍼 파라미터만 조정한다"""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_world()
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
    """
    base_link 의 실제 월드 pose 를 읽어 Lula 에 다시 알려준다.

    카터를 옮기면 팔 base_link 의 월드 pose 도 같이 바뀐다. 카터 위치에
    오프셋을 더해 '추정' 하면 chassis_link 가 카터 prim 원점에 있다는 검증
    안 된 가정이 끼어들고, 그 가정이 틀리면 IK 는 어긋난 좌표계 안에서
    멀쩡히 해를 찾는다 — 팔은 그럴듯한 자세로 가는데 흡착컵만 그 오프셋만큼
    빗나간다. 겉보기엔 '좌표를 못 잡는' 것으로 보인다. 그래서 항상 실측한다.
    """
    pos, quat = get_world_pose(BASE_LINK_PATH)
    lula.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
    return pos, quat


# ══════════════════════════════════════════════════════════════
#  베이스 순간이동 (+ 페이로드 동반)
# ══════════════════════════════════════════════════════════════
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

    팔은 따로 옮기지 않는다. 카터와 한 아티큘레이션이라 카터를 옮기면 관절
    각도를 유지한 채 그대로 따라온다. 따로 옮기려 들면 같은 몸을 두 번 건드려
    오히려 터진다.
    """

    def __init__(self, robot, payload=None):
        self._robot = robot
        self._payload = payload
        # 기대 베이스 계산에 쓸 픽업쪽 실측 베이스. main 이 set_pick_side_base 로 채운다.
        self._pick_side_base = None

    def set_pick_side_base(self, base_pos):
        self._pick_side_base = np.array(base_pos, dtype=float)

    def park(self, pos_xyz, yaw_deg):
        """베이스만 옮긴다. 아무것도 들고 있지 않을 때 쓴다 (사전 검증 / 리셋)"""
        quat = yaw_quat_wxyz(yaw_deg)
        self._robot.set_world_pose(position=np.array(pos_xyz, dtype=float),
                                   orientation=quat)
        self._zero_robot_velocity()

    def carry(self, wp_from, wp_to, world, lula):
        """
        카터 + 매거진을 같은 강체 변환으로 함께 옮긴다.

        둘은 흡착 조인트로 물려 있다. 하나만 옮기면 조인트가 거대한 구속
        위반을 보고 매거진을 놓아 버린다. 관절 각도는 손대지 않으므로 팔
        모양과 흡착 상대 자세가 그대로 유지된다.
        """
        from_pos, from_yaw = wp_from
        to_pos, to_yaw = wp_to
        from_quat, to_quat = yaw_quat_wxyz(from_yaw), yaw_quat_wxyz(to_yaw)

        # 옮기기 전에 먼저 읽는다. 베이스를 먼저 옮기면 뒤 계산이 오염된다
        pay_pose = None
        ee_before = None
        if self._payload is not None:
            try:
                pay_pose = self._payload.get_world_pose()
                ee_before = self._robot.end_effector.get_world_pose()
            except Exception as exc:
                print(f"   !! payload   순간이동 전 pose 읽기 실패: {exc}")

        self._robot.set_world_pose(position=np.array(to_pos, dtype=float),
                                   orientation=to_quat)
        self._zero_robot_velocity()

        if pay_pose is not None:
            new_pos, new_quat = rigid_map(pay_pose[0], pay_pose[1],
                                          from_pos, from_quat, to_pos, to_quat)
            self._payload.set_world_pose(new_pos, new_quat)

        # USD 가 물리 결과를 받아쓴 뒤여야 base_link 실측이 맞는다. 이 동안은
        # apply_action 을 안 하므로 팔은 관절 드라이브 목표를 그대로 유지한다.
        for _ in range(TELEPORT_SYNC_STEPS):
            world.step(render=not HEADLESS)
        base_pos, _ = sync_ik_base_pose(lula)

        self._report(from_pos, from_quat, to_pos, to_quat, base_pos,
                     pay_pose, ee_before)

    def _zero_robot_velocity(self):
        # 아티큘레이션에는 관절 속도를 눌러 주는 쪽이 본질이다. 루트 속도 API 는
        # 버전에 따라 텐서 크기 문제로 터질 수 있어 전부 예외로 감싼다.
        try:
            self._robot.set_joint_velocities(np.zeros(self._robot.num_dof))
        except Exception:
            pass
        for fn in ("set_linear_velocity", "set_angular_velocity"):
            try:
                getattr(self._robot, fn)(np.zeros(3))
            except Exception:
                pass

    def _report(self, from_pos, from_quat, to_pos, to_quat, base_pos,
                pay_pose, ee_before):
        """
        옮긴 결과를 검증한다.

        set_world_pose() 에 회전을 준 경로는 WP_PICK(yaw 0)만 쓰던 12_ 에서
        한 번도 지나가지 않았다. 회전이 안 먹으면 팔 베이스가 엉뚱한 곳에
        남는데, 그 상태로도 IK 는 조용히 해를 찾아버려 원인이 안 보인다.
        그래서 기대 베이스와 실측 베이스를 나란히 찍는다.
        """
        if self._pick_side_base is not None:
            want, _ = rigid_map(self._pick_side_base, np.array([1.0, 0.0, 0.0, 0.0]),
                                from_pos, from_quat, to_pos, to_quat)
            err = float(np.linalg.norm(np.array(base_pos) - want))
            mark = "ok" if err < 0.02 else "!! 회전이 안 먹었거나 루트 해석이 다르다"
            print(f"   팔 베이스     실측 {vec(base_pos)}  기대 {vec(want)}  "
                  f"오차 {err*1000:.1f} mm  {mark}")

        # 매거진이 그리퍼 기준으로 제자리에 남았는지. 여기가 어긋나면 흡착
        # 조인트가 구속 위반을 먹고 곧 놓는다.
        if pay_pose is None or ee_before is None:
            return
        try:
            ee_after = self._robot.end_effector.get_world_pose()
            pay_after = self._payload.get_world_pose()
        except Exception:
            return
        rel_before = quat_to_matrix(ee_before[1]).T @ (pay_pose[0] - ee_before[0])
        rel_after = quat_to_matrix(ee_after[1]).T @ (pay_after[0] - ee_after[0])
        err = float(np.linalg.norm(rel_after - rel_before))
        mark = "ok" if err < 0.02 else "!! 매거진이 그리퍼 기준으로 밀렸다"
        print(f"   매거진 상대   {vec(rel_before, 4)} -> {vec(rel_after, 4)}  "
              f"오차 {err*1000:.1f} mm  {mark}")


# ══════════════════════════════════════════════════════════════
#  Pick & Place FSM
# ══════════════════════════════════════════════════════════════
class PnPFSM:
    """
       0 APPROACH   매거진 위 접근 대기 높이로              ┐
       1 DESCEND    이번 시도의 흡착 높이까지 하강           │
       2 GRIP       흡착하고 붙었는지 확인                   │ 픽업 정거장
                      붙었으면 -> HOLD                      │ (WP_PICK)
                      아니면   -> 간격을 낮춰 DESCEND 로     │
                      사다리를 다 쓰면 -> FAIL_SLIP          │
       3 HOLD       제자리에서 버틴다 + 실제 파지 간격 측정  │
       4 LIFT       선반에서 수직 인출     <- 파지 판정       │
       5 CARRY      운반 자세로 접는다                      ┘
       6 TELEPORT   카터 + 매거진 함께 컨베이어 앞으로  <- 유지 판정  ┐
       7 APPROACH2  벨트 위 대기 높이로                              │ 배치 정거장
       8 LOWER      놓을 높이까지 하강                               │ (WP_PLACE)
       9 RELEASE    흡착 해제                                        │
      10 RETREAT    위로 빠지기            <- 배치 판정              ┘
      11 DONE

    0~5 는 12_pick_test.py 의 PickFSM 과 동작이 같다 (HOLD 에서 파지 간격을
    재는 것만 추가됐다 — 그 값으로 놓을 높이를 잡는다).

    상태 6 부터 베이스/툴 yaw 가 배치 정거장 것으로 바뀐다. 순간이동은
    CARRY(5) 가 '끝나는 순간' advance() 안에서 일어난다 — 메인 루프가 상태 6
    의 목표/툴 자세를 읽는 시점에는 이미 옮겨진 뒤라, 픽업쪽 베이스로
    컨베이어 좌표를 풀어 보는 프레임이 생기지 않는다. 순간이동이 강체 변환이고
    관절 각도를 건드리지 않으므로, 옮긴 직후의 실제 TCP 는 carry_place 와
    정확히 같다 — 그래서 상태 6 은 그 자리를 목표로 잡고 가만히 버티면 된다.

    HOLD 부터 LOWER 까지는 매 스텝 붙어 있는지 감시한다.
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "HOLD", "LIFT", "CARRY",
             "TELEPORT", "APPROACH2", "LOWER", "RELEASE", "RETREAT", "DONE"]
    S_APPROACH, S_DESCEND, S_GRIP, S_HOLD, S_LIFT, S_CARRY = 0, 1, 2, 3, 4, 5
    S_TELEPORT, S_APPROACH2, S_LOWER, S_RELEASE, S_RETREAT = 6, 7, 8, 9, 10
    DONE_STATE = 11

    PICK_SIDE_LAST = S_CARRY          # 이 단계까지가 픽업쪽 베이스/yaw
    WATCH_STATES = (S_HOLD, S_LIFT, S_CARRY, S_TELEPORT, S_APPROACH2, S_LOWER)
    WATCH_EVERY = 10

    def __init__(self, robot, gripper, teleporter, world, lula, pick_base):
        self._robot = robot
        self._gripper = gripper
        self._teleporter = teleporter
        self._world = world
        self._lula = lula
        self.pick_base = np.array(pick_base, dtype=float)
        self.reset()

    # ── 초기화 ──────────────────────────────────────────
    def reset(self):
        center_xy, top_z, _ = measure_prim(FLANGE_PATH)
        self.center_xy = center_xy
        self.obj_top = top_z
        # 매거진 전체 높이 — 놓을 높이를 잡는 데 쓴다 (flange 만 재면 안 된다)
        self.height = measure_prim(MAGAZINE_PATH)[2]

        self.attempt = 0
        self.z_at_grip = None
        self.grip_ok = False
        self.dropped = False
        self.teleported = False
        self.released = False
        self.done = False
        self.fail_code = None

        self.lift_rise_m = 0.0
        self.tilt_at_lift_deg = 0.0
        self.place_xy_err_m = 0.0
        self.place_dz_m = 0.0
        self.tilt_at_place_deg = 0.0

        # 실제로 붙은 간격. HOLD 끝에서 실측해 놓을 높이에 반영한다.
        # 그 전까지는 첫 시도 목표값을 쓴다 (사전 검증이 숫자를 필요로 한다).
        self.caught_gap = GRIP_GAPS[0]

        self.pick_base_quat = yaw_quat_wxyz(WP_PICK[1])
        self.place_base_quat = yaw_quat_wxyz(WP_PLACE[1])
        # 배치쪽 팔 베이스는 추정하지 않는다. 순간이동이 강체 변환이므로
        # 픽업쪽 '실측' 베이스에 같은 변환을 먹이면 정확하다.
        self.place_base = rigid_map(
            self.pick_base, np.array([1.0, 0.0, 0.0, 0.0]),
            WP_PICK[0], self.pick_base_quat, WP_PLACE[0], self.place_base_quat)[0]

        self.state = self.S_APPROACH
        self.step = 0
        self.start = None
        self.gripper = "open"
        self.ik_fail_streak = 0
        self._rebuild()

    def _rebuild(self):
        """이번 시도의 흡착 높이 / 실측 파지 간격으로 웨이포인트를 다시 만든다"""
        cx, cy = self.center_xy
        grip_z     = self.obj_top + GRIP_GAPS[self.attempt]
        approach_z = self.obj_top + APPROACH_HEIGHT_OFFSET
        lift_z     = self.obj_top + LIFT_HEIGHT_OFFSET
        self.grip_z = grip_z

        carry_pick  = self.pick_base  + quat_to_matrix(self.pick_base_quat)  @ CARRY_OFFSET
        carry_place = self.place_base + quat_to_matrix(self.place_base_quat) @ CARRY_OFFSET

        # 컵은 매거진 윗면보다 caught_gap 만큼 위에 있다. 매거진 바닥을 벨트
        # 상판에 딱 앉히는 TCP 높이가 touch_z, 거기서 PLACE_DROP 만큼 띄워 놓는다.
        px, py = PLACE_XY
        touch_z     = BELT_TOP_Z + self.height + self.caught_gap
        release_z   = touch_z + PLACE_DROP
        approach2_z = release_z + PLACE_APPROACH_CLEAR
        self.touch_z = touch_z

        self.waypoints = [
            np.array([cx, cy, approach_z]),      #  0 APPROACH
            np.array([cx, cy, grip_z]),          #  1 DESCEND
            np.array([cx, cy, grip_z]),          #  2 GRIP
            np.array([cx, cy, grip_z]),          #  3 HOLD
            np.array([cx, cy, lift_z]),          #  4 LIFT
            carry_pick,                          #  5 CARRY
            carry_place,                         #  6 TELEPORT
            np.array([px, py, approach2_z]),     #  7 APPROACH2
            np.array([px, py, release_z]),       #  8 LOWER
            np.array([px, py, release_z]),       #  9 RELEASE
            np.array([px, py, approach2_z]),     # 10 RETREAT
        ]

    # ── 조회 ────────────────────────────────────────────
    def carter_yaw_for(self, state):
        return WP_PICK[1] if state <= self.PICK_SIDE_LAST else WP_PLACE[1]

    def base_for(self, state):
        return self.pick_base if state <= self.PICK_SIDE_LAST else self.place_base

    def tool_quat(self):
        return tool_quat_for(self.carter_yaw_for(self.state))

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            return self.waypoints[self.state]
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    def note_ik(self, solved):
        self.ik_fail_streak = 0 if solved else self.ik_fail_streak + 1
        if self.ik_fail_streak >= MAX_IK_FAIL_STEPS:
            # current_target() 은 done 이 서면 마지막 웨이포인트를 돌려주므로,
            # 실패한 '이 단계의' 목표를 직접 집어 찍는다
            target = self.waypoints[self.state]
            load = two_link_load(target, self.base_for(self.state))[1]
            self.done = True
            self.fail_code = FAIL_UNREACHABLE
            print(f"   !! IK 연속 실패 {self.ik_fail_streak} 프레임  "
                  f"단계 {self.NAMES[self.state]}  목표 {vec(target)}  "
                  f"2링크 부하 {load*100:.1f}%")

    def timed_out(self):
        self.done = True
        self.fail_code = FAIL_TIMEOUT
        print(f"   !! 스텝 상한 {MAX_TRIAL_STEPS} 초과  단계 {self.NAMES[self.state]}")

    # ── 진행 ────────────────────────────────────────────
    def advance(self):
        if self.done:
            return

        if self.start is None:
            self._enter_state()

        self.step += 1
        self._watch_drop()

        if self.step < self.n_steps:
            return

        if self.state == self.S_GRIP:
            if not self._check_grip():
                return                       # 재시도로 DESCEND 로 되돌아갔다
        elif self.state == self.S_HOLD:
            self._measure_caught_gap()
        elif self.state == self.S_LIFT:
            self._judge_grasp()
        elif self.state == self.S_CARRY:
            self._do_teleport()              # 여기서 옮긴다. 다음 상태부터 배치쪽 좌표계
        elif self.state == self.S_TELEPORT:
            self._judge_teleport()
        elif self.state == self.S_RETREAT:
            self._judge_place()

        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            if self.fail_code is None:
                self.fail_code = FAIL_OK
            print(f"   [{self.DONE_STATE}] PNP DONE  fail_code={self.fail_code}")

    def _do_teleport(self):
        """
        CARRY 가 끝난 자리에서 카터 + 매거진을 함께 옮긴다.

        advance() 의 상태 전이 안에서 부르므로, 메인 루프가 다음에 상태 6 을
        볼 때는 base_link 실측·IK 베이스 동기까지 끝나 있다. 옮긴 직후 실제
        TCP 는 carry_place(waypoints[6]) 와 같다 (강체 변환 + 관절 각도 유지).
        """
        print()
        print(f"   {'─' * 56}")
        print(f"   순간이동  카터 {vec(WP_PICK[0])} yaw {WP_PICK[1]:+.0f}"
              f"  ->  {vec(WP_PLACE[0])} yaw {WP_PLACE[1]:+.0f}")
        self._teleporter.carry(WP_PICK, WP_PLACE, self._world, self._lula)
        self.teleported = True
        print(f"   {'─' * 56}")

    def _enter_state(self):
        self.start = get_tcp_pose(self._robot)
        self.goal = self.waypoints[self.state]

        if self.state == self.S_GRIP:
            self.z_at_grip = measure_prim(MAGAZINE_PATH)[1]
            self._gripper.close()
            self.gripper = "close"
            self.n_steps, dist = GRIP_WAIT, 0.0
        elif self.state == self.S_HOLD:
            self.n_steps, dist = HOLD_WAIT, 0.0
        elif self.state == self.S_TELEPORT:
            # 이미 옮겨진 뒤다. 실측 TCP 에서 carry_place 로 미세 보정하며 버틴다
            dist = float(np.linalg.norm(self.goal - self.start))
            self.n_steps = TELEPORT_SETTLE
        elif self.state == self.S_RELEASE:
            self._gripper.open()
            self.gripper = "open"
            self.released = True
            self.n_steps, dist = RELEASE_WAIT, 0.0
        else:
            self.n_steps, dist = steps_for(self.start, self.goal)

        load = two_link_load(self.goal, self.base_for(self.state))[1]
        print(f"   [{self.state:2d}] {self.NAMES[self.state]:9s}"
              f" goal {vec(self.goal)}  {dist:.4f} m  {self.n_steps} steps"
              f"  부하 {load*100:5.1f}%  gripper {self.gripper}")

    # ── 드롭 감시 ───────────────────────────────────────
    def _watch_drop(self):
        if self.state not in self.WATCH_STATES or not self.grip_ok or self.dropped:
            return
        if self.step % self.WATCH_EVERY:
            return
        if holding(self._gripper.gripped()):
            return
        self.dropped = True
        self.done = True
        self.fail_code = FAIL_SLIP
        print(f"   !! 놓쳤다  단계 {self.NAMES[self.state]}  {self.step}/{self.n_steps} 스텝"
              f"  매거진 윗면 {measure_prim(MAGAZINE_PATH)[1]:.4f}"
              f"  status {self._gripper.status()}")

    # ── 재시도 (12_pick_test.py 와 동일) ────────────────
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
        self.state = self.S_DESCEND
        self.step = 0
        self.start = None
        return False

    # ── 실측 ────────────────────────────────────────────
    def _measure_caught_gap(self):
        """
        컵과 매거진 윗면의 실제 간격을 잰다. 놓을 높이가 이 값만큼 달라진다.

        흡착 조인트가 매거진을 컵 쪽으로 당기므로 목표 간격(GRIP_GAPS)보다
        작아지는 게 보통이다. HOLD 가 끝난 뒤(조인트가 안정된 뒤) 재야 맞다.
        """
        tcp = get_tcp_pose(self._robot)
        mag_top = measure_prim(MAGAZINE_PATH)[1]
        raw = float(tcp[2] - mag_top)
        self.caught_gap = float(np.clip(raw, 0.0, MAX_GRIP_DISTANCE))
        note = "" if abs(raw - self.caught_gap) < 1e-9 else \
            f"  (실측 {raw*1000:+.1f} mm 를 0~{MAX_GRIP_DISTANCE*1000:.0f} mm 로 잘랐다)"
        print(f"   파지 간격     실측 {self.caught_gap*1000:+.1f} mm"
              f"  (목표 {GRIP_GAPS[self.attempt]*1000:+.0f} mm){note}")
        self._rebuild()
        print(f"   배치 높이     벨트 {BELT_TOP_Z:.3f} + 매거진 {self.height:.3f}"
              f" + 간격 {self.caught_gap:.3f} -> touch {self.touch_z:.4f}"
              f"  release {self.touch_z + PLACE_DROP:.4f}")

    # ── 판정 ────────────────────────────────────────────
    def _judge_grasp(self):
        """12_pick_test.py 의 S4 기준 그대로: 5 mm 리프트 후 유지, 기울기 <= 5 도"""
        now = measure_prim(MAGAZINE_PATH)[1]
        self.lift_rise_m = now - self.z_at_grip
        _, quat = get_world_pose(MAGAZINE_PATH)
        self.tilt_at_lift_deg = tilt_deg_from_quat(quat)

        ok = (self.lift_rise_m >= LIFT_OK_MIN_M) and (self.tilt_at_lift_deg <= TILT_MAX_DEG)
        print(f"   파지 판정  rise {self.lift_rise_m*1000:+.1f} mm (>= {LIFT_OK_MIN_M*1000:.0f} mm)  "
              f"tilt {self.tilt_at_lift_deg:.2f} deg (<= {TILT_MAX_DEG:.0f})  "
              f"-> {'성공' if ok else '실패'}")
        if not ok:
            self.fail_code = FAIL_SLIP
            self.done = True

    def _judge_teleport(self):
        """순간이동을 건너고도 아직 들고 있는지 본다"""
        ok = holding(self._gripper.gripped())
        print(f"   순간이동 판정  {'유지' if ok else '놓쳤다'}"
              f"  매거진 윗면 {measure_prim(MAGAZINE_PATH)[1]:.4f}"
              f"  status {self._gripper.status()}")
        if not ok:
            self.fail_code = FAIL_SLIP
            self.done = True
            print(f"      흡착 조인트가 순간이동을 못 버텼다. TELEPORT_SETTLE"
                  f"({TELEPORT_SETTLE}) 을 늘리거나 CARRY_OFFSET 을 베이스에 더"
                  f" 당겨 관성을 줄인다")

    def _judge_place(self):
        """벨트 위에 제대로 앉았는지 본다 (RETREAT 로 팔이 빠진 뒤라 안정돼 있다)"""
        center_xy, top_z, height = measure_prim(MAGAZINE_PATH)
        bottom_z = top_z - height
        _, quat = get_world_pose(MAGAZINE_PATH)

        self.place_xy_err_m = float(np.linalg.norm(center_xy - PLACE_XY))
        self.place_dz_m = float(bottom_z - BELT_TOP_Z)
        self.tilt_at_place_deg = tilt_deg_from_quat(quat)

        ok = (self.place_xy_err_m <= PLACE_XY_TOL_M
              and abs(self.place_dz_m) <= PLACE_Z_TOL_M
              and self.tilt_at_place_deg <= TILT_MAX_DEG)

        print()
        print(f"   {'─' * 56}")
        print(f"   배치 판정  중심 ({center_xy[0]:+.3f}, {center_xy[1]:+.3f})  "
              f"목표 ({PLACE_XY[0]:+.3f}, {PLACE_XY[1]:+.3f})  "
              f"오차 {self.place_xy_err_m*1000:.1f} mm (<= {PLACE_XY_TOL_M*1000:.0f})")
        print(f"              바닥 z {bottom_z:.4f}  벨트 상판 {BELT_TOP_Z:.3f}  "
              f"차이 {self.place_dz_m*1000:+.1f} mm (<= {PLACE_Z_TOL_M*1000:.0f})")
        print(f"              tilt {self.tilt_at_place_deg:.2f} deg "
              f"(<= {TILT_MAX_DEG:.0f})  -> {'성공' if ok else '실패'}")
        if not ok and self.place_dz_m > PLACE_Z_TOL_M:
            print(f"      벨트에 안 닿고 떨어졌다. PLACE_DROP 을 줄이거나 "
                  f"BELT_TOP_Z({BELT_TOP_Z:.3f}) 를 확인한다")
        elif not ok and self.place_dz_m < -PLACE_Z_TOL_M:
            print("      벨트를 파고들었다. BELT_TOP_Z 나 파지 간격 실측을 확인한다")
        print(f"   {'─' * 56}")
        print()
        if not ok:
            self.fail_code = FAIL_PLACE_POSE


# ══════════════════════════════════════════════════════════════
#  결과 레코드
# ══════════════════════════════════════════════════════════════
@dataclasses.dataclass
class TrialResult:
    trial: int
    fail_code: int
    cycle_time_s: float
    lift_rise_mm: float
    tilt_at_lift_deg: float
    place_xy_err_mm: float
    place_dz_mm: float
    tilt_at_place_deg: float

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
#  사전 검증 — 전 웨이포인트에서 IK 가 풀리는지 먼저 확인
# ══════════════════════════════════════════════════════════════
def reachability_check(world, robot, lula, solver, teleporter, fsm):
    """
    두 정거장을 실제로 돌면서 그 정거장이 담당하는 웨이포인트 전부를 Lula 로
    풀어 본다. 12_pick_test.py 는 APPROACH 한 점만 봤는데, 배치쪽은 2링크
    부하가 커서 구간마다 결과가 갈린다.

    한계: compute_inverse_kinematics 는 현재 관절 각도를 초기값으로 쓴다.
    여기서는 준비 자세에서 전 구간을 재기 때문에, 다른 자세에서 출발하면
    풀리는 지점이 FAILED 로 나올 수 있다 — 보수적인 검사다.
    """
    section("REACHABILITY CHECK")
    all_ok = True

    stations = (
        ("PICK ", WP_PICK,  range(0, PnPFSM.PICK_SIDE_LAST + 1)),
        ("PLACE", WP_PLACE, range(PnPFSM.PICK_SIDE_LAST + 1, PnPFSM.DONE_STATE)),
    )
    for tag, wp, states in stations:
        pos, yaw = wp
        teleporter.park(pos, yaw)
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            world.step(render=not HEADLESS)
        base_pos, _ = sync_ik_base_pose(lula)
        quat = tool_quat_for(yaw)

        print(f"   [{tag}] 카터 {vec(pos)} yaw {yaw:+.0f}"
              f"   팔 베이스 {vec(base_pos)}   툴 yaw {GRIPPER_YAW_DEG - yaw:+.0f}")
        for s in states:
            target = fsm.waypoints[s]
            flange = tcp_to_flange(target, quat)
            _, solved = solver.compute_inverse_kinematics(
                target_position=flange, target_orientation=quat)
            dist, load = two_link_load(target, base_pos)
            radial = float(np.linalg.norm(
                np.array(target) - (np.array(base_pos) + np.array([0, 0, SHOULDER_Z]))))
            print(f"      [{s:2d}] {PnPFSM.NAMES[s]:9s} tcp {vec(target)}"
                  f"  반경 {radial:.3f}/{SPEC_REACH:.2f}"
                  f"  2링크 {dist:.3f}/{TWO_LINK:.3f} = {load*100:5.1f}%"
                  f"  -> {'SOLVED' if solved else 'FAILED'}")
            all_ok = all_ok and solved

    if not all_ok:
        print()
        print("   !! FAILED 가 있다. 이 순서로 손본다")
        print("      1) PLACE_APPROACH_CLEAR 를 줄인다 (부하가 가장 큰 구간)")
        print("      2) PLACE_XY 의 x 를 줄여 카터에 붙인다 (벨트 한계 x 4.12)")
        print("      3) WP_PLACE 의 x 를 키워 카터를 벨트에 붙인다 (차체 x+0.25 < 4.2)")

    # 트라이얼 루프를 시작하기 전 WP_PICK 으로 되돌려 둔다
    teleporter.park(*WP_PICK)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    return all_ok


# ══════════════════════════════════════════════════════════════
#  한 사이클 실행
# ══════════════════════════════════════════════════════════════
def run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter, pick_base):
    t0 = time.time()

    teleporter.park(*WP_PICK)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    fsm = PnPFSM(robot, gripper, teleporter, world, lula, pick_base)

    step = 0
    while not fsm.done:
        world.step(render=not HEADLESS)

        # 베이스가 사이클 도중에 움직이므로 매 프레임 실측해 IK 에 넣는다.
        # 12_ 는 명시적 텔레포트 직후에만 불렀는데, 여기서는 순간이동이
        # FSM 안에서 일어나 호출 시점을 놓치기 쉽다. 매 프레임이 안전하다.
        sync_ik_base_pose(lula)

        target_tcp = fsm.current_target()
        target_quat = fsm.tool_quat()          # 정거장마다 베이스 yaw 를 따라간다
        flange_target = tcp_to_flange(target_tcp, target_quat)

        action, solved = solver.compute_inverse_kinematics(
            target_position=flange_target, target_orientation=target_quat)
        if solved:
            robot.apply_action(action)
        fsm.note_ik(solved)
        fsm.advance()

        if step % LOG_INTERVAL == 0:
            tcp = get_tcp_pose(robot)
            name = PnPFSM.NAMES[min(fsm.state, PnPFSM.DONE_STATE)]
            print(f"   {name:9s} cmd {vec(target_tcp)}  tcp {vec(tcp)}"
                  f"  err {np.linalg.norm(np.array(target_tcp) - tcp)*1000:5.1f}mm"
                  f"  solved={solved}")
        step += 1
        if step >= MAX_TRIAL_STEPS:
            fsm.timed_out()

    return TrialResult(
        trial=trial_idx, fail_code=fsm.fail_code,
        cycle_time_s=time.time() - t0,
        lift_rise_mm=fsm.lift_rise_m * 1000.0,
        tilt_at_lift_deg=fsm.tilt_at_lift_deg,
        place_xy_err_mm=fsm.place_xy_err_m * 1000.0,
        place_dz_mm=fsm.place_dz_m * 1000.0,
        tilt_at_place_deg=fsm.tilt_at_place_deg,
    )


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = MagazinePnPTask(name="magazine_pnp_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)

    payload = MagazinePayload(MAGAZINE_XFORM_PATH)
    payload.initialize()

    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    magazine_spawn_pos, magazine_spawn_quat = payload.get_world_pose()
    print(f"   magazine     spawn {vec(magazine_spawn_pos)}  "
          f"높이 {measure_prim(MAGAZINE_PATH)[2]*1000:.1f} mm")

    section("SOLVER")
    base_pos0, base_quat0 = get_world_pose(BASE_LINK_PATH)
    lula, solver = create_ik_solver(robot, base_pos0, base_quat0)
    gripper = SurfaceGripperCtl(task.gripper_node_path)
    teleporter = BaseTeleporter(robot, payload)

    # 픽업 정거장의 팔 베이스를 실측해 둔다. 배치쪽 베이스와 순간이동 검증이
    # 이 값을 기준으로 계산된다.
    teleporter.park(*WP_PICK)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    pick_base, _ = sync_ik_base_pose(lula)
    teleporter.set_pick_side_base(pick_base)
    print(f"   팔 베이스     픽업 실측 {vec(pick_base)}")

    plan = PnPFSM(robot, gripper, teleporter, world, lula, pick_base)
    print(f"   팔 베이스     배치 계산 {vec(plan.place_base)}  (강체 변환)")

    if not reachability_check(world, robot, lula, solver, teleporter, plan):
        print("\n   웨이포인트를 먼저 조정하세요")
        simulation_app.close()
        return

    def do_cycle(trial_idx):
        # 매거진/그리퍼/베이스를 시작 상태로 되돌린 뒤 한 사이클 실행한다.
        # Stop 으로 물리가 초기화돼도 이 함수가 다시 명시적으로 상태를 맞춘다.
        gripper.open()
        payload.initialize()
        payload.set_world_pose(magazine_spawn_pos, magazine_spawn_quat)
        teleporter.park(*WP_PICK)
        for _ in range(SETTLE_STEPS):
            world.step(render=not HEADLESS)
        set_ready_pose(robot)
        gripper.reinit()
        sync_ik_base_pose(lula)

        section("RUN")
        result = run_trial(trial_idx, world, robot, lula, solver,
                           gripper, teleporter, pick_base)
        section("RESULT")
        print(f"   trial {trial_idx + 1}  fail_code={result.fail_code}"
              f"  {'성공' if result.success else '실패'}"
              f"  cycle_time={result.cycle_time_s:.2f}s")
        print(f"   파지         rise {result.lift_rise_mm:+.1f} mm  "
              f"tilt {result.tilt_at_lift_deg:.2f} deg")
        print(f"   배치         xy {result.place_xy_err_mm:.1f} mm  "
              f"dz {result.place_dz_mm:+.1f} mm  "
              f"tilt {result.tilt_at_place_deg:.2f} deg")
        return result

    do_cycle(0)

    # 결과 상태로 정지해서 계속 띄워둔다. Stop 했다가 다시 Play 를 누르면
    # (재생 상태 False -> True 전환을 감지해) 한 사이클 더 실행한다.
    section("HOLD")
    print("   결과 상태로 정지. Stop 후 Play 를 누르면 다시 실행합니다."
          " 창을 닫으면 종료됩니다.")
    was_playing = True
    trial_idx = 1
    while simulation_app.is_running():
        world.step(render=not HEADLESS)
        playing = world.is_playing()
        if playing and not was_playing:
            do_cycle(trial_idx)
            trial_idx += 1
        was_playing = playing

    simulation_app.close()


if __name__ == "__main__":
    main()

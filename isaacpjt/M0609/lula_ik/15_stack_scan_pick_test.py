"""
단위 테스트 — PKG-OUT 선반의 스택을 QR 로 스캔·확인한 뒤 집기 (SCAN + PICK)

    isaac_python 15_stack_scan_pick_test.py   (기본이 GUI)
    (헤드리스로 돌리려면)  PICK_HEADLESS=1 isaac_python 15_stack_scan_pick_test.py

13_stack_scan_pick_place.py 의 씬 구성(스택 스폰·GT PICK)을 뼈대로 쓰되,
PLACE(내려놓기) 단계는 뺀다 — "12_pick_test.py 처럼 만들어줘"(사용자 지시)라
12_pick_test.py 의 뼈대(여러 판 반복 TRIALS, REACHABILITY CHECK, SUMMARY,
GUI Stop/Play 재실행)를 따른다. SCAN 은 처음엔 GT 만 찍고 지나갔는데(사용자
지시로) 실제로 손목카메라 + cobot3_perception.qr_pose 로 QR 을 디코드해
확인하고 나서야 PICK 으로 넘어가도록 바꿨다 — QR 이 안 읽히면 그 트라이얼은
실패(FAIL_NOT_FOUND)로 끝난다.

시나리오
────────
  1) 스택이 "컨베이어벨트로 나와서" PKG-OUT 선반(OutputShelf/ShelfDeck) 위에
     이미 놓여 있는 상태에서 시작한다(아래 "스택을 어떻게 놓는가" 절 참고).
  2) 로봇을 WP_SCAN 에서 STACK_APPROACH_CREEP_M 만큼 뒤로 뺀 대기점으로
     텔레포트한 뒤, drive_to() 로 마지막 구간만 실제로(조금씩, 매 스텝
     물리를 돌리며) 크립해 WP_SCAN 에 선다 — task_manager.py 의
     STACK_NAV(Nav2, 대기점까지) → STACK_CREEP_IN(cmd_vel, 마지막 접근)
     과 같은 순서다(2026-09-25). 거기서 손목카메라를 스택의 QR 라벨
     (qr_label_py, 서쪽을 본다)쪽으로 겨눠 한 장 찍어 estimate_qr_pose() 로
     디코드한다. 기대 ID(EXPECTED_QR_ID)와 다르거나 아예 안 읽히면 SCAN
     실패로 트라이얼을 끝낸다("QR 을 확인하고 잡으러 가야 한다" — 사용자
     지시. GT 로 대충 확인하고 바로 PICK 으로 넘어가던 이전 버전을 고쳤다).
  3) QR 확인에 성공하면 **그 자리(WP_SCAN)에서 움직이지 않고** flange_plate
     (top-grasp 지점, 매거진과 같은 규약)를 GT 로 읽어 바로 PICK 한다.
     ★ 2026-09-25: 예전엔 여기서 WP_PICK 이라는 별도 정차점으로 다시
     재이동했다(옆에서 QR 보기와 위에서 내려찍기는 요구하는 기하가 다르다고
     판단해서) — 그런데 그 재이동이 실제 로봇에서 manipulator 를 제대로
     못 세우는 원인이었다(사용자 실측: 지금 이 자리에서는 안 부딪힌다).
     그래서 WP_PICK 을 없애고 WP_SCAN 한 점에서 SCAN 도 PICK 도 다 한다.
     TRIALS 판을 반복한다(12_pick_test.py 와 동일하게 PICK_TRIALS 로 조절).

★ 스택을 어떻게 놓는가 — 컨베이어를 실제로 돌리지 않는다
──────────────────────────────────────────────────────────
  실제 파이프라인은: 로봇이 PKG-01 벨트에 매거진을 놓음 -> packaging_flow.py
  (씬에 OmniScriptingAPI 로 이미 붙어 있다 - PackagingUnloaderZone 참고)가
  그 매거진을 감지해 스택으로 바꾸고 -> 언로더 벨트를 따라 ~16초에 걸쳐
  슬라이드시켜 -> PKG-OUT 선반 칸에 내려놓는다.

  이 스크립트는 그 이동 과정 자체를 재현하지 않고, packaging_flow.py의
  _spawn_stack()/_move_to_shelf() 가 만드는 "최종 결과"(선반 칸 위의 스택,
  같은 에셋·같은 변환)만 씬 로드 시점에 직접 재현한다(spawn_stack_on_shelf,
  13_stack_scan_pick_place.py 와 동일). world.reset() 이전(Task.set_up_scene
  안)에 payload 를 붙이므로, 물리 씬이 처음 빌드될 때부터
  PhysicsRigidBodyAPI/CollisionAPI 가 함께 등록된다.

  자산·좌표 출처:
    · payload "../assets/F3_STKB_1.usda" — packaging_flow.py 의
      ORANGE_STACK_PAYLOAD 그대로.
    · 선반 칸 좌표(SHELF_X/SHELF_Z/SHELF_SLOT_Y) — packaging_flow.py 의 원래
      값(3.25, 0.64, 2.25...)은 실제 OutputShelf/ShelfDeck 과 무관한 자리를
      가리켜서(사용자 지적으로 확인) 쓰지 않는다 — ShelfDeck 자신의 로컬
      translate 를 그대로 재사용하도록 바꿨다. 아래 SHELF_X/SHELF_Z/
      SHELF_SLOT_Y 상수 옆 주석 참고. packaging_flow.py 자체는 건드리지
      않는다(그 값이 ShelfDeck 과 안 맞는 건 이 테스트 범위를 넘는다).
    · flange_plate — F3_STKB_1.usda 에 매거진과 같은 이름의 top-grasp
      서브프림이 있다(스택은 매거진을 90도 돌린 것뿐이라는 팀 합의,
      shelves.yaml PKG-OUT 주석 참고). measure_prim() 으로 그대로 잰다.
    · QR 라벨(qr_label_py, 서쪽을 본다)과 그 내용("F3-STKB-1") — 아래
      QR_LABEL_PATH/EXPECTED_QR_ID 상수 옆 주석 참고.

★ 정차점(WP_SCAN) — shelves.yaml PKG-OUT 은 그대로 못 쓴다
──────────────────────────────────────────────────────────────
  shelves.yaml PKG-OUT 의 waypoint_start(2.713, 0.696)/arm_teach_pose 는
  스택 위치가 바뀌어서 못 쓴다. WP_SCAN 을 다시 찾았고, SCAN 과 PICK 둘 다
  이 한 점에서 한다(2026-09-25 — 아래 "베이스 텔레포트 waypoint" 절 참고).

중요 — 실행해서 확인한 것들(2026-09-24):
  · SCAN(QR 디코드) 5/5, PICK(흡착+리프트) 3/5 성공 — project-plan S4 가
    요구하는 것도 100% 가 아니라 반복 성공률이라, 흡착 재시도 사다리가
    가끔 다 소진되는 것(fail_code=3) 자체는 실패가 아니다. PICK_TRIALS 를
    키워 돌려 성공률을 더 확인해 볼 수 있다.
  · flange_plate 흡착이 매거진과 같은 파라미터(GRIP_GAPS 등)로 붙는 것은
    위 5 판 실행으로 확인했다 — 스택 표면 재질/무게(1.0 kg, 매거진과 같은
    급)가 같다는 가정이 맞았다.
"""

import os
import shutil

from isaacsim import SimulationApp

HEADLESS = os.environ.get("PICK_HEADLESS", "0") == "1"

# 몇 판을 연달아 돌릴지 — 12_pick_test.py 와 동일한 용도(반복 흡착 성공률 확인).
#   PICK_TRIALS=10 PICK_HEADLESS=1 isaac_python 15_stack_scan_pick_test.py
TRIALS = max(1, int(os.environ.get("PICK_TRIALS", "1")))
simulation_app = SimulationApp({"headless": HEADLESS})

import dataclasses
import math
import sys
import time
from pathlib import Path

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

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
GRASP_YAML       = WS_ROOT / "src/cobot3_bringup/config/grasp.yaml"

# QR 검출/자세추정은 ROS 패키지 쪽 코드를 그대로 쓴다(ROS 의존성 없음 —
# 12_pick_test.py 의 FlangeVision 과 같은 이유, 노드와 Isaac 테스트가 같은
# 코드를 쓴다).
sys.path.insert(0, str(WS_ROOT / "src/cobot3_perception"))


# ══════════════════════════════════════════════════════════════
#  씬 prim 경로
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/Robots/nova_carter1/m0609"
BASE_XFORM_PATH = "/World/Robots/nova_carter1"
EE_LINK_NAME    = "link_6"
EE_LINK_PATH    = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"
BASE_LINK_PATH  = f"{ROBOT_PRIM_PATH}/base_link"
GRIPPER_PRIM    = f"{ROBOT_PRIM_PATH}/short_gripper"
CAMERA_PRIM     = f"{GRIPPER_PRIM}/rsd455/RSD455/Camera_OmniVision_OV9782_Color"

ARTICULATION_ROOT_CANDIDATES = [
    f"{BASE_XFORM_PATH}/chassis_link",
    BASE_XFORM_PATH,
]

# ── 스택 스폰 — packaging_flow.py 의 STACK_ROOT/SHELF_* 그대로(모듈
#    docstring "스택을 어떻게 놓는가" 절 참고) ─────────────────────────
SPAWNED_STACKS_PATH = "/World/Environment/PackagingUnloaderZone/SpawnedStacks"
STACK_ID          = "test_1"
STACK_XFORM_PATH  = f"{SPAWNED_STACKS_PATH}/{STACK_ID}"
STACK_PATH        = STACK_XFORM_PATH   # payload defaultPrim 이 이 경로에 별칭된다
FLANGE_NAME       = "flange_plate"
FLANGE_PATH       = f"{STACK_PATH}/{FLANGE_NAME}"

# QR 라벨 — F3_STKB_1.usda 에 양쪽 벽에 하나씩(qr_label_ny/qr_label_py) 붙어
# 있다. spawn 시 z 90° 회전을 걸면 qr_label_py 쪽이 world -X(서쪽)를 보게
# 된다(shelves.yaml PKG-OUT 주석의 실측과 같다). carrier_code.py 의 코드
# 규칙대로 F3_STKB_1.usda 의 QR 내용은 "F3-STKB-1"(대시, 파일명의 언더스코어
# 아님).
QR_LABEL_PATH  = f"{STACK_PATH}/qr_label_py"
EXPECTED_QR_ID = "F3-STKB-1"

# packaging_flow.ORANGE_STACK_PAYLOAD 그대로 쓰되 절대경로로 바꾼다 —
# 13_stack_scan_pick_place.py 와 같은 이유(별도 인메모리 스테이지에 참조로
# 얹는 방식이라 상대경로가 작업 디렉터리 기준으로 풀려 버린다).
STACK_PAYLOAD = str(ISAACPJT_DIR / "assets/F3_STKB_1.usda")

# ★ 실측(2026-09-24) — packaging_flow.py 의 SHELF_X/SHELF_Z/SHELF_SLOT_Y
#   (3.25, 0.64, 2.25...) 를 그대로 썼더니 실제 선반(OutputShelf/ShelfDeck)
#   과 전혀 다른 자리(x 0.6 m+ 떨어짐, 바닥까지 떨어짐)에 놓였다 — 그래서
#   지난 버전은 안 보이는 스캐폴딩 콜라이더(_spawn_shelf_support)를 임시로
#   깔아 스택을 공중에 띄워 뒀었다. 사용자 지시로, 실제 ShelfDeck 프림
#   (실측: world x [3.876,4.051], y [0.271,1.671],
#   top z 0.54) 위에 스택이 오도록 이 테스트만 좌표를 다시 잡는다 —
#   packaging_flow.py 자체는 건드리지 않는다(그 값이 ShelfDeck 과 안 맞는
#   건 이 테스트 범위를 넘는 별개 문제로 보인다).
# ★ SHELF_X/SHELF_SLOT_Y 는 SpawnedStacks 밑에 붙이는 "로컬" 좌표다(스택
#   prim 의 부모, spawn_stack_on_shelf() 참고). SpawnedStacks 는 OutputShelf
#   와 형제 프림이라 부모(PackagingUnloaderZone) 기준 로컬 오프셋이 둘 다
#   같다(both translate y -1.8286350742021307) — 그래서 ShelfDeck **자신의
#   로컬** translate (3.9633, 2.8, 0.48, 실측한
#   /World/.../OutputShelf/ShelfDeck 의 xformOp:translate 원본값) 를 그대로
#   가져다 쓰면 SpawnedStacks 밑에서도 같은 world 자리가 나온다. (world y 를
#   직접 넣으면 -1.8286 을 또 빼야 하는데 처음에 그걸 깜빡해 스택이 세계
#   y=-0.86 으로 떨어져 바닥을 뚫은 적이 있다 — 실측.)
# ★ 실측(사용자 지시) — ShelfDeck 중심(3.9633)에 놓으니 스택이 컨베이어
#   (BeltTop/ConveyorFrame, world x [4.05,8.75], 즉 선반 동쪽에 바로 붙어
#   있다)쪽에서 너무 안쪽에 있어 보였다. 동쪽(+X, 컨베이어 방향)으로
#   0.08 m 밀었다 — ShelfDeck 동쪽 끝(4.051)에 더 가깝다(스택이 이
#   좁은 덱(0.175 m 깊이)보다 넓어서(x 반폭 0.137 m) 어느 쪽으로 놓든
#   테두리를 약간 넘는다).
SHELF_X = 3.963325659785515 + 0.08   # = ShelfDeck 로컬 x + 동쪽 오프셋
SHELF_Z = 0.54                  # ShelfDeck 윗면 world z(부모 z 오프셋 0 이라 로컬=world)
SHELF_SLOT_Y = (2.8,)           # = ShelfDeck 로컬 y (형제라 그대로 재사용)
STACK_SLOT_INDEX = 0


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
#  베이스 텔레포트 waypoint — 실제 ShelfDeck 기준 새 스택 위치(3.9633,
#  0.9714, top 0.734)에 맞춰 잡았고, REACHABILITY CHECK(SOLVED)와 실제
#  SCAN+PICK 실행(QR 5/5 디코드, PICK 3/5 성공 — 흡착 재시도 사다리가 늘
#  붙는 건 아니다, project-plan S4 가 요구하는 것도 100% 가 아니라 성공률)
#  로 확인했다.
#
#  ★ 2026-09-25: WP_PICK(팔이 위에서 내려찍는 별도 정차점)을 없앴다 —
#  QR 관측과 top-grasp 접근이 요구하는 기하가 달라 정차점을 나눴었는데,
#  실제 로봇에서 그 SCAN→PICK 재이동이 오히려 manipulator 를 제대로 못
#  세우는 원인이었다(사용자 실측 — WP_SCAN 자리에서는 안 부딪힌다).
#  이제 WP_SCAN 한 점에서 SCAN 도 PICK 도 다 한다.
# ══════════════════════════════════════════════════════════════
CARTER_Z = 0.07963398335074101

# QR 라벨(서쪽을 보는 qr_label_py) 관측 겸 top-grasp 접근 자리(제자리 PICK).
# ★ 2026-09-25(5): x=3.52/y=0.85/yaw=-90 은 REACHABILITY CHECK·접근 크립
#   여유 둘 다 통과했지만, 손목카메라가 로봇 몸체에 가려졌다(사용자 실측 —
#   IK 는 각도만 보지 자기폐색은 안 본다). 대신 capture_pose.py 로 사용자가
#   직접 슬라이더로 잡은 자세(taught_poses.yaml pkg_out_pick, captured_at
#   2026-09-25 23:17:17)를 그대로 쓴다 — shelves.yaml PKG-OUT 의 새
#   waypoint_start 와 같은 값이다. 이 자리에서 REACHABILITY CHECK 랑
#   접근 크립을 다시 확인해본다.
WP_SCAN = (np.array([3.53718, 0.52602, CARTER_Z]), -82.82924)

# ★ 2026-09-25: 사용자 지적 — 이 스크립트의 SCAN(AIM QR)은 지금까지
#   qr_aim_flange_pose() 로 매번 "새 목표"를 계산해 거기로 서보하고 있었다
#   — 실제 생산 경로(carrier_code_reader._observe() → sim_backend.
#   observe_pose(joints_deg=...) → _servo_joint_deg())는 그게 아니라
#   shelves.yaml 의 arm_teach_pose 를 **그대로 관절 목표로** 서보한다,
#   IK 를 다시 안 푼다. 즉 지금까지 이 테스트는 "사용자가 티칭해준 그 자세"
#   를 검증한 게 아니었다. taught_poses.yaml 의 pkg_out_pick(=WP_SCAN 을
#   캡처할 때의 그 자세, joints_deg) 을 그대로 쓴다 — servo_joint_deg() 참고.
TAUGHT_JOINTS_DEG = [-179.9989, 35.5247, 111.4977, -74.5018, 73.5002, 55.5003]

# ★ 2026-09-26: "크립 시작점에서 SCAN"(STAGING_SCAN_JOINTS_DEG) 실험은
#   되돌렸다 — 거기서 QR 을 찾자마자 크립을 멈추면 PICK 이 그 자리에서
#   파지점까지 팔 도달거리(약 0.9m) 밖이라 못 닿는다. PICK 이 되는
#   자리(WP_SCAN, 사용자가 카메라 안 가리는 걸 직접 확인하고 티칭한 자세)
#   에서 그대로 SCAN 하는 이 방식으로 돌아간다 — 대신 크립 자체가 방어
#   정지 없이 끝나게 하는 건 별도로 (COLLISION_SAFE_MARGIN_M 주석 참고).

# 손목카메라를 QR 라벨 정면에서 이만큼 띄워 놓고 찍는다(관측 standoff).
QR_STANDOFF_M = 0.40

# ── STACK_NAV 접근 크립 (2026-09-25) ────────────────────────────────────
# task_manager.py 의 STACK_APPROACH_CREEP_M 과 같은 값 — Nav2(여기서는
# teleport)로 이만큼 뒤로 뺀 대기점까지만 크게 이동하고, 마지막은
# drive_to() 로 실제 cmd_vel 크립처럼 조금씩(물리 스텝마다) 접근해 WP_SCAN
# 에 정확히 선다. PLACE 의 CREEP_IN 과 같은 패턴이다.
# ★ 2026-09-26: 1m -> 2m (사용자 지시, task_manager.py 와 같은 값 유지).
STACK_APPROACH_CREEP_M = 2.0


def back_off(pos_yaw, dist_m):
    """(pos xyz, yaw_deg) 가 보는 방향의 반대로 dist_m 만큼 뺀 대기점
    (같은 yaw, 같은 z). task_manager.py 의 _back_off() 와 동일한 계산이지만
    이 파일의 (np.array, yaw_deg) 좌표 관례를 따른다 — STACK_NAV 대기점을
    만든다."""
    pos, yaw_deg = pos_yaw
    yaw = math.radians(yaw_deg)
    back = pos.copy()
    back[0] -= dist_m * math.cos(yaw)
    back[1] -= dist_m * math.sin(yaw)
    return (back, yaw_deg)


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정 — 12_pick_test.py / 13_stack_scan_pick_place.py 재사용
#  (스택도 매거진과 같은 1 kg 급)
# ══════════════════════════════════════════════════════════════
COAXIAL_FORCE_LIMIT = 200.0   # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 100.0   # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.03    # m

SUCTION_FACE_Z = 0.161
TCP_OFFSET     = np.array([0.0, 0.0, SUCTION_FACE_Z])

# GRIP 재시도 사다리. 흡착면을 대상 윗면보다 이만큼 위에 둔다(음수=살짝 누름).
# 앞에서부터 재시도. 실험으로 바꿔 볼 수 있게 열어 둔다:
#   PICK_GRIP_GAPS=0.002,0.000,-0.003
GRIP_GAPS = [float(v) for v in
             os.environ.get("PICK_GRIP_GAPS", "0.005,0.002,0.000,-0.003").split(",")]


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
APPROACH_HEIGHT_OFFSET = 0.15
LIFT_HEIGHT_OFFSET     = 0.10

# ── OBSERVE(손목캠 파지점 재확인) — grasp.yaml flange_vision 그대로 ──────
# ★ 2026-09-25: 실제 robot1 이 STACK_PICK 에서 NO_FLANGE(3) x3 로 얼어붙은
#   원인을 mission.log 로 추적한 결과, 실패 지점은 APPROACH/DESCEND 가
#   아니라 그 앞의 OBSERVE(pick_place_server._execute → sim_backend.
#   pick_observe_flange)였다 — "관측 자세 IK 실패". 이 값은 APPROACH_
#   HEIGHT_OFFSET(0.15)보다 10cm 더 높이(observe_cam_height_m=0.25) 손목캠을
#   띄워야 해서, 이전에 REACHABILITY CHECK 로 검증한 APPROACH 보다 더 빡빡한
#   목표다 — 이 테스트가 그동안 그 사실 자체를 몰랐다(PickFSM 에 OBSERVE
#   단계가 없다, GT 로 바로 APPROACH 부터 시작했다). 값 출처:
#   src/cobot3_bringup/config/grasp.yaml 의 flange_vision.observe_cam_height_m
#   / observe_image_offset_px — 바뀌면 여기도 같이 고칠 것.
OBSERVE_CAM_HEIGHT_M = 0.25
OBSERVE_IMAGE_OFFSET_PX = (0.0, -110.0)

GRIP_WAIT    = 90
HOLD_WAIT    = 120
SETTLE_STEPS = 90
BASE_MOVE_SETTLE_STEPS = 60

TCP_SPEED = 0.002
MIN_STEPS = 90
MAX_STEPS = 600

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG     = 0.0

MAX_IK_FAIL_STEPS = 30

LOG_INTERVAL = 60


# ══════════════════════════════════════════════════════════════
#  성공 판정 기준 — project-plan.html S4 그대로 적용
# ══════════════════════════════════════════════════════════════
LIFT_OK_MIN_M        = 0.005
TILT_MAX_DEG         = 5.0


# ══════════════════════════════════════════════════════════════
#  fail_code — project-plan.html 체계 + 확장(6)
# ══════════════════════════════════════════════════════════════
FAIL_OK          = 0
FAIL_NOT_FOUND   = 1   # QR 미인식/기대 ID 불일치 (SCAN 단계)
FAIL_SLIP        = 3
FAIL_COLLISION   = 4   # 재이동 중 차체가 대상과 너무 가까워져 방어 정지
FAIL_TIMEOUT     = 5
FAIL_UNREACHABLE = 6
FAIL_NO_FLANGE   = 7   # OBSERVE: IK 는 풀렸는데 detect_flange 가 못 찾음


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 12_pick_test.py 와 동일
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
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
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def observe_tcp_from_prior(prior_world, R_l6_opt, t_l6_cam, K, target_quat,
                           cam_height_m=OBSERVE_CAM_HEIGHT_M,
                           image_offset_px=OBSERVE_IMAGE_OFFSET_PX):
    """sim_backend.pick_observe_flange() 의 observe_tcp 계산을 그대로 옮긴 것.

    prior_world 위로 손목캠을 cam_height_m 만큼 띄워 top-down 으로 내려다볼
    tool0(=TCP, TCP_OFFSET 포함) 목표를 구한다. R_l6_opt 는 StackQRScan 이
    이미 R_l6_cam @ R_cam_opt 로 합쳐 갖고 있다(그 클래스 주석) — sim_backend
    가 따로 쓰는 R_l6_cam/R_cam_opt 와 같은 값이라 그대로 재사용한다.
    """
    R_tool = quat_to_matrix(target_quat)
    R_wo = R_tool @ R_l6_opt
    du, dv = image_offset_px
    p_c = np.array([du / K[0, 0] * cam_height_m, dv / K[1, 1] * cam_height_m, cam_height_m])
    cam_pos = np.asarray(prior_world, dtype=float) - R_wo @ p_c
    tool0_pos = cam_pos - R_tool @ t_l6_cam
    return tool0_pos + R_tool @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


def ease(alpha):
    a = float(np.clip(alpha, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def tilt_deg_from_quat(quat_wxyz):
    R = quat_to_matrix(quat_wxyz)
    up = R @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def quat_from_matrix(R):
    """회전행렬 -> (w,x,y,z) 쿼터니언 (Shepperd's method)."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _quat_xyzw_to_mat(q):
    x, y, z, w = q
    return quat_to_matrix([w, x, y, z])


# 손목카메라 광학 프레임을 QR 라벨(서쪽 벽, world -X 를 본다)쪽으로 겨눌 때
# 원하는 자세 — forward(광학 +Z) = world +X, down(광학 +Y) = world -Z,
# right(광학 +X) = world -Y. FlangeVision(12_pick_test.py) 의 위에서 내려
# 보는 자세와 달리 옆에서 수평으로 보는 자세라 따로 정의한다.
R_OPT_WORLD_QR_VIEW = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def qr_aim_flange_pose(R_l6_opt, t_l6_cam, label_center, standoff_m=QR_STANDOFF_M):
    """라벨 정면(world -X 쪽) standoff_m 만큼 떨어진 자리에서 카메라가 라벨을
    수평으로 정면으로 보도록 하는 flange(link_6) 목표 (position, quat_wxyz).

    FlangeVision.observe_tcp()(12_pick_test.py)와 같은 방식 — 원하는 카메라
    world pose 에서 거꾸로 링크6(tool0) pose 를 구한다. 다만 여기서는 흡착
    TCP_OFFSET 을 안 쓴다(카메라를 겨누는 것이지 흡착 접촉점을 잡는 게
    아니다) — solver 에는 flange(=link_6) 목표를 직접 준다.
    """
    R_tool_world = R_OPT_WORLD_QR_VIEW @ R_l6_opt.T
    cam_pos = np.asarray(label_center, dtype=float) + np.array([-standoff_m, 0.0, 0.0])
    flange_pos = cam_pos - R_tool_world @ t_l6_cam
    return flange_pos, quat_from_matrix(R_tool_world)


# ══════════════════════════════════════════════════════════════
#  USD 조회 유틸 — 12_pick_test.py / 13_stack_scan_pick_place.py 와 동일
# ══════════════════════════════════════════════════════════════
def stage():
    return omni.usd.get_context().get_stage()


def find_prim_path(root_path, name):
    root = stage().GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def resolve_articulation_root(candidates, fallback):
    for path in candidates:
        prim = stage().GetPrimAtPath(path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    return fallback


def get_world_pose(prim_path):
    """prim 의 world (pos, quat_wxyz) — Gf.Transform 으로 분해한다.

    ★ 실측 — Matrix4d.ExtractRotationQuat() 은 스케일이 낀 행렬(우리 스택은
    xformOp:scale=2)에서 스케일을 제대로 나누지 않고 norm 1.44 짜리 가짜
    쿼터니언을 준다(12_pick_test.py/13_stack_scan_pick_place.py 도 이
    함수를 그대로 썼는데, 매거진은 스케일이 없어 안 드러났던 버그다).
    reset_stack_pose() 로 그 값을 그대로 넣으면 PhysX 가
    "PxRigidDynamic::setGlobalPose: pose is not valid" 로 거부하고, LIFT
    판정의 tilt_deg_from_quat() 도 조용히 틀린 값을 낸다. Gf.Transform 은
    스케일/시어를 제대로 나눠 단위 쿼터니언을 준다(실측 norm 1.0)."""
    prim = stage().GetPrimAtPath(prim_path)
    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = Gf.Transform(m).GetRotation().GetQuat()
    imag = q.GetImaginary()
    return np.array([t[0], t[1], t[2]]), np.array([q.GetReal(), imag[0], imag[1], imag[2]])


def measure_prim(prim_path):
    prim = stage().GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        raise RuntimeError(f"{prim_path} 의 바운딩박스가 비어 있다")
    lo, hi = rng.GetMin(), rng.GetMax()
    center_xy = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
    return center_xy, float(hi[2]), float(hi[2] - lo[2])


def reset_prim_pose(prim_path, pos, quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """prim 의 월드 pose 를 직접 갈아끼운다 — 물리 뷰 핸들이 없을 때의
    폴백(reset_stack_pose 참고)."""
    xform = UsdGeom.Xformable(stage().GetPrimAtPath(prim_path))
    xform.ClearXformOpOrder()
    q = Gf.Quatd(quat_wxyz[0], Gf.Vec3d(*quat_wxyz[1:]))
    m = Gf.Matrix4d().SetRotate(q)
    m.SetTranslateOnly(Gf.Vec3d(*pos))
    xform.AddTransformOp().Set(m)


def spawn_stack_on_shelf(stage_, stack_path, slot_index=STACK_SLOT_INDEX):
    """packaging_flow.py 의 _spawn_stack()+_move_to_shelf() 최종 결과(선반
    칸 위의 스택)를 직접 재현한다 — 모듈 docstring "스택을 어떻게 놓는가"
    절 참고. Task.set_up_scene() 안(= world.reset() 이전)에서만 불러야
    한다 — 그래야 payload 의 물리 API 가 첫 물리씬 빌드에 같이 등록된다."""
    prim = stage_.DefinePrim(stack_path, "Xform")
    prim.GetPayloads().AddPayload(STACK_PAYLOAD)
    # ★ 실측 — 여기서 Load() 를 안 하면 flange_plate 등 payload 자식이 전혀
    #   합성되지 않아 measure_prim() 이 "Invalid prim: null prim"으로 죽는다.
    prim.Load()
    xform = UsdGeom.XformCommonAPI(prim)
    xform.SetTranslate(Gf.Vec3d(SHELF_X, SHELF_SLOT_Y[slot_index], SHELF_Z))
    xform.SetRotate(Gf.Vec3f(0.0, 0.0, 90.0))
    # ★ 2026-09-26: packaging_flow.py(production, ORANGE_STACK_PAYLOAD)도
    #   이 자산을 2.0 배로 스폰한다 — 지금까지 이 테스트에서 "스택"이라고
    #   본 크기가 그 2.0 배 크기다. 사용자 지시로 그걸 절반(=1.0, F3_STKB_1.
    #   usda 원본 그대로, x/y/z 전부)으로 줄인다 — 원본 에셋 파일이나
    #   packaging_flow.py(생산)는 안 건드린다, 이 테스트에서 스폰하는
    #   인스턴스만 작아진다.
    xform.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
    # ★ SHELF_X/Z 가 이제 실제 ShelfDeck(PhysicsCollisionAPI 있음) 위를
    #   가리키므로, 예전에 썼던 안 보이는 스캐폴딩 콜라이더는 더 필요 없다.

    # ★ 2026-09-26(2): F3_STKB_1.usda 의 physics:mass=1(kg) 은 명시적
    #   오버라이드라 위 SetScale() 과 무관하게 그대로다 — 부피는 0.5^3=1/8로
    #   줄었는데 질량은 그대로면 밀도가 8배가 되어(그만큼 흡착 접촉면당
    #   하중이 커진다), LIFT 도중 놓치는 사례가 나왔다(사용자 실측
    #   fail_code=3, GRIP 은 성공했는데 LIFT 10/90 스텝에서 놓침). 자산
    #   자체의 문서 의도("uniform density")대로 부피비 그대로 질량도
    #   줄인다 — 여기서도 이 테스트가 스폰하는 인스턴스만 바꾼다
    #   (F3_STKB_1.usda/packaging_flow.py 는 그대로).
    UsdPhysics.MassAPI(prim).CreateMassAttr().Set(1.0 * (0.5 ** 3))


# 스택의 리지드바디 핸들 — world.reset() 뒤에 채운다(12_pick_test.py 의
# bind_magazine_rigid 와 같은 이유: 트라이얼 사이에 USD 쓰기만으로 위치를
# 되돌리면 물리 스텝이 다음 프레임에 PhysX 값으로 덮어써 버린다).
_stack_rigid = None


def bind_stack_rigid():
    global _stack_rigid
    try:
        from isaacsim.core.prims import SingleRigidPrim
        _stack_rigid = SingleRigidPrim(prim_path=STACK_XFORM_PATH, name="stack_rigid")
        _stack_rigid.initialize()
        print(f"   stack        리지드바디 핸들 확보 {STACK_XFORM_PATH}")
    except Exception as exc:
        _stack_rigid = None
        print(f"   !! stack     리지드바디 핸들 실패 ({exc}) — "
              f"USD 쓰기로 떨어진다. 트라이얼 간 위치가 안 맞을 수 있다.")


def reset_stack_pose(spawn_pos, spawn_quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """트라이얼 사이에 스택을 원래 선반 자리로 되돌린다(속도도 함께
    턴다 — 12_pick_test.py 의 reset_magazine_pose 와 같은 이유: 안 털면
    직전 트라이얼의 운동량이 남아 스택이 트라이얼마다 조금씩 밀려난다)."""
    if _stack_rigid is not None:
        try:
            _stack_rigid.set_world_pose(
                position=np.asarray(spawn_pos, dtype=float),
                orientation=np.asarray(spawn_quat_wxyz, dtype=float))
            _stack_rigid.set_linear_velocity(np.zeros(3))
            _stack_rigid.set_angular_velocity(np.zeros(3))
            return
        except Exception as exc:
            print(f"   !! stack     물리 리셋 실패 ({exc}) — USD 쓰기로 떨어진다")
    reset_prim_pose(STACK_PATH, spawn_pos, spawn_quat_wxyz)


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 제어 — 12_pick_test.py 와 동일
# ══════════════════════════════════════════════════════════════
class SurfaceGripperCtl:
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
class StackScanPickTask(BaseTask):
    """simple_factory_layout.usda 를 그대로 올리고, 스택 하나를 PKG-OUT
    선반 칸에 스폰한 뒤 로봇/그리퍼 파라미터를 조정한다."""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_world()
        self._spawn_stack()
        self._setup_arm_drives()
        self._configure_gripper_limits()
        # ★ 2026-09-24: 로봇<->스택 충돌 필터링을 뺐다(_filter_gripper_target_
        #   collision, 아래 남겨둔 정의 참고) — 사용자 지시로 "이동하다가
        #   스택을 건드려 넘어뜨리는" 실패를 재현해 보는 게 이번 목적이라,
        #   그걸 가리는 필터를 켜 두면 애초에 재현이 안 된다. 원래 이 필터는
        #   텔레포트로 스택 옆에 "순간이동"할 때 한 프레임 만에 깊이 겹쳐
        #   물리 솔버가 스택을 튕겨 보내는 걸 막으려던 것이었다 — WP_SCAN->
        #   WP_PICK 구간을 이제 BaseTeleporter.drive_to() 로 실제로(조금씩)
        #   주행시키므로(아래), 그 원인 자체가 없어져 필터가 더 필요 없다.
        self._register_robot(scene)
        print("   scene        ready")

    def _load_world(self):
        stg = stage()
        world_prim = stg.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stg, "/World").GetPrim()
        world_prim.GetReferences().AddReference(WORLD_USD)
        for _ in range(15):
            simulation_app.update()
        print(f"   USD          loaded  {WORLD_USD}")

    def _spawn_stack(self):
        # world.reset() 이전에 붙인다 — spawn_stack_on_shelf() 독스트링 참고.
        spawn_stack_on_shelf(stage(), STACK_XFORM_PATH)
        for _ in range(5):
            simulation_app.update()
        print(f"   stack        spawned  {STACK_XFORM_PATH}  (slot {STACK_SLOT_INDEX})")

    def _setup_arm_drives(self):
        count = 0
        for prim in Usd.PrimRange(stage().GetPrimAtPath(ROBOT_PRIM_PATH)):
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
        node_path = find_prim_path(GRIPPER_PRIM, "SurfaceGripper")
        if node_path is None:
            raise RuntimeError(f"SurfaceGripper node not found under {GRIPPER_PRIM}")
        node = stage().GetPrimAtPath(node_path)
        node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
        node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
        node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)
        self._gripper_node_path = node_path
        print(f"   gripper      {node_path}")
        print(f"   grip limits  coaxial {COAXIAL_FORCE_LIMIT:.0f} N  "
              f"shear {SHEAR_FORCE_LIMIT:.0f} N  "
              f"maxGripDistance {MAX_GRIP_DISTANCE*1000:.0f} mm")

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
    pos, quat = get_world_pose(BASE_LINK_PATH)
    lula.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
    return pos, quat


# ══════════════════════════════════════════════════════════════
#  베이스 텔레포트 — 12_pick_test.py 와 동일
# ══════════════════════════════════════════════════════════════
def yaw_quat_wxyz(yaw_deg):
    return quat_from_axis([0, 0, 1], yaw_deg)


BASE_DRIVE_STEP_M = 0.01
BASE_DRIVE_MIN_STEPS = 60
BASE_DRIVE_MAX_STEPS = 500

# ── 방어 코드 — 차체 사각형 vs 대상 사각형 여유거리 ────────────────────
# shelves.yaml 의 실측 차체 사각형(앞 0.14 · 뒤 0.607 · 반폭 0.25+패딩 0.03,
# PKG-OUT/SHELF-A 주석 참고)을 그대로 쓴다. "SCAN->PICK 이동 중 스택을
# 건드려 넘어뜨렸다"(사용자 보고)를 재현해 보니, base_link 기준 최소거리가
# 0.521 m 인데도 이 사각형(뒤 0.607 m!)과 스택 자체 반폭(0.137 m)을 감안하면
# 실제 여유는 9 cm 밖에 안 됐다 — teleport() 기반 검증(REACHABILITY CHECK 등)
# 은 끝점의 IK 만 보지, 그 경로/끝점에서 차체가 실제로 얼마나 가까워지는지는
# 한 번도 확인한 적이 없었다.
CARTER_FOOTPRINT_FRONT_M = 0.14
CARTER_FOOTPRINT_REAR_M  = 0.607
CARTER_FOOTPRINT_HALF_WIDTH_M = 0.28   # 반폭 0.25 + 패딩 0.03

# 이 밑으로 떨어지면 drive_to() 가 그 자리에서 멈춘다(FAIL_COLLISION).
# ★ 2026-09-25: pkg_out_pick 접근 크립 — 80mm 에서 27/99 스텝 전, 60mm 에서
#   8/99 스텝 전까지 좁혀졌다(둘 다 GUI로 직접 보고 사용자가 "여유있다"
#   확인). 50mm 로 낮췄더니 [+3.481 +0.971]에서 여전히 걸렸다(대기점을
#   1m/2m 어느 쪽으로 둬도 이 지점은 그대로 — 목적지 근처 스택 옆을
#   지나는 지점이라 대기 거리와 무관하다). 2026-09-26: 더 낮춘다 — 여기서
#   직접 조정하며 테스트할 것.
COLLISION_SAFE_MARGIN_M = 0.003


def carter_footprint_corners(pos_xy, yaw_deg):
    """base_link (pos_xy, yaw_deg) 에서의 차체 사각형 네 꼭짓점(world, 순서
    있음) — 회전된 그대로, AABB 로 감싸지 않는다(아래 rect_clearance_m
    참고)."""
    yaw = math.radians(yaw_deg)
    fwd = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    p = np.asarray(pos_xy, dtype=float)
    return [
        p + fwd * CARTER_FOOTPRINT_FRONT_M + left * CARTER_FOOTPRINT_HALF_WIDTH_M,
        p + fwd * CARTER_FOOTPRINT_FRONT_M - left * CARTER_FOOTPRINT_HALF_WIDTH_M,
        p - fwd * CARTER_FOOTPRINT_REAR_M - left * CARTER_FOOTPRINT_HALF_WIDTH_M,
        p - fwd * CARTER_FOOTPRINT_REAR_M + left * CARTER_FOOTPRINT_HALF_WIDTH_M,
    ]


def _point_seg_dist(p, a, b):
    """점 p 와 선분 ab 사이 최단거리."""
    ab = b - a
    denom = float(np.dot(ab, ab))
    t = float(np.dot(p - a, ab) / denom) if denom > 1e-12 else 0.0
    t = max(0.0, min(1.0, t))
    return float(np.linalg.norm(p - (a + t * ab)))


def _point_aabb_dist(p, aabb):
    """점 p 와 축정렬 사각형(xmin,xmax,ymin,ymax) 사이 최단거리(안이면 0)."""
    xmin, xmax, ymin, ymax = aabb
    dx = max(xmin - p[0], 0.0, p[0] - xmax)
    dy = max(ymin - p[1], 0.0, p[1] - ymax)
    return math.hypot(dx, dy)


def _polygons_overlap_sat(poly_a, poly_b):
    """두 볼록사각형이 겹치는지 SAT(분리축 정리)로 확인 — 네 변의 법선
    (사각형이라 축 4개, 평행한 변은 축이 같아 사실상 2+2)만 검사하면
    충분하다(두 사각형 모두 볼록이라 이 변의 법선들이 곧 분리축 후보다)."""
    for poly in (poly_a, poly_b):
        n = len(poly)
        for i in range(n):
            edge = poly[(i + 1) % n] - poly[i]
            axis = np.array([-edge[1], edge[0]])
            norm = np.linalg.norm(axis)
            if norm < 1e-12:
                continue
            axis = axis / norm
            proj_a = [float(np.dot(v, axis)) for v in poly_a]
            proj_b = [float(np.dot(v, axis)) for v in poly_b]
            if max(proj_a) < min(proj_b) or max(proj_b) < min(proj_a):
                return False
    return True


def rect_clearance_m(car_corners, stack_aabb):
    """회전된 차체 사각형(꼭짓점 4개, 순서 있음)과 축정렬 스택 사각형 사이
    **정확한** 최단거리. 겹치면 0.0.

    ★ 2026-09-26: 예전 stack_clearance_m() 은 차체를 "회전된 사각형 -> 그
    네 꼭짓점을 다시 축정렬 박스(AABB)로 감싸기" 로 근사했다 — yaw 가
    90도 배수가 아니면(WP_SCAN 의 -82.83°처럼) 실제보다 더 넓게(더
    가깝게) 잡혀서, 진짜 위험은 그대로인데 계산만 더 보수적으로 나온다.
    여기서는 감싸지 않고 회전된 사각형 그대로 스택 AABB 와의 최단거리를
    잰다 — 같은 차체 모델을 더 정확히 재는 것뿐이라 실제 충돌 위험은
    그대로 두고, 근사가 깎아 먹던 여유거리만 되찾는다(사용자 지시)."""
    xmin, xmax, ymin, ymax = stack_aabb
    stack_poly = [
        np.array([xmin, ymin]), np.array([xmax, ymin]),
        np.array([xmax, ymax]), np.array([xmin, ymax]),
    ]
    car_poly = [np.asarray(c, dtype=float) for c in car_corners]

    if _polygons_overlap_sat(car_poly, stack_poly):
        return 0.0

    best = min(_point_aabb_dist(v, stack_aabb) for v in car_poly)
    n = len(car_poly)
    for v in stack_poly:
        for i in range(n):
            best = min(best, _point_seg_dist(v, car_poly[i], car_poly[(i + 1) % n]))
    return best


def stack_clearance_m(base_pos_xy, base_yaw_deg, stack_aabb):
    """이 base pose 에서 차체 사각형과 스택 AABB 사이 여유거리."""
    return rect_clearance_m(carter_footprint_corners(base_pos_xy, base_yaw_deg), stack_aabb)


def measure_prim_aabb_xy(prim_path):
    """prim 의 world AABB 를 (xmin,xmax,ymin,ymax) 로 — stack_clearance_m 용."""
    prim = stage().GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    return float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1])


class BaseTeleporter:
    def __init__(self, robot):
        self._robot = robot
        self._last_yaw_deg = 0.0

    def teleport(self, pos_xyz, yaw_deg):
        quat = yaw_quat_wxyz(yaw_deg)
        self._robot.set_world_pose(position=np.array(pos_xyz), orientation=quat)
        try:
            self._robot.set_linear_velocity(np.zeros(3))
            self._robot.set_angular_velocity(np.zeros(3))
        except Exception:
            pass
        self._last_yaw_deg = yaw_deg

    def drive_to(self, world, target_pos_xyz, target_yaw_deg, render, label="",
                 stack_aabb=None):
        """실제 patrol_to(cmd_vel 직접주행)를 흉내낸다 — teleport() 처럼
        한 프레임에 순간이동하지 않고, 현재 위치에서 목표까지 잘게 쪼개
        조금씩 옮기며 매 스텝 물리를 돌린다. ★ 왜 필요한가 — 사용자가
        실제 로봇에서 "SCAN 자리에서 PICK 자리로 이동하다가 스택을 건드려
        넘어뜨렸다"고 보고했는데, teleport() 는 중간 경로를 아예 지나가지
        않아(순간이동) 이 경로상 충돌을 원천적으로 재현할 수 없었다. 여기
        서는 매 스텝 world.step() 을 실제로 돌리므로, 도중에 로봇 콜라이더가
        스택과 겹치면 물리 솔버가 그 결과를 그대로 반영한다(충돌 필터링을
        뺀 것과 세트 — StackScanPickTask.set_up_scene 주석 참고).

        ★ 방어 코드 — stack_aabb 를 주면 매 스텝 차체 사각형(carter_footprint_
        aabb) 여유거리를 확인하다가 COLLISION_SAFE_MARGIN_M 밑으로 떨어지면
        "닿기 전에" 그 자리에서 멈추고 False 를 돌려준다(호출부가 FAIL_
        COLLISION 으로 처리한다). Nav2(navigate_to) 대신 patrol_to(cmd_vel
        직접주행)를 쓰기로 하면서 Nav2 의 장애물 회피/정지를 포기한 거라,
        이 정도 최소한의 근접 정지는 이쪽에서 대신 넣어야 한다(사용자 지시).

        반환: True 면 목표까지 안전하게 도착, False 면 방어 정지로 중단."""
        start_pos, _ = self._robot.get_world_pose()
        start_pos = np.array(start_pos, dtype=float)
        goal = np.array(target_pos_xyz, dtype=float)
        start_yaw = self._last_yaw_deg
        dist = float(np.linalg.norm(goal[:2] - start_pos[:2]))
        n_steps = int(np.clip(dist / BASE_DRIVE_STEP_M, BASE_DRIVE_MIN_STEPS, BASE_DRIVE_MAX_STEPS))
        print(f"   [drive] {label:9s} {vec(start_pos)} -> {vec(goal)}  "
              f"{dist:.3f} m  {n_steps} steps")
        min_clearance = float("inf")
        for i in range(1, n_steps + 1):
            a = ease(i / float(n_steps))
            pos = start_pos + a * (goal - start_pos)
            yaw = start_yaw + a * (target_yaw_deg - start_yaw)

            if stack_aabb is not None:
                clearance = stack_clearance_m(pos[:2], yaw, stack_aabb)
                min_clearance = min(min_clearance, clearance)
                if clearance < COLLISION_SAFE_MARGIN_M:
                    print(f"   !! [drive] {label:9s} 여유거리 {clearance*1000:.0f} mm "
                          f"(< {COLLISION_SAFE_MARGIN_M*1000:.0f} mm) — {i}/{n_steps} 스텝에서 "
                          f"방어 정지 {vec(pos)}")
                    self._last_yaw_deg = yaw
                    return False

            self.teleport(pos, yaw)
            world.step(render=render)
        self._last_yaw_deg = target_yaw_deg
        if stack_aabb is not None:
            print(f"   [drive] {label:9s} 최소 여유거리 {min_clearance*1000:.0f} mm "
                  f"(>= {COLLISION_SAFE_MARGIN_M*1000:.0f} mm 기준)")
        return True


def move_to_flange_pose(world, robot, solver, target_pos, target_quat, label):
    """현재 flange(link_6) pose 에서 목표까지 smoothstep 보간 + 매 스텝 IK로
    옮긴다 — 12_pick_test.py 의 servo_tcp() 와 같은 방식이지만, 흡착
    TCP_OFFSET 을 거치지 않고 flange 좌표를 직접 다룬다(카메라 조준용 —
    qr_aim_flange_pose() 가 이미 flange 목표를 준다)."""
    start = robot.end_effector.get_world_pose()[0]
    n_steps, dist = steps_for(start, target_pos)
    print(f"   [-] {label:9s} goal {vec(target_pos)}  {dist:.4f} m  {n_steps} steps")
    fail = 0
    for i in range(1, n_steps + 1):
        world.step(render=not HEADLESS)
        pos = start + ease(i / float(n_steps)) * (target_pos - start)
        action, solved = solver.compute_inverse_kinematics(
            target_position=pos, target_orientation=target_quat)
        if solved:
            robot.apply_action(action)
            fail = 0
        else:
            fail += 1
            if fail >= MAX_IK_FAIL_STEPS:
                return False
    return True


def servo_joint_deg(world, robot, target_joints_deg, n_steps=SETTLE_STEPS):
    """관절 목표(도)로 매 스텝 다시 명령해 부드럽게 움직인다.

    sim_backend.RigBackend._servo_joint_deg() 와 똑같은 방식이다 —
    IK 를 안 푼다, 그냥 관절 공간에서 직선 보간한다. carrier_code_reader.
    _observe() → observe_pose(joints_deg=...) 가 실제로 하는 일이 이거다
    (qr_aim_flange_pose()/move_to_flange_pose() 는 그 목표를 *찾을 때*나
    쓰는 거지, 실제 관측 자세로 가는 방법이 아니다)."""
    idx = [robot.get_dof_index(j) for j in ARM_JOINTS]
    start_deg = np.degrees(robot.get_joint_positions()[idx])
    target_deg = np.array(target_joints_deg, dtype=float)
    for i in range(1, n_steps + 1):
        world.step(render=not HEADLESS)
        cur_deg = start_deg + ease(i / float(n_steps)) * (target_deg - start_deg)
        robot.apply_action(ArticulationAction(
            joint_positions=np.deg2rad(cur_deg), joint_indices=idx))


# ══════════════════════════════════════════════════════════════
#  QR 외부 디코더 — sim_backend.py 와 완전히 같은 코드다(2026-09-26).
#  Isaac 번들 cv2 에 WeChat 이 없어서(hasattr 확인) 시스템 python3 에
#  opencv-contrib-python 을 설치해 그쪽으로 디코딩만 넘긴다. 이 테스트가
#  "QR 이 안 보인다"(실은 cv2.QRCodeDetector 가 크롭 마진 52px 에서만
#  실패)를 밝혀낸 그 자리다 — sim_backend.py 를 고치면서 이 테스트도
#  같은 경로로 검증되게 맞춘다.
# ══════════════════════════════════════════════════════════════
QR_DECODE_PYTHON = os.environ.get("QR_DECODE_PYTHON", "/usr/bin/python3")

_QR_SERVER_SRC = (
    "import sys, cv2\n"
    "d = cv2.wechat_qrcode_WeChatQRCode()\n"
    "sys.stdout.write('READY\\n'); sys.stdout.flush()\n"
    "for line in sys.stdin:\n"
    "    p = line.strip()\n"
    "    if not p:\n"
    "        continue\n"
    "    try:\n"
    "        img = cv2.imread(p)\n"
    "        r = d.detectAndDecode(img)\n"
    "        t = [x for x in (r[0] if isinstance(r, tuple) else r) if x]\n"
    "        out = t[0] if t else ''\n"
    "    except Exception:\n"
    "        out = ''\n"
    "    sys.stdout.write(out + '\\n'); sys.stdout.flush()\n"
)

_qr_srv = None
_qr_srv_failed = False


def _qr_decoder_env():
    env = dict(os.environ)
    env.pop("PYTHONHOME", None)
    env.pop("PYTHONPATH", None)
    return env


def _qr_server():
    global _qr_srv, _qr_srv_failed
    if _qr_srv is not None or _qr_srv_failed:
        return _qr_srv
    try:
        in_r, in_w = os.pipe()
        out_r, out_w = os.pipe()
        exe = shutil.which(QR_DECODE_PYTHON) or QR_DECODE_PYTHON
        pid = os.posix_spawn(
            exe, [exe, "-c", _QR_SERVER_SRC], _qr_decoder_env(),
            file_actions=[
                (os.POSIX_SPAWN_DUP2, in_r, 0),
                (os.POSIX_SPAWN_DUP2, out_w, 1),
                (os.POSIX_SPAWN_CLOSE, in_w),
                (os.POSIX_SPAWN_CLOSE, out_r),
            ],
        )
        os.close(in_r); os.close(out_w)
        rf = os.fdopen(out_r, "r")
        if rf.readline().strip() != "READY":
            raise RuntimeError("디코더가 READY 를 안 보냈다")
        _qr_srv = (pid, in_w, rf)
        print(f"   QR 외부 디코더(WeChat) 상주 시작  pid={pid}  ({QR_DECODE_PYTHON})")
    except Exception as e:
        print(f"   !! 외부 QR 디코더를 못 띄웠다: {type(e).__name__}: {e}")
        _qr_srv_failed = True
        _qr_srv = None
    return _qr_srv


def _external_qr_decode(image):
    global _qr_srv, _qr_srv_failed
    srv = _qr_server()
    if srv is None:
        return ""
    import cv2
    pid, w_fd, rf = srv
    tmp = Path("/tmp") / "_cobot3_extdecode_test.png"
    try:
        if not cv2.imwrite(str(tmp), image):
            return ""
        os.write(w_fd, (str(tmp) + "\n").encode())
        return rf.readline().strip()
    except Exception as e:
        print(f"   !! 외부 QR 디코더 통신 실패: {type(e).__name__}: {e}")
        _qr_srv, _qr_srv_failed = None, True
        return ""


# ══════════════════════════════════════════════════════════════
#  QR 스캔 — 손목카메라로 스택의 qr_label_py 를 찍어 cobot3_perception.qr_pose
#  로 디코드/확인한다(12_pick_test.py 의 FlangeVision 과 같은 재사용 방식 —
#  frames.yaml 카메라 내부/외부 파라미터를 그대로 쓴다)
# ══════════════════════════════════════════════════════════════
class StackQRScan:
    def __init__(self):
        import yaml
        from cobot3_perception.qr_pose import estimate_qr_pose, set_external_decoder
        self._estimate_qr_pose = estimate_qr_pose
        set_external_decoder(_external_qr_decode)

        frames = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
        ci = frames["wrist_camera"]["camera_info_observed"]
        self.K = np.array(ci["k"], dtype=float).reshape(3, 3)
        self.dist = np.array(ci["d"], dtype=float)
        self.resolution = (ci["width"], ci["height"])
        st = frames["static_transforms"]
        self.t_l6_cam = np.array(st["m0609_tool0__camera_link"]["xyz"], dtype=float)
        self.R_l6_opt = (_quat_xyzw_to_mat(st["m0609_tool0__camera_link"]["quat_xyzw"])
                         @ _quat_xyzw_to_mat(
                             st["camera_link__camera_color_optical_frame"]["quat_xyzw"]))

        import omni.replicator.core as rep
        self._rp = rep.create.render_product(CAMERA_PRIM, self.resolution)
        self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self._rgb.attach([self._rp])
        self._depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        self._depth.attach([self._rp])
        print(f"   qr camera    {CAMERA_PRIM}  {self.resolution}")

    def capture_raw(self, world, render_steps=15):
        """rgb/depth 한 장 + 그 순간 카메라 world pose(R_wo, p_wo). None 이면 캡처 실패.

        run_trial_real_pipeline() 의 OBSERVE(detect_flange)가 QR 디코드 없이
        이 원본 캡처만 재사용한다 — capture_and_decode() 와 캡처 로직을
        하나로 둔다."""
        for _ in range(render_steps):
            world.step(render=True)
        img = np.asarray(self._rgb.get_data())
        if img.size == 0:
            return None
        bgr = img[:, :, :3][:, :, ::-1].copy()
        depth = np.asarray(self._depth.get_data(), dtype=np.float64)
        depth = depth.reshape(bgr.shape[:2]) if depth.size == bgr.shape[0] * bgr.shape[1] else None

        l6_p, l6_q = get_world_pose(EE_LINK_PATH)
        R_l6 = quat_to_matrix(l6_q)
        R_wo = R_l6 @ self.R_l6_opt
        p_wo = l6_p + R_l6 @ self.t_l6_cam
        return bgr, depth, R_wo, p_wo

    def capture_and_decode(self, world, expected_id=EXPECTED_QR_ID, render_steps=15):
        raw = self.capture_raw(world, render_steps=render_steps)
        if raw is None:
            return None
        bgr, depth, R_wo, p_wo = raw
        return self._estimate_qr_pose(bgr, depth, self.K, self.dist, R_wo, p_wo,
                                      expected_id=expected_id)


# ══════════════════════════════════════════════════════════════
#  Pick 단계 FSM — 12_pick_test.py / 13_stack_scan_pick_place.py 와 동일
#  (FLANGE_PATH/STACK_PATH 만 교체)
# ══════════════════════════════════════════════════════════════
class PickFSM:
    """
      0 APPROACH  1 DESCEND  2 GRIP(재시도)  3 HOLD  4 LIFT  5 DONE
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "HOLD", "LIFT", "DONE"]
    DONE_STATE = 5

    def __init__(self, robot, gripper, center_xy=None, obj_top=None):
        self._robot = robot
        self._gripper = gripper
        # center_xy/obj_top 를 주면 GT(measure_prim) 대신 그 값으로 판다 —
        # 실제 파이프라인처럼 비전(detect_flange)이 잰 파지점을 쓰는 경로
        # 검증용(run_trial_real_pipeline, PICK_REAL_PIPELINE=1).
        self._override_center_xy = center_xy
        self._override_obj_top = obj_top
        self.reset()

    def reset(self):
        if self._override_center_xy is not None and self._override_obj_top is not None:
            center_xy, top_z = self._override_center_xy, self._override_obj_top
        else:
            center_xy, top_z, _height = measure_prim(FLANGE_PATH)
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
            return self.waypoints[self.state]
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
                self.z_at_grip = measure_prim(STACK_PATH)[1]
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
        now = measure_prim(STACK_PATH)[1]
        self.lift_rise_m = now - self.z_at_grip
        _, quat = get_world_pose(STACK_PATH)
        self.tilt_at_lift_deg = tilt_deg_from_quat(quat)

        ok = (self.lift_rise_m >= LIFT_OK_MIN_M) and (self.tilt_at_lift_deg <= TILT_MAX_DEG)
        print(f"   판정  rise {self.lift_rise_m*1000:+.1f} mm (>= {LIFT_OK_MIN_M*1000:.0f} mm)  "
              f"tilt {self.tilt_at_lift_deg:.2f} deg (<= {TILT_MAX_DEG:.0f})  "
              f"-> {'성공' if ok else '실패'}")
        if not ok:
            self.fail_code = FAIL_SLIP


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
#  사전 검증 — WP_SCAN 한 점에서 QR 조준·top-grasp 접근 둘 다 실제로 IK 로
#  풀리는지 먼저 확인
# ══════════════════════════════════════════════════════════════
def reachability_check(world, robot, lula, solver, teleporter, target_quat, qr_scan):
    """QR 조준(SCAN)과 top-grasp 접근(PICK) 둘 다 **같은 정차점(WP_SCAN)**
    에서 IK 가 풀리는지 본다.

    ★ 2026-09-25: 예전엔 WP_SCAN 과 WP_PICK 두 정차점을 따로 뒀다(옆에서
      보기 vs 위에서 내려찍기가 요구하는 기하가 다르다고 판단해서) — 그런데
      실제 로봇에서 그 사이 재이동(STACK_PICK_MOVE)이 오히려 manipulator
      를 제대로 못 세우는 원인이었다(사용자 실측, 지금 이 자리에서는 안
      부딪힌다). 그래서 WP_PICK 을 없애고 WP_SCAN 한 점에서 SCAN 도 PICK
      도 다 한다 — 이 함수도 그 한 점에서 두 IK 를 순서대로 확인한다.
    """
    section("REACHABILITY CHECK")
    all_ok = True

    pos, yaw = WP_SCAN
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    label_xy, label_top_z, label_h = measure_prim(QR_LABEL_PATH)
    label_center = np.array([label_xy[0], label_xy[1], label_top_z - label_h / 2.0])
    aim_pos, aim_quat = qr_aim_flange_pose(qr_scan.R_l6_opt, qr_scan.t_l6_cam, label_center)
    _, solved_scan = solver.compute_inverse_kinematics(
        target_position=aim_pos, target_orientation=aim_quat)
    print(f"   WP_SCAN   aim flange {vec(aim_pos)}  -> {'SOLVED' if solved_scan else 'FAILED'}")
    all_ok = all_ok and solved_scan

    center_xy, top_z, _ = measure_prim(FLANGE_PATH)
    approach_tcp = np.array([center_xy[0], center_xy[1], top_z + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)
    _, solved_pick = solver.compute_inverse_kinematics(
        target_position=flange_target, target_orientation=target_quat)
    print(f"   WP_SCAN   approach tcp(제자리 PICK) {vec(approach_tcp)}  -> "
          f"{'SOLVED' if solved_pick else 'FAILED'}")
    all_ok = all_ok and solved_pick

    if not all_ok:
        print("   !! 하나 이상 FAILED — WP_SCAN 좌표를 조정해야 한다")

    return all_ok


# ══════════════════════════════════════════════════════════════
#  QR 조준 IK 를 풀어 관절값을 그대로 저장 (PICK_SAVE_TEACH=<이름>)
#
#  ★ 2026-09-25: 사용자가 capture_pose.py 슬라이더로 손수 grasp 지점(top-
#  grasp tcp)을 맞추려다 세 번 다 못 맞췄다 — FK(순방향)로 xyz 를 정확히
#  맞추는 건 원래 어렵다. 그런데 arm_teach_pose 는 애초에 grasp 자세가
#  아니라 **QR 관측(SCAN) 자세**다(shelves.yaml 의 다른 선반들도 전부 "QR-
#  조준 IK 를 풀어 나온 값"이라고 적혀 있다) — PICK 자체(APPROACH/OBSERVE/
#  GRIP)는 pick_place_server 가 매 사이클 실시간으로 IK 를 푼다, 고정
#  관절값을 안 쓴다. reachability_check() 가 이미 이 자리에서 QR 조준 IK
#  를 풀어 SOLVED 를 확인했으니, 그 해를 그대로 관절값으로 뽑아 capture_
#  pose.py 와 같은 스키마로 taught_poses.yaml 에 저장한다 — 사람이 손으로
#  맞출 필요가 없다.
# ══════════════════════════════════════════════════════════════
def save_teach_pose(name, world, robot, lula, solver, qr_scan):
    import yaml

    out_yaml = WS_ROOT / "isaacpjt/tools/out/taught_poses.yaml"

    label_xy, label_top_z, label_h = measure_prim(QR_LABEL_PATH)
    label_center = np.array([label_xy[0], label_xy[1], label_top_z - label_h / 2.0])
    aim_pos, aim_quat = qr_aim_flange_pose(qr_scan.R_l6_opt, qr_scan.t_l6_cam, label_center)
    action, solved = solver.compute_inverse_kinematics(
        target_position=aim_pos, target_orientation=aim_quat)
    if not solved:
        print(f"   !! save_teach_pose({name}): QR 조준 IK 가 안 풀린다 — 저장 안 함")
        return False

    robot.apply_action(action)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)

    idx = [robot.get_dof_index(j) for j in ARM_JOINTS]
    jp = robot.get_joint_positions()
    joints_rad = [float(jp[i]) for i in idx]
    joints_deg = [float(np.degrees(v)) for v in joints_rad]

    # ★ BASE_XFORM_PATH("/World/Robots/nova_carter1")는 정적 부모 프림이라
    #   teleport() 가 실제로 움직이는 chassis_link/m0609 base_link 가 아니다
    #   — 여기서 읽으면 스폰 좌표가 그대로 나온다(실측). base_link_world 는
    #   팔 베이스(BASE_LINK_PATH, sync_ik_base_pose 가 실제로 쓰는 그 프림)
    #   로 대신 읽는다.
    base_p, base_q = get_world_pose(BASE_LINK_PATH)
    arm_p, _ = get_world_pose(ROBOT_PRIM_PATH)
    l6_p, l6_q = get_world_pose(EE_LINK_PATH)
    cam_p, _ = get_world_pose(CAMERA_PRIM)

    captured = {}
    if out_yaml.exists():
        captured = yaml.safe_load(out_yaml.read_text(encoding="utf-8")) or {}
    captured[name] = dict(
        base_waypoint="pkg_out_pick",
        base_link_world=[round(float(v), 5) for v in base_p],
        base_link_quat_wxyz=[round(float(v), 6) for v in base_q],
        m0609_base_world=[round(float(v), 5) for v in arm_p],
        joints=dict(zip(ARM_JOINTS, [round(v, 4) for v in joints_deg])),
        joints_deg=[round(v, 4) for v in joints_deg],
        joints_rad=[round(v, 6) for v in joints_rad],
        tool0_world=[round(float(v), 5) for v in l6_p],
        tool0_quat_wxyz=[round(float(v), 6) for v in l6_q],
        camera_optical_world=[round(float(v), 5) for v in cam_p],
        qr_in_view=[],
        captured_at=time.strftime("%Y-%m-%d %H:%M:%S"),
        source="15_stack_scan_pick_test.py PICK_SAVE_TEACH (QR 조준 IK 계산값)",
    )
    out_yaml.parent.mkdir(parents=True, exist_ok=True)
    out_yaml.write_text(
        yaml.safe_dump(captured, allow_unicode=True, sort_keys=False), encoding="utf-8")

    print(f"\n   [{name}] QR 조준 IK 로 계산해 저장")
    print("      관절(deg) " + ", ".join(f"{v:7.2f}" for v in joints_deg))
    print(f"      tool0     {np.round(l6_p, 4).tolist()}")
    print(f"      -> {out_yaml}")
    return True


# ══════════════════════════════════════════════════════════════
#  T_QR_grasp_xyz 실측 검증/보정 — grasp.yaml 의 "★ 미실측" 값을 처음으로
#  Isaac 안에서 GT 와 직접 비교한다 (2026-09-25, mission.log STACK_PICK
#  NO_FLANGE 진단 이어서 — sweep_wp_pick_x 는 WP_PICK 이 이미 IK 로는 풀리는
#  자리임을 보여줬다. 그런데도 실제 로봇은 가끔 실패했다 — GT 대신 "QR 추정
#  + T_QR_grasp_xyz" 로 만든 prior 가 얼마나 정확한지 한 번도 본 적이 없어서
#  다음 용의자로 넘어온다.
#
#  sim_backend.scan_qr() 의 base_link 왕복(world -> base_link -> world)은
#  수학적으로 상쇄된다 — 그래서 world 프레임에서 바로 검증할 수 있다:
#      prior_world = qr_center_world + R_qr_world(yaw) @ T_QR_grasp_xyz
#  (pick_place_server._flange_pose_from_qr 의 좌표계 뒤집기 두 번(carrier_
#  code_reader 규약 변환 + 그 역변환)도 쿼터니언 부호만 바뀌고 회전 자체는
#  그대로라 상쇄된다 — 직접 대수로 확인했다.)
# ══════════════════════════════════════════════════════════════
def R_qr_world_from_yaw(yaw_rad):
    """sim_backend.scan_qr() 과 똑같은 재구성 — 라벨은 항상 upright 라
    y_qr(이미지 아래) 은 세계 -Z 로 고정, yaw 하나로 x/z 를 정한다."""
    x_qr_world = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])
    y_qr_world = np.array([0.0, 0.0, -1.0])
    z_qr_world = np.cross(x_qr_world, y_qr_world)
    return np.column_stack([x_qr_world, y_qr_world, z_qr_world])


def calibrate_grasp_offset(world, robot, lula, solver, teleporter, qr_scan, variant="tray_blue",
                           n_frames=5):
    section(f"T_QR_grasp_xyz 검증 — {variant}")
    import yaml
    from cobot3_perception.qr_pose import aggregate_qr_poses

    pos, yaw = WP_SCAN
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    # 팔을 QR 라벨 쪽으로 겨눈다 — run_trial 의 SCAN 단계와 똑같다. 이걸
    # 빼먹으면 팔이 READY 자세인 채로 캡처해서 QR 이 화면에 아예 안 잡힌다
    # (처음 돌렸을 때 "QR 이 안 보인다" x5 로 이렇게 실패했었다).
    label_xy, label_top_z, label_h = measure_prim(QR_LABEL_PATH)
    label_center = np.array([label_xy[0], label_xy[1], label_top_z - label_h / 2.0])
    aim_pos, aim_quat = qr_aim_flange_pose(qr_scan.R_l6_opt, qr_scan.t_l6_cam, label_center)
    aimed = move_to_flange_pose(world, robot, solver, aim_pos, aim_quat, "AIM QR")
    if not aimed:
        print("   !! QR 조준 IK 실패 — 검증 못 함")
        return None

    obs_list = [qr_scan.capture_and_decode(world) for _ in range(n_frames)]
    obs_list = [o for o in obs_list if o is not None]
    agg = aggregate_qr_poses(obs_list) if obs_list else None
    if agg is None or not agg.ok:
        print(f"   !! QR 추정 실패 — 검증 못 함 ({(agg.reason if agg else '캡처 실패')})")
        return None

    center_xy, top_z, _ = measure_prim(FLANGE_PATH)
    gt_flange_world = np.array([center_xy[0], center_xy[1], top_z])

    R_qr_world = R_qr_world_from_yaw(agg.yaw_rad)

    grasp_doc = yaml.safe_load(GRASP_YAML.read_text(encoding="utf-8"))
    current_T = np.array(grasp_doc["magazines"][variant]["T_QR_grasp_xyz"], dtype=float)

    prior_world = agg.center_world + R_qr_world @ current_T
    err_world = gt_flange_world - prior_world
    err_qr = R_qr_world.T @ err_world   # QR 프레임(x=오른쪽,y=아래,z=안쪽)으로 봤을 때 오차

    corrected_T = R_qr_world.T @ (gt_flange_world - agg.center_world)

    print(f"   QR 추정  n_used={agg.n_used}/{agg.n_total}  "
          f"center_world={vec(agg.center_world)}  yaw={math.degrees(agg.yaw_rad):.2f}deg  "
          f"pos_std_mm={vec(agg.pos_std_mm, 2)}")
    print(f"   GT flange  {vec(gt_flange_world)}")
    print(f"   현재 T_QR_grasp_xyz {variant} = {vec(current_T, 4)}")
    print(f"     -> prior_world = {vec(prior_world)}")
    print(f"     -> 오차(world) = {vec(err_world, 4)}  ({np.linalg.norm(err_world)*1000:.1f} mm)")
    print(f"     -> 오차(QR 프레임 x/y/z) = {vec(err_qr, 4)}")
    print(f"   보정된 T_QR_grasp_xyz {variant} = {vec(corrected_T, 4)}  "
          f"(delta {vec(corrected_T - current_T, 4)})")

    pos, yaw0 = WP_SCAN
    teleporter.teleport(pos, yaw0)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    return {"current_T": current_T, "corrected_T": corrected_T,
           "err_world_mm": float(np.linalg.norm(err_world) * 1000.0),
           "agg": agg, "gt_flange_world": gt_flange_world}


# ══════════════════════════════════════════════════════════════
#  한 트라이얼 실행 — SCAN(QR 확인) -> PICK
# ══════════════════════════════════════════════════════════════
def run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter, qr_scan):
    t0 = time.time()

    # ── 0) 접근 — STACK_APPROACH_CREEP_M 뒤 대기점까지 텔레포트하고,
    #   마지막은 drive_to() 로 실제 cmd_vel 크립처럼(매 스텝 물리를 돌리며)
    #   조금씩 다가가 WP_SCAN 에 정확히 선다 — task_manager.py 의
    #   STACK_NAV(Nav2, 대기점까지) → STACK_CREEP_IN(cmd_vel, 마지막 접근)
    #   과 같은 순서다(2026-09-25). 옛날엔 이 자리에서 SCAN 만 하고, SCAN
    #   성공 뒤 WP_PICK 으로 다시 재이동해서 PICK 했는데, 그 재이동이
    #   manipulator 를 제대로 못 세우는 원인이었다(사용자 실측) — 이제
    #   SCAN 도 PICK 도 이 접근 한 번으로 도착한 이 자리에서 다 한다.
    stack_aabb = measure_prim_aabb_xy(STACK_PATH)
    stage_pos, stage_yaw = back_off(WP_SCAN, STACK_APPROACH_CREEP_M)
    teleporter.teleport(stage_pos, stage_yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)

    stack_xy_before, _ = get_world_pose(STACK_PATH)
    pos, yaw = WP_SCAN
    safe = teleporter.drive_to(world, pos, yaw, render=not HEADLESS, label="접근 크립",
                               stack_aabb=stack_aabb)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    stack_xy_after, _ = get_world_pose(STACK_PATH)
    bump_mm = float(np.linalg.norm(stack_xy_after[:2] - stack_xy_before[:2])) * 1000.0
    if bump_mm > 5.0:
        print(f"   !! 스택이 접근 크립 중 밀렸다  {vec(stack_xy_before)} -> "
              f"{vec(stack_xy_after)}  ({bump_mm:.1f} mm)")

    if not safe:
        return TrialResult(
            trial=trial_idx, fail_code=FAIL_COLLISION, cycle_time_s=time.time() - t0,
            lift_rise_mm=0.0, tilt_deg=0.0,
        )

    section(f"SCAN  trial {trial_idx + 1}")
    # ★ 2026-09-25: qr_aim_flange_pose()+move_to_flange_pose() 로 매번 IK 를
    #   새로 풀어 거기로 서보하던 걸 없앴다 — 실제 경로(carrier_code_reader.
    #   _observe → observe_pose(joints_deg=...))는 IK 를 안 푼다, 사용자가
    #   티칭한 TAUGHT_JOINTS_DEG 로 그냥 서보한다(servo_joint_deg 독스트링).
    print(f"   [-] 관측 자세    joints(deg) {[round(v, 1) for v in TAUGHT_JOINTS_DEG]}")
    servo_joint_deg(world, robot, TAUGHT_JOINTS_DEG)
    aimed = True

    obs = qr_scan.capture_and_decode(world) if aimed else None
    qr_ok = obs is not None and obs.ok and obs.decoded == EXPECTED_QR_ID
    if not aimed:
        print("   !! QR 조준 IK 실패")
    elif obs is None:
        print("   QR    캡처 실패 (이미지 없음)")
    else:
        print(f"   QR    {'OK' if obs.ok else 'FAIL'}  decoded={obs.decoded!r}"
              f"  (기대 {EXPECTED_QR_ID!r})  reason={obs.reason}")

    if not qr_ok:
        return TrialResult(
            trial=trial_idx, fail_code=FAIL_NOT_FOUND, cycle_time_s=time.time() - t0,
            lift_rise_mm=0.0, tilt_deg=0.0,
        )

    # ── 1) PICK — 제자리(WP_SCAN)에서 바로. 재이동 없다(2026-09-25) ──────
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS // 2):
        world.step(render=not HEADLESS)

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

    return TrialResult(
        trial=trial_idx, fail_code=pick_fsm.fail_code,
        cycle_time_s=time.time() - t0,
        lift_rise_mm=pick_fsm.lift_rise_m * 1000.0,
        tilt_deg=pick_fsm.tilt_at_lift_deg,
    )


# ══════════════════════════════════════════════════════════════
#  실제 파이프라인 재현 — SCAN(QR) -> OBSERVE(prior+detect_flange) -> PICK
#  (2026-09-25, PICK_REAL_PIPELINE=1)
#
#  run_trial() 은 GT(measure_prim)로 바로 잡아서 grasp.yaml 의
#  T_QR_grasp_xyz/OBSERVE 경로를 한 번도 실제로 타지 않았다 — 그래서
#  T_QR_grasp_xyz 가 2배 틀려 있던 걸 이 테스트가 여태 못 잡았다. 이 함수는
#  pick_place_server._execute()/sim_backend.pick_observe_flange() 가 하는
#  일(QR prior 계산 -> observe_tcp IK -> detect_flange)을 그대로 옮겨서,
#  grasp.yaml 교정이 실제로 pick 을 성공시키는지 GT 를 안 거치고 확인한다.
# ══════════════════════════════════════════════════════════════
def run_trial_real_pipeline(trial_idx, world, robot, lula, solver, gripper, teleporter, qr_scan,
                            variant="tray_blue"):
    import yaml

    t0 = time.time()

    # ── 0) 접근 — run_trial() 과 같은 크립(2026-09-25). 재이동 없이 이
    #   자리에서 SCAN 도 OBSERVE/PICK 도 다 한다.
    stack_aabb = measure_prim_aabb_xy(STACK_PATH)
    stage_pos, stage_yaw = back_off(WP_SCAN, STACK_APPROACH_CREEP_M)
    teleporter.teleport(stage_pos, stage_yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)

    pos, yaw = WP_SCAN
    safe = teleporter.drive_to(world, pos, yaw, render=not HEADLESS, label="접근 크립",
                               stack_aabb=stack_aabb)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    if not safe:
        return TrialResult(trial=trial_idx, fail_code=FAIL_COLLISION,
                           cycle_time_s=time.time() - t0, lift_rise_mm=0.0, tilt_deg=0.0)

    section(f"SCAN(real)  trial {trial_idx + 1}")
    # ★ 2026-09-25: run_trial() 과 같은 이유로 IK 서보 대신 TAUGHT_JOINTS_DEG
    #   로 직접 서보한다(observe_pose(joints_deg=...) 와 동일 경로).
    print(f"   [-] 관측 자세    joints(deg) {[round(v, 1) for v in TAUGHT_JOINTS_DEG]}")
    servo_joint_deg(world, robot, TAUGHT_JOINTS_DEG)

    # sim_backend.scan_qr() 과 같은 n_frames=3 평균.
    from cobot3_perception.qr_pose import aggregate_qr_poses
    obs_list = [qr_scan.capture_and_decode(world) for _ in range(3)]
    obs_list = [o for o in obs_list if o is not None]
    agg = aggregate_qr_poses(obs_list) if obs_list else None
    qr_ok = agg is not None and agg.ok and agg.decoded == EXPECTED_QR_ID
    if not qr_ok:
        print(f"   QR    FAIL  reason={(agg.reason if agg else '캡처 실패')}")
        return TrialResult(trial=trial_idx, fail_code=FAIL_NOT_FOUND,
                           cycle_time_s=time.time() - t0, lift_rise_mm=0.0, tilt_deg=0.0)
    print(f"   QR    OK  decoded={agg.decoded!r}  n_used={agg.n_used}/{agg.n_total}")

    # ── 1) 제자리(WP_SCAN)에서 바로. 재이동 없다(2026-09-25) ─────────────
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS // 2):
        world.step(render=not HEADLESS)

    # ── 2) OBSERVE — pick_place_server._flange_pose_from_qr() 와 같은 식으로
    #    prior_world 를 만들고(_flange_pose_from_qr 의 base_link 왕복이
    #    상쇄되는 건 calibrate_grasp_offset() 주석에서 이미 대수로 확인했다),
    #    sim_backend.pick_observe_flange() 와 같은 식으로 손목캠을 그 위로
    #    띄운 뒤 detect_flange() 로 진짜 파지점을 잰다.
    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
    grasp_doc = yaml.safe_load(GRASP_YAML.read_text(encoding="utf-8"))
    entry = grasp_doc["magazines"][variant]
    fv = grasp_doc["flange_vision"]
    T = np.array(entry["T_QR_grasp_xyz"], dtype=float)
    R_qr_world = R_qr_world_from_yaw(agg.yaw_rad)
    prior_world = agg.center_world + R_qr_world @ T

    observe_tcp = observe_tcp_from_prior(
        prior_world, qr_scan.R_l6_opt, qr_scan.t_l6_cam, qr_scan.K, target_quat,
        cam_height_m=float(fv["observe_cam_height_m"]),
        image_offset_px=tuple(fv["observe_image_offset_px"]))
    observed = move_to_flange_pose(
        world, robot, solver, tcp_to_flange(observe_tcp, target_quat), target_quat, "OBSERVE")
    if not observed:
        print(f"   !! OBSERVE IK 실패  observe_tcp={vec(observe_tcp)}  prior={vec(prior_world)}")
        return TrialResult(trial=trial_idx, fail_code=FAIL_UNREACHABLE,
                           cycle_time_s=time.time() - t0, lift_rise_mm=0.0, tilt_deg=0.0)

    raw = qr_scan.capture_raw(world)
    if raw is None:
        print("   !! OBSERVE 캡처 실패")
        return TrialResult(trial=trial_idx, fail_code=FAIL_NO_FLANGE,
                           cycle_time_s=time.time() - t0, lift_rise_mm=0.0, tilt_deg=0.0)
    bgr, depth, R_opt, p_opt = raw

    from cobot3_perception.flange_topview import detect_flange
    flange_obs = detect_flange(
        bgr, qr_scan.K, qr_scan.dist, R_opt, p_opt,
        expected_center_world=prior_world, top_z_prior=float(prior_world[2]),
        yaw_prior_rad=math.atan2(R_qr_world[1, 0], R_qr_world[0, 0]),
        color=entry.get("color"), flange_size_m=tuple(entry["flange_size"][:2]),
        depth=depth if fv.get("use_depth", True) else None,
        depth_band_m=float(fv["depth_band_m"]), search_radius_m=float(fv["search_radius_m"]),
        size_tol=float(fv["size_tol"]), min_fill=float(fv["min_fill"]),
        max_depth_correction_m=float(fv["max_depth_correction_m"]))
    if not flange_obs.ok:
        print(f"   !! detect_flange 실패: {flange_obs.reason}  prior={vec(prior_world)}")
        return TrialResult(trial=trial_idx, fail_code=FAIL_NO_FLANGE,
                           cycle_time_s=time.time() - t0, lift_rise_mm=0.0, tilt_deg=0.0)

    gt_center_xy, gt_top_z, _ = measure_prim(FLANGE_PATH)
    err_mm = float(np.linalg.norm(
        flange_obs.center_world - np.array([gt_center_xy[0], gt_center_xy[1], gt_top_z]))) * 1000.0
    print(f"   OBSERVE OK  detect_flange center={vec(flange_obs.center_world)}  "
          f"GT={vec([gt_center_xy[0], gt_center_xy[1], gt_top_z])}  오차 {err_mm:.1f}mm")

    # ── 3) APPROACH -> DESCEND -> GRIP -> HOLD -> LIFT — 이제부터는 비전이
    #    잰 파지점(GT 아님)으로 판다.
    pick_fsm = PickFSM(robot, gripper,
                       center_xy=flange_obs.center_world[:2],
                       obj_top=float(flange_obs.center_world[2]))
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

    return TrialResult(
        trial=trial_idx, fail_code=pick_fsm.fail_code,
        cycle_time_s=time.time() - t0,
        lift_rise_mm=pick_fsm.lift_rise_m * 1000.0,
        tilt_deg=pick_fsm.tilt_at_lift_deg,
    )


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = StackScanPickTask(name="stack_scan_pick_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    stack_spawn_pos, stack_spawn_quat = get_world_pose(STACK_PATH)
    print(f"   stack pose   {vec(stack_spawn_pos, 4)}  quat {vec(stack_spawn_quat, 4)}")
    bind_stack_rigid()

    section("SOLVER")
    base_pos0, base_quat0 = get_world_pose(BASE_LINK_PATH)
    lula, solver = create_ik_solver(robot, base_pos0, base_quat0)
    gripper = SurfaceGripperCtl(task.gripper_node_path)
    teleporter = BaseTeleporter(robot)
    qr_scan = StackQRScan()

    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
    reachable = reachability_check(world, robot, lula, solver, teleporter, target_quat, qr_scan)
    if not reachable:
        print("\n   WP_SCAN 좌표를 먼저 조정하세요")
        simulation_app.close()
        return

    # PICK_SAVE_TEACH=<이름> 이면 QR 조준 IK 를 그 이름으로 taught_poses.yaml
    # 에 저장하고 끝낸다 — save_teach_pose() 독스트링 참고.
    save_teach = os.environ.get("PICK_SAVE_TEACH", "")
    if save_teach:
        save_teach_pose(save_teach, world, robot, lula, solver, qr_scan)
        simulation_app.close()
        return

    # PICK_DUMP_FRAME=<경로> 면 접근 크립 + TAUGHT_JOINTS_DEG 서보까지 마친
    # 뒤 손목캠 한 장을 그 경로에 PNG 로 떠 놓고 끝낸다 — "QR 이 안 보인다"
    # 는 게 실제로 뭘 보고 있는지 눈으로 확인하려는 것(2026-09-25, 사용자
    # 지시 — "네가 직접 캡쳐해서 봐봐").
    dump_frame = os.environ.get("PICK_DUMP_FRAME", "")
    if dump_frame:
        stack_aabb = measure_prim_aabb_xy(STACK_PATH)
        stage_pos, stage_yaw = back_off(WP_SCAN, STACK_APPROACH_CREEP_M)
        teleporter.teleport(stage_pos, stage_yaw)
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            world.step(render=not HEADLESS)
        pos, yaw = WP_SCAN
        teleporter.drive_to(world, pos, yaw, render=not HEADLESS, label="접근 크립",
                            stack_aabb=stack_aabb)
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            world.step(render=not HEADLESS)
        sync_ik_base_pose(lula)

        print(f"   [-] 관측 자세    joints(deg) {[round(v, 1) for v in TAUGHT_JOINTS_DEG]}")
        servo_joint_deg(world, robot, TAUGHT_JOINTS_DEG)

        raw = qr_scan.capture_raw(world)
        if raw is None:
            print("   !! 캡처 실패 (이미지 없음)")
        else:
            bgr, depth, R_wo, p_wo = raw
            import cv2
            Path(dump_frame).parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(dump_frame, bgr)
            obs = qr_scan._estimate_qr_pose(
                bgr, depth, qr_scan.K, qr_scan.dist, R_wo, p_wo, expected_id=EXPECTED_QR_ID)
            print(f"   QR    {'OK' if obs.ok else 'FAIL'}  decoded={obs.decoded!r}"
                  f"  reason={obs.reason}")
            print(f"   -> {dump_frame}")
        simulation_app.close()
        return

    # PICK_OBSERVE_HEIGHT_CHECK=1 이면 OBSERVE IK 를 지금 스택 높이와, 반절
    # 높이로 줄였을 때(grasp_z 를 그만큼 낮춰서) 둘 다 계산만 해 본다 —
    # 실제로 에셋을 안 줄이고 "낮추면 풀리는지" 를 먼저 확인하려는 것
    # (2026-09-26, 사용자 질문: "스택 크기를 반절로 줄이면 잡을 수 있을까").
    if os.environ.get("PICK_OBSERVE_HEIGHT_CHECK", "0") == "1":
        stack_aabb = measure_prim_aabb_xy(STACK_PATH)
        stage_pos, stage_yaw = back_off(WP_SCAN, STACK_APPROACH_CREEP_M)
        teleporter.teleport(stage_pos, stage_yaw)
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            world.step(render=not HEADLESS)
        pos, yaw = WP_SCAN
        teleporter.drive_to(world, pos, yaw, render=not HEADLESS, label="접근 크립",
                            stack_aabb=stack_aabb)
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            world.step(render=not HEADLESS)
        sync_ik_base_pose(lula)

        center_xy, top_z, stack_h = measure_prim(FLANGE_PATH)
        deck_z = top_z - stack_h  # 대략 선반/컨베이어 상판 높이
        target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
        for label, z in (("지금 높이", top_z), ("반절 높이", deck_z + stack_h / 2.0)):
            prior_world = np.array([center_xy[0], center_xy[1], z])
            observe_tcp = observe_tcp_from_prior(
                prior_world, qr_scan.R_l6_opt, qr_scan.t_l6_cam, qr_scan.K, target_quat)
            _, solved = solver.compute_inverse_kinematics(
                target_position=tcp_to_flange(observe_tcp, target_quat),
                target_orientation=target_quat)
            print(f"   OBSERVE IK  {label}  grasp_z={z:.3f}  "
                  f"observe_tcp={vec(observe_tcp)}  -> {'SOLVED' if solved else 'FAILED'}")
        simulation_app.close()
        return

    # PICK_CALIBRATE=1 이면 T_QR_grasp_xyz 실측 검증만 하고 끝낸다.
    if os.environ.get("PICK_CALIBRATE", "0") == "1":
        calibrate_grasp_offset(world, robot, lula, solver, teleporter, qr_scan, variant="tray_blue")
        simulation_app.close()
        return

    def do_pick(trial_idx):
        # 스택/그리퍼/베이스를 시작 상태로 되돌린 뒤 SCAN+PICK 한 판을
        # 실행한다. 순서가 중요하다(12_pick_test.py 와 같은 이유) — 로봇
        # 쪽을 다 세운 다음, 맨 마지막에 스택을 제자리로 놓는다.
        #
        # ★ 2026-09-25: run_trial()/run_trial_real_pipeline() 이 이제 자기
        #   시작점에서 직접 접근 크립(STACK_APPROACH_CREEP_M)을 한다 — 여기
        #   서는 WP_SCAN 이 아니라 그 대기점에 세워 둔다. WP_SCAN 에 먼저
        #   세웠다가 run_trial 이 다시 대기점으로 물러났다 크립해 오는
        #   낭비를 없앤다.
        gripper.open()
        for _ in range(20):                      # 릴리즈가 실제로 처리되게
            world.step(render=not HEADLESS)

        pos, yaw = back_off(WP_SCAN, STACK_APPROACH_CREEP_M)
        teleporter.teleport(pos, yaw)
        for _ in range(SETTLE_STEPS):
            world.step(render=not HEADLESS)
        set_ready_pose(robot)
        for _ in range(SETTLE_STEPS // 2):       # 팔이 멎을 때까지
            world.step(render=not HEADLESS)

        reset_stack_pose(stack_spawn_pos, tuple(stack_spawn_quat))
        for _ in range(30):                      # 리셋이 물리에 반영되게
            world.step(render=not HEADLESS)
        gripper.reinit()

        section("RUN")
        if os.environ.get("PICK_REAL_PIPELINE", "0") == "1":
            result = run_trial_real_pipeline(
                trial_idx, world, robot, lula, solver, gripper, teleporter, qr_scan)
        else:
            result = run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter, qr_scan)
        print(f"   trial {trial_idx + 1} -> fail_code={result.fail_code}  "
              f"cycle_time={result.cycle_time_s:.2f}s")
        return result

    results = [do_pick(i) for i in range(TRIALS)]

    if TRIALS > 1:
        section("SUMMARY")
        ok = [r for r in results if r.success]
        print(f"   성공 {len(ok)} / {TRIALS}")
        if ok:
            ts = [r.cycle_time_s for r in ok]
            rs = [r.lift_rise_mm for r in ok]
            tl = [r.tilt_deg for r in ok]
            print(f"   cycle  평균 {sum(ts)/len(ts):.2f}s  "
                  f"최소 {min(ts):.2f}  최대 {max(ts):.2f}")
            print(f"   rise   최소 {min(rs):.1f} mm  (기준 {LIFT_OK_MIN_M*1000:.0f} mm)")
            print(f"   tilt   최대 {max(tl):.2f} deg  (기준 {TILT_MAX_DEG:.0f} deg)")
        bad = [r for r in results if not r.success]
        for r in bad:
            print(f"   실패 trial {r.trial + 1}  fail_code={r.fail_code}")

    # 헤드리스는 판정이 끝나면 더 할 일이 없다. 창도 없어서 Stop/Play 도 못 한다.
    if HEADLESS:
        simulation_app.close()
        return

    # pick 결과 상태로 정지해서 계속 띄워둔다. Stop 했다가 다시 Play 를 누르면
    # (재생 상태 False -> True 전환을 감지해) SCAN+PICK 을 한 번 더 실행한다.
    #
    # ★ 실측 — Stop 은 물리 씬(아티큘레이션 뷰 포함)을 허문다. Play 로 다시
    #   세워도 Python 이 들고 있던 robot(SingleManipulator) 핸들은 그 허문
    #   뷰를 계속 가리켜서, 재생 전환을 감지하고 바로 do_pick() 을 부르면
    #   "Attempted to compute inverse kinematics for an uninitialized robot
    #   Articulation" 에러 뒤에 apply_action() 이 None 액션을 받아 죽는다
    #   (AttributeError: 'NoneType' object has no attribute 'joint_positions'
    #   -> Kit 셧다운 중 세그폴트까지 났다). teleporter/solver 도 같은
    #   robot 핸들을 들고 있어서 robot.initialize() 한 번이면 셋 다 새
    #   뷰를 다시 잡는다 — main() 맨 앞에서 한 것과 동일.
    section("HOLD")
    print("   pick 결과 상태로 정지. Stop 후 Play 를 누르면 다시 SCAN+PICK 합니다."
          " 창을 닫으면 종료됩니다.")
    was_playing = True
    trial_idx = TRIALS
    while simulation_app.is_running():
        world.step(render=not HEADLESS)
        playing = world.is_playing()
        if playing and not was_playing:
            robot.initialize()
            do_pick(trial_idx)
            trial_idx += 1
        was_playing = playing

    simulation_app.close()


if __name__ == "__main__":
    main()

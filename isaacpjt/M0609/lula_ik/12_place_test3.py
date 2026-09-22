"""
단위 테스트 3 — 스택이 포장 라인 출력 자리에 이미 나와 있다고 치고("포장은
끝났다치고"), 그 앞에서 관측 자세로 대기하다가 집어서 테스트 스테이션(검사
벨트)에 놓기.

    isaac_python 12_place_test3.py   (GUI 기본. 헤드리스: PICK_HEADLESS=1)
    SIM_WORLD_USD 로 다른 씬을 줄 수도 있다(기본값이 이미 아래 테스트 씬).

12_place_test.py 를 그대로 뼈대로 쓴다 — Nav2 없이 베이스를 텔레포트한다.
파지점은 **손목 카메라로 QR 을 인식해 PnP 로 낸다**(12_place_test2.py 의
QrVision/grasp_from_qr 과 같은 경로) — GT(measure_prim)는 "얼마나 맞았는지
채점"에만 쓴다. 성공 판정도 12_place_test.py 와 같은 기준(project-plan.html
S4/S6)을 쓴다.

★★ 스택은 테스트 씬(simple_factory_layout_test.usda)에 이미 놓여 있다
──────────────────────────────────────────────────────────────────
  이전 판은 packaging_flow.py(매거진→스택 변환, OmniScriptingAPI)를
  standalone 스크립트에서 돌리는 게 안 돼서(스크립팅 보안 팝업 + 벨트
  물리가 참조-로딩 경로에서 기대대로 안 붙음) 그 대안으로 스택을 직접
  스폰해 자유낙하시키는 방식을 썼었다. 실측 결과 **그 방식도 문제가
  있었다** — 방금 payload 로 붙인 PhysicsRigidBodyAPI/PhysicsCollisionAPI
  가 PhysX 에 등록되기 전에 먼저 중력을 받아 ShelfDeck(0.54)을 그냥
  관통해 바닥까지 떨어졌다(수직으로만 관통하니 로봇도 같은 XY 위에 서
  있어서 "떨어진 스택 바로 옆" 처럼 보였다 — short_gripper payload 가
  붙자마자 SurfaceGripper 노드를 못 찾던 것과 같은 종류의 지연이다).

  runtime 스폰 자체를 없애는 쪽으로 바꿨다 — simple_factory_layout_test.usda
  (task_manager scenario:=static_test 용으로 만든 씬)에 스택 하나가
  `/World/Environment/PackagingUnloaderZone/SpawnedStacks/stack_1` 로
  ShelfDeck 윗면(z=0.54) 바로 위(z=0.542, 2mm 여유)에 미리 놓여 있다 —
  이걸 그냥 쓴다. WORLD_USD 기본값이 이 씬이다.

시나리오
────────
  1) 씬을 열면 stack_1 이 이미 있다 — EXISTING_STACK_PATH 로 찾아서
     wait_for_settle() 로 짧게(붙어 있던 2mm 여유가 가라앉는 정도) 안정을
     기다린다. 로봇은 그동안 관측 자세를 유지한다.
       ★ 관측 자세 티칭 필요 — 아래 절 참고.
  2) 안정된 자리 앞으로 다시 서서, 강성을 SCAN(1e5)으로 낮추고 손목캠으로
     stack_1 의 QR 라벨(F3-STKO-1, 40mm, qr_label_ny/py)을 여러 장 찍어
     PnP 로 자세를 낸다(estimate_qr_pose/aggregate_qr_poses) — 거기서
     grasp.yaml 의 tray_1_orange 와 같은 T_QR_grasp_xyz 로 흡착점을 뽑는다
     (grasp_from_qr). 인식 실패면 FAIL_NO_QR 로 중단한다.
  3) 강성을 PICK(1e8)으로 되돌리고 그 QR 파지점으로 PICK, 검사 벨트
     (TestingZone, BELT_Y_TESTING)로 옮겨 PLACE 한다.

★ 관측 자세(observe pose) — 티칭이 필요하다, 그리고 이제는 필수다
───────────────────────────────────────────────────────────────
  taught_poses.yaml 에는 SHELF-A 매거진을 보는 자세 셋(shelf_1_top_scan,
  shelf_1_top_close_centered, s1_bottom_scan)만 있다. 이 스크립트가 대기하는
  자리(OutputShelf 앞, 대략 x=3.87, y=0.42)를 보는 자세는 **아무도 티칭한
  적이 없다.** GT 를 읽던 예전 판에서는 이 자세가 안 맞아도(READY 폴백)
  동작이 막히지 않았지만, 지금은 이 자세로 실제 QR 을 화면에 담아야 인식이
  된다 — 안 찍혀 있으면(READY 폴백) 카메라가 stack_1 을 안 보고 있을
  가능성이 커서 scan_qr() 이 거의 확실히 실패한다.

  티칭하는 법 (capture_pose.py 에 이 자리를 "packaging_output" 정차 지점으로
  이미 추가해 뒀다):

      CAPTURE_BASE=packaging_output isaac_python isaacpjt/tools/capture_pose.py

  슬라이더로 자세를 잡고(stack_1 의 -Y 면 QR 라벨이 화면에 들어오도록,
  grasp.yaml flange_vision.observe_cam_height_m=0.25 근처 거리) 이름을
  "packaging_output_scan" 으로 Capture 하면 taught_poses.yaml 에 덧붙는다
  — 이 스크립트의 load_observe_joints_deg() 가 그 이름을 자동으로 찾아 쓴다.

★ 이 스크립트가 검증하지 못한 것
──────────────────────────────────
  1. 매거진→스택 변환 자체(packaging_flow.py 를 standalone 스크립트에서
     돌리는 문제 — 위 절 참고). 이 스크립트는 그 변환을 거치지 않고 이미
     스택이 있는 테스트 씬을 쓴다.
  2. QR 인식이 실제로 되는지 — 관측 자세가 안 타칭돼 있으면(위 절)
     scan_qr() 이 거의 확실히 실패한다. 티칭돼 있어도 라벨이 40mm 라
     매거진(50mm)보다 작고, 씬 조명이 어두워 대비를 펴야(cv2.normalize)
     디코딩됐다(12_place_test2.py 실측) — 여기서도 같은 보정을 넣었지만
     이 씬에서 실제로 읽히는지는 돌려봐야 안다.
  3. stations.yaml 의 TEST-01 place_pose 는 이제
     {x: 3.85, y: -2.705314596908152, theta: 0.0}(BELT_BASE_X/BELT_Y_TESTING,
     12_place_test2.py 에서 그대로 가져온 값)로 채워져 있다 — 다만 이 값은
     simple_factory_layout.usda 를 재서 낸 것이지, 이 트라이얼을 Isaac 에서
     끝까지 돌려 PLACE 성공을 확인한 값은 아니다(stations.yaml 쪽 주석도
     같이 남겨 뒀다). 이 트라이얼이 실제로 통과하면 그 "미확인" 주석을
     지울 것.
"""

import os
import sys

from isaacsim import SimulationApp

HEADLESS = os.environ.get("PICK_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

# ★ packaging_flow.py(매거진→스택 변환)는 아예 안 쓴다 — 모듈 docstring
#   "매거진→스택 변환은 안 한다" 절 참고. 그래서 omni.kit.scripting 확장도
#   더 이상 켤 필요가 없다.

import ctypes
import dataclasses
import math
import time
from pathlib import Path
from typing import Optional

import numpy as np
import omni.usd
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)
from isaacsim.core.utils.types import ArticulationAction


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR   = Path(__file__).resolve().parent
M0609_DIR  = THIS_DIR.parent
ISAACPJT_DIR = M0609_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD        = os.environ.get("SIM_WORLD_USD") or str(
    ISAACPJT_DIR / "worlds/simple_factory_layout_test.usda")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")
TAUGHT_POSES     = ISAACPJT_DIR / "tools/out/taught_poses.yaml"
FRAMES_YAML      = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"

# qr_pose 는 numpy+cv2 만 쓴다 — rclpy 를 import 하지 않아 Isaac 의 python 에서
# 그대로 불린다(12_place_test2.py QrVision 독스트링과 같은 이유).
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
CAMERA_PRIM     = f"{GRIPPER_PRIM}/rsd455/RSD455/Camera_OmniVision_OV9782_Color"

ARTICULATION_ROOT_CANDIDATES = [
    f"{BASE_XFORM_PATH}/chassis_link",
    BASE_XFORM_PATH,
]

# packaging_flow.py 가 스폰했을 스택들이 자식으로 붙는 곳(12_place_test2.py
# 와 동일 경로). simple_factory_layout_test.usda 에는 이미 stack_1 이 하나
# 저장돼 있다 — 이 스크립트는 아무것도 스폰하지 않고 그 prim 을 그대로 쓴다
# (모듈 docstring "스택은 테스트 씬에 이미 놓여 있다" 절 참고).
SPAWNED_STACKS_PATH = "/World/Environment/PackagingUnloaderZone/SpawnedStacks"
EXISTING_STACK_PATH = f"{SPAWNED_STACKS_PATH}/stack_1"


# ══════════════════════════════════════════════════════════════
#  로봇 관절 / 드라이브
# ══════════════════════════════════════════════════════════════
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS      = 1e8   # 파지/이송
DRIVE_STIFFNESS_SCAN = 1e5   # 관측(QR 스캔) — 1e8 이면 잔진동으로 디코딩이 깨진다
                              # (sim_backend.py/12_place_test2.py 실측과 동일)
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# capture_pose.py 로 CAPTURE_BASE=packaging_output 자리에서 티칭해 이 이름
# 으로 캡처하면 load_observe_joints_deg() 가 자동으로 찾아 쓴다 — 모듈
# docstring "관측 자세" 절 참고. 아직 없으면 READY 로 폴백한다.
OBSERVE_POSE_NAME = "packaging_output_scan"


# ══════════════════════════════════════════════════════════════
#  씬 기하 — 12_place_test2.py 가 이미 실측/검증한 값을 그대로 가져온다
# ══════════════════════════════════════════════════════════════
CARTER_Z   = 0.07963398335074101     # 베이스 텔레포트 z (씬의 스폰 z 와 같다)
BELT_TOP_Z = 0.5407278772166425      # 세 벨트 모두 동일

BELT_Y_PACKAGING = 4.6                    # +X 로 흐른다 → 포장 트리거(7.45)로 실려 간다
BELT_Y_TESTING   = -2.705314596908152     # +X 로 흐른다 — 검사 벨트(TestingZone)

BELT_PLACE_X = 4.30   # 벨트에 올려놓는 x (프레임 앞면 4.2 보다 안쪽)
BELT_BASE_X  = 3.85   # 벨트 앞에 서는 베이스 x

# 출력 자리 — simple_factory_layout_test.usda 에 이미 놓여 있는 stack_1 의
# world 좌표를 직접 재서 낸 값이다:
#   PackagingUnloaderZone 은 identity, SpawnedStacks 로컬 translate
#   (0, -1.8286350742021307, 0), stack_1 로컬 translate
#   (3.83, 3.0586350742021307, 0.542) → world (3.83, 1.23, 0.542).
# ShelfDeck 윗면 world z 는 0.54(OutputShelf 로컬 translate y=-1.8286350742021307,
# ShelfDeck 로컬 translate (3.8694130739117156, 2.8000000000000003, 0.48)
# scale (0.35, 1.4, 0.12) → 윗면 (3.8694, 0.9714, 0.54)) 라서, stack_1 은 덱
# 중심(0.9714)이 아니라 그보다 +y 로 약 26cm 치우친 자리에 z=0.542(덱 위
# 2mm)로 놓여 있다 — 중심이 아니어도 덱 폭(y 반길이 0.7) 안이라 문제없다.
STACK_XY = (3.83, 1.23)
OUTPUT_STANDOFF_M  = 0.55   # SHELF_STANDOFF_M(12_place_test2.py)과 같은 규약 — 대상 앞 0.55 m

# (베이스 world xyz, yaw_deg) — yaw 는 world +x 축 기준
WP_STAGE_OUTPUT = (
    np.array([STACK_XY[0], STACK_XY[1] - OUTPUT_STANDOFF_M, CARTER_Z]),
    0.0,
)
WP_PLACE_TEST = (np.array([BELT_BASE_X, BELT_Y_TESTING, CARTER_Z]), 0.0)

STACK_SETTLE_TIMEOUT_S = 10.0   # 이미 놓여 있는 스택이 안정되길 기다리는 상한(2mm 여유뿐이라 짧다)
STACK_SETTLE_TOL_M     = 0.002
STACK_SETTLE_FRAMES    = 60

# PLACE 사전 점검용 대략 높이 — 정확한 스택 높이는 실제로 집어봐야 안다.
# F3_STKO_1/F3_STKB_1.usda 문서상 전체 높이가 대략 77.7 mm 다.
STACK_HEIGHT_GUESS_M = 0.08


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정 — 12_place_test.py 와 동일(같은 1 kg 급 대상)
# ══════════════════════════════════════════════════════════════
COAXIAL_FORCE_LIMIT = 200.0   # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 100.0   # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.03    # m

SUCTION_FACE_Z = 0.161   # link_6 로컬 +Z 로 흡착면까지 거리 (short_gripper 실측)
TCP_OFFSET     = np.array([0.0, 0.0, SUCTION_FACE_Z])

GRIP_GAPS = [0.005, 0.002, 0.000, -0.003]


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
APPROACH_HEIGHT_OFFSET = 0.15
LIFT_HEIGHT_OFFSET     = 0.10
PLACE_DROP             = 0.005

GRIP_WAIT    = 90
HOLD_WAIT    = 120
RELEASE_WAIT = 90
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
#  성공 판정 기준 — 12_place_test.py 와 동일 (project-plan.html S4/S6)
# ══════════════════════════════════════════════════════════════
LIFT_OK_MIN_M        = 0.005
TILT_MAX_DEG         = 5.0
PLACE_POS_TOL_M      = 0.002
PLACE_ORIENT_TOL_DEG = 1.0


# ══════════════════════════════════════════════════════════════
#  fail_code — project-plan.html 체계 + 확장(6, 7)
# ══════════════════════════════════════════════════════════════
FAIL_OK          = 0
FAIL_SLIP        = 3
FAIL_TIMEOUT     = 5
FAIL_UNREACHABLE = 6
FAIL_PLACE_ERROR = 7
FAIL_NO_QR       = 8   # 12_place_test2.py 와 같은 코드 — QR 인식 실패


# ══════════════════════════════════════════════════════════════
#  카메라 / QR 인식 — 12_place_test2.py 의 QrVision/CarrierSpec 을 로봇
#  한 대짜리로 단순화해 가져온다. GT(measure_prim) 대신 손목 카메라로 QR
#  을 읽어 파지점을 낸다 — 모듈 docstring 참고.
# ══════════════════════════════════════════════════════════════
@dataclasses.dataclass
class CarrierSpec:
    """QR 한 장에서 파지점까지 가는 데 필요한 값 전부."""
    name: str
    expected_id: Optional[str]   # 디코드 문자열. None 이면 아무거나 받는다
    qr_side_m: float             # 라벨 한 변 (m)
    t_qr_grasp: np.ndarray       # QR 프레임에서 흡착점까지 (x_qr, y_qr, z_qr)

    @property
    def data_side_m(self) -> float:
        """QR 데이터 영역 한 변 — 29모듈 중 데이터가 21모듈(12_place_test2.py
        CarrierSpec 과 동일한 유도)."""
        return self.qr_side_m * 21.0 / 29.0


# T_QR_grasp 값 출처: grasp.yaml 의 tray_1_orange/tray_2_blue 를 그대로
# 가져온다(그 파일 주석 — 매거진과 같은 규칙으로 에셋 기하에서 유도, 매거진
# 두 값을 역산해 규칙이 맞는 것을 확인했다). 이 테스트 씬의 stack_1 은
# F3_STKO_1.usda(주황) payload 라 "F3-STKO" 를 쓴다.
STACK_SPECS = {
    "F3-STKO": CarrierSpec("stack_orange", None, 0.040, np.array([0.1373, -0.0530, 0.06845])),
    "F3-STKB": CarrierSpec("stack_blue",   None, 0.040, np.array([0.1373, -0.0570, 0.06845])),
}

QR_FRAMES       = 3    # 여러 장 찍어 중앙값으로 모은다(aggregate_qr_poses)
QR_SETTLE_STEPS = 30   # 강성을 SCAN 으로 낮춘 뒤 흔들림이 가라앉을 시간
DEPTH_AOV_NAME  = "DistanceToImagePlaneSD"   # ★ Replicator 애노테이터 이름이 아니다
                                              # (sim_backend.py DEPTH_AOV_NAME 참고)

with open(FRAMES_YAML, "r", encoding="utf-8") as _fh:
    _frames = yaml.safe_load(_fh)
_st = _frames["static_transforms"]
_CI = _frames["wrist_camera"]["camera_info_observed"]
CAMERA_K    = np.array(_CI["k"], dtype=float).reshape(3, 3)
CAMERA_DIST = np.array(_CI["d"], dtype=float)
# R_L6_CAM/R_CAM_OPT 는 quat_to_matrix() 가 정의된 뒤에 계산한다(아래 참고).


# ══════════════════════════════════════════════════════════════
#  회전 유틸
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


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return quat_to_matrix([w, x, y, z])


# frames.yaml 은 xyzw 순서라 quat_to_matrix(wxyz) 정의 뒤에야 계산할 수 있다
# (위 "카메라 / QR 인식" 절에서 CAMERA_K/CAMERA_DIST 는 이미 읽어 뒀다).
R_L6_CAM  = quat_xyzw_to_mat(_st["m0609_tool0__camera_link"]["quat_xyzw"])
T_L6_CAM  = np.array(_st["m0609_tool0__camera_link"]["xyz"], dtype=float)
R_CAM_OPT = quat_xyzw_to_mat(_st["camera_link__camera_color_optical_frame"]["quat_xyzw"])


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


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


def orientation_error_deg(quat_a, quat_b):
    dot = float(np.clip(abs(np.dot(quat_a, quat_b)), -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(dot)))


# ══════════════════════════════════════════════════════════════
#  USD 조회 유틸
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
    prim = stage().GetPrimAtPath(prim_path)
    cache = UsdGeom.XformCache()
    m = cache.GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
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


def filter_collision(path_a, path_b):
    a = stage().GetPrimAtPath(path_a)
    rel = UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel()
    rel.AddTarget(Sdf.Path(path_b))


def reset_prim_pose(prim_path, pos, quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """prim 의 월드 pose 를 직접 갈아끼운다(12_place_test.py 의
    reset_magazine_pose 를 임의 prim 에 쓸 수 있게 일반화한 것)."""
    xform = UsdGeom.Xformable(stage().GetPrimAtPath(prim_path))
    xform.ClearXformOpOrder()
    q = Gf.Quatd(quat_wxyz[0], Gf.Vec3d(*quat_wxyz[1:]))
    m = Gf.Matrix4d().SetRotate(q)
    m.SetTranslateOnly(Gf.Vec3d(*pos))
    xform.AddTransformOp().Set(m)


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 제어 — 12_place_test.py 와 동일
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


def set_arm_stiffness(value):
    """관측(SCAN, 낮게) ↔ 파지/이송(PICK, 높게) 를 오간다 — DRIVE_STIFFNESS_SCAN
    정의부 주석 참고. OutputPickPlaceTask._setup_arm_drives() 는 부팅 시
    DRIVE_STIFFNESS(PICK 값)로 한 번 세팅만 하므로, 관측 단계에서 낮췄다가
    PICK 전에 다시 여기로 되돌려야 한다."""
    for prim in Usd.PrimRange(stage().GetPrimAtPath(ROBOT_PRIM_PATH)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        for drive_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
            if drive:
                drive.GetStiffnessAttr().Set(value)


# ══════════════════════════════════════════════════════════════
#  손목 카메라 — QR 스캔용 캡쳐. sim_backend.py _ensure_camera_warm()/
#  _capture_frame() 을 로봇 한 대짜리로 그대로 옮겼다(같은 이유: Isaac Sim
#  5.1.0 rc.19 에서 rep.create.render_product() 의 rgb 애노테이터가 항상
#  빈 프레임을 줘서, omni.kit.widget.viewport.capture 로 우회한다 — GUI
#  필요, headless 불가).
# ══════════════════════════════════════════════════════════════
def _pycapsule_to_bytes(capsule, size):
    ctypes.pythonapi.PyCapsule_GetName.restype = ctypes.c_char_p
    ctypes.pythonapi.PyCapsule_GetName.argtypes = [ctypes.py_object]
    name = ctypes.pythonapi.PyCapsule_GetName(capsule)
    ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.c_void_p
    ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    ptr = ctypes.pythonapi.PyCapsule_GetPointer(capsule, name)
    if not ptr:
        raise RuntimeError("PyCapsule 에서 포인터를 못 가져왔다")
    return bytes((ctypes.c_uint8 * size).from_address(ptr))


_capture_ready = [False]     # 뷰포트에 depth AOV 를 한 번 등록했는지
_pending_capture = [None]    # GC 되면 콜백이 안 온다 — 끝날 때까지 붙잡아둔다


def optical_pose():
    l6_p, l6_q = get_world_pose(EE_LINK_PATH)
    R_l6 = quat_to_matrix(l6_q)
    return R_l6 @ R_L6_CAM @ R_CAM_OPT, l6_p + R_l6 @ T_L6_CAM


def capture_frame(world, timeout_frames=180):
    from omni.kit.viewport.utility import get_active_viewport
    from omni.kit.widget.viewport.capture import MultiAOVByteCapture

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("활성 뷰포트를 찾을 수 없다")

    result = {}

    def _on_rgb(buffer, buffer_size, width, height, byte_format):
        raw = _pycapsule_to_bytes(buffer, buffer_size)
        arr = np.frombuffer(raw, dtype=np.uint8, count=buffer_size)
        result["rgb"] = arr.reshape(height, width, 4)[:, :, :3][:, :, ::-1].copy()

    def _on_depth(buffer, buffer_size, width, height, byte_format):
        raw = _pycapsule_to_bytes(buffer, buffer_size)
        arr = np.frombuffer(raw, dtype=np.float32, count=width * height)
        result["depth"] = arr.reshape(height, width).astype(np.float64).copy()

    cap = MultiAOVByteCapture(["", DEPTH_AOV_NAME], [_on_rgb, _on_depth])
    _pending_capture[0] = cap
    viewport.schedule_capture(cap)
    for _ in range(timeout_frames):
        world.step(render=True)
        if "rgb" in result and "depth" in result:
            _pending_capture[0] = None
            return result["rgb"], result["depth"]
    _pending_capture[0] = None
    raise RuntimeError("프레임 캡쳐 타임아웃 — rgb/depth 콜백이 오지 않았다")


def ensure_camera_warm(world, force=False):
    from omni.kit.viewport.utility import get_active_viewport, add_aov_to_viewport
    import carb.settings

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError(
            "활성 뷰포트를 찾을 수 없다 — QR 스캔은 GUI 에서만 된다(헤드리스면 안 통한다)")
    viewport.camera_path = CAMERA_PRIM
    if force or not _capture_ready[0]:
        carb.settings.get_settings().set("/app/hydra/renderSettings/saveUsdAttributes", False)
        add_aov_to_viewport(viewport, DEPTH_AOV_NAME)
        _capture_ready[0] = False
    for _ in range(30):
        world.step(render=True)
    capture_frame(world, timeout_frames=240)   # 워밍업 검증 — 실패하면 예외를 올린다
    _capture_ready[0] = True


def scan_qr(world, spec, n_frames=QR_FRAMES):
    """여러 장을 찍어 중앙값으로 모은다(aggregate_qr_poses) — 12_place_test2.py
    QrVision.scan() 과 동일."""
    import cv2
    obs = []
    for i in range(n_frames):
        bgr, depth = capture_frame(world)
        R_opt, p_opt = optical_pose()
        # ★ 씬 조명이 어두워 흰 라벨이 낮게 찍힌다 — 대비를 펴 주면 디코딩된다
        #   (12_place_test2.py 실측과 동일). 기하는 안 건드린다.
        bgr = cv2.normalize(bgr, None, 0, 255, cv2.NORM_MINMAX)
        o = estimate_qr_pose(bgr, depth, CAMERA_K, CAMERA_DIST, R_opt, p_opt,
                             expected_id=spec.expected_id,
                             data_side_m=spec.data_side_m)   # ★ 스택은 40mm 라 기본값과 다르다
        obs.append(o)
        print(f"      frame {i}  ok={o.ok}  decoded={o.decoded!r}  {o.reason}")
    return aggregate_qr_poses(obs)


def grasp_from_qr(agg, spec):
    """QR 자세 → 흡착점(월드). sim_backend.scan_qr/12_place_test2.py 의
    grasp_from_qr() 과 같은 QR 프레임 구성이다:
      x_qr : yaw 로 정해지는 수평 방향(라벨의 '오른쪽')
      y_qr : 월드 -Z (라벨의 '아래'. 캐리어가 똑바로 서 있다는 가정)
      z_qr : x×y (라벨 안쪽)
    """
    yaw = float(agg.yaw_rad)
    x_qr = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y_qr = np.array([0.0, 0.0, -1.0])
    R_qr = np.column_stack([x_qr, y_qr, np.cross(x_qr, y_qr)])
    return np.asarray(agg.center_world, dtype=float) + R_qr @ spec.t_qr_grasp, yaw


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
class OutputPickPlaceTask(BaseTask):
    """simple_factory_layout.usda 를 그대로 올리고 로봇/그리퍼 파라미터만 조정한다.

    12_place_test.py 의 MagazinePickPlaceTask 와 달리 TestItem 비활성화나
    그리퍼<->매거진 필터링은 하지 않는다 — 이 스크립트는 매거진을 팔로
    집지 않는다(스텝 1 이 텔레포트로 끝난다). 그리퍼<->스택 필터링은 스택이
    스폰된 뒤에야 경로를 알 수 있어 main() 에서 나중에 건다.
    """

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_world()
        self._setup_arm_drives()
        self._configure_gripper_limits()
        self._register_robot(scene)
        print("   scene        ready")

    def _load_world(self):
        world_prim = stage().GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage(), "/World").GetPrim()
        world_prim.GetReferences().AddReference(WORLD_USD)
        for _ in range(15):
            simulation_app.update()
        print(f"   USD          loaded  {WORLD_USD}")

    def _setup_arm_drives(self):
        """nova_carter1/m0609 의 팔 6축에만 Drive 를 강화한다(nova_carter2 는 안 건드림)"""
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


def joints_deg_action(robot, joints_deg):
    """관절 목표(도) 하나를 ArticulationAction 으로. 관측 자세를 매 스텝
    다시 걸어 유지하는 데 쓴다(안 걸면 팔이 흘러내린다 — 12_place_test2.py
    carry_pose_action 과 같은 이유)."""
    idx = np.array([robot.get_dof_index(name) for name in ARM_JOINTS])
    target = np.deg2rad(np.array(joints_deg, dtype=float))
    return ArticulationAction(joint_positions=target, joint_indices=idx)


def load_observe_joints_deg():
    """taught_poses.yaml 에서 OBSERVE_POSE_NAME 을 찾아 관절값(도)을 준다.

    아직 안 찍혀 있으면(capture_pose.py 로 티칭 전) READY 로 폴백하면서
    경고를 찍는다 — carrier_code_reader.py 의 FALLBACK_POSE_BY_PATROL_TARGET
    과 같은 패턴. 모듈 docstring "관측 자세" 절 참고.
    """
    if TAUGHT_POSES.exists():
        taught = yaml.safe_load(TAUGHT_POSES.read_text(encoding="utf-8")) or {}
        entry = taught.get(OBSERVE_POSE_NAME)
        if entry is not None:
            print(f"   관측 자세  taught_poses.yaml 의 '{OBSERVE_POSE_NAME}' 사용")
            return list(entry["joints_deg"])
    print(f"   !! '{OBSERVE_POSE_NAME}' 이 taught_poses.yaml 에 없다 — READY 로 폴백한다.")
    print(f"      티칭하려면: CAPTURE_BASE=packaging_output "
          f"isaac_python isaacpjt/tools/capture_pose.py")
    return list(READY_JOINTS_DEG)


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
#  베이스 텔레포트 — 12_place_test.py 와 동일
# ══════════════════════════════════════════════════════════════
def yaw_quat_wxyz(yaw_deg):
    return quat_from_axis([0, 0, 1], yaw_deg)


class BaseTeleporter:
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
#  Pick / Place FSM — 12_place_test2.py 의 일반화판(임의 carrier_path +
#  grasp_world/target_xy 를 받는다). 12_place_test.py 의 것은 매거진의
#  flange_plate 에 박혀 있어 그대로는 못 쓴다.
# ══════════════════════════════════════════════════════════════
class PickFSM:
    NAMES = ["APPROACH", "DESCEND", "GRIP", "HOLD", "LIFT", "DONE"]
    DONE_STATE = 5

    def __init__(self, robot, gripper, grasp_world, carrier_path):
        self._robot = robot
        self._gripper = gripper
        self._carrier = carrier_path
        self.grasp_world = np.asarray(grasp_world, dtype=float)
        self.reset()

    def reset(self):
        self.attempt = 0
        self.z_at_grip = None
        self.grip_ok = False
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
        cx, cy, top = self.grasp_world
        grip_z = top + GRIP_GAPS[self.attempt]
        self.waypoints = [
            np.array([cx, cy, top + APPROACH_HEIGHT_OFFSET]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, top + LIFT_HEIGHT_OFFSET]),
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
            self.done, self.fail_code = True, FAIL_UNREACHABLE

    def advance(self):
        if self.done:
            return
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            if self.state == 2:
                self.z_at_grip = measure_prim(self._carrier)[1]
                self._gripper.close()
                self.gripper = "close"
                self.n_steps, dist = GRIP_WAIT, 0.0
            elif self.state == 3:
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
        if self.state == 2 and not self._check_grip():
            return
        if self.state == 4:
            self._judge()
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            self.fail_code = self.fail_code if self.fail_code is not None else FAIL_OK
            print(f"   [{self.DONE_STATE}] PICK DONE  fail_code={self.fail_code}")

    def _watch_drop(self):
        if self.state not in (3, 4) or not self.grip_ok or self.done:
            return
        if self.step % 10 or holding(self._gripper.gripped()):
            return
        print(f"   !! 놓쳤다  단계 {self.NAMES[self.state]}  {self.step}/{self.n_steps} 스텝")
        self.done, self.fail_code = True, FAIL_SLIP

    def _check_grip(self):
        ok = holding(self._gripper.gripped())
        print(f"   ── 시도 {self.attempt + 1}/{len(GRIP_GAPS)}  "
              f"간격 {GRIP_GAPS[self.attempt]*1000:+.0f} mm  "
              f"status {self._gripper.status()}  -> {'붙었다' if ok else '안 붙었다'}")
        if ok:
            self.grip_ok = True
            return True
        self.attempt += 1
        if self.attempt >= len(GRIP_GAPS):
            self.done, self.fail_code = True, FAIL_SLIP
            print("   흡착 실패 — GRIP_GAPS 를 다 써봤다")
            return False
        self._gripper.open()
        self.gripper = "open"
        self._rebuild()
        self.state, self.step, self.start = 1, 0, None
        return False

    def _judge(self):
        now = measure_prim(self._carrier)[1]
        self.lift_rise_m = now - self.z_at_grip
        _, quat = get_world_pose(self._carrier)
        self.tilt_at_lift_deg = tilt_deg_from_quat(quat)
        ok = (self.lift_rise_m >= LIFT_OK_MIN_M) and (self.tilt_at_lift_deg <= TILT_MAX_DEG)
        print(f"   판정  rise {self.lift_rise_m*1000:+.1f} mm (>= {LIFT_OK_MIN_M*1000:.0f} mm)  "
              f"tilt {self.tilt_at_lift_deg:.2f} deg (<= {TILT_MAX_DEG:.0f})  "
              f"-> {'성공' if ok else '실패'}")
        if not ok:
            self.fail_code = FAIL_SLIP


class PlaceFSM:
    """벨트 윗면에 내려놓는다. surface_z(벨트면) + carrier_height 로 목표 높이를 낸다."""

    NAMES = ["MOVE", "LOWER", "RELEASE", "RETREAT", "DONE"]
    DONE_STATE = 4

    def __init__(self, robot, gripper, target_xy, surface_z, carrier_height):
        self._robot = robot
        self._gripper = gripper
        gx, gy = target_xy
        ref_z = float(surface_z) + float(carrier_height)
        self.waypoints = [
            np.array([gx, gy, ref_z + APPROACH_HEIGHT_OFFSET]),
            np.array([gx, gy, ref_z + PLACE_DROP]),
            np.array([gx, gy, ref_z + PLACE_DROP]),
            np.array([gx, gy, ref_z + APPROACH_HEIGHT_OFFSET]),
        ]
        self.done = False
        self.fail_code = None
        self.state = 0
        self.step = 0
        self.start = None
        self.gripper = "close"
        self.ik_fail_streak = 0

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            return self.waypoints[self.state]
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    note_ik = PickFSM.note_ik

    def advance(self):
        if self.done:
            return
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            if self.state == 2:
                self._gripper.open()
                self.gripper = "open"
                self.n_steps, dist = RELEASE_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)
            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}  {dist:.4f} m  {self.n_steps} steps"
                  f"  gripper {self.gripper}")
        self.step += 1
        if self.state in (0, 1) and self.step % 10 == 0 and not holding(self._gripper.gripped()):
            self.done, self.fail_code = True, FAIL_SLIP
            print(f"   !! 이송 중 놓쳤다  단계 {self.NAMES[self.state]}")
            return
        if self.step < self.n_steps:
            return
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            self.fail_code = self.fail_code if self.fail_code is not None else FAIL_OK
            print(f"   [{self.DONE_STATE}] PLACE DONE  fail_code={self.fail_code}")


# ══════════════════════════════════════════════════════════════
#  결과 레코드
# ══════════════════════════════════════════════════════════════
@dataclasses.dataclass
class TrialResult:
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
#  스텝 1 — 이미 놓여 있는 스택을 찾아 안정을 기다린다
#  (테스트 씬에 stack_1 이 이미 있다 — 모듈 docstring 참고. 아무것도
#  스폰하지 않는다 — 예전 판은 여기서 런타임에 직접 스폰했는데, 방금 붙인
#  payload 의 PhysicsRigidBodyAPI/PhysicsCollisionAPI 가 PhysX 에 등록되기
#  전에 먼저 중력을 받아 ShelfDeck 을 그냥 관통해 바닥까지 떨어지는 문제가
#  실측됐다 — 그 경로 자체를 없앤다.)
# ══════════════════════════════════════════════════════════════
def wait_for_settle(world, robot, hold_action, prim_path, timeout_s=STACK_SETTLE_TIMEOUT_S):
    """prim 이 안정될 때까지 기다린 뒤 (위치, 자세, payload 이름) 을 준다.
    stack_1 은 ShelfDeck 위 2mm 에 저장돼 있어(모듈 docstring 참고) 오래
    걸리지 않는다 — 그 2mm 가 가라앉는 것만 기다리는 정도다.

    ★ 좌표를 박지 않는다 — STACK_XY/저장된 z 를 그대로 믿지 않고 여기서
      실측한 자리를 PICK 목표로 쓴다. 씬 파일이 나중에 바뀌어도(위치를
      조금 옮기거나 물리가 살짝 다르게 안착해도) 안전하다.

    hold_action: 대기 중 매 스텝 다시 걸어 줄 관절 목표(관측 자세). 안 걸면
    팔이 흘러내린다.
    """
    section("WAIT — 스택 안정 대기 (관측 자세 유지)")
    t0 = time.time()
    payload = stage().GetPrimAtPath(prim_path).GetAttribute("flow:stackPayload")
    payload_name = Path(str(payload.Get())).stem if payload and payload.Get() else "?"

    last_pos, last_quat = None, None
    stable = 0
    while stable < STACK_SETTLE_FRAMES and time.time() - t0 < timeout_s:
        robot.apply_action(hold_action)
        world.step(render=not HEADLESS)
        pos, quat = get_world_pose(prim_path)
        stable = stable + 1 if (last_pos is not None
                                 and np.linalg.norm(pos - last_pos) < STACK_SETTLE_TOL_M) else 0
        last_pos, last_quat = pos, quat
    if stable < STACK_SETTLE_FRAMES:
        print(f"   !! {timeout_s:.0f}s 안에 완전히 안 멈췄다 — 마지막 자리로 계속 진행한다"
              f"  {vec(last_pos, 4)}")
    else:
        print(f"   멈춘 자리 {vec(last_pos, 4)}")
    return last_pos, last_quat, payload_name


# ══════════════════════════════════════════════════════════════
#  사전 점검 — 테스트 스테이션 PLACE 접근이 IK 로 풀리는지 먼저 확인
# ══════════════════════════════════════════════════════════════
def reachability_check_place(world, robot, lula, solver, teleporter, target_quat):
    section("REACHABILITY CHECK — 테스트 스테이션 PLACE 접근")
    pos, yaw = WP_PLACE_TEST
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    approach_tcp = np.array([BELT_PLACE_X, BELT_Y_TESTING,
                              BELT_TOP_Z + STACK_HEIGHT_GUESS_M + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)
    _, solved = solver.compute_inverse_kinematics(
        target_position=flange_target, target_orientation=target_quat)
    print(f"   WP_PLACE_TEST  approach tcp {vec(approach_tcp)}  -> {'SOLVED' if solved else 'FAILED'}")
    if not solved:
        print("   !! FAILED — BELT_BASE_X/BELT_Y_TESTING 좌표를 조정해야 한다")
    return solved


# ══════════════════════════════════════════════════════════════
#  스텝 3 — PICK(실측 자리) → TRANSPORT(텔레포트) → PLACE(테스트 스테이션)
# ══════════════════════════════════════════════════════════════
def run_trial(world, robot, lula, solver, gripper, teleporter, target_quat,
              stack_path, stack_pos, stack_quat, payload_name, observe_action):
    t0 = time.time()

    # ── 실측 자리 앞으로 정밀 이동 ──────────────────────────────
    px = float(stack_pos[0])
    py = float(stack_pos[1]) - OUTPUT_STANDOFF_M
    teleporter.teleport(np.array([px, py, CARTER_Z]), 0.0)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    # ── OBSERVE — 손목캠으로 QR 을 읽어 파지점을 낸다 ─────────────
    # ★ GT(measure_prim) 로 좌표를 미리 아는 대신, 매거진 pick 과 같은
    #   경로(estimate_qr_pose → grasp_from_qr)로 실제로 인식한다 — GT 는
    #   '맞았는지 채점' 에만 쓴다(12_place_test2.py 와 같은 원칙).
    section("OBSERVE — QR 인식")
    key = "F3-STKB" if "STKB" in payload_name else "F3-STKO"
    stack_spec = STACK_SPECS[key]
    print(f"   스택 사양   {stack_spec.name}  라벨 {stack_spec.qr_side_m*1000:.0f} mm  "
          f"data_side {stack_spec.data_side_m*1000:.2f} mm")

    set_arm_stiffness(DRIVE_STIFFNESS_SCAN)   # 잔진동으로 디코딩이 깨지지 않는 값으로
    for _ in range(QR_SETTLE_STEPS):
        robot.apply_action(observe_action)
        world.step(render=not HEADLESS)
    ensure_camera_warm(world)
    agg = scan_qr(world, stack_spec)
    set_arm_stiffness(DRIVE_STIFFNESS)        # PICK 은 다시 파지 강성으로

    if not agg.ok:
        print(f"\n   스택 QR 인식 실패({agg.reason}) — 중단. 관측 자세가 실제로 QR 을"
              " 보고 있는지 확인할 것(모듈 docstring '관측 자세' 절 참고).")
        return TrialResult(
            fail_code=FAIL_NO_QR, cycle_time_s=time.time() - t0,
            lift_rise_mm=0.0, tilt_deg=0.0,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    grasp_world, _yaw = grasp_from_qr(agg, stack_spec)
    # height 는 PLACE 목표 높이 계산에 그대로 필요해서 GT 로 잰다 — 파지점
    # 자체(grasp_world)만 QR 로 바꾼 것이다.
    center_xy, top_z, height = measure_prim(stack_path)
    gt = np.array([center_xy[0], center_xy[1], top_z])
    err_mm = float(np.linalg.norm(grasp_world - gt)) * 1000.0
    print(f"   [{payload_name}] QR 파지점 {vec(grasp_world, 4)}   GT {vec(gt, 4)}   "
          f"차이 {err_mm:.1f} mm   높이 {height*1000:.1f} mm")

    # ── 1) PICK ────────────────────────────────────────────
    pick_fsm = PickFSM(robot, gripper, grasp_world, stack_path)
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
            fail_code=pick_fsm.fail_code, cycle_time_s=time.time() - t0,
            lift_rise_mm=pick_fsm.lift_rise_m * 1000.0, tilt_deg=pick_fsm.tilt_at_lift_deg,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    # 잡은 뒤에는 차체·팔과도 필터링 — 주행/텔레포트 진동에 스쳐 놓치지
    # 않게(carrier_code_reader 관련 최근 개선과 같은 이유).
    filter_collision(BASE_XFORM_PATH, stack_path)
    filter_collision(ROBOT_PRIM_PATH, stack_path)

    # ── 2) TRANSPORT (텔레포트, 주행 없음) ─────────────────
    # 12_place_test.py 와 같은 이유: 베이스 아티큘레이션을 텔레포트하면
    # 거리와 무관하게 흡착이 즉시 풀린다. 팔을 먼저 목적지 접근 위치로
    # 보내고, 거기서 물체를 그리퍼 위치로 옮겨 붙인 뒤 재흡착한다.
    pos, yaw = WP_PLACE_TEST
    teleporter.teleport(pos, yaw)
    sync_ik_base_pose(lula)
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    approach_tcp = np.array([BELT_PLACE_X, BELT_Y_TESTING,
                              BELT_TOP_Z + height + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)

    def _hold_ik_target(n_steps):
        for _ in range(n_steps):
            action, solved = solver.compute_inverse_kinematics(
                target_position=flange_target, target_orientation=target_quat)
            if solved:
                robot.apply_action(action)
            world.step(render=not HEADLESS)

    _hold_ik_target(MAX_STEPS)

    tcp_now = get_tcp_pose(robot)
    new_origin = np.array([tcp_now[0], tcp_now[1], tcp_now[2] - height])
    reset_prim_pose(stack_path, new_origin, tuple(stack_quat))

    gripper.close()
    reattached = False
    for _ in range(GRIP_WAIT):
        action, solved = solver.compute_inverse_kinematics(
            target_position=flange_target, target_orientation=target_quat)
        if solved:
            robot.apply_action(action)
        world.step(render=not HEADLESS)
        if holding(gripper.gripped()):
            reattached = True
            break
    print(f"   텔레포트 후 재흡착  -> {'붙었다' if reattached else '안 붙었다'}  tcp={get_tcp_pose(robot)}")
    if not reattached:
        return TrialResult(
            fail_code=FAIL_SLIP, cycle_time_s=time.time() - t0,
            lift_rise_mm=pick_fsm.lift_rise_m * 1000.0, tilt_deg=pick_fsm.tilt_at_lift_deg,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    _hold_ik_target(BASE_MOVE_SETTLE_STEPS)
    sync_ik_base_pose(lula)

    # ── 3) PLACE ───────────────────────────────────────────
    place_fsm = PlaceFSM(robot, gripper, (BELT_PLACE_X, BELT_Y_TESTING), BELT_TOP_Z, height)
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

    fail_code = place_fsm.fail_code
    place_pos_err_mm = None
    place_orient_err_deg = None

    if fail_code == FAIL_OK:
        for _ in range(SETTLE_STEPS):
            world.step(render=not HEADLESS)
        final_pos, final_quat = get_world_pose(stack_path)
        target_xy = np.array([BELT_PLACE_X, BELT_Y_TESTING])
        place_pos_err_mm = float(np.linalg.norm(final_pos[:2] - target_xy)) * 1000.0
        place_orient_err_deg = orientation_error_deg(final_quat, stack_quat)
        if (place_pos_err_mm > PLACE_POS_TOL_M * 1000.0
                or place_orient_err_deg > PLACE_ORIENT_TOL_DEG):
            fail_code = FAIL_PLACE_ERROR
        print(f"   배치 오차   pos {place_pos_err_mm:.2f} mm (<= {PLACE_POS_TOL_M*1000:.0f})  "
              f"orient {place_orient_err_deg:.2f} deg (<= {PLACE_ORIENT_TOL_DEG:.0f})")

    return TrialResult(
        fail_code=fail_code, cycle_time_s=time.time() - t0,
        lift_rise_mm=pick_fsm.lift_rise_m * 1000.0, tilt_deg=pick_fsm.tilt_at_lift_deg,
        place_pos_error_mm=place_pos_err_mm, place_orient_error_deg=place_orient_err_deg,
    )


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = OutputPickPlaceTask(name="output_pick_place_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    section("SOLVER")
    base_pos0, base_quat0 = get_world_pose(BASE_LINK_PATH)
    lula, solver = create_ik_solver(robot, base_pos0, base_quat0)
    gripper = SurfaceGripperCtl(task.gripper_node_path)
    teleporter = BaseTeleporter(robot)
    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)

    reachable = reachability_check_place(world, robot, lula, solver, teleporter, target_quat)
    if not reachable:
        print("\n   BELT_BASE_X/BELT_Y_TESTING 좌표를 먼저 조정하세요")
        simulation_app.close()
        return

    # ── STEP 1 — 출력 자리 앞에서 관측 자세로 대기 ──────────────
    section("STEP 1 — 관측 자세로 대기 (스택 출력 자리 앞)")
    stage_pos, stage_yaw = WP_STAGE_OUTPUT
    teleporter.teleport(stage_pos, stage_yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    observe_joints_deg = load_observe_joints_deg()
    observe_action = joints_deg_action(robot, observe_joints_deg)
    for _ in range(SETTLE_STEPS):
        robot.apply_action(observe_action)
        world.step(render=not HEADLESS)
    print(f"   관측 자세 고정  {observe_joints_deg}")

    # ── STEP 2 — 이미 놓여 있는 스택(stack_1)의 안정을 기다린다 ──
    stack_path = EXISTING_STACK_PATH
    filter_collision(GRIPPER_PRIM, stack_path)
    stack_pos, stack_quat, payload_name = wait_for_settle(world, robot, observe_action, stack_path)

    # ── STEP 3 — PICK → 테스트 스테이션 PLACE ───────────────────
    section("STEP 3 — PICK → 테스트 스테이션 PLACE")
    result = run_trial(world, robot, lula, solver, gripper, teleporter, target_quat,
                        stack_path, stack_pos, stack_quat, payload_name, observe_action)

    section("결과")
    print(f"   fail_code={result.fail_code}  cycle_time={result.cycle_time_s:.2f}s  "
          f"lift_rise={result.lift_rise_mm:+.1f} mm  tilt={result.tilt_deg:.2f} deg")
    if result.place_pos_error_mm is not None:
        print(f"   place_pos_error={result.place_pos_error_mm:.2f} mm  "
              f"place_orient_error={result.place_orient_error_deg:.2f} deg")

    section("HOLD")
    print("   결과 상태로 정지. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=not HEADLESS)
    simulation_app.close()


if __name__ == "__main__":
    main()

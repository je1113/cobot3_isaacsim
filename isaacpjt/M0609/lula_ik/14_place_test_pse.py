"""
단위 테스트 — 매거진을 집어 컨베이어 앞 바닥 스테이징 지점에 내려놓기 (PICK + PLACE)

    isaac_python 14_place_test_pse.py   (기본이 GUI)
    (헤드리스로 돌리려면)  PICK_HEADLESS=1 isaac_python 14_place_test_pse.py

12_pick_test.py 의 PICK 을 그대로 쓰고, 12_place_test.py 가 실측으로 찾아낸 이송/
배치 방식을 그대로 따른다. 이 파일의 목적은 같은 시나리오를 한 파일에서 돌리면서
사전 검증(REACHABILITY CHECK)을 조금 더 촘촘히 찍어 보는 것이다.

  PICK        APPROACH -> DESCEND -> GRIP(재시도) -> HOLD -> LIFT
  TRANSPORT   텔레포트 -> 준비자세 리셋 -> 배치 접근점 유지 -> 매거진 재부착
  PLACE       MOVE -> LOWER -> RELEASE -> RETREAT


─── 이전 버전(순간이동으로 들고 가기)이 왜 틀렸는가 ───────────────

이 파일의 앞 버전은 "카터와 매거진을 같은 강체 변환으로 함께 옮기면 흡착이
유지된다"는 전제로 짰다. 12_place_test.py 가 실측으로 확인한 사실과 정면으로
어긋난다. 그 파일 run_trial 주석에 이렇게 적혀 있다.

  · 베이스 아티큘레이션에 set_world_pose() 를 호출하면 거리와 무관하게 흡착이
    즉시 풀린다 (실측)
  · PICK 은 선반 옆(+y)으로, PLACE 는 컨베이어 앞(+x)으로 뻗어야 해서 잡았던
    상대 위치를 그대로 들고 오면 WP_PLACE 에서 그 y 오프셋(약 0.55 m)이 IK 로
    안 풀린다 (실측)
  · PICK 직후의 관절 각도를 IK 시드로 그대로 쓰면 10 m 떨어진 새 목표에서는
    600 스텝 내내 solved=False 가 된다 — 준비자세로 리셋해야 한다 (실측)
  · IK 목표를 한 번만 걸고 world.step() 만 반복하면 팔이 다른 자세로 흘러내린다.
    매 스텝 다시 명령해야 한다 (실측)

그래서 "붙든 채 옮기기" 를 버리고, 팔을 먼저 검증된 배치 접근점으로 보낸 뒤
매거진을 그리퍼 밑으로 옮겨 다시 흡착하는 12_place_test.py 방식을 따른다.
위 네 가지는 전부 Isaac 안에서 확인된 것이고, 이 환경에서는 재현할 수 없다.


─── 좌표는 12_place_test.py 를 그대로 따른다 ──────────────────

WP_PICK / WP_PLACE / PLACE_TARGET_XY 는 12_place_test.py 값 그대로다. WP_PLACE 는
한때 3.35 였다가 8c455f6 에서 4.0 으로 고쳐졌다 — URDF 로 FK/IK 를 직접 풀어 봐도
그 수정이 맞다.

    carter x = 3.35  ->  팔 베이스 x = 3.143,  수평 0.856 m,  부하 113~118%  해 없음
    carter x = 4.00  ->  팔 베이스 x = 3.793,  수평 0.206 m,  부하  36~ 50%  여유 있음

차체 앞면(carter x + 0.14 = 4.14)과 ConveyorFrame 앞면(x = 4.2) 간격은 6 cm 로
좁은 편이니, 카터를 더 붙이는 방향으로는 조정하지 않는다. 아래 REACHABILITY CHECK
가 실행 시 Lula 로 다시 확인해 준다.


─── 검증 상태 ─────────────────────────────────────────────────

Isaac 에서 돌려본 것은 아니다. 이 환경에는 Isaac 이 없다. 확인한 것은
  · PICK 경로가 12_pick_test.py 와, PlaceFSM 이 12_place_test.py 와 파라미터·함수
    단위로 같다는 것 (주석 제외 동작 코드 기계적 대조)
  · WP_PLACE 도달성 (URDF FK/IK 직접 구현, 12_pick_test.py 의 알려진 성공/실패
    지점으로 모델을 먼저 맞춘 뒤 적용)
  · prim 경로가 현재 레이아웃에 실제로 존재한다는 것 (usd-core 로 레이아웃 파싱)
흡착이 재부착되는지, 팔이 접근점에 실제로 도달하는지는 실행해 봐야 안다.


중요 — 이 씬에서 확인이 필요한 값들:
  · WP_PICK 은 12_pick_test.py 에서 성공한 값 그대로다.
  · 레이아웃이 /World/Robots/..., /World/Magazines/... 로 재편됐다. 경로가 틀리면
    _wait_for_scene 이 즉시 "레이아웃에 없는 prim 경로" 로 죽는다.
  · 컨베이어 위 TestItem 은 현재 레이아웃에서 제거됐다 (12_place_test.py 에 있는
    비활성화 코드는 이 파일에 없다).
  · 컨베이어는 8c455f6 에서 전체가 낮아졌다 (BeltTop 상판 0.770 -> 0.541,
    ConveyorFrame 윗면 0.675 -> 0.466). 배치 지점이 벨트가 아니라 그 앞
    바닥(z=0)이라 이 스크립트의 좌표에는 영향이 없다.
  · physx 텐서 속도 에러(getVelocities expected 12 / received 24)는 8c455f6 에서
    레이아웃의 rsd455 RSD455 프림 리지드 바디와 IMU 를 끄는 것으로 해결됐다.
    이 스크립트가 따로 할 일은 없다.
"""

import os

from isaacsim import SimulationApp

HEADLESS = os.environ.get("PICK_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import dataclasses
import time
from pathlib import Path
from typing import Optional

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
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
#  씬 prim 경로 — 12_pick_test.py / 12_place_test.py 와 동일
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

MAGAZINE_XFORM_PATH = "/World/Magazines/shelf_1_magaines/top_magazines/magazine_1_orange"
# payload 로 합성되면 magazine_1_orange_qr.usda 의 defaultPrim("Magazine")이 이
# 경로 자체에 별칭(alias)되므로, 그 자식(flange_plate 등)은 별도의 "Magazine"
# 서브프림이 아니라 MAGAZINE_XFORM_PATH 바로 아래에 붙는다.
MAGAZINE_PATH = MAGAZINE_XFORM_PATH
FLANGE_NAME   = "flange_plate"                      # top-grasp 지점 (파지용 손잡이)
FLANGE_PATH   = f"{MAGAZINE_PATH}/{FLANGE_NAME}"

# 씬 합성이 끝났는지 판단할 prim 들 (라벨, 탐색 루트, prim 이름).
# 셋 다 참조/페이로드 '안'에 있는 prim 이라, 이게 보이면 합성이 끝난 것이다.
REQUIRED_PRIMS = [
    ("m0609 joint_6",           ROBOT_PRIM_PATH,     "joint_6"),
    ("short_gripper 흡착 노드",  GRIPPER_PRIM,        "SurfaceGripper"),
    ("magazine flange_plate",   MAGAZINE_XFORM_PATH, FLANGE_NAME),
]
SCENE_LOAD_TIMEOUT_S = 180.0   # 원격 payload(S3) 첫 다운로드까지 감안한 상한


# ══════════════════════════════════════════════════════════════
#  로봇 관절 / 드라이브 — 12_pick_test.py 와 동일
# ══════════════════════════════════════════════════════════════
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# URDF 실측 — 사전 검증 리포트의 참고 수치 계산용 (제어에는 안 쓴다)
SHOULDER_Z = 0.1345           # base_link -> joint_1
LINK_2     = 0.411            # joint_1 -> joint_3
LINK_3     = 0.368            # joint_3 -> joint_4
WRIST_Z    = 0.121            # joint_5 -> joint_6
TWO_LINK   = LINK_2 + LINK_3  # 어깨 -> 손목중심 최대
SPEC_REACH = LINK_2 + LINK_3 + WRIST_Z   # 0.900, 카탈로그 도달반경


# ══════════════════════════════════════════════════════════════
#  베이스 텔레포트 waypoint
# ══════════════════════════════════════════════════════════════
#   magazine_1_orange  (-6.5, 2.0, 0.6)  Shelf_01 Level_2 위
#   PLACE 목표          (4.0, 0.0, 바닥)  ConveyorFrame(x>=4.2) 바로 앞
#
# [WP_PICK] 12_pick_test.py 에서 성공한 값 그대로다. Shelf_01 은 선반판이 x 방향
# 2.5 m 전체를 덮고 있어(world y:[1.867,2.867]) 선반 옆으로 접근하면 선반을
# 관통한다. 차체는 통로 방향(yaw=0)으로 세우고 팔만 옆(+y)으로 뻗어 집는다.
# base_y=1.45 는 선반 앞면과 약 17.9 cm 간격이 남는 지점이다.
#
# [WP_PLACE] ConveyorFrame 충돌체가 x>=4.2 를 꽉 채우고 있어 배치 목표를 그 앞
# 바닥(x=4.0)으로 잡고, 베이스도 그 앞에 yaw=0 으로 세운다. 12_place_test.py 값
# 그대로다. 차체 앞면(x+0.14=4.14)과 프레임 앞면(4.2) 간격이 6 cm 뿐이라 카터를
# 더 붙이는 방향으로는 조정하지 않는다.
WP_PICK  = (np.array([-6.5, 1.45, 0.07963398335074101]), 0.0)
WP_PLACE = (np.array([4.0, 0.0, 0.07963398335074101]), 0.0)

# PLACE 목표 xy — ConveyorFrame 바로 앞 바닥. z 는 매거진 높이를 런타임에 재서 쓴다.
PLACE_TARGET_XY = np.array([4.0, 0.0])
FLOOR_Z = 0.0


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정 — 12_pick_test.py 와 동일
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
#  동작 파라미터 — 12_pick_test.py / 12_place_test.py 와 동일
# ══════════════════════════════════════════════════════════════
APPROACH_HEIGHT_OFFSET = 0.15   # 대상 윗면 기준 접근 대기 높이
LIFT_HEIGHT_OFFSET     = 0.10   # 대상 원래 위치 기준 들고 이동할 높이
PLACE_DROP             = 0.005  # 놓을 때 이만큼 높게 둬서 매거진이 튀지 않게 한다

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
GRIPPER_YAW_DEG    = 0.0
# 두 정거장 모두 카터 yaw 가 0 이라 툴 목표 자세는 한 번만 만들어 두고 계속 쓴다.

MAX_IK_FAIL_STEPS = 30   # 연속 IK 실패 허용치 — 넘으면 UNREACHABLE 로 중단

LOG_INTERVAL = 60


# ══════════════════════════════════════════════════════════════
#  성공 판정 기준 — docs/project-plan.html S4/S6
# ══════════════════════════════════════════════════════════════
LIFT_OK_MIN_M        = 0.005   # 5 mm 리프트 후 유지되면 파지 성공
TILT_MAX_DEG         = 5.0     # 파지 중 기울기 허용치
PLACE_POS_TOL_M      = 0.002   # 배치 위치오차 허용치 (2 mm)
PLACE_ORIENT_TOL_DEG = 1.0     # 배치 자세오차 허용치


# ══════════════════════════════════════════════════════════════
#  fail_code — docs/project-plan.html 체계 + 확장(6, 7)
# ══════════════════════════════════════════════════════════════
FAIL_OK          = 0
FAIL_NOT_FOUND   = 1   # (미사용 — GT pose 라 인식 실패 케이스 없음)
FAIL_LOW_CONF    = 2   # (미사용)
FAIL_SLIP        = 3   # 파지 실패(흡착 재시도 소진) 또는 이송 중 낙하
FAIL_COLLISION   = 4   # (미구현 — 접촉 리포트 연동은 추후 확장)
FAIL_TIMEOUT     = 5   # (미사용)
FAIL_UNREACHABLE = 6   # IK 미해 지속 (확장)
FAIL_PLACE_ERROR = 7   # 배치 오차 허용치 초과 (확장)


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 12_pick_test.py 와 동일
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


def orientation_error_deg(quat_a, quat_b):
    """두 쿼터니언 사이의 회전각 차이(도)"""
    dot = float(np.clip(abs(np.dot(quat_a, quat_b)), -1.0, 1.0))
    return float(np.degrees(2.0 * np.arccos(dot)))


def two_link_load(tcp, base_pos):
    """
    어깨(joint_1)에서 손목중심까지의 거리 / 2링크 한계. 참고용 숫자다 — 제어에는
    안 쓰고 사전 검증 리포트에만 찍는다.

    툴을 수직 하향으로 유지하는 이 작업에서는 도달반경(0.9)보다 이 값이 IK 실패를
    잘 예측한다. 12_pick_test.py 실측 대조: LIFT +0.10 m -> 88.5% 풀림(성공 사례),
    +0.23 m -> 98.3% 안 풀림. FAILED 가 났을 때 "얼마나 모자란지" 를 보는 용도다.
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


def set_magazine_pose(pos, quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """
    매거진의 위치를 USD 로 직접 다시 쓴다.

    12_place_test.py 와 같은 방식이다 — USD 로 prim 의 pose 를 다시 쓰면 물리
    엔진이 그 rigid body pose 를 따라간다. 트라이얼 사이 리셋과, 텔레포트 후
    매거진을 그리퍼 밑으로 옮겨 재부착할 때 쓴다.
    """
    stage = omni.usd.get_context().get_stage()
    xform = UsdGeom.Xformable(stage.GetPrimAtPath(MAGAZINE_XFORM_PATH))
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
        print(f"   USD          loaded  {WORLD_USD}")
        self._wait_for_scene()

    def _wait_for_scene(self):
        """
        참조/페이로드 합성이 실제로 끝날 때까지 기다린다.

        12_ 들은 simulation_app.update() 를 15번(약 0.25 s) 돌리고 넘어간다.
        nova_carter1 과 short_gripper 는 S3 원격 payload 라 Kit 이 비동기로
        받아오는데, 그 안에 안 들어오면 Usd.PrimRange 의 기본 predicate 가
        미로드 서브트리를 건너뛰어 "arm drives 0 / SurfaceGripper node not found"
        로 죽는다. 캐시가 따뜻하면 15번 안에 들어와 되는 것처럼 보일 뿐이다.

        경로 자체가 레이아웃에 없으면 기다려도 소용없으므로 즉시 죽인다. 레이아웃
        계층이 /World/nova_carter1 -> /World/Robots/nova_carter1 로 바뀌었을 때
        "arm drives 0" 만 찍히고 원인이 안 보였던 적이 있다.
        """
        stage = omni.usd.get_context().get_stage()

        roots = (BASE_XFORM_PATH, ROBOT_PRIM_PATH, GRIPPER_PRIM, MAGAZINE_XFORM_PATH)
        bad = [p for p in roots if not stage.GetPrimAtPath(p).IsValid()]
        if bad:
            world = stage.GetPrimAtPath("/World")
            top = [c.GetName() for c in world.GetChildren()] if world.IsValid() else []
            raise RuntimeError(
                f"레이아웃에 없는 prim 경로: {bad}\n"
                f"      /World 바로 아래: {top}\n"
                f"      레이아웃 계층이 바뀌었으면 ROBOT_PRIM_PATH / BASE_XFORM_PATH / "
                f"MAGAZINE_XFORM_PATH 를 12_pick_test.py 와 맞춘다")

        # 스테이지 로드 규칙이 LoadAll 이 아니어도 되게 페이로드를 명시적으로 요청한다
        for path in (BASE_XFORM_PATH, GRIPPER_PRIM, MAGAZINE_XFORM_PATH):
            try:
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid() and not prim.IsLoaded():
                    stage.Load(path)
            except Exception as exc:
                print(f"   !! payload   {path} Load 요청 실패: {exc}")

        t0 = time.time()
        updates = 0
        while True:
            missing = [label for label, root, name in REQUIRED_PRIMS
                       if find_prim_path(root, name) is None]
            if not missing:
                break
            if time.time() - t0 > SCENE_LOAD_TIMEOUT_S:
                lines = []
                for path in roots:
                    prim = stage.GetPrimAtPath(path)
                    state = ("없음" if not prim.IsValid() else
                             f"loaded={prim.IsLoaded()} active={prim.IsActive()} "
                             f"children={len(prim.GetChildren())}")
                    lines.append(f"      {path}: {state}")
                raise RuntimeError(
                    f"씬 로드가 {SCENE_LOAD_TIMEOUT_S:.0f} s 안에 안 끝났다. 아직 없는 것: {missing}\n"
                    + "\n".join(lines) + "\n"
                    "      nova_carter1/short_gripper 는 S3 원격 payload 다 — 네트워크와\n"
                    "      에셋 캐시를 보고, 레이아웃의 file:/home/rokey/... 절대경로가\n"
                    "      이 PC 에 있는지 확인한다")
            simulation_app.update()
            updates += 1
            if updates % 120 == 0:
                print(f"   loading      {time.time() - t0:5.1f} s  아직 없음: {missing}")

        print(f"   scene prims  ready  ({updates} updates, {time.time() - t0:.1f} s)")

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
        if count == 0:
            raise RuntimeError(
                f"{ROBOT_PRIM_PATH} 아래에서 팔 관절 Drive 를 하나도 못 찾았다 — "
                f"씬 합성이 안 끝났거나 경로가 다르다")
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

    주의: 이 호출은 흡착을 즉시 풀어 버린다(12_place_test.py 실측). 그래서 매거진을
    들고 있는 상태에서는 부르지 않는다 — LIFT 가 끝나 매거진이 떨어진 뒤에 부르고,
    도착해서 다시 붙인다(run_trial 의 TRANSPORT 참고).
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
#  Pick 단계 FSM — 12_pick_test.py 와 동일
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
        elif self.state == 4:                         # LIFT 끝 -> 파지 판정
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
#  사전 검증 — 두 정거장에서 실제로 IK 가 풀리는지 먼저 확인
# ══════════════════════════════════════════════════════════════
def reachability_check(world, robot, lula, solver, teleporter, target_quat, place_ref_z):
    """
    트라이얼을 돌리기 전에 각 정거장의 웨이포인트에서 Lula 가 실제로 푸는지 본다.
    FAILED 가 나오면 그 좌표를 조정해야 한다. 2링크 부하는 참고 수치다 —
    100% 를 넘으면 자세와 무관하게 기하학적으로 못 닿는다는 뜻이다.

    한계: compute_inverse_kinematics 는 현재 관절 각도를 초기값으로 쓴다. 여기서는
    준비 자세에서 재기 때문에, 다른 자세에서 출발하면 풀리는 지점이 FAILED 로
    나올 수 있다 — 보수적인 검사다.
    """
    section("REACHABILITY CHECK")
    all_ok = True

    def probe(tag, wp, targets):
        nonlocal all_ok
        pos, yaw = wp
        teleporter.teleport(pos, yaw)
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            world.step(render=not HEADLESS)
        base_pos, _ = sync_ik_base_pose(lula)
        print(f"   [{tag}] 카터 {vec(pos)} yaw {yaw:+.0f}   팔 베이스 {vec(base_pos)}")
        for name, tcp in targets:
            flange_target = tcp_to_flange(tcp, target_quat)
            _, solved = solver.compute_inverse_kinematics(
                target_position=flange_target, target_orientation=target_quat)
            dist, load = two_link_load(tcp, base_pos)
            print(f"      {name:10s} tcp {vec(tcp)}  2링크 {dist:.3f}/{TWO_LINK:.3f}"
                  f" = {load*100:5.1f}%  -> {'SOLVED' if solved else 'FAILED'}")
            all_ok = all_ok and solved

    center_xy, top_z, _ = measure_prim(FLANGE_PATH)
    probe("PICK ", WP_PICK, [
        ("APPROACH", np.array([center_xy[0], center_xy[1], top_z + APPROACH_HEIGHT_OFFSET])),
        ("GRIP",     np.array([center_xy[0], center_xy[1], top_z + GRIP_GAPS[0]])),
        ("LIFT",     np.array([center_xy[0], center_xy[1], top_z + LIFT_HEIGHT_OFFSET])),
    ])
    gx, gy = PLACE_TARGET_XY
    probe("PLACE", WP_PLACE, [
        ("APPROACH", np.array([gx, gy, place_ref_z + APPROACH_HEIGHT_OFFSET])),
        ("LOWER",    np.array([gx, gy, place_ref_z + PLACE_DROP])),
    ])

    if not all_ok:
        print()
        print("   !! 하나 이상 FAILED — 해당 정거장의 좌표를 조정해야 한다")
        print("      부하가 100% 를 넘으면 카터를 목표 쪽으로 더 붙인다")
        print("      (PLACE 는 차체 앞면 x+0.14 가 ConveyorFrame 4.2 를 넘지 않는 선까지)")

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
              place_ref_z, place_target_quat_ref):
    t0 = time.time()

    def fail(code, pick_fsm):
        return TrialResult(
            trial=trial_idx, fail_code=code,
            cycle_time_s=time.time() - t0,
            lift_rise_mm=pick_fsm.lift_rise_m * 1000.0,
            tilt_deg=pick_fsm.tilt_at_lift_deg,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)

    # ── 1) PICK ────────────────────────────────────────────
    pos, yaw = WP_PICK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

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
        return fail(pick_fsm.fail_code, pick_fsm)

    # ── 2) TRANSPORT (텔레포트, 주행 없음) ─────────────────
    # 12_place_test.py 가 실측으로 확인한 것들을 그대로 따른다.
    #   · 베이스에 set_world_pose() 를 부르면 거리와 무관하게 흡착이 즉시 풀린다.
    #     그래서 "붙든 채 옮기기" 를 포기하고, 도착한 뒤 다시 붙인다.
    #   · PICK 은 선반 옆(+y)으로, PLACE 는 컨베이어 앞(+x)으로 뻗어야 해서 잡았던
    #     상대 위치를 그대로 들고 오면 WP_PLACE 에서 그 y 오프셋이 IK 로 안 풀린다.
    section("TRANSPORT")
    pos, yaw = WP_PLACE
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    # 텔레포트 직후 USD 가 물리 결과를 받아쓴 뒤에 읽어야 base_link 실측이 맞다.
    # (reachability_check 과 12_pick_test.py 가 쓰는 순서와 같다)
    base_pos, _ = sync_ik_base_pose(lula)
    print(f"   카터         {vec(np.array(pos))} yaw {yaw:+.0f}   팔 베이스 {vec(base_pos)}")

    # PICK(LIFT) 직후의 관절 각도를 그대로 IK 시드로 쓰면, 약 10 m 떨어진 새 목표
    # 에서는 계속 안 풀리는 나쁜 시드가 되어 팔이 아예 안 움직인다 (12_place_test.py
    # 실측 — 600 스텝 내내 solved=False). 지금은 아무것도 안 들고 있으니 준비자세로
    # 리셋해 깨끗한 시드로 다시 IK 를 건다.
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    # IK 목표를 한 번만 걸고 world.step() 만 반복하면 팔이 다른 자세로 흘러내린다
    # (12_place_test.py 실측). 매 스텝 다시 명령하면서 도착을 기다린다.
    approach_tcp = np.array([PLACE_TARGET_XY[0], PLACE_TARGET_XY[1],
                             place_ref_z + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)

    def hold_ik_target(n_steps):
        solved_any = False
        for _ in range(n_steps):
            action, solved = solver.compute_inverse_kinematics(
                target_position=flange_target, target_orientation=target_quat)
            if solved:
                robot.apply_action(action)
                solved_any = True
            world.step(render=not HEADLESS)
        return solved_any

    solved_any = hold_ik_target(MAX_STEPS)
    arrive_err = float(np.linalg.norm(get_tcp_pose(robot) - approach_tcp))
    print(f"   접근점 도달   목표 {vec(approach_tcp)}  실제 {vec(get_tcp_pose(robot))}"
          f"  오차 {arrive_err*1000:.1f} mm  (IK solved={solved_any})")
    if not solved_any or arrive_err > 0.05:
        print("   !! 접근점에 못 갔다 — WP_PLACE / PLACE_TARGET_XY 를 조정해야 한다")
        return fail(FAIL_UNREACHABLE, pick_fsm)

    # 팔이 접근 위치에 자리잡았으면, 매거진을 그리퍼(흡착면) 바로 밑에 옮겨 붙인다.
    # get_tcp_pose() 가 흡착면 월드 위치이므로, 매거진 flange_plate 윗면이 거기
    # 닿도록 원점(= 바닥면 기준)을 매거진 높이만큼 내려서 잡는다.
    tcp_now = get_tcp_pose(robot)
    new_origin = np.array([tcp_now[0], tcp_now[1], tcp_now[2] - place_ref_z])
    set_magazine_pose(new_origin, tuple(place_target_quat_ref))

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
    print(f"   재흡착       {'붙었다' if reattached else '안 붙었다'}  "
          f"status {gripper.status()}  tcp {vec(get_tcp_pose(robot))}")
    if not reattached:
        return fail(FAIL_SLIP, pick_fsm)

    hold_ik_target(BASE_MOVE_SETTLE_STEPS)
    sync_ik_base_pose(lula)

    # ── 3) PLACE ───────────────────────────────────────────
    section("PLACE")
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

    fail_code = place_fsm.fail_code
    place_pos_err_mm = None
    place_orient_err_deg = None

    if fail_code == FAIL_OK:
        for _ in range(SETTLE_STEPS):
            world.step(render=not HEADLESS)
        final_pos, final_quat = get_world_pose(MAGAZINE_PATH)
        place_pos_err_mm = float(np.linalg.norm(final_pos[:2] - PLACE_TARGET_XY)) * 1000.0
        place_orient_err_deg = orientation_error_deg(final_quat, place_target_quat_ref)
        if (place_pos_err_mm > PLACE_POS_TOL_M * 1000.0
                or place_orient_err_deg > PLACE_ORIENT_TOL_DEG):
            fail_code = FAIL_PLACE_ERROR
        print(f"   배치 오차   pos {place_pos_err_mm:.2f} mm (<= {PLACE_POS_TOL_M*1000:.0f})  "
              f"orient {place_orient_err_deg:.2f} deg (<= {PLACE_ORIENT_TOL_DEG:.0f})")

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
    place_ref_z = FLOOR_Z + magazine_height
    print(f"   magazine h   {magazine_height*1000:.1f} mm  ->  place_ref_z={place_ref_z:.3f} m")

    section("SOLVER")
    base_pos0, base_quat0 = get_world_pose(BASE_LINK_PATH)
    lula, solver = create_ik_solver(robot, base_pos0, base_quat0)
    gripper = SurfaceGripperCtl(task.gripper_node_path)
    teleporter = BaseTeleporter(robot)

    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
    reachable = reachability_check(world, robot, lula, solver, teleporter,
                                   target_quat, place_ref_z)
    if not reachable:
        print("\n   WP_PICK/WP_PLACE 좌표를 먼저 조정하세요")
        simulation_app.close()
        return

    def do_pick_place(trial_idx):
        # 매거진/그리퍼/베이스를 시작 상태로 되돌린 뒤 pick+place 한 판을 실행한다.
        # Stop 으로 물리가 초기화돼도 이 함수가 다시 명시적으로 상태를 맞춘다.
        gripper.open()
        set_magazine_pose(magazine_spawn_pos, tuple(magazine_spawn_quat))
        pos, yaw = WP_PICK
        teleporter.teleport(pos, yaw)
        for _ in range(SETTLE_STEPS):
            world.step(render=not HEADLESS)
        set_ready_pose(robot)
        gripper.reinit()

        section("RUN")
        result = run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter,
                           place_ref_z, magazine_spawn_quat)
        section("RESULT")
        print(f"   trial {trial_idx + 1}  fail_code={result.fail_code}"
              f"  {'성공' if result.success else '실패'}"
              f"  cycle_time={result.cycle_time_s:.2f}s")
        print(f"   파지         rise {result.lift_rise_mm:+.1f} mm  "
              f"tilt {result.tilt_deg:.2f} deg")
        if result.place_pos_error_mm is not None:
            print(f"   배치         pos {result.place_pos_error_mm:.2f} mm  "
                  f"orient {result.place_orient_error_deg:.2f} deg")
        return result

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

    simulation_app.close()


if __name__ == "__main__":
    main()

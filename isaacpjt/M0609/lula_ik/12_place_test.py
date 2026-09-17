"""
단위 테스트 — nova_carter1/m0609 로 magazine_1_orange 를 집어 컨베이어 앞
바닥 스테이징 지점에 내려놓기 (PICK + PLACE)

    isaac_python 12_place_test.py   (기본이 GUI)
    (헤드리스로 돌리려면)  PICK_HEADLESS=1 isaac_python 12_place_test.py

12_pick_test.py 의 PICK 로직(흡착 PickFSM, 재시도 사다리 포함)을 그대로 쓰고,
그 뒤에 PLACE 단계를 추가한다.

PLACE 목표 지점에 대해 — 원래 이 시나리오는 PackagingZone 컨베이어 위
TestItem 의 초기 위치(x=4.5, z=1.05)에 내려놓는 것이었다. 하지만 그 지점은
ConveyorFrame 충돌체(x>=4.2 를 꽉 채움) 안쪽이라, 이 팔(스펙 도달거리 0.9 m)
로는 베이스가 충돌 없이 접근할 수 있는 범위에서 절대 닿지 않는다(접근 방향/
IK 시드를 여러 조합으로 바꿔봐도 동일 — 12_pick_test.py 이전 버전의 조사
내용 참고). 그래서 PLACE 목표를 ConveyorFrame 바로 앞, 바닥(z=0) 스테이징
지점(x=4.0, y=0.0)으로 바꿨다. TestItem 은 이 지점과 겹쳐 보여 헷갈리므로
씬 로드 후 비활성화한다(원래 충돌체가 없는 순수 시각적 prim이라 물리적으로는
문제 없었지만, 시각적으로 방해가 된다).

  - 인식(비전) 없이 USD 에서 매거진의 실제(GT) pose 를 직접 읽는다.
  - Nav2 주행 없이 베이스(nova_carter1)를 WP_PICK -> WP_PLACE 로 직접 순간
    이동시킨다(주행 시뮬레이션 없음). 텔레포트할 때마다 Lula 솔버에 새 base
    pose 를 다시 알려준다.
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

MAGAZINE_XFORM_PATH = "/World/Magazines/shelf_1_magaines/top_magazines/magazine_1_orange"
# payload 로 합성되면 magazine_1_orange.usda 의 defaultPrim("Magazine")이 이
# 경로 자체에 별칭(alias)되므로, 그 자식(flange_plate 등)은 별도의 "Magazine"
# 서브프림이 아니라 MAGAZINE_XFORM_PATH 바로 아래에 붙는다.
MAGAZINE_PATH = MAGAZINE_XFORM_PATH
FLANGE_NAME           = "flange_plate"                      # top-grasp 지점 (파지용 손잡이)
FLANGE_PATH            = f"{MAGAZINE_PATH}/{FLANGE_NAME}"

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
#   magazine_1_orange  (-6.5, 2.0, 0.6)  Shelf_01 위
#   PLACE 목표          (4.0, 0.0, 바닥)  ConveyorFrame(x>=4.2) 바로 앞
#
# [WP_PICK] Shelf_01 은 Level_1/Level_2 두 단 선반판이 x 방향 2.5 m 전체를
# 덮고 있어(world x:[-7.05,-4.55], y:[1.867,2.867]), 선반 옆(x축)으로
# 접근하면 선반 몸체를 관통하는 경로가 된다. 차체는 통로 방향(yaw=0, x축
# 정렬)으로 세워두고 팔만 옆(+y)으로 뻗어 집는다. base_x=-6.50,
# base_y=1.42~1.72 m 구간 전체가 APPROACH/LIFT 둘 다 SOLVED 이면서 선반
# 앞면(y=1.867)과의 간격도 0~18 cm 확보된다. 아래 값(1.45)은 그 구간의
# 여유 있는 지점(간격 약 17.9 cm)이다.
#
# [WP_PLACE] ConveyorFrame 충돌체가 x>=4.2 를 꽉 채우고 있어, PLACE 목표를
# x=4.0(프레임 바로 앞 바닥)으로 잡고 베이스도 그 앞(yaw=0, 차체 정면이
# 컨베이어를 향함)에 세운다. base_x 를 3.56~3.95 m 구간에서 스윕한 결과
# 전 구간이 APPROACH/LOWER 둘 다 SOLVED 이면서 ConveyorFrame 앞면(x=4.2)
# 과의 간격도 13~52 cm 확보된다. 아래 값(3.85)은 그 구간의 중간 지점
# (간격 약 25 cm)이다.

# (베이스 world xyz, yaw_deg) — yaw 는 world +x 축 기준
WP_PICK  = (np.array([-6.5, 1.45, 0.07963398335074101]), 0.0)
WP_PLACE = (np.array([3.35, 0.0, 0.07963398335074101]), 0.0)

# PLACE 목표 xy — ConveyorFrame(x>=4.2) 바로 앞 바닥. z 는 매거진 자체
# 높이만큼 띄운 값(런타임에 measure_prim 으로 재서 계산)을 쓴다.
PLACE_TARGET_XY = np.array([4.0, 0.0])
FLOOR_Z = 0.0


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
FAIL_NOT_FOUND   = 1   # (미사용 — GT pose 라 인식 실패 케이스 없음)
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
              place_ref_z, place_target_quat_ref):
    t0 = time.time()

    # ── 1) PICK ────────────────────────────────────────────
    pos, yaw = WP_PICK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

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

    # ── 2) TRANSPORT (텔레포트, 주행 없음) ─────────────────
    # 베이스 아티큘레이션에 set_world_pose() 를 호출하면(거리와 무관하게) 흡착이
    # 즉시 풀린다 — 실측으로 확인. 게다가 PICK 은 선반 옆(+y)으로 뻗어 잡고
    # PLACE 는 컨베이어 앞(+x)으로 뻗어야 해서, 잡았던 상대 위치를 그대로
    # 들고 오면 WP_PLACE 에서는 그 y 오프셋(약 0.55 m)이 IK 로 안 풀리는
    # 지점이 된다(실측으로 확인). 그래서 "떨어진 물체를 쫓아가 다시 잡기"
    # 대신 — 팔을 먼저 이미 검증된 PLACE 접근 위치로 보내고, 거기서 매거진을
    # 그리퍼 위치에 바로 갖다 놓은 뒤 흡착한다.
    pos, yaw = WP_PLACE
    teleporter.teleport(pos, yaw)
    sync_ik_base_pose(lula)

    # PICK(LIFT) 직후의 관절 각도를 그대로 IK 시드로 쓰면, 그 각도가 이
    # 새 목표(약 10 m 떨어진 곳)에서는 계속 안 풀리는 나쁜 시드가 되어 팔이
    # 아예 안 움직인다(실측으로 확인 — 600 스텝 내내 solved=False). 아직
    # 아무것도 붙들고 있지 않으니 지금 관절을 ready pose 로 리셋해 깨끗한
    # 시드로 다시 IK 를 건다.
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    # 팔이 실제로 approach_tcp 에 도착할 때까지 매 스텝 같은 IK 목표를 계속
    # 재적용하면서 기다린다. 목표를 한 번만 걸고 world.step() 만 반복하면
    # (드라이브가 계속 그 목표를 유지할 거라 기대했지만) 실제로는 팔이 다른
    # 자세로 흘러내리는 걸 실측으로 확인했다 — 그래서 매 스텝 다시 명령한다.
    approach_tcp = np.array([PLACE_TARGET_XY[0], PLACE_TARGET_XY[1],
                              place_ref_z + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)

    def _hold_ik_target(n_steps):
        for _ in range(n_steps):
            action, solved = solver.compute_inverse_kinematics(
                target_position=flange_target, target_orientation=target_quat)
            if solved:
                robot.apply_action(action)
            world.step(render=not HEADLESS)

    _hold_ik_target(MAX_STEPS)

    # 팔이 접근 위치에 자리잡았으면, 매거진을 그리퍼(흡착면) 바로 아래에
    # 옮겨 붙인다. get_tcp_pose() 는 흡착면(suction face) 월드 위치이므로,
    # 매거진의 flange_plate 윗면이 거기 닿도록 원점(=바닥면 기준)을 그만큼
    # 내려서 잡는다 (place_ref_z 는 FLOOR_Z=0 기준 매거진 높이와 같다).
    tcp_now = get_tcp_pose(robot)
    new_origin = np.array([tcp_now[0], tcp_now[1], tcp_now[2] - place_ref_z])
    reset_magazine_pose(new_origin, tuple(place_target_quat_ref))

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
            trial=trial_idx, fail_code=FAIL_SLIP,
            cycle_time_s=time.time() - t0,
            lift_rise_mm=pick_fsm.lift_rise_m * 1000.0,
            tilt_deg=pick_fsm.tilt_at_lift_deg,
            place_pos_error_mm=None, place_orient_error_deg=None,
        )

    _hold_ik_target(BASE_MOVE_SETTLE_STEPS)
    sync_ik_base_pose(lula)
    print(f"   DEBUG after settle  tcp={get_tcp_pose(robot)}  gripped={gripper.gripped()}")

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
    reachable = reachability_check(world, robot, lula, solver, teleporter, target_quat, place_ref_z)
    if not reachable:
        print("\n   WP_PICK/WP_PLACE 좌표를 먼저 조정하세요")
        simulation_app.close()
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

        section("RUN")
        result = run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter,
                            place_ref_z, magazine_spawn_quat)
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

    simulation_app.close()


if __name__ == "__main__":
    main()

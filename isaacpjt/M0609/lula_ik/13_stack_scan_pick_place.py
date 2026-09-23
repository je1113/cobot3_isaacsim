"""
단위 테스트 — PKG-OUT 선반의 스택을 관측(티칭 자세) 후 집어 TEST-01(검사
투입 벨트)에 내려놓기 (SCAN + PICK + PLACE)

    isaac_python 13_stack_scan_pick_place.py   (기본이 GUI)
    (헤드리스로 돌리려면)  PICK_HEADLESS=1 isaac_python 13_stack_scan_pick_place.py

12_place_test.py 를 그대로 뼈대로 쓴다 — 인식(비전) 없이 USD 에서 GT pose 를
직접 읽고, Nav2 없이 베이스를 텔레포트한다. 12_place_test3.py 처럼 손목 카메라
QR PnP 를 쓰지 않는다(사용자 지시 — "12_place_test.py 처럼 만들어줘").

시나리오
────────
  1) 스택이 "컨베이어벨트로 나와서" PKG-OUT 선반 첫 칸에 이미 놓여 있는
     상태에서 시작한다(아래 "스택을 어떻게 놓는가" 절 참고).
  2) 로봇을 PKG-OUT 앞 정차점(shelves.yaml waypoint_start)으로 텔레포트하고,
     같은 파일의 arm_teach_pose(스택 전용 — SHELF-A 관측 자세와 값이 같다)로
     팔을 세워 "스캔"한다. QR 디코드는 하지 않는다 — GT 로 스택이 실제로
     그 자리에 있는지만 확인한다(측정값을 콘솔에 찍는다).
  3) flange_plate(top-grasp 지점, 매거진과 같은 규약)를 GT 로 읽어 PICK 하고,
     TEST-01(TestingZone) 벨트 앞으로 텔레포트해 벨트 위에 PLACE 한다.

★ 스택을 어떻게 놓는가 — 컨베이어를 실제로 돌리지 않는다
──────────────────────────────────────────────────────────
  실제 파이프라인은: 로봇이 PKG-01 벨트에 매거진을 놓음 -> packaging_flow.py
  (씬에 OmniScriptingAPI 로 이미 붙어 있다 - PackagingUnloaderZone 참고)가
  그 매거진을 감지해 스택으로 바꾸고 -> 언로더 벨트를 따라 ~16초에 걸쳐
  슬라이드시켜 -> PKG-OUT 선반 칸에 내려놓는다.

  이 스크립트는 그 이동 과정 자체를 재현하지 않고, packaging_flow.py의
  _spawn_stack()/_move_to_shelf() 가 만드는 "최종 결과"(선반 칸 위의 스택,
  같은 에셋·같은 변환)만 씬 로드 시점에 직접 재현한다(spawn_stack_on_shelf).
  이유:
    · 12_place_test3.py 가 이미 겪은 문제(standalone 스크립트에서 벨트
      물리가 참조-로딩 경로로는 기대대로 안 붙음)와 별개로,
    · 이 테스트가 검증하는 것은 SCAN(티칭 자세)+PICK+PLACE 지 컨베이어
      자체의 물리(이미 씬에 붙어 있고 이 테스트와 무관하게 동작한다)가
      아니다. 최종 상태만 있으면 된다.
    · world.reset() 이전(Task.set_up_scene 안)에 payload 를 붙이므로, 물리
      씬이 처음 빌드될 때부터 PhysicsRigidBodyAPI/CollisionAPI 가 함께
      등록된다 — 12_place_test3.py 모듈 docstring 이 기록한 "런타임에 붙인
      payload 가 등록되기 전에 중력을 받아 바닥까지 관통하는" 문제를 이
      순서 자체로 피한다.

  자산·좌표 출처:
    · payload "../assets/F3_STKB_1.usda" — packaging_flow.py 의
      ORANGE_STACK_PAYLOAD 그대로(이름과 반대로 보이지만, 씬의
      PackagingUnloaderZone.flow:orangeStackPayload 도 같은 값이다 — 에셋
      파일명 접미사가 곧 색을 뜻하지 않는다).
    · 선반 칸 좌표(SHELF_X/SHELF_Z/SHELF_SLOT_Y) — packaging_flow.py 그대로.
    · flange_plate — F3_STKB_1.usda 에 매거진과 같은 이름의 top-grasp
      서브프림이 있다(스택은 매거진을 90도 돌린 것뿐이라는 팀 합의,
      shelves.yaml PKG-OUT 주석 참고). measure_prim() 으로 그대로 잰다.

★ 관측(SCAN) 자세와 정차점 — shelves.yaml PKG-OUT 그대로
──────────────────────────────────────────────────────
  waypoint_start (2.713, 0.696, theta=-90°) 는 선반 첫 칸(SHELF_SLOT_Y[0])을
  보는 정차점이고, arm_teach_pose 는 SHELF-A 관측 자세와 같은 값이다(팀
  지시 — "스택은 매거진을 90도 돌린 것일 뿐이라 로봇과의 상대 위치가 같다").
  이 스크립트는 그 두 값을 그대로 가져다 쓴다 — 별도로 캡처한 적 없다.

★ TEST-01(TestingZone) 벨트 좌표 — 12_place_test3.py 그대로
──────────────────────────────────────────────────────────
  BELT_TOP_Z/BELT_Y_TESTING/BELT_PLACE_X/BELT_BASE_X 는 12_place_test3.py 가
  이미 실측/검증한 값이다(TestingZone 은 PackagingZone 과 벨트 x 범위·폭·
  높이가 동일하다 — stations.yaml TEST-01 주석). stations.yaml 의 TEST-01
  place_pose(Nav2 정차 기준)는 로봇이 Nav2+creep 으로 서는 자리라 이 텔레포트
  테스트의 BELT_BASE_X 와는 다른 좌표계다 — 벨트 위 배치 목표(BELT_PLACE_X,
  BELT_Y_TESTING)는 같다.

중요 — 이 씬에서 확인이 필요한 값들:
  · WP_STACK/WP_PLACE_TEST 가 실제로 IK 로 풀리는지는 REACHABILITY CHECK 가
    실행 시 다시 확인해 준다. FAILED 면 좌표를 조정한다.
  · flange_plate 흡착이 매거진과 같은 파라미터(GRIP_GAPS 등)로 붙는지는
    스택 표면 재질/무게(1.0 kg, 매거진과 같은 급)가 같다는 가정에 기댄다 —
    실행해서 확인해야 한다.
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

# packaging_flow.ORANGE_STACK_PAYLOAD 그대로 쓰되 절대경로로 바꾼다 — 그
# 스크립트는 WORLD_USD 자체를 편집 타깃으로 잡고 돌아서(ctx.open_stage) 상대
# 경로가 isaacpjt/worlds/ 기준으로 풀리지만, 이 스크립트는 World(...)가 만든
# 별도 인메모리 스테이지에 WORLD_USD 를 참조로 얹는 방식(12_place_test.py와
# 동일)이라 우리 스테이지의 편집 타깃(anchor 없는 세션 레이어)에서는 같은
# 상대경로가 전혀 다른 곳(작업 디렉터리 기준)으로 풀린다 — 실측(flange_plate
# 가 "Invalid prim: null prim"으로 안 잡힘, payload 자체가 안 풀린 것).
STACK_PAYLOAD = str(ISAACPJT_DIR / "assets/F3_STKB_1.usda")
SHELF_X = 3.25
SHELF_Z = 0.64
SHELF_SLOT_Y = (2.25, 2.62, 2.99, 3.36)
STACK_SLOT_INDEX = 0   # shelves.yaml PKG-OUT waypoint_start 가 보는 첫 칸


# ══════════════════════════════════════════════════════════════
#  로봇 관절 / 드라이브
# ══════════════════════════════════════════════════════════════
ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# shelves.yaml PKG-OUT(=SHELF-A) arm_teach_pose, rad -> deg. 별도로 캡처한
# 적 없다 — 모듈 docstring "관측(SCAN) 자세와 정차점" 절 참고.
OBSERVE_JOINTS_RAD = [0.235391, 0.670242, 2.617961, 1.579533, 1.570796, 1.500994]
OBSERVE_JOINTS_DEG = list(np.degrees(OBSERVE_JOINTS_RAD))
SCAN_HOLD_STEPS = 60


# ══════════════════════════════════════════════════════════════
#  베이스 텔레포트 waypoint
# ══════════════════════════════════════════════════════════════
CARTER_Z = 0.07963398335074101

# shelves.yaml PKG-OUT waypoint_start 그대로(theta -1.570796 rad = -90°).
WP_STACK = (np.array([2.713, 0.696, CARTER_Z]), -90.0)

# TestingZone(TEST-01) 벨트 앞 — 12_place_test3.py 가 검증한 값 그대로.
BELT_TOP_Z       = 0.5407278772166425
BELT_Y_TESTING   = -2.705314596908152
BELT_PLACE_X     = 4.30
BELT_BASE_X      = 3.85
WP_PLACE_TEST = (np.array([BELT_BASE_X, BELT_Y_TESTING, CARTER_Z]), 0.0)


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정 — 12_place_test.py 재사용(스택도 매거진과 같은 1 kg 급)
# ══════════════════════════════════════════════════════════════
COAXIAL_FORCE_LIMIT = 200.0
SHEAR_FORCE_LIMIT   = 100.0
MAX_GRIP_DISTANCE   = 0.03

SUCTION_FACE_Z = 0.161
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


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 12_place_test.py 와 동일
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
#  USD 조회 유틸 — 12_place_test.py 와 동일(reset_magazine_pose 는
#  임의 prim 에 쓸 수 있게 12_place_test3.py 의 reset_prim_pose 로 가져온다)
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
    """prim 의 월드 pose 를 직접 갈아끼운다 — 트라이얼 사이에 스택을 원래
    선반 자리로 되돌리거나, PLACE 텔레포트 뒤 그리퍼 밑으로 옮길 때 쓴다."""
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
    #   합성되지 않아 measure_prim() 이 "Invalid prim: null prim" 으로 죽는다.
    #   스테이지가 이미 열린 뒤에 붙인 payload 는 자동으로 로드되지 않는다
    #   (magazine_spawner.py 의 _spawn() 도 CopySpec 뒤 반드시 prim.Load() 를
    #   부른다 — 같은 이유).
    prim.Load()
    xform = UsdGeom.XformCommonAPI(prim)
    xform.SetTranslate(Gf.Vec3d(SHELF_X, SHELF_SLOT_Y[slot_index], SHELF_Z))
    xform.SetRotate(Gf.Vec3f(0.0, 0.0, 90.0))
    xform.SetScale(Gf.Vec3f(2.0, 2.0, 2.0))
    _spawn_shelf_support(stage_, slot_index)


def _spawn_shelf_support(stage_, slot_index):
    """이 칸 바로 밑에 얇은 정적 콜라이더를 깐다 — 이 테스트에서만 쓰는
    스캐폴딩이다.

    ★ 실측 — 실제 씬의 OutputShelf/ShelfDeck 콜라이더는 world x 약
      3.876~4.051 에만 있는데(local translate 3.9633, scale 0.175 -> 반너비
      0.0875), packaging_flow.py 가 스택을 놓는 자리(SHELF_X=3.25)는 그
      범위 밖이다 — 0.6 m 넘게 떨어져 있다. 그래서 payload 의
      PhysicsRigidBodyAPI 가 실제로 로드되자(prim.Load() 주석 참고) 받쳐줄
      게 없어 중력으로 바닥(z=0)까지 떨어졌다(실측). 이 씬 파일을 고치는
      대신(그건 이 테스트의 범위를 넘는다 — 모듈 docstring 참고) 우리
      테스트에서만 쓰는 임시 받침을 깐다."""
    path = f"{SPAWNED_STACKS_PATH}/_test_shelf_support"
    cube = UsdGeom.Cube.Define(stage_, path)
    cube.CreateSizeAttr(1.0)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    thickness = 0.02
    xform = UsdGeom.XformCommonAPI(cube.GetPrim())
    xform.SetTranslate(Gf.Vec3d(SHELF_X, SHELF_SLOT_Y[slot_index], SHELF_Z - thickness / 2.0))
    xform.SetScale(Gf.Vec3f(0.6, 0.6, thickness))
    UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()


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


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
class StackScanPickPlaceTask(BaseTask):
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
        self._filter_gripper_target_collision()
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

    def _filter_gripper_target_collision(self):
        filter_collision(GRIPPER_PRIM, STACK_XFORM_PATH)
        print(f"   collision    {GRIPPER_PRIM} <-> {STACK_XFORM_PATH} 필터링")

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
    """관측(SCAN) 자세를 매 스텝 다시 걸어 유지하는 데 쓴다 — 안 걸면
    팔이 흘러내린다(12_place_test3.py carry_pose_action 과 같은 이유)."""
    idx = np.array([robot.get_dof_index(name) for name in ARM_JOINTS])
    target = np.deg2rad(np.array(joints_deg, dtype=float))
    return ArticulationAction(joint_positions=target, joint_indices=idx)


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
#  Pick 단계 FSM — 12_place_test.py 와 동일(FLANGE_PATH/STACK_PATH 만 교체)
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
            np.array([cx, cy, approach_z]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, lift_z]),
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

            if self.state == 2:
                self.z_at_grip = measure_prim(STACK_PATH)[1]
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

        if self.state == 2:
            if not self._check_grip():
                return
        elif self.state == 4:
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
#  Place 단계 FSM — 12_place_test.py 와 동일(바닥 대신 벨트 윗면 기준)
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
            np.array([gx, gy, approach_z]),
            np.array([gx, gy, lower_z]),
            np.array([gx, gy, lower_z]),
            np.array([gx, gy, approach_z]),
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
#  사전 검증 — WP_STACK / WP_PLACE_TEST 에서 실제로 IK 가 풀리는지 확인
# ══════════════════════════════════════════════════════════════
def reachability_check(world, robot, lula, solver, teleporter, target_quat):
    section("REACHABILITY CHECK")
    all_ok = True

    pos, yaw = WP_STACK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    center_xy, top_z, _ = measure_prim(FLANGE_PATH)
    approach_tcp = np.array([center_xy[0], center_xy[1], top_z + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)
    _, solved = solver.compute_inverse_kinematics(
        target_position=flange_target, target_orientation=target_quat)
    print(f"   WP_STACK       approach tcp {vec(approach_tcp)}  -> {'SOLVED' if solved else 'FAILED'}")
    all_ok = all_ok and solved

    pos, yaw = WP_PLACE_TEST
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    _, _, stack_height = measure_prim(STACK_PATH)
    approach_tcp = np.array([BELT_PLACE_X, BELT_Y_TESTING,
                              BELT_TOP_Z + stack_height + APPROACH_HEIGHT_OFFSET])
    flange_target = tcp_to_flange(approach_tcp, target_quat)
    _, solved = solver.compute_inverse_kinematics(
        target_position=flange_target, target_orientation=target_quat)
    print(f"   WP_PLACE_TEST  approach tcp {vec(approach_tcp)}  -> {'SOLVED' if solved else 'FAILED'}")
    all_ok = all_ok and solved

    if not all_ok:
        print("   !! 하나 이상 FAILED — WP_STACK/WP_PLACE_TEST 좌표를 조정해야 한다")

    pos, yaw = WP_STACK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)
    return all_ok


# ══════════════════════════════════════════════════════════════
#  한 트라이얼 실행 — SCAN(관측 자세, GT 확인) -> PICK -> TRANSPORT -> PLACE
# ══════════════════════════════════════════════════════════════
def run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter,
              place_target_quat_ref):
    t0 = time.time()

    # ── 0) SCAN — PKG-OUT 앞에서 티칭 자세로 GT 확인 ────────────
    pos, yaw = WP_STACK
    teleporter.teleport(pos, yaw)
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        world.step(render=not HEADLESS)
    sync_ik_base_pose(lula)

    section("SCAN — 관측(티칭) 자세 유지")
    observe_action = joints_deg_action(robot, OBSERVE_JOINTS_DEG)
    for _ in range(SCAN_HOLD_STEPS):
        robot.apply_action(observe_action)
        world.step(render=not HEADLESS)
    center_xy, top_z, _ = measure_prim(FLANGE_PATH)
    # ★ 스택 전체 높이는 여기, PICK 전에 딱 한 번 잰다 — PICK 뒤에는 흡착이
    #   풀린 스택이 어디에 어떻게 놓여 있을지 몰라(텔레포트로 흡착이 즉시
    #   풀린다 — 12_place_test.py 와 같은 사실) 그때 다시 재면 값이 틀어질
    #   수 있다. TRANSPORT/PLACE 는 전부 이 값을 그대로 재사용한다.
    _, _, stack_height = measure_prim(STACK_PATH)
    print(f"   SCAN  관측 자세 {SCAN_HOLD_STEPS} 스텝 유지 (joints_deg={[round(v,1) for v in OBSERVE_JOINTS_DEG]})")
    print(f"   SCAN  대상 확인  flange center {vec([*center_xy, top_z])}  높이 {stack_height*1000:.1f} mm  "
          f"(GT 확인 — QR 디코드는 하지 않는다)")

    # ── 1) PICK ────────────────────────────────────────────
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

    # 잡은 뒤에는 차체·팔과도 필터링 — 텔레포트 진동에 스쳐 놓치지 않게
    # (12_place_test3.py 의 개선을 그대로 가져온다).
    filter_collision(BASE_XFORM_PATH, STACK_PATH)
    filter_collision(ROBOT_PRIM_PATH, STACK_PATH)

    # ── 2) TRANSPORT (텔레포트, 주행 없음) ─────────────────
    # 12_place_test.py 와 같은 이유: 베이스 아티큘레이션을 텔레포트하면
    # 거리와 무관하게 흡착이 즉시 풀린다. 팔을 먼저 목적지 접근 위치로
    # 보내고, 거기서 스택을 그리퍼 위치로 옮겨 붙인 뒤 재흡착한다.
    pos, yaw = WP_PLACE_TEST
    teleporter.teleport(pos, yaw)
    sync_ik_base_pose(lula)
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    approach_tcp = np.array([BELT_PLACE_X, BELT_Y_TESTING,
                              BELT_TOP_Z + stack_height + APPROACH_HEIGHT_OFFSET])
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
    new_origin = np.array([tcp_now[0], tcp_now[1], tcp_now[2] - stack_height])
    reset_prim_pose(STACK_PATH, new_origin, tuple(place_target_quat_ref))

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

    # ── 3) PLACE — TEST-01(TestingZone) 벨트 위 ─────────────
    place_fsm = PlaceFSM(robot, gripper, (BELT_PLACE_X, BELT_Y_TESTING),
                          BELT_TOP_Z + stack_height)
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
    target_xy = np.array([BELT_PLACE_X, BELT_Y_TESTING])

    if fail_code == FAIL_OK:
        for _ in range(SETTLE_STEPS):
            world.step(render=not HEADLESS)
        final_pos, final_quat = get_world_pose(STACK_PATH)
        place_pos_err_mm = float(np.linalg.norm(final_pos[:2] - target_xy)) * 1000.0
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
    task = StackScanPickPlaceTask(name="stack_scan_pick_place_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    stack_spawn_pos, stack_spawn_quat = get_world_pose(STACK_PATH)
    print(f"   stack pose   {vec(stack_spawn_pos, 4)}  quat {vec(stack_spawn_quat, 4)}")

    section("SOLVER")
    base_pos0, base_quat0 = get_world_pose(BASE_LINK_PATH)
    lula, solver = create_ik_solver(robot, base_pos0, base_quat0)
    gripper = SurfaceGripperCtl(task.gripper_node_path)
    teleporter = BaseTeleporter(robot)

    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
    reachable = reachability_check(world, robot, lula, solver, teleporter, target_quat)
    if not reachable:
        print("\n   WP_STACK/WP_PLACE_TEST 좌표를 먼저 조정하세요")
        simulation_app.close()
        return

    def do_pick_place(trial_idx):
        gripper.open()
        reset_prim_pose(STACK_PATH, stack_spawn_pos, tuple(stack_spawn_quat))
        pos, yaw = WP_STACK
        teleporter.teleport(pos, yaw)
        for _ in range(SETTLE_STEPS):
            world.step(render=not HEADLESS)
        set_ready_pose(robot)
        gripper.reinit()

        section("RUN")
        result = run_trial(trial_idx, world, robot, lula, solver, gripper, teleporter,
                            stack_spawn_quat)
        print(f"   trial {trial_idx + 1} -> fail_code={result.fail_code}  "
              f"cycle_time={result.cycle_time_s:.2f}s")

    do_pick_place(0)

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

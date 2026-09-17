"""
카터 위 팔로 매거진 픽앤플레이스 — 선반 -> (순간이동) -> 컨베이어

    isaac_python test_pnp.py

simple_factory_layout.usda 를 열고, 거기 이미 있는
/World/nova_carter1/m0609 (흡착 그리퍼 + RealSense 장착) 를 그대로 쓴다.
로봇도 그리퍼도 매거진도 새로 만들지 않는다.

시나리오

  1. 카터가 선반 앞으로 순간이동한다 (yaw -90, 팔이 선반 쪽을 향한다)
  2. 선반 2단의 주황 매거진을 흡착으로 집는다
  3. 팔을 접은 채 카터가 컨베이어 앞으로 순간이동한다 (yaw 180)
  4. 벨트 낮은 쪽 끝에 내려놓는다

  APPROACH -> DESCEND -> GRIP -> HOLD -> LIFT -> CARRY
            -> TELEPORT -> APPROACH2 -> LOWER -> RELEASE -> RETREAT

순간이동은 카터 + 팔 + 들고 있는 매거진 셋을 같은 강체 변환으로 함께 옮긴다.
팔은 arm_mount_joint 로 카터 섀시에 물려 있고 매거진은 흡착 조인트로 팔에
물려 있으므로, 셋이 같이 움직여야 조인트가 안 끊긴다. 관절 각도는 건드리지
않고, 직후 속도를 전부 0 으로 눌러 가속 스파이크를 없앤다.

레이아웃에서 읽은 제약
  선반 Level_2 상판 z 0.595,  전면 기둥 y 1.878~1.928  -> 카터는 y < 1.867
  컨베이어 상판 z 0.770,  프레임 x 4.2 부터            -> 카터는 x < 4.200
  사이드레일 y +-0.660 (상단 z 1.080)                  -> 벨트는 옆이 막혀 있어
                                                          x 낮은 쪽 끝에서 넣는다

내일 할 일은 아래 "설정" 블록의 카터 정차 위치만 손보는 것이다.
집는 높이는 매거진 실측 바운딩박스에서 자동으로 나온다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

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
THIS_DIR  = Path(__file__).resolve().parent          # isaacpjt/
M0609_DIR = THIS_DIR / "M0609"

WORLD_USD        = str(THIS_DIR / "worlds/simple_factory_layout.usda")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ══════════════════════════════════════════════════════════════
#
#   설정 — 내일 여기만 바꾼다
#
# ══════════════════════════════════════════════════════════════

# ── 레이아웃 안에서 쓸 prim (전부 이미 존재한다. 만들지 않는다) ──
CARTER_PRIM_PATH = "/World/nova_carter1"
ROBOT_PRIM_PATH  = f"{CARTER_PRIM_PATH}/m0609"
GRIPPER_PRIM     = f"{ROBOT_PRIM_PATH}/short_gripper"
TARGET_PATH      = "/World/magazine_1_orange"     # 선반 2단 주황 매거진

#   ArticulationView 를 걸 경로. None 이면 자동 탐색한다.
#   팔은 arm_mount_joint 로 카터에 물려 있어서 PhysX 가 둘을 하나의
#   아티큘레이션으로 본다. m0609 경로로 걸면
#     "Pattern '/World/nova_carter1/m0609' did not match any articulations"
#   가 뜨고 world.reset() 이 죽는다. 자동 탐색이 틀리면 여기에 직접 적는다.
ARTICULATION_PATH = None

# ── 1) 선반 앞 정차 (픽업) ────────────────────────────────────
#   yaw 180 이면 카터가 선반과 나란히 서고, 팔은 몸통의 짧은 축(+-0.25 m)을
#   가로질러 선반 쪽(world +Y)으로 뻗는다. 카터를 선반에 정면으로 붙이면
#   몸통이 뒤로 0.607 m 나 있어 선반을 파고든다 (아래 CARTER_FOOTPRINT 참고).
PICK_CARTER_POS = np.array([-6.706, 1.467, 0.080])
PICK_CARTER_YAW = 180.0
MAGAZINE_XY     = np.array([-6.500, 2.000])       # 매거진 world xy

# ── 2) 컨베이어 앞 정차 (배치) ────────────────────────────────
#   yaw 90 이면 팔이 몸통 짧은 축을 가로질러 벨트 쪽(world +X)으로 뻗는다.
#   픽업과 같은 로컬 방향(-Y)이라 joint_1 스윙이 양쪽 0 도가 된다.
PLACE_CARTER_POS = np.array([3.800, 0.206, 0.080])
PLACE_CARTER_YAW = 90.0
PLACE_XY         = np.array([4.400, 0.000])       # 벨트 위 놓을 자리
PLACE_SURFACE_Z  = 0.770                          # 벨트 상판 높이

# ── 3) 여유 높이 ──────────────────────────────────────────────
#   M0609 도달반경은 약 0.9 m. 시작할 때 REACH 표로 전 구간 확인한다.
PICK_APPROACH_CLEAR  = 0.20   # 매거진 윗면 위 이만큼에서 접근 대기
PICK_LIFT_CLEAR      = 0.20   # 집고 매거진 윗면 기준 이만큼 들어올린다
PLACE_APPROACH_CLEAR = 0.15   # 벨트 위 이만큼에서 대기
PLACE_DROP           = 0.005  # 놓을 때 이만큼 높게 두어 튀지 않게 한다

#   운반 자세. 팔 베이스 기준 상대 좌표다. 로컬 -Y 로 둔 이유는 두 정거장
#   모두 팔이 그 방향으로 뻗기 때문이다. 같은 방향이라 joint_1 스윙이 0 이
#   된다. 반대쪽에 두면 1 kg 을 흡착으로 물고 180도 휘둘러야 한다.
CARRY_OFFSET = np.array([0.00, -0.30, 0.50])

# ── 4) 카터 몸통과 장애물 (간섭 검사용) ──────────────────────
#   Nav2 설정에서 가져온 실측 footprint. 원점이 중심이 아니다.
#   앞으로 0.14 m, 뒤로 0.607 m — 몸통이 뒤로 길다.
#   src/nova_carter/carter_navigation/params/carter_navigation_params.yaml
#     footprint: "[ [0.14, 0.25], [0.14, -0.25], [-0.607, -0.25], [-0.607, 0.25] ]"
CARTER_FOOTPRINT = np.array([[0.14, 0.25], [0.14, -0.25],
                             [-0.607, -0.25], [-0.607, 0.25]])
CARTER_MARGIN = 0.10   # m, 이보다 가까우면 경고

#   레이아웃에서 읽은 장애물 AABB (x0, x1, y0, y1)
OBSTACLES = {
    "선반 Shelf_01":   (-6.852, -4.352,  1.867,  2.867),
    "컨베이어 프레임":  ( 4.200,  8.600, -0.675,  0.675),
}

# ── 5) 흡착 ───────────────────────────────────────────────────
#   coaxial 500 N 은 실제로 성공을 확인한 값이다.
#   shear 는 확인된 값이 아니고, 기존 2:1 비율을 유지해 250 으로 올렸다.
#   흡착은 걸리는데 옮기다 놓친다면 여기부터 본다.
COAXIAL_FORCE_LIMIT = 500.0   # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 250.0   # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.03    # m, 이 안에 들어오면 붙는다

# ══════════════════════════════════════════════════════════════
#   설정 끝 — 아래는 보통 건드리지 않는다
# ══════════════════════════════════════════════════════════════


GRIPPER_NODE = f"{GRIPPER_PRIM}/SurfaceGripper"
TARGET_NAME  = TARGET_PATH.rsplit("/", 1)[-1]

EE_LINK_NAME = "link_6"
EE_LINK_PATH = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 팔은 카터 섀시에 arm_mount_joint 로 고정돼 있다. 이 오프셋은 USD 에서
# 실측해 채운다 (기본값은 레이아웃에 적힌 값). 순간이동 뒤 IK 베이스를
# 다시 잡을 때 쓴다.
ARM_LOCAL_OFFSET = np.array([-0.20649390288330727, 0.0, 0.5492008321030571])

# link_6 로컬 +Z 방향으로 흡착면까지의 거리
SUCTION_FACE_Z = 0.161
SUCTION_INSET  = 0.0025
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
# ── 프로브 (가드 무브) ───────────────────────────────────────
#   정확한 z 를 몰라도 되게 만든다. 예상 윗면보다 한참 위에서 흡착을 켠 채
#   천천히 내려가면서, 붙는 순간 멈춘다.
#
#   흡착 그리퍼라 힘 센서가 필요 없다. maxGripDistance(30 mm) 안에 물체가
#   들어오면 조인트가 붙고 get_gripped_objects() 가 채워지므로, 그게 곧
#   "닿았다" 신호다. 내려가는 내내 포착 범위가 쓸리며 지나가기 때문에
#   높이를 몰라도 걸린다.
PROBE_START_ABOVE = 0.10    # 예상 윗면보다 이만큼 위에서 시작
PROBE_MAX_BELOW   = 0.03    # 예상 윗면보다 이만큼 아래까지만. 그 아래는 누르는 것
PROBE_SPEED       = 0.0005  # m/step. 60 Hz 기준 약 30 mm/s

#   안 붙었는데 못 따라가면 뭔가에 눌린 것이다. 힘 센서 없이 접촉을 아는 법.
PROBE_CONTACT_ERR = 0.004   # 명령 TCP 와 실제 TCP 가 이만큼 벌어지면 접촉
PROBE_CONTACT_N   = 5       # 연속 이만큼 넘어야 인정 (순간 튐 걸러내기)
PROBE_SETTLE      = 30      # 시작 직후 이 스텝 동안은 추종 오차를 안 본다

HOLD_WAIT       = 120   # 들기 전에 제자리에서 버티는 스텝
RELEASE_WAIT    = 90
TELEPORT_SETTLE = 120   # 순간이동 직후 안정될 때까지
LIFT_OK_MIN     = 0.03  # 물체가 이만큼 올라가면 흡착 성공

TCP_SPEED = 0.002       # 스텝당 TCP 이동 거리(m). 느릴수록 가속 스파이크가 작다
MIN_STEPS = 90
MAX_STEPS = 600

SETTLE_STEPS = 60       # 씬이 안정될 때까지

# 툴(link_6 로컬 +Z)이 바닥을 향하게.
#   yaw 는 정거장의 카터 yaw 를 따라간다 (PnPFSM.tool_quat 참고).
#   월드 기준으로 고정해 두면 베이스가 180/90 도 돌아간 정거장에서 손목이
#   그만큼 비틀린 해를 찾아야 해서, 관절 한계에 걸리거나 기괴한 자세가 나온다.
APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0    # 툴축 추가 회전. 흡착컵은 축대칭이라 보통 0

LOG_INTERVAL = 60


# ══════════════════════════════════════════════════════════════
#  회전 유틸
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
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    return np.concatenate([[np.cos(half)], (a / np.linalg.norm(a)) * np.sin(half)])


def yaw_quat(deg):
    """카터는 바닥을 굴러다니므로 z축 회전만 쓴다"""
    return quat_from_axis([0.0, 0.0, 1.0], deg)


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    """roll, pitch 로 접근 방향을 정하고 yaw 를 마지막에 곱해 툴축 회전만 준다"""
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


def arm_base_of(carter_pos, carter_yaw_deg):
    """카터 정차 위치에서 팔 베이스 world 위치를 구한다"""
    return np.array(carter_pos) + quat_to_matrix(yaw_quat(carter_yaw_deg)) @ ARM_LOCAL_OFFSET


def tcp_to_flange(tcp_pos, quat):
    """흡착면 목표를 플랜지(link_6) 목표로 바꾼다"""
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose(robot):
    """현재 플랜지 pose 로부터 흡착면의 월드 위치를 구한다"""
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  물체 측정
# ══════════════════════════════════════════════════════════════
def measure():
    """
    대상의 현재 월드 바운딩박스를 잰다.

    캐시를 매번 새로 만드는 이유는 물리가 물체를 움직인 뒤의 값이 필요해서다.
    반환값은 (중심 xy, 윗면 z, 높이).
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(TARGET_PATH)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        raise RuntimeError(f"{TARGET_PATH} 의 바운딩박스가 비어 있다")
    lo, hi = rng.GetMin(), rng.GetMax()
    center_xy = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
    return center_xy, float(hi[2]), float(hi[2] - lo[2])


def top_z():
    return measure()[1]


def describe_target():
    """물체의 물리 구성을 찍는다. 흡착이 안 걸릴 때 여기부터 본다"""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(TARGET_PATH)

    bodies, colliders, offsets, mass = [], 0, set(), 0.0
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.append(p.GetPath().pathString)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            colliders += 1
            a = p.GetAttribute("physxCollision:contactOffset")
            offsets.add(a.Get() if a and a.Get() is not None else "기본(0.02)")
        if p.HasAPI(UsdPhysics.MassAPI):
            m = p.GetAttribute("physics:mass").Get()
            if m:
                mass += m

    print(f"   rigid body   {bodies if bodies else '없음 — 흡착해도 안 딸려온다'}")
    print(f"   colliders    {colliders}개   contactOffset {offsets}")
    if mass > 0:
        print(f"   mass         {mass:.3f} kg = {mass*9.81:.2f} N   "
              f"(coaxial {COAXIAL_FORCE_LIMIT:.0f} N = "
              f"{COAXIAL_FORCE_LIMIT/(mass*9.81):.1f}배)")
    return bodies


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 제어
# ══════════════════════════════════════════════════════════════
class SurfaceGripperCtl:
    """서피스 그리퍼는 관절이 없어 열기와 닫기 두 가지뿐이다"""

    def __init__(self, path):
        self._path = path
        self._view = None
        self.reinit(verbose=True)

    def reinit(self, verbose=False):
        """world.reset() 이후에는 핸들이 무효가 될 수 있어 다시 잡는다"""
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
        """흡착"""
        if self._view is not None:
            self._view.apply_gripper_action(np.array([1.0]))
        else:
            self._command("close_gripper")

    def open(self):
        """해제"""
        if self._view is not None:
            self._view.apply_gripper_action(np.array([-1.0]))
        else:
            self._command("open_gripper")

    def gripped(self):
        """붙어 있는 물체 목록. 실패해도 루프가 죽지 않게 한다"""
        try:
            if self._view is not None:
                return self._view.get_gripped_objects()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripped_objects(self._path)
        except Exception:
            return None

    def status(self):
        """열림/닫힘 상태. 명령이 먹었는지 확인용"""
        try:
            if self._view is not None:
                return self._view.get_surface_gripper_status()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripper_status(self._path)
        except Exception:
            return None


def holding(gripped):
    """get_gripped_objects 결과가 실제로 뭔가를 들고 있는지 판단한다"""
    if not gripped:
        return False
    # 배치 API 는 [[...]] 로 감싸서 돌려준다. 안쪽이 비면 아무것도 안 들었다
    flat = []
    for item in gripped:
        flat.extend(item) if isinstance(item, (list, tuple)) else flat.append(item)
    return any(str(x).strip() not in ("", "None") for x in flat)


# ══════════════════════════════════════════════════════════════
#  순간이동시 같이 옮겨야 하는 것들
# ══════════════════════════════════════════════════════════════
class Movable:
    """
    카터(아티큘레이션), 팔(아티큘레이션), 매거진(강체) 을 같은 방식으로 다룬다.

    시뮬이 도는 중에는 USD xform 을 고쳐도 물리 물체가 안 움직인다.
    물리 쪽 API 를 써야 하는데 종류마다 클래스가 달라 여기서 흡수한다.
    """

    def __init__(self, path, kind, obj=None):
        self._path = path
        self._kind = kind          # "articulation" | "rigid"
        self._obj = obj
        self._batch = False

        if obj is not None:
            print(f"   movable      {kind:<13} {path}  (기존 핸들 사용)")
            return

        tries = ([("SingleArticulation", False), ("Articulation", True)]
                 if kind == "articulation" else
                 [("SingleRigidPrim", False), ("RigidPrim", True)])
        import isaacsim.core.prims as prims
        for cls_name, batch in tries:
            cls = getattr(prims, cls_name, None)
            if cls is None:
                continue
            try:
                self._obj = cls(paths=[path]) if batch else cls(prim_path=path)
                self._batch = batch
                print(f"   movable      {kind:<13} {path}  ({cls_name})")
                return
            except Exception:
                continue
        print(f"   !! movable   {path} 핸들을 못 잡았다 — 순간이동 때 안 따라온다")

    @property
    def ok(self):
        return self._obj is not None

    def initialize(self):
        try:
            self._obj.initialize()
        except Exception:
            pass

    def get_world_pose(self):
        if self._batch:
            pos, quat = self._obj.get_world_poses()
            return np.array(pos[0], dtype=float), np.array(quat[0], dtype=float)
        pos, quat = self._obj.get_world_pose()
        return np.array(pos, dtype=float), np.array(quat, dtype=float)

    def set_world_pose(self, pos, quat):
        if self._batch:
            self._obj.set_world_poses(np.array([pos], dtype=float),
                                      np.array([quat], dtype=float))
        else:
            self._obj.set_world_pose(position=np.array(pos, dtype=float),
                                     orientation=np.array(quat, dtype=float))

    def zero_velocity(self):
        """
        순간이동 직후 남은 속도가 조인트 한계를 넘기지 않게 눌러 준다.

        단일 prim 에 배치 API 를 부르면 shape 이 안 맞는다. 예외로 걸러지면
        다행이지만 조용히 먹어 버리면 엉뚱한 값이 들어가므로 갈라서 부른다.
        """
        obj = self._obj
        if self._kind == "articulation":
            # 아티큘레이션에는 관절 속도만 건드린다. 루트 속도 API 는
            # 내부에서 get -> 수정 -> set 을 하는데, 뷰 크기가 어긋나 있으면
            # 그 getter 에서 텐서 크기 에러가 난다
            try:
                obj.set_joint_velocities(np.zeros(obj.num_dof))
            except Exception:
                pass
            return

        if self._batch:
            try:
                obj.set_velocities(np.zeros((1, 6)))
            except Exception:
                pass
        else:
            for fn in ("set_linear_velocity", "set_angular_velocity"):
                try:
                    getattr(obj, fn)(np.zeros(3))
                except Exception:
                    pass


# ══════════════════════════════════════════════════════════════
#  순간이동
# ══════════════════════════════════════════════════════════════
def teleport(movables, lula, from_pos, from_quat, to_pos, to_quat):
    """
    카터 + 팔 + 매거진을 같은 강체 변환으로 함께 옮긴다.

    셋은 조인트로 물려 있다 (카터-팔은 arm_mount_joint, 팔-매거진은 흡착).
    하나만 옮기면 조인트가 거대한 구속 위반을 보고 터지므로 전부 같이 옮긴다.
    관절 각도는 손대지 않아 팔 모양과 흡착 상대 자세가 그대로 유지된다.
    """
    R_from = quat_to_matrix(from_quat)
    R_to   = quat_to_matrix(to_quat)
    dq     = quat_mul(to_quat, quat_conj(from_quat))

    # 먼저 전부 읽는다. 하나씩 읽고 쓰면 앞에서 옮긴 게 뒤 계산에 섞인다
    poses = []
    for mv in movables:
        if not mv.ok:
            poses.append(None)
            continue
        try:
            poses.append(mv.get_world_pose())
        except Exception:
            poses.append(None)

    for mv, pose in zip(movables, poses):
        if pose is None:
            continue
        pos, quat = pose
        rel = R_from.T @ (pos - from_pos)
        try:
            mv.set_world_pose(to_pos + R_to @ rel, quat_mul(dq, quat))
            mv.zero_velocity()
        except Exception as exc:
            print(f"   !! {mv._path} 이동 실패: {exc}")

    # IK 솔버는 팔 베이스 기준이다. 카터가 옮겨졌으니 다시 알려 준다
    lula.set_robot_base_pose(
        robot_position=to_pos + R_to @ ARM_LOCAL_OFFSET,
        robot_orientation=to_quat,
    )

    # 카터가 진짜 갔는지 확인한다.
    #   아티큘레이션의 world pose 는 루트 '링크'(섀시) 기준이라 카터 prim
    #   위치와 다를 수 있다. 그래서 절대 위치가 아니라 '이동량'을 본다.
    if movables and movables[0].ok and poses[0] is not None:
        try:
            before = poses[0][0]
            after, _ = movables[0].get_world_pose()
            want = to_pos + R_to @ (R_from.T @ (before - from_pos))
            err = float(np.linalg.norm(after - want))
            mark = "ok" if err < 0.02 else "!! 안 옮겨졌다"
            print(f"   carter       {vec(before)} -> {vec(after)}   "
                  f"기대 {vec(want)}   오차 {err*1000:.1f} mm  {mark}")
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════
#  궤적 + 상태 기계
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def ease(alpha):
    """
    smoothstep 가감속.

    선형 보간은 구간이 시작하는 한 스텝에서 속도가 0 -> 최고속으로 튄다.
    60 Hz 기준 가속도가 14 m/s2 까지 올라가고, 1 kg 짜리를 물고 있으면
    그 순간 힘이 흡착 한계를 넘어 그리퍼가 놓아 버린다.
    양 끝 속도가 0 이 되게 하면 최대 가속도가 0.6 m/s2 수준으로 떨어진다.
    """
    a = float(np.clip(alpha, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


class PnPFSM:
    """
       0 HOVER      매거진 예상 위치 위 PROBE_START_ABOVE 높이
       1 PROBE      흡착을 켠 채 천천히 하강. 붙는 순간 멈춘다 (가드 무브)
       2 HOLD       붙은 자리에서 잠깐 버틴다. 잡자마자 놓치는지 여기서 걸린다
       3 LIFT       선반에서 수직으로 인출   <- 파지 성공 판정
       4 CARRY      팔을 베이스 쪽으로 접는다 (순간이동 준비)
       5 TELEPORT   카터 + 팔 + 매거진 함께 컨베이어 앞으로
       6 APPROACH2  벨트 위 대기 높이로
       7 LOWER      가드 하강. 벨트에 닿으면 멈춘다
       8 RELEASE    해제
       9 RETREAT    위로 빠지기
      10 DONE

    PROBE 와 LOWER 가 이 설계의 핵심이다. 둘 다 "정확한 z 를 안다"는 전제를
    버리고, 내려가면서 신호가 올 때 멈춘다.
      PROBE  - 흡착이 걸리면(get_gripped_objects) 멈춘다
      LOWER  - 명령 TCP 를 실제가 못 따라오면(눌렸다) 멈춘다
    덕분에 선반 높이나 벨트 높이가 몇 cm 틀려도 동작한다.

    HOLD 부터 LOWER 까지는 매 스텝 붙어 있는지 감시한다.
    """

    NAMES = ["HOVER", "PROBE", "HOLD", "LIFT", "CARRY",
             "TELEPORT", "APPROACH2", "LOWER", "RELEASE", "RETREAT", "DONE"]
    S_HOVER, S_PROBE, S_HOLD, S_LIFT, S_CARRY = 0, 1, 2, 3, 4
    S_TELEPORT, S_APPROACH2, S_LOWER, S_RELEASE, S_RETREAT = 5, 6, 7, 8, 9
    DONE_STATE = 10
    PICK_SIDE_LAST = S_CARRY          # 여기까지가 픽업쪽 베이스
    PROBE_STATES = (S_PROBE, S_LOWER)
    WATCH_STATES = (S_HOLD, S_LIFT, S_CARRY, S_TELEPORT, S_APPROACH2, S_LOWER)
    WATCH_EVERY = 10

    def __init__(self, robot, gripper, lula, movables):
        self._robot = robot
        self._gripper = gripper
        self._lula = lula
        self._movables = movables
        self.reset()

    # ── 초기화 ──────────────────────────────────────────
    def reset(self):
        center_xy, t_z, height = measure()
        self.center_xy = center_xy
        self.obj_top = t_z              # 예상값일 뿐이다. 프로브가 실제를 찾는다
        self.height = height

        self.z_at_grip = None
        self.grip_ok = False
        self.dropped = False
        self.teleported = False
        self.done = False
        self.contact_hits = 0
        self.caught_gap = 0.0           # 실제로 붙은 간격. 놓는 높이에 쓴다
        self.catch_tcp = None

        self.pick_quat  = yaw_quat(PICK_CARTER_YAW)
        self.place_quat = yaw_quat(PLACE_CARTER_YAW)
        self.pick_base  = arm_base_of(PICK_CARTER_POS, PICK_CARTER_YAW)
        self.place_base = arm_base_of(PLACE_CARTER_POS, PLACE_CARTER_YAW)

        cx, cy = center_xy
        px, py = PLACE_XY
        print(f"   target       {TARGET_NAME}  center ({cx:+.3f}, {cy:+.3f})  "
              f"예상 윗면 {t_z:.4f}  height {height*1000:.1f} mm")
        print(f"   pick  carter {vec(PICK_CARTER_POS)}  yaw {PICK_CARTER_YAW:+.1f}"
              f"  -> 팔 베이스 {vec(self.pick_base)}")
        print(f"   place carter {vec(PLACE_CARTER_POS)}  yaw {PLACE_CARTER_YAW:+.1f}"
              f"  -> 팔 베이스 {vec(self.place_base)}")
        print(f"   probe        윗면 +{PROBE_START_ABOVE*1000:.0f} mm 에서 시작해 "
              f"-{PROBE_MAX_BELOW*1000:.0f} mm 까지  "
              f"{PROBE_SPEED*1000:.1f} mm/step")

        self.state = self.S_HOVER
        self.step = 0
        self.start = None
        self.gripper = "open"
        self._rebuild()

    def _rebuild(self):
        """웨이포인트를 다시 만든다. 프로브 단계의 목표는 '허용 최심부'다"""
        cx, cy = self.center_xy
        px, py = PLACE_XY

        hover_z = self.obj_top + PROBE_START_ABOVE
        probe_z = self.obj_top - PROBE_MAX_BELOW      # 여기까지만 내려간다
        lift_z  = self.obj_top + PICK_LIFT_CLEAR

        carry_pick  = self.pick_base  + quat_to_matrix(self.pick_quat)  @ CARRY_OFFSET
        carry_place = self.place_base + quat_to_matrix(self.place_quat) @ CARRY_OFFSET

        # 벨트쪽. 명목 접촉 높이에서 PROBE_MAX_BELOW 만큼 더 내려갈 수 있게 둔다
        touch_z  = PLACE_SURFACE_Z + self.height + self.caught_gap
        self.touch_z = touch_z
        place_hi = touch_z + PLACE_APPROACH_CLEAR
        place_lo = touch_z - PROBE_MAX_BELOW

        hold_wp = self.catch_tcp if self.catch_tcp is not None \
            else np.array([cx, cy, self.obj_top])

        self.waypoints = [
            np.array([cx, cy, hover_z]),     # 0 HOVER
            np.array([cx, cy, probe_z]),     # 1 PROBE     (최심부, 보통 도중에 멈춘다)
            hold_wp,                         # 2 HOLD      (붙은 자리)
            np.array([cx, cy, lift_z]),      # 3 LIFT
            carry_pick,                      # 4 CARRY
            carry_place,                     # 5 TELEPORT
            np.array([px, py, place_hi]),    # 6 APPROACH2
            np.array([px, py, place_lo]),    # 7 LOWER     (최심부, 보통 도중에 멈춘다)
            np.array([px, py, touch_z]),     # 8 RELEASE   (접촉하면 그 자리로 덮어쓴다)
            np.array([px, py, place_hi]),    # 9 RETREAT
        ]

    # ── 조회 ────────────────────────────────────────────
    def base_for(self, state):
        """그 단계에서 팔 베이스가 어디에 있는가 (도달거리 계산용)"""
        return self.pick_base if state <= self.PICK_SIDE_LAST else self.place_base

    def carter_yaw_for(self, state):
        """그 단계에서 카터가 어느 쪽을 보고 있는가"""
        return PICK_CARTER_YAW if state <= self.PICK_SIDE_LAST else PLACE_CARTER_YAW

    def tool_quat(self):
        """
        이번 단계의 툴 목표 자세.

        베이스 yaw 를 빼서 넣는다. roll 180 도가 툴 Z 축을 뒤집기 때문에
        툴 프레임 yaw 는 월드에서 반대로 돈다. 더하면 베이스와 정반대로
        돌아가 손목이 두 배로 비틀린다. (수치로 확인했다)
        """
        return make_target_quat(
            APPROACH_ROLL_DEG, APPROACH_PITCH_DEG,
            GRIPPER_YAW_DEG - self.carter_yaw_for(self.state),
        )

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            return self.waypoints[self.state]
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    # ── 진행 ────────────────────────────────────────────
    def advance(self):
        if self.done:
            return

        if self.start is None:
            self._enter_state()

        self.step += 1
        self._watch_drop()

        # 프로브 단계는 끝까지 가기 전에 멈출 수 있다
        if self.state in self.PROBE_STATES and self._probe_stop():
            return

        if self.step < self.n_steps:
            return

        if self.state == self.S_PROBE:
            self._probe_exhausted()
            if self.done:
                return
        elif self.state == self.S_LIFT:
            self._judge_grasp()
        elif self.state == self.S_TELEPORT:
            self._judge_teleport()
        elif self.state == self.S_LOWER:
            self._lower_exhausted()
        elif self.state == self.S_RELEASE:
            self._judge_place()

        self._goto(self.state + 1)

    def _goto(self, state):
        self.state = state
        self.step = 0
        self.start = None
        self.contact_hits = 0
        if self.state >= self.DONE_STATE:
            self.done = True
            print(f"   [{self.DONE_STATE}] DONE")

    def _enter_state(self):
        self.start = get_tcp_pose(self._robot)
        self.goal = self.waypoints[self.state]

        if self.state == self.S_PROBE:
            # 흡착을 미리 켜 둔다. 내려가다 포착 범위에 들어오면 알아서 붙는다
            self.z_at_grip = top_z()
            self._gripper.close()
            self.gripper = "close"
            dist = float(np.linalg.norm(self.goal - self.start))
            self.n_steps = max(int(dist / PROBE_SPEED), MIN_STEPS)
        elif self.state == self.S_HOLD:
            self.n_steps, dist = HOLD_WAIT, 0.0
        elif self.state == self.S_TELEPORT:
            print()
            print(f"   {'─' * 56}")
            print(f"   순간이동  카터 {vec(PICK_CARTER_POS)} yaw {PICK_CARTER_YAW:+.0f}"
                  f"  ->  {vec(PLACE_CARTER_POS)} yaw {PLACE_CARTER_YAW:+.0f}")
            teleport(self._movables, self._lula,
                     PICK_CARTER_POS, self.pick_quat,
                     PLACE_CARTER_POS, self.place_quat)
            self.teleported = True
            self.start = self.goal
            self.n_steps, dist = TELEPORT_SETTLE, 0.0
        elif self.state == self.S_LOWER:
            dist = float(np.linalg.norm(self.goal - self.start))
            self.n_steps = max(int(dist / PROBE_SPEED), MIN_STEPS)
        elif self.state == self.S_RELEASE:
            self._gripper.open()
            self.gripper = "open"
            self.n_steps, dist = RELEASE_WAIT, 0.0
        else:
            self.n_steps, dist = steps_for(self.start, self.goal)

        print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
              f" goal {vec(self.goal)}  {dist:.4f} m  {self.n_steps} steps"
              f"  gripper {self.gripper}")

    # ── 가드 무브 ───────────────────────────────────────
    def _tracking_error(self):
        """명령한 TCP 를 실제가 얼마나 못 따라오고 있는가"""
        return float(np.linalg.norm(self.current_target() - get_tcp_pose(self._robot)))

    def _pressed(self):
        """추종 오차가 연속으로 임계를 넘으면 뭔가에 눌린 것으로 본다"""
        if self.step <= PROBE_SETTLE:
            return False
        if self._tracking_error() > PROBE_CONTACT_ERR:
            self.contact_hits += 1
        else:
            self.contact_hits = 0
        return self.contact_hits >= PROBE_CONTACT_N

    def _probe_stop(self):
        """멈춰야 하면 처리하고 True. 계속 내려가도 되면 False"""
        tcp = get_tcp_pose(self._robot)

        if self.state == self.S_PROBE:
            if holding(self._gripper.gripped()):
                self.grip_ok = True
                self.catch_tcp = self.current_target().copy()
                actual_top = top_z()
                self.caught_gap = (tcp[2] + SUCTION_INSET) - actual_top
                print()
                print(f"   {'─' * 56}")
                print(f"   붙었다      {self.step}/{self.n_steps} 스텝 "
                      f"({self.step * PROBE_SPEED * 1000:.0f} mm 내려와서)")
                print(f"   흡착점 z    {tcp[2] + SUCTION_INSET:.4f}   "
                      f"물체 윗면 {actual_top:.4f}   간격 {self.caught_gap*1000:+.1f} mm")
                print(f"   예상 대비   {(actual_top - self.obj_top)*1000:+.1f} mm "
                      f"(예상 {self.obj_top:.4f})")
                print(f"   {'─' * 56}")
                print()
                self._rebuild()            # 실제 간격으로 놓는 높이를 다시 잡는다
                self._goto(self.S_HOLD)
                return True

            if self._pressed():
                print()
                print(f"   !! 닿았는데 안 붙었다  tcp {vec(tcp)}  "
                      f"추종오차 {self._tracking_error()*1000:.1f} mm")
                print(f"      흡착면이 플랜지(80x80)를 빗나갔거나 "
                      f"COAXIAL_FORCE_LIMIT({COAXIAL_FORCE_LIMIT:.0f} N)가 모자라다")
                print()
                self.done = True
                return True

        elif self.state == self.S_LOWER:
            if self._pressed():
                self.waypoints[self.S_RELEASE] = self.current_target().copy()
                print(f"   벨트에 닿았다  tcp {vec(tcp)}  "
                      f"{self.step}/{self.n_steps} 스텝  여기서 놓는다")
                self._goto(self.S_RELEASE)
                return True

        return False

    def _probe_exhausted(self):
        """끝까지 내려갔는데 안 붙었다"""
        print()
        print(f"   {'─' * 56}")
        print(f"   흡착 실패 — 예상 윗면 {self.obj_top:.4f} 기준 "
              f"+{PROBE_START_ABOVE*1000:.0f} ~ -{PROBE_MAX_BELOW*1000:.0f} mm "
              f"를 다 훑었다")
        print(f"   높이 문제가 아니다. 다음을 보자")
        print(f"     - rigid body 경로가 위에 찍혔는지 (없으면 물리 물체가 아니다)")
        print(f"     - 흡착면 xy 가 플랜지(80x80 mm) 위에 오는지")
        print(f"     - MAX_GRIP_DISTANCE({MAX_GRIP_DISTANCE*1000:.0f} mm) 를 키워보기")
        print(f"   {'─' * 56}")
        print()
        self.done = True

    def _lower_exhausted(self):
        """끝까지 내려갔는데 접촉을 못 느꼈다. 그래도 놓는 게 낫다"""
        print(f"   !! 접촉 없이 최심부까지 내려왔다  "
              f"PLACE_SURFACE_Z({PLACE_SURFACE_Z:.3f}) 가 실제보다 높을 수 있다")
        print(f"      여기서 놓는다 (낙하 {PROBE_MAX_BELOW*1000:.0f} mm 이내)")

    # ── 드롭 감시 ───────────────────────────────────────
    def _watch_drop(self):
        """잡고 있다가 놓치는 순간을 잡아낸다"""
        if self.state not in self.WATCH_STATES or not self.grip_ok:
            return
        if self.step % self.WATCH_EVERY or self.dropped:
            return
        if holding(self._gripper.gripped()):
            return

        self.dropped = True
        tcp = get_tcp_pose(self._robot)
        print(f"   !! 놓쳤다  단계 {self.NAMES[self.state]}  "
              f"{self.step}/{self.n_steps} 스텝  "
              f"tcp {vec(tcp)}  물체 윗면 {top_z():.4f}")
        print(f"      이 단계의 목표 {vec(self.goal)}  "
              f"status {self._gripper.status()}")

    # ── 판정 ────────────────────────────────────────────
    def _judge_grasp(self):
        """물체가 실제로 딸려 올라왔는지 높이 차로 판정한다"""
        now = top_z()
        rise = now - self.z_at_grip
        ok = rise >= LIFT_OK_MIN

        print()
        print(f"   {'─' * 56}")
        print(f"   파지        {'성공' if ok else '실패'}   "
              f"(간격 {self.caught_gap*1000:+.1f} mm 에서 붙음)")
        print(f"   물체 윗면   {self.z_at_grip:.4f} -> {now:.4f}  ({rise*1000:+.1f} mm)")
        print(f"   붙은 물체   {self._gripper.gripped()}")
        if not ok:
            print(f"   붙긴 했는데 들다가 놓쳤다 -> COAXIAL_FORCE_LIMIT 을 올리거나 "
                  f"TCP_SPEED 를 낮춘다")
        print(f"   {'─' * 56}")
        print()

    def _judge_teleport(self):
        """순간이동을 건너고도 아직 들고 있는지 본다"""
        ok = holding(self._gripper.gripped())
        print(f"   순간이동 후  {'유지' if ok else '놓쳤다'}   "
              f"물체 윗면 {top_z():.4f}   status {self._gripper.status()}")
        if not ok:
            print(f"      흡착 조인트가 순간이동을 못 버텼다.")
            print(f"      TELEPORT_SETTLE({TELEPORT_SETTLE}) 를 늘리거나,")
            print(f"      CARRY_OFFSET 을 베이스에 더 가깝게 당겨 관성을 줄인다.")
        print(f"   {'─' * 56}")
        print()

    def _judge_place(self):
        """벨트 위에 제대로 놓였는지 본다"""
        center_xy, t_z, _ = measure()
        want = PLACE_SURFACE_Z + self.height
        dz = t_z - want
        dxy = float(np.linalg.norm(center_xy - PLACE_XY))

        print()
        print(f"   {'─' * 56}")
        print(f"   배치        중심 ({center_xy[0]:+.3f}, {center_xy[1]:+.3f})  "
              f"목표 ({PLACE_XY[0]:+.3f}, {PLACE_XY[1]:+.3f})  오차 {dxy*1000:.1f} mm")
        print(f"   윗면 z      {t_z:.4f}  기대 {want:.4f}  ({dz*1000:+.1f} mm)")
        if abs(dz) > 0.02:
            print(f"      벨트 상판에 안 앉았다. PLACE_SURFACE_Z 를 확인하자")
        print(f"   {'─' * 56}")
        print()


# ══════════════════════════════════════════════════════════════
#  씬 준비 — 전부 이미 있는 것을 찾아 쓴다
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


def filter_collision(path_a, path_b):
    """
    두 prim 사이의 충돌을 끈다.

    흡착 조인트는 물체를 그리퍼 쪽으로 당기는데, 같은 자리에서 콜라이더는
    물체를 밀어낸다. 둘이 싸우면 그 반력이 흡착 힘으로 읽혀 한계를 넘고
    그리퍼가 놓아 버린다. 1 kg 짜리에서 특히 크게 나타난다.
    """
    stage = omni.usd.get_context().get_stage()
    a = stage.GetPrimAtPath(path_a)
    rel = UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel()
    rel.AddTarget(Sdf.Path(path_b))
    return True


def read_arm_local_offset():
    """
    팔이 카터 섀시에 어디에 얹혀 있는지 USD 에서 실측한다.
    레이아웃을 고쳐 팔 위치가 바뀌어도 코드를 안 고쳐도 되게 한다.
    """
    global ARM_LOCAL_OFFSET
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()
    carter = stage.GetPrimAtPath(CARTER_PRIM_PATH)
    arm = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not (carter.IsValid() and arm.IsValid()):
        return
    try:
        c = cache.GetLocalToWorldTransform(carter)
        a = cache.GetLocalToWorldTransform(arm)
        local = a * c.GetInverse()
        ARM_LOCAL_OFFSET = np.array(local.ExtractTranslation())
        print(f"   arm offset   카터 기준 {vec(ARM_LOCAL_OFFSET)}  (USD 실측)")
    except Exception as exc:
        print(f"   arm offset   실측 실패, 기본값 사용: {exc}")


def disable_nested_articulations():
    """
    다른 아티큘레이션 안에 들어 있는 아티큘레이션 루트를 꺼 준다.

    팔은 arm_mount_joint 로 카터에 물려 PhysX 에서는 카터와 한 몸이 됐는데,
    USD 에는 ArticulationRootAPI 가 그대로 남아 있다. 그러면 Isaac 은
    USD 기준으로 4개(카터 2 + 팔 2)를 세고 PhysX 는 2개만 만들어,
    매 스텝 이 에러가 난다.

        Incompatible size of velocity tensor in function getVelocities:
        expected total size 12, received total size 24 with shape (4, 6)

    12 = 2 x 6 (PhysX 가 만든 아티큘레이션), 24 = 4 x 6 (USD 가 센 개수).
    바깥쪽 루트의 자손인 루트를 꺼서 개수를 맞춘다.
    """
    stage = omni.usd.get_context().get_stage()
    roots = [str(prim.GetPath()) for prim in stage.Traverse()
             if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    nested = [r for r in roots
              if any(r != o and r.startswith(o + "/") for o in roots)]

    if not nested:
        print(f"   articulation 중첩 루트 없음 (전체 {len(roots)}개)")
        return

    try:
        from pxr import PhysxSchema
    except ImportError as exc:
        print(f"   !! PhysxSchema 를 못 불러왔다, 중첩 루트를 못 끈다: {exc}")
        return

    for path in nested:
        prim = stage.GetPrimAtPath(path)
        try:
            api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            api.CreateArticulationEnabledAttr().Set(False)
            print(f"   articulation 중첩 루트 비활성화  {path}")
        except Exception as exc:
            print(f"   !! {path} 비활성화 실패: {exc}")


def find_articulation_root():
    """
    ArticulationView 를 걸 실제 경로를 찾는다.

    팔 단독 경로(/World/nova_carter1/m0609)로는 안 잡힌다. arm_mount_joint
    로 카터 섀시에 물려 있어서 PhysX 가 카터와 팔을 하나의 아티큘레이션으로
    합쳐 버리기 때문이다. 합쳐진 쪽의 루트(=카터)에 걸어야 한다.

    팔 관절은 그 아티큘레이션의 DOF 안에 이름으로 들어 있으므로,
    Lula 솔버는 ArticulationSubset 으로 joint_1~6 만 골라 쓴다.
    """
    global ARTICULATION_PATH
    if ARTICULATION_PATH:
        print(f"   articulation {ARTICULATION_PATH}  (설정값 고정)")
        return ARTICULATION_PATH

    stage = omni.usd.get_context().get_stage()
    roots = [str(prim.GetPath())
             for prim in Usd.PrimRange(stage.GetPrimAtPath(CARTER_PRIM_PATH))
             if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print(f"   articulation ArticulationRootAPI 후보 {roots if roots else '없음'}")

    # 팔 단독 경로는 절대 고르지 않는다. 런타임에 아티큘레이션이 아니라는 게
    # 이미 확인됐다 (여기서 한 번 터졌다). 문자열 길이로 고르면 m0609 쪽이
    # chassis_link 보다 짧아서 하필 그걸 집는다.
    usable = [r for r in roots if r != ROBOT_PRIM_PATH]

    if CARTER_PRIM_PATH in usable or not usable:
        # 정상 경로. 후보가 아예 안 잡히는 경우(페이로드 합성 타이밍)도 여기로 온다
        ARTICULATION_PATH = CARTER_PRIM_PATH
    else:
        # 경로 깊이가 가장 얕은 것이 바깥쪽 루트다 (문자열 길이가 아니다)
        ARTICULATION_PATH = min(usable, key=lambda r: r.count("/"))

    print(f"   articulation {ARTICULATION_PATH}  <- 여기에 건다")
    return ARTICULATION_PATH


class PnPTask(BaseTask):
    """레이아웃을 열고, 이미 있는 카터/팔/그리퍼/매거진을 연결만 한다"""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._require_prims()
        disable_nested_articulations()
        read_arm_local_offset()
        self._park_carter()
        self._setup_gripper_limits()

        filter_collision(GRIPPER_PRIM, TARGET_PATH)
        print(f"   collision    {GRIPPER_PRIM} <-> {TARGET_PATH} 필터링")

        self._setup_arm_drives()
        self._register_robot(scene)
        print("   scene        ready")

    def _require_prims(self):
        """네 개가 다 있어야 한다. 없으면 여기서 바로 알려 준다"""
        stage = omni.usd.get_context().get_stage()
        for label, path in (("carter", CARTER_PRIM_PATH),
                            ("arm", ROBOT_PRIM_PATH),
                            ("gripper", GRIPPER_PRIM),
                            ("target", TARGET_PATH)):
            if not stage.GetPrimAtPath(path).IsValid():
                raise RuntimeError(
                    f"{label} prim 이 없다: {path}\n"
                    f"  {Path(WORLD_USD).name} 안의 경로를 확인하자. "
                    f"레이아웃이 절대경로로 에셋을 참조하고 있으면 "
                    f"이 PC 에서 안 열릴 수 있다")
            print(f"   found        {label:<8} {path}")

    def _park_carter(self):
        """시작 전에 카터를 선반 앞에 세워 둔다 (USD 단계에서 한다)"""
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(CARTER_PRIM_PATH)
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*PICK_CARTER_POS))
        xf.AddRotateZOp().Set(float(PICK_CARTER_YAW))
        simulation_app.update()
        print(f"   carter park  {vec(PICK_CARTER_POS)}  yaw {PICK_CARTER_YAW:+.1f}")

    def _setup_gripper_limits(self):
        """
        그리퍼는 레이아웃에 이미 link_6 에 FixedJoint 로 붙어 있다.
        여기서는 흡착 한계만 넣는다. 시뮬 중에는 안 건드린다.
        """
        stage = omni.usd.get_context().get_stage()
        node = stage.GetPrimAtPath(GRIPPER_NODE)
        if not node.IsValid():
            found = find_prim_path(GRIPPER_PRIM, "SurfaceGripper")
            if found:
                node = stage.GetPrimAtPath(found)
        if not node.IsValid():
            print(f"   !! gripper   SurfaceGripper 노드를 못 찾았다 ({GRIPPER_NODE})")
            return

        for attr, value in (("isaac:coaxialForceLimit", COAXIAL_FORCE_LIMIT),
                            ("isaac:shearForceLimit", SHEAR_FORCE_LIMIT),
                            ("isaac:maxGripDistance", MAX_GRIP_DISTANCE)):
            a = node.GetAttribute(attr)
            if a:
                a.Set(value)
        print(f"   grip limits  coaxial {COAXIAL_FORCE_LIMIT:.0f} N  "
              f"shear {SHEAR_FORCE_LIMIT:.0f} N  "
              f"maxGripDistance {MAX_GRIP_DISTANCE*1000:.0f} mm")

    def _setup_arm_drives(self):
        """IK 결과를 팔이 따라가도록 관절 Drive 를 강화한다"""
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

    def _register_robot(self, scene):
        """서피스 그리퍼는 관절이 없으므로 gripper 인자 없이 등록한다"""
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")
        # prim_path 는 팔이 아니라 합쳐진 아티큘레이션의 루트(카터)다.
        # end_effector 만 팔의 link_6 를 가리킨다
        self._robot = scene.add(
            SingleManipulator(
                prim_path=find_articulation_root(),
                name="carter_m0609",
                end_effector_prim_path=ee_path,
            )
        )
        print(f"   EE frame     {ee_path}")

    @property
    def robot(self):
        return self._robot


def set_ready_pose(robot):
    """
    카터와 팔이 한 아티큘레이션이라 DOF 에 바퀴 관절이 섞여 있다.
    q[:6] 으로 넣으면 앞 6개가 바퀴일 수 있어 엉뚱한 데를 돌린다.
    이름으로 팔 관절만 찾아서 넣는다.
    """
    names = list(robot.dof_names)
    q = robot.get_joint_positions()
    if q is None:
        q = np.zeros(robot.num_dof)
    q = np.array(q, dtype=float)

    missing = []
    for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
        if name in names:
            q[names.index(name)] = np.deg2rad(deg)
        else:
            missing.append(name)
    if missing:
        print(f"   !! dof       팔 관절을 아티큘레이션에서 못 찾았다: {missing}")
        print(f"      dof_names = {names}")
    robot.set_joint_positions(q)


def create_ik_solver(robot):
    """
    lula 객체도 같이 돌려준다.
    순간이동 뒤에 set_robot_base_pose 로 팔 베이스를 다시 알려 줘야 하기 때문이다.
    """
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_robot_base_pose(
        robot_position=arm_base_of(PICK_CARTER_POS, PICK_CARTER_YAW),
        robot_orientation=yaw_quat(PICK_CARTER_YAW),
    )
    print(f"   controlled   {', '.join(lula.get_joint_names())}")
    solver = ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )
    return solver, lula


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_status(robot, solved, fsm, target_tcp):
    if fsm.done:
        return
    name = fsm.NAMES[fsm.state]
    if not solved:
        print(f"   {name:9s} IK FAILED  target {vec(target_tcp)}")
        return
    tcp = get_tcp_pose(robot)
    print(f"   {name:9s} tcp {vec(tcp)}   top {top_z():.4f}   "
          f"gripper {fsm.gripper}")


def carter_box(pos, yaw_deg):
    """카터 몸통을 world AABB 로 만든다 (실측 footprint 기준)"""
    c, s_ = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    rot = np.array([[c, -s_], [s_, c]])
    w = (rot @ CARTER_FOOTPRINT.T).T + np.array(pos[:2])
    return w[:, 0].min(), w[:, 0].max(), w[:, 1].min(), w[:, 1].max()


def check_clearance():
    """
    카터 몸통이 선반/컨베이어를 파고드는지 본다.

    이 검사가 없어서 한 번 틀렸다. footprint 원점이 중심이 아니라
    뒤로 0.607 m 나 있는데 +-0.375 로 어림잡아, 선반을 140 mm,
    컨베이어를 107 mm 파고드는 자리에 세워 뒀었다.
    """
    worst = None
    for tag, pos, yaw in (("픽업", PICK_CARTER_POS, PICK_CARTER_YAW),
                          ("배치", PLACE_CARTER_POS, PLACE_CARTER_YAW)):
        x0, x1, y0, y1 = carter_box(pos, yaw)
        print(f"   {tag}  yaw {yaw:+6.1f}   몸통 x[{x0:+.3f}, {x1:+.3f}]  "
              f"y[{y0:+.3f}, {y1:+.3f}]")

        for name, (ox0, ox1, oy0, oy1) in OBSTACLES.items():
            # 겹치면 음수, 떨어져 있으면 두 축 중 큰 간격
            gx = max(ox0 - x1, x0 - ox1)
            gy = max(oy0 - y1, y0 - oy1)
            gap = max(gx, gy)
            if gap >= CARTER_MARGIN:
                continue
            mark = "!! 충돌" if gap < 0 else "?  여유 부족"
            print(f"        {name:<14} {gap*1000:+7.0f} mm  {mark}")
            if worst is None or gap < worst[1]:
                worst = (f"{tag}/{name}", gap)

    if worst is None:
        print(f"   -> 두 정거장 모두 장애물에서 {CARTER_MARGIN*1000:.0f} mm 이상 여유")
    else:
        print(f"   -> 가장 위험한 곳 {worst[0]}  {worst[1]*1000:+.0f} mm. "
              f"카터를 장애물 반대쪽으로 물리자")


def check_reach(fsm):
    """
    실제로 IK 에 넣을 웨이포인트가 팔이 닿는 범위인지 본다.
    M0609 의 도달반경은 약 0.9 m 다.

    흡착 지점보다 LIFT / 대기 지점이 먼저 반경을 넘는다. 카터 정차 위치를
    잘못 잡으면 IK 가 통째로 실패하는데 그때 원인이 좌표인지 솔버인지
    헷갈리므로, 시작할 때 단계별 거리를 미리 찍어 둔다.
    """
    reach = 0.90
    warn = 0
    for i, wp in enumerate(fsm.waypoints):
        base = fsm.base_for(i)
        d = float(np.linalg.norm(np.array(wp) - base))
        if d >= reach:
            mark, warn = "!! 반경 밖", warn + 1
        elif d >= reach * 0.95:
            mark = "?  아슬아슬"
        else:
            mark = "ok"
        print(f"   [{i:2d}] {fsm.NAMES[i]:9s} {vec(wp)}  팔 베이스에서 {d:.3f} m  {mark}")

    if warn:
        print(f"   -> {warn}개가 반경 밖이다. 카터를 목표 쪽으로 당기거나 "
              f"_CLEAR 값을 줄이자")
    else:
        print(f"   -> 전 구간 반경 {reach:.2f} m 안쪽")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def open_world():
    """
    World 를 만들기 전에 스테이지를 열어야 한다.
    World 가 현재 스테이지를 붙잡은 뒤에 open_stage 로 갈아끼우면
    내부 참조가 전부 무효가 된다.
    """
    omni.usd.get_context().open_stage(WORLD_USD)
    for _ in range(30):
        simulation_app.update()
    print(f"   world        {Path(WORLD_USD).name}")


def main():
    section("SCENE")
    open_world()

    world = World(stage_units_in_meters=1.0)
    task = PnPTask(name="pnp_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()

    names = list(robot.dof_names)
    found = [n for n in ARM_JOINTS if n in names]
    print(f"   dof          {robot.num_dof}개  팔 관절 {len(found)}/6 확인")
    if len(found) < 6:
        print(f"   !! dof_names {names}")
        print(f"      팔 관절이 이 아티큘레이션에 없다. ARTICULATION_PATH 를 "
              f"직접 지정해야 한다")

    set_ready_pose(robot)

    section("MOVABLES")
    # 순간이동 때 같이 옮길 것. 카터가 첫 번째여야 한다 (검증에 쓴다).
    #
    # 팔은 따로 넣지 않는다. 카터와 한 아티큘레이션으로 합쳐져 있어서
    # 카터를 옮기면 팔이 관절 각도를 유지한 채 그대로 따라온다.
    # 따로 옮기려 들면 같은 몸을 두 번 건드려 오히려 터진다.
    movables = [
        Movable(find_articulation_root(), "articulation", obj=robot),
        Movable(TARGET_PATH, "rigid"),
    ]
    for mv in movables:
        mv.initialize()

    # 씬이 안정될 시간을 준다. 실측은 이 뒤에 해야 맞다
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    section("TARGET")
    describe_target()

    section("SOLVER")
    ik_solver, lula = create_ik_solver(robot)
    gripper = SurfaceGripperCtl(GRIPPER_NODE)

    section("PLAN")
    fsm = PnPFSM(robot, gripper, lula, movables)

    section("CLEARANCE")
    check_clearance()

    section("REACH")
    check_reach(fsm)

    section("RUN")
    print("   press Play in the viewport")

    was_playing = False
    step = 0
    ik_bad = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간 처음부터 다시 한다
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            # 카터를 선반 앞으로 되돌린다. 앞 시행에서 옮겨 뒀을 수 있다
            pick_quat = yaw_quat(PICK_CARTER_YAW)
            # 카터를 선반 앞으로 되돌린다. 팔은 같은 아티큘레이션이라
            # 따라온다. world.reset() 이 이미 되돌렸어도 한 번 더 눌러 둔다
            movables[0].initialize()
            try:
                movables[0].set_world_pose(PICK_CARTER_POS, pick_quat)
                movables[0].zero_velocity()
            except Exception:
                pass
            lula.set_robot_base_pose(
                robot_position=arm_base_of(PICK_CARTER_POS, PICK_CARTER_YAW),
                robot_orientation=pick_quat,
            )
            set_ready_pose(robot)
            movables[1].initialize()
            gripper.reinit()
            gripper.open()
            for _ in range(SETTLE_STEPS):
                world.step(render=True)
            fsm.reset()
            step = 0
            ik_bad = 0

        if is_playing:
            target_tcp = fsm.current_target()
            target_quat = fsm.tool_quat()          # 정거장마다 베이스를 따라간다
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)
                ik_bad = 0
            else:
                ik_bad += 1
                # 몇 프레임 튀는 건 흔하다. 계속 실패하면 자세/좌표 문제다
                if ik_bad in (30, 300, 3000):
                    print(f"   !! IK 연속 실패 {ik_bad} 프레임  "
                          f"단계 {fsm.NAMES[fsm.state]}  목표 {vec(target_tcp)}")
                    print(f"      팔 베이스에서 "
                          f"{np.linalg.norm(target_tcp - fsm.base_for(fsm.state)):.3f} m"
                          f"  (반경 약 0.9 m)")

            fsm.advance()

            if step % LOG_INTERVAL == 0:
                print_status(robot, solved, fsm, target_tcp)
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()

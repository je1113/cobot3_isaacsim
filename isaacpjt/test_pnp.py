"""
픽업존 -> (순간이동) -> 테스팅 스테이션 로더  픽앤플레이스

    isaac_python test_pnp.py

10_suction_magazine.py 를 베이스로 하고, 가운데에 순간이동 단계를 넣었다.
AMR 주행이 아직 없으므로 "스토커 앞에 서 있다가 로더 앞으로 이동했다" 를
베이스 순간이동으로 대신한다.

  APPROACH -> DESCEND -> GRIP -> HOLD -> LIFT -> CARRY
            -> TELEPORT -> APPROACH2 -> LOWER -> RELEASE -> RETREAT

순간이동은 로봇 베이스와 들고 있는 매거진을 같은 변환으로 함께 옮긴다.
관절 각도는 건드리지 않으므로 흡착 조인트의 상대 자세가 그대로 보존되고,
직후에 양쪽 속도를 0 으로 눌러 가속 스파이크가 흡착 한계를 넘지 않게 한다.

──────────────────────────────────────────────────────────────
내일 할 일은 아래 "설정" 블록의 좌표만 바꾸는 것이다.
그 밖의 높이는 전부 실측 바운딩박스에서 자동으로 나온다.
──────────────────────────────────────────────────────────────

베이스가 된 10_suction_magazine.py 의 독스트링에는 아직
"매거진은 안 붙었다" 가 남아 있다. 다만 그 뒤 커밋에서
힘한계 60->200 N, TCP 속도 4->2 mm/step, HOLD 단계 추가,
그리퍼<->물체 충돌 필터링이 들어갔으므로 그 값들을 그대로 가져왔다.
흡착이 여전히 안 걸리면 GRIP 재시도 로그부터 본다.
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
from isaacsim.storage.native import get_assets_root_path


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent          # isaacpjt/
M0609_DIR = THIS_DIR / "M0609"
ASSETS_DIR = THIS_DIR / "assets"

ROBOT_USD        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim/m0609_isaac_sim.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

GRIPPER_USD = (get_assets_root_path()
               + "/Isaac/Robots/UniversalRobots/ur10/grippers/short_gripper.usd")

MAGAZINE_1_ORANGE = str(ASSETS_DIR / "magazine_1_orange.usda")  # 250 x 140 x 142, 플랜지 80x80x6
MAGAZINE_2_BLUE   = str(ASSETS_DIR / "magazine_2_blue.usda")    # 300 x 200 x 106, 플랜지 100x60x10
TRAY_1_ORANGE     = str(ASSETS_DIR / "tray_1_orange.usda")      # 6장, 329 x 136 x 78
TRAY_2_BLUE       = str(ASSETS_DIR / "tray_2_blue.usda")        # 8장, 329 x 136 x 97


# ══════════════════════════════════════════════════════════════
#
#   설정 — 내일 여기만 바꾼다
#
# ══════════════════════════════════════════════════════════════

# ── 씬 ────────────────────────────────────────────────────────
# 팩토리 레이아웃 위에서 돌리려면 경로를 넣는다. None 이면 빈 씬을 만든다.
#   예: WORLD_USD = str(THIS_DIR / "worlds/simple_factory_layout.usda")
WORLD_USD = None

# 받침대를 코드로 세운다. 레이아웃에 이미 선반/로더가 있으면 False.
BUILD_STANDS = True

# ── 집을 물체 ─────────────────────────────────────────────────
TARGET_USD  = MAGAZINE_1_ORANGE
TARGET_NAME = "magazine"

# ── 픽업존 ────────────────────────────────────────────────────
PICK_BASE_POS  = np.array([0.00, 0.00, 0.00])   # 로봇 베이스 (world xyz)
PICK_BASE_YAW  = 0.0                            # deg, z축 회전
PICK_SURFACE_Z = 0.20                           # 매거진이 놓인 면의 높이
MAGAZINE_XY    = np.array([0.45, 0.20])         # 매거진 위치 (world xy)

# ── 테스팅 스테이션 로더 ──────────────────────────────────────
PLACE_BASE_POS  = np.array([3.00, 0.00, 0.00])  # 순간이동 후 로봇 베이스
PLACE_BASE_YAW  = 0.0
PLACE_SURFACE_Z = 0.35                          # 로더 상판 높이
PLACE_XY        = np.array([3.45, 0.00])        # 놓을 자리 (world xy)

# ── 여유 높이 (면 높이를 바꾸면 따라온다) ─────────────────────
#   M0609 의 도달반경은 약 0.9 m 다. 이 값을 키우면 LIFT/대기 지점이
#   반경 밖으로 나가 IK 가 통째로 실패한다. 시작할 때 REACH 표로 확인한다.
PICK_APPROACH_CLEAR  = 0.20   # 매거진 윗면 위 이만큼에서 접근 대기
PICK_LIFT_CLEAR      = 0.20   # 집고 매거진 윗면 기준 이만큼 들어올린다
PLACE_APPROACH_CLEAR = 0.15   # 로더 위 이만큼에서 대기
PLACE_DROP           = 0.005  # 놓을 때 이만큼 높게 두어 튀지 않게 한다

# 순간이동 중 팔 자세. 베이스 기준 상대 좌표라 두 스테이션에서 같은 모양이 된다.
CARRY_OFFSET = np.array([0.30, 0.00, 0.50])

# ══════════════════════════════════════════════════════════════
#   설정 끝 — 아래는 보통 건드리지 않는다
# ══════════════════════════════════════════════════════════════


TARGET_PATH = f"/World/{TARGET_NAME}"


# ══════════════════════════════════════════════════════════════
#  로봇 설정
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
EE_LINK_PATH    = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정  (10_suction_magazine.py 최신값 그대로)
# ══════════════════════════════════════════════════════════════
GRIPPER_PRIM = f"{ROBOT_PRIM_PATH}/surface_gripper"
GRIPPER_NODE = f"{GRIPPER_PRIM}/SurfaceGripper"

# short_gripper 는 로컬 +X 로 뻗는다. 툴축은 link_6 로컬 +Z 이므로 Y축 -90도.
MOUNT_QUAT   = Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0))
MOUNT_OFFSET = Gf.Vec3f(0.0, 0.0, 0.0)

COAXIAL_FORCE_LIMIT = 200.0   # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 100.0   # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.03    # m, 이 안에 들어오면 붙는다

SUCTION_FACE_Z = 0.161        # link_6 로컬 +Z 로 흡착면까지
SUCTION_INSET  = 0.0025       # 실제 흡착점은 팁 바깥면보다 이만큼 안쪽
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
# GRIP 재시도 사다리. 흡착면을 물체 윗면보다 이만큼 위에 둔다.
GRIP_GAPS = [0.005, 0.002, 0.000, -0.003]

GRIP_WAIT        = 90    # 흡착 명령 후 붙었는지 보기까지
HOLD_WAIT        = 120   # 들기 전에 제자리에서 버티는 스텝
RELEASE_WAIT     = 90
TELEPORT_SETTLE  = 120   # 순간이동 직후 안정될 때까지
LIFT_OK_MIN      = 0.03  # 물체가 이만큼 올라가면 흡착 성공

TCP_SPEED = 0.002        # 스텝당 TCP 이동 거리(m). 느릴수록 가속 스파이크가 작다
MIN_STEPS = 90
MAX_STEPS = 600

SETTLE_STEPS = 60        # 물체가 면에 앉을 때까지

# 툴(link_6 로컬 +Z)이 바닥을 향하게
APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0

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
    """베이스는 바닥을 굴러다니므로 z축 회전만 쓴다"""
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
#  들고 있는 물체 — 순간이동 때 같이 옮겨야 한다
# ══════════════════════════════════════════════════════════════
class HeldObject:
    """
    매거진 강체를 pose 단위로 다룬다.

    시뮬이 도는 중에는 USD xform 을 고쳐도 강체가 안 움직인다. 물리 쪽 API 로
    써야 하는데 버전마다 단일/배치 클래스가 달라, 되는 것을 찾아 여기서 흡수한다.
    """

    def __init__(self, path):
        self._path = path
        self._prim = None
        self._batch = False

        try:
            from isaacsim.core.prims import SingleRigidPrim
            self._prim = SingleRigidPrim(prim_path=path)
            print("   rigid api    SingleRigidPrim")
            return
        except Exception:
            pass

        try:
            from isaacsim.core.prims import RigidPrim
            self._prim = RigidPrim(paths=[path])
            self._batch = True
            print("   rigid api    RigidPrim (배치)")
            return
        except Exception as exc:
            print(f"   rigid api    사용 불가 — 순간이동 시 매거진이 안 따라온다: {exc}")

    @property
    def ok(self):
        return self._prim is not None

    def initialize(self):
        try:
            self._prim.initialize()
        except Exception:
            pass

    def get_world_pose(self):
        if self._batch:
            pos, quat = self._prim.get_world_poses()
            return np.array(pos[0], dtype=float), np.array(quat[0], dtype=float)
        pos, quat = self._prim.get_world_pose()
        return np.array(pos, dtype=float), np.array(quat, dtype=float)

    def set_world_pose(self, pos, quat):
        if self._batch:
            self._prim.set_world_poses(
                np.array([pos], dtype=float), np.array([quat], dtype=float))
        else:
            self._prim.set_world_pose(
                position=np.array(pos, dtype=float),
                orientation=np.array(quat, dtype=float))

    def zero_velocity(self):
        """순간이동 직후 남은 속도가 흡착 한계를 넘기지 않게 눌러 준다"""
        try:
            if self._batch:
                self._prim.set_velocities(np.zeros((1, 6)))
            else:
                self._prim.set_linear_velocity(np.zeros(3))
                self._prim.set_angular_velocity(np.zeros(3))
        except Exception:
            pass


def zero_robot_velocity(robot):
    try:
        robot.set_joint_velocities(np.zeros(robot.num_dof))
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════
#  순간이동
# ══════════════════════════════════════════════════════════════
def teleport_base(robot, lula, held, from_pos, from_quat, to_pos, to_quat):
    """
    로봇 베이스와 들고 있는 매거진을 같은 강체 변환으로 함께 옮긴다.

    관절 각도는 손대지 않는다. 베이스와 물체가 같이 움직이므로 흡착 조인트가
    보는 상대 자세는 변하지 않고, 그래서 조인트가 끊기지 않는다.
    옮긴 뒤 IK 솔버에도 새 베이스를 알려 줘야 이후 월드 목표가 맞는다.
    """
    obj_pos, obj_quat = held.get_world_pose() if held.ok else (None, None)

    # 베이스
    try:
        robot.set_world_pose(position=to_pos, orientation=to_quat)
    except Exception as exc:
        print(f"   !! 베이스 순간이동 실패: {exc}")
    zero_robot_velocity(robot)

    # 매거진 — 베이스 기준 상대 자세를 그대로 유지한 채 따라 옮긴다
    if held.ok and obj_pos is not None:
        R_from = quat_to_matrix(from_quat)
        R_to   = quat_to_matrix(to_quat)
        rel    = R_from.T @ (obj_pos - from_pos)
        dq     = quat_mul(to_quat, quat_conj(from_quat))

        held.set_world_pose(to_pos + R_to @ rel, quat_mul(dq, obj_quat))
        held.zero_velocity()

    # IK 솔버 재조준
    lula.set_robot_base_pose(robot_position=to_pos, robot_orientation=to_quat)

    # 진짜 옮겨졌는지 확인한다. 고정베이스 아티큘레이션은 안 따라올 수 있다
    try:
        actual, _ = robot.get_world_pose()
        err = float(np.linalg.norm(np.array(actual) - to_pos))
        mark = "ok" if err < 0.01 else "!! 안 옮겨졌다"
        print(f"   base         {vec(to_pos)}  실제 {vec(actual)}  오차 {err*1000:.1f} mm  {mark}")
        if err >= 0.01:
            print("      고정베이스라 set_world_pose 가 안 먹는 경우다.")
            print("      PLACE_BASE_POS 를 PICK_BASE_POS 와 같게 두고")
            print("      PLACE_XY 만 팔이 닿는 범위로 잡아 먼저 확인해 보자.")
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
       0 APPROACH   픽업존 매거진 위로
       1 DESCEND    이번 시도의 흡착 높이까지 하강
       2 GRIP       흡착하고 붙었는지 확인
                      붙었으면 -> HOLD
                      아니면   -> 간격을 낮춰 DESCEND 로 되돌아간다
                      사다리를 다 쓰면 -> 포기하고 DONE
       3 HOLD       제자리에서 잠깐 버틴다. 잡자마자 놓치는지 여기서 걸린다
       4 LIFT       들어올리기          <- 파지 성공 판정
       5 CARRY      베이스 앞 운반 자세로 (순간이동 전에 팔을 접는다)
       6 TELEPORT   베이스 + 매거진 함께 로더 앞으로
       7 APPROACH2  로더 위로
       8 LOWER      놓을 높이까지 하강
       9 RELEASE    해제
      10 RETREAT    위로 빠지기
      11 DONE

    HOLD 부터 LOWER 까지는 매 스텝 붙어 있는지 감시한다.
    놓치는 순간의 단계, 경과 스텝, TCP, 물체 높이를 한 줄로 남긴다.
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "HOLD", "LIFT", "CARRY",
             "TELEPORT", "APPROACH2", "LOWER", "RELEASE", "RETREAT", "DONE"]
    DONE_STATE = 11
    WATCH_STATES = (3, 4, 5, 6, 7, 8)     # HOLD ~ LOWER
    WATCH_EVERY = 10

    def __init__(self, robot, gripper, lula, held):
        self._robot = robot
        self._gripper = gripper
        self._lula = lula
        self._held = held
        self.reset()

    # ── 초기화 ──────────────────────────────────────────
    def reset(self):
        center_xy, t_z, height = measure()
        self.center_xy = center_xy
        self.obj_top = t_z
        self.height = height

        self.attempt = 0
        self.z_at_grip = None
        self.grip_ok = False
        self.dropped = False
        self.teleported = False
        self.done = False

        self.pick_quat = yaw_quat(PICK_BASE_YAW)
        self.place_quat = yaw_quat(PLACE_BASE_YAW)

        # 놓는 높이는 GRIP 이 실제로 성공한 간격을 알고 나서 확정한다.
        # 일단 첫 간격으로 깔아 둔다.
        self.place_gap = GRIP_GAPS[0]

        cx, cy = center_xy
        px, py = PLACE_XY
        print(f"   target       {TARGET_NAME}  center ({cx:+.3f}, {cy:+.3f})  "
              f"top z {t_z:.4f}  height {height*1000:.1f} mm")
        print(f"   pick  base   {vec(PICK_BASE_POS)}  yaw {PICK_BASE_YAW:+.1f} deg  "
              f"surface z {PICK_SURFACE_Z:.3f}")
        print(f"   place base   {vec(PLACE_BASE_POS)}  yaw {PLACE_BASE_YAW:+.1f} deg  "
              f"surface z {PLACE_SURFACE_Z:.3f}")
        print(f"   place xy     ({px:+.3f}, {py:+.3f})")
        print(f"   grip gaps    {[f'{g*1000:+.0f}mm' for g in GRIP_GAPS]}  "
              f"(앞에서 실패하면 다음 값으로 더 내려간다)")

        self.state = 0
        self.step = 0
        self.start = None
        self.gripper = "open"
        self._rebuild()

    def _rebuild(self):
        """이번 시도의 흡착 높이로 웨이포인트를 다시 만든다"""
        cx, cy = self.center_xy
        px, py = PLACE_XY

        grip_z = self.obj_top + GRIP_GAPS[self.attempt]
        self.grip_z = grip_z

        # 픽업쪽 — 매거진 실측 윗면 기준
        approach_z = self.obj_top + PICK_APPROACH_CLEAR
        lift_z     = self.obj_top + PICK_LIFT_CLEAR

        # 운반 자세 — 베이스 기준이라 두 스테이션에서 같은 모양이 된다
        carry_pick  = PICK_BASE_POS  + quat_to_matrix(self.pick_quat)  @ CARRY_OFFSET
        carry_place = PLACE_BASE_POS + quat_to_matrix(self.place_quat) @ CARRY_OFFSET

        # 놓는쪽 — 로더 상판에 매거진 바닥이 닿게
        place_top   = PLACE_SURFACE_Z + self.height
        release_z   = place_top + self.place_gap + PLACE_DROP
        hover_z     = place_top + PLACE_APPROACH_CLEAR
        self.release_z = release_z

        self.waypoints = [
            np.array([cx, cy, approach_z]),        #  0 APPROACH
            np.array([cx, cy, grip_z]),            #  1 DESCEND
            np.array([cx, cy, grip_z]),            #  2 GRIP
            np.array([cx, cy, grip_z]),            #  3 HOLD
            np.array([cx, cy, lift_z]),            #  4 LIFT
            carry_pick,                            #  5 CARRY
            carry_place,                           #  6 TELEPORT (이동 직후 이미 여기)
            np.array([px, py, hover_z]),           #  7 APPROACH2
            np.array([px, py, release_z]),         #  8 LOWER
            np.array([px, py, release_z]),         #  9 RELEASE
            np.array([px, py, hover_z]),           # 10 RETREAT
        ]

    # ── 진행 ────────────────────────────────────────────
    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            return self.waypoints[self.state]
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    def advance(self):
        if self.done:
            return

        if self.start is None:
            self._enter_state()

        self.step += 1
        self._watch_drop()

        if self.step < self.n_steps:
            return

        if self.state == 2:                           # GRIP 끝 -> 붙었나
            if not self._check_grip():
                return                                # 재시도로 되돌아갔다
        elif self.state == 4:                         # LIFT 끝 -> 파지 판정
            self._judge_grasp()
        elif self.state == 6:                         # TELEPORT 끝 -> 아직 들고 있나
            self._judge_teleport()
        elif self.state == 9:                         # RELEASE 끝 -> 배치 판정
            self._judge_place()

        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            print(f"   [{self.DONE_STATE}] DONE")

    def _enter_state(self):
        self.start = get_tcp_pose(self._robot)
        self.goal = self.waypoints[self.state]

        if self.state == 2:                           # GRIP
            self.z_at_grip = top_z()
            self._gripper.close()
            self.gripper = "close"
            self.n_steps, dist = GRIP_WAIT, 0.0
        elif self.state == 3:                         # HOLD
            self.n_steps, dist = HOLD_WAIT, 0.0
        elif self.state == 6:                         # TELEPORT
            print()
            print(f"   {'─' * 56}")
            print(f"   순간이동  {vec(PICK_BASE_POS)} -> {vec(PLACE_BASE_POS)}")
            teleport_base(self._robot, self._lula, self._held,
                          PICK_BASE_POS, self.pick_quat,
                          PLACE_BASE_POS, self.place_quat)
            self.teleported = True
            # 관절이 그대로라 TCP 는 이미 새 CARRY 위치에 있다. 안정만 기다린다
            self.start = self.goal
            self.n_steps, dist = TELEPORT_SETTLE, 0.0
        elif self.state == 9:                         # RELEASE
            self._gripper.open()
            self.gripper = "open"
            self.n_steps, dist = RELEASE_WAIT, 0.0
        else:
            self.n_steps, dist = steps_for(self.start, self.goal)

        print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
              f" goal {vec(self.goal)}  {dist:.4f} m  {self.n_steps} steps"
              f"  gripper {self.gripper}")

    # ── 드롭 감시 ───────────────────────────────────────
    def _watch_drop(self):
        """잡고 있다가 놓치는 순간을 잡아낸다"""
        if self.state not in self.WATCH_STATES or not self.grip_ok:
            return
        if self.step % self.WATCH_EVERY:
            return
        if self.dropped:
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

    # ── 재시도 ──────────────────────────────────────────
    def _check_grip(self):
        """붙었으면 True. 아니면 간격을 낮춰 DESCEND 로 되돌리고 False"""
        gripped = self._gripper.gripped()
        ok = holding(gripped)
        tcp = get_tcp_pose(self._robot)
        suction_z = tcp[2] + SUCTION_INSET      # 툴이 아래를 보므로 흡착점은 위쪽
        gap = suction_z - self.obj_top

        print(f"   ── 시도 {self.attempt + 1}/{len(GRIP_GAPS)}  "
              f"목표간격 {GRIP_GAPS[self.attempt]*1000:+.0f} mm  "
              f"실제 흡착점 {suction_z:.4f} (물체 윗면 {self.obj_top:.4f}, "
              f"거리 {gap*1000:+.1f} mm / 한계 {MAX_GRIP_DISTANCE*1000:.0f} mm)")
        print(f"      status {self._gripper.status()}   gripped {gripped}   "
              f"-> {'붙었다' if ok else '안 붙었다'}")

        if ok:
            self.grip_ok = True
            # 실제로 성공한 간격으로 놓는 높이를 다시 잡는다
            self.place_gap = GRIP_GAPS[self.attempt]
            self._rebuild()
            print(f"      놓는 높이 재계산  release z {self.release_z:.4f} "
                  f"(간격 {self.place_gap*1000:+.0f} mm 반영)")
            return True

        self.attempt += 1
        if self.attempt >= len(GRIP_GAPS):
            print()
            print(f"   {'─' * 56}")
            print(f"   흡착 실패 — 간격 {GRIP_GAPS[0]*1000:+.0f} ~ "
                  f"{GRIP_GAPS[-1]*1000:+.0f} mm 를 다 해봤다")
            print(f"   높이 문제가 아니다. 다음을 보자")
            print(f"     - rigid body 경로가 위에 찍혔는지 (없으면 물리 물체가 아니다)")
            print(f"     - 같은 자리에 큐브를 놓고 되는지 (되면 에셋 문제)")
            print(f"     - MAX_GRIP_DISTANCE({MAX_GRIP_DISTANCE*1000:.0f} mm) 를 더 키워보기")
            print(f"   {'─' * 56}")
            print()
            self.done = True
            return False

        # 더 내려가서 다시
        self._gripper.open()
        self.gripper = "open"
        self._rebuild()
        self.state = 1                                 # DESCEND 로
        self.step = 0
        self.start = None
        print(f"      -> {GRIP_GAPS[self.attempt]*1000:+.0f} mm 로 더 내려가서 재시도")
        return False

    # ── 판정 ────────────────────────────────────────────
    def _judge_grasp(self):
        """물체가 실제로 딸려 올라왔는지 높이 차로 판정한다"""
        now = top_z()
        rise = now - self.z_at_grip
        ok = rise >= LIFT_OK_MIN

        print()
        print(f"   {'─' * 56}")
        print(f"   파지        {'성공' if ok else '실패'}   "
              f"(간격 {GRIP_GAPS[self.attempt]*1000:+.0f} mm 에서 붙음)")
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
        obj_top = top_z()
        print(f"   순간이동 후  {'유지' if ok else '놓쳤다'}   "
              f"물체 윗면 {obj_top:.4f}   status {self._gripper.status()}")
        if not ok:
            print(f"      흡착 조인트가 순간이동을 못 버텼다.")
            print(f"      TELEPORT_SETTLE({TELEPORT_SETTLE}) 를 늘리거나,")
            print(f"      CARRY_OFFSET 을 베이스에 더 가깝게 당겨 관성을 줄인다.")
        print(f"   {'─' * 56}")
        print()

    def _judge_place(self):
        """로더 위에 제대로 놓였는지 본다"""
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
            print(f"      로더 상판에 안 앉았다. PLACE_SURFACE_Z 를 확인하자")
        print(f"   {'─' * 56}")
        print()


# ══════════════════════════════════════════════════════════════
#  씬 구성
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
    잡을 대상과 그리퍼는 어차피 붙어 있어야 하므로 충돌을 볼 필요가 없다.
    """
    stage = omni.usd.get_context().get_stage()
    a = stage.GetPrimAtPath(path_a)
    rel = UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel()
    rel.AddTarget(Sdf.Path(path_b))
    return True


def has_ground_plane():
    """바닥이 이미 있는지 본다. 두 번 깔면 물체가 낀다"""
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if "groundplane" in name or "ground_plane" in name:
            return True
        if prim.IsA(UsdGeom.Plane) and prim.HasAPI(UsdPhysics.CollisionAPI):
            return True
    return False


class PnPTask(BaseTask):
    """로봇, 그리퍼, 매거진, (필요하면) 받침대를 얹는다"""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        if WORLD_USD:
            self._open_world()

        self._load_robot()

        if has_ground_plane():
            print("   ground       already present, skip")
        else:
            scene.add_default_ground_plane()
            print("   ground       added")

        if BUILD_STANDS:
            self._build_stands(scene)

        self._attach_surface_gripper()
        self._load_target()
        filter_collision(GRIPPER_PRIM, TARGET_PATH)
        print(f"   collision    {GRIPPER_PRIM} <-> {TARGET_PATH} 필터링 (접촉 간섭 제거)")
        self._setup_arm_drives()
        self._register_robot(scene)
        print("   scene        ready")

    def _open_world(self):
        """팩토리 레이아웃 위에 얹고 싶을 때"""
        omni.usd.get_context().open_stage(WORLD_USD)
        for _ in range(20):
            simulation_app.update()
        print(f"   world        {Path(WORLD_USD).name}")

    def _load_robot(self):
        """
        m0609_isaac_sim.usd 의 defaultPrim 은 /m0609 다.
        /World/m0609 를 만들어 거기에 참조를 걸면 링크들이 그 아래로 들어온다.
        """
        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath("/World").IsValid():
            UsdGeom.Xform.Define(stage, "/World")
        robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM_PATH)
        robot.GetPrim().GetReferences().AddReference(ROBOT_USD)

        # 픽업존 자리에 세워 둔다. 순간이동은 시뮬 중에 물리 API 로 한다
        xf = UsdGeom.Xformable(robot.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*PICK_BASE_POS))
        xf.AddRotateZOp().Set(float(PICK_BASE_YAW))

        for _ in range(15):
            simulation_app.update()
        print(f"   robot        {ROBOT_PRIM_PATH}  @ {vec(PICK_BASE_POS)} "
              f"yaw {PICK_BASE_YAW:+.1f}")

    def _build_stands(self, scene):
        """
        픽업 선반과 로더 상판을 단순한 고정 박스로 세운다.
        레이아웃 USD 를 쓰면 BUILD_STANDS 를 False 로 두고 이 단계를 건너뛴다.
        """
        from isaacsim.core.api.objects import FixedCuboid

        for tag, xy, top_z_ in (("pick", MAGAZINE_XY, PICK_SURFACE_Z),
                                ("place", PLACE_XY, PLACE_SURFACE_Z)):
            if top_z_ <= 0.001:
                print(f"   stand        {tag} 생략 (surface z = 0)")
                continue
            scene.add(FixedCuboid(
                prim_path=f"/World/stand_{tag}",
                name=f"stand_{tag}",
                position=np.array([xy[0], xy[1], top_z_ / 2.0]),
                scale=np.array([0.60, 0.60, top_z_]),
                color=np.array([0.55, 0.57, 0.60]),
            ))
            print(f"   stand        {tag}  top z {top_z_:.3f}  @ "
                  f"({xy[0]:+.3f}, {xy[1]:+.3f})")

    def _attach_surface_gripper(self):
        """흡착 그리퍼를 참조로 올리고 link_6 에 FixedJoint 로 묶는다"""
        stage = omni.usd.get_context().get_stage()

        grip = UsdGeom.Xform.Define(stage, GRIPPER_PRIM)
        grip.GetPrim().GetReferences().AddReference(GRIPPER_USD)
        simulation_app.update()

        # 물리가 스냅하기 전에 시각 위치를 맞춰 둔다 (첫 프레임 튐 방지)
        cache = UsdGeom.XformCache()
        link6_world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(EE_LINK_PATH))
        local = Gf.Matrix4d().SetRotate(Gf.Quatd(MOUNT_QUAT))
        local.SetTranslateOnly(Gf.Vec3d(MOUNT_OFFSET))

        xf = UsdGeom.Xformable(grip.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local * link6_world)

        # localRot0 에 마운트 회전을 넣으면
        # link6_frame * MOUNT_QUAT == gripper_frame 이 강제된다
        joint = UsdPhysics.FixedJoint.Define(
            stage, f"{EE_LINK_PATH}/surface_gripper_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(EE_LINK_PATH)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(GRIPPER_PRIM)])
        joint.CreateLocalPos0Attr().Set(MOUNT_OFFSET)
        joint.CreateLocalRot0Attr().Set(MOUNT_QUAT)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))

        # 파지 한계는 여기서 한 번만 넣는다 (시뮬 중에는 안 건드린다)
        node = stage.GetPrimAtPath(GRIPPER_NODE)
        node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
        node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
        node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)

        print(f"   gripper      {GRIPPER_PRIM} -> {EE_LINK_PATH}")
        print(f"   grip limits  coaxial {COAXIAL_FORCE_LIMIT:.0f} N  "
              f"shear {SHEAR_FORCE_LIMIT:.0f} N  "
              f"maxGripDistance {MAX_GRIP_DISTANCE*1000:.0f} mm")

    def _load_target(self):
        """매거진 USD 를 참조로 올린다. 물리는 에셋에 이미 들어 있다"""
        stage = omni.usd.get_context().get_stage()
        xform = UsdGeom.Xform.Define(stage, TARGET_PATH)
        xform.GetPrim().GetReferences().AddReference(TARGET_USD)

        # 면보다 살짝 위에 띄워 두고 SETTLE_STEPS 동안 앉힌다
        spawn = Gf.Vec3d(float(MAGAZINE_XY[0]), float(MAGAZINE_XY[1]),
                         float(PICK_SURFACE_Z) + 0.005)
        xf = UsdGeom.Xformable(xform.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(spawn)
        simulation_app.update()
        print(f"   target       {TARGET_PATH}  <- {Path(TARGET_USD).name}  "
              f"spawn {tuple(round(v, 3) for v in spawn)}")

    def _setup_arm_drives(self):
        """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
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
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=ee_path,
            )
        )
        print(f"   EE frame     {ee_path}")

    @property
    def robot(self):
        return self._robot


def set_ready_pose(robot):
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


def create_ik_solver(robot):
    """
    lula 객체도 같이 돌려준다.
    순간이동 뒤에 set_robot_base_pose 로 베이스를 다시 알려 줘야 하기 때문이다.
    """
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_robot_base_pose(
        robot_position=PICK_BASE_POS,
        robot_orientation=yaw_quat(PICK_BASE_YAW),
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


def check_reach(fsm):
    """
    실제로 IK 에 넣을 웨이포인트가 팔이 닿는 범위인지 본다.
    M0609 의 도달반경은 약 0.9 m 다.

    흡착 지점보다 LIFT / 대기 지점이 먼저 반경을 넘는다. 좌표를 잘못 넣으면
    IK 가 통째로 실패하는데 그때 원인이 좌표인지 솔버인지 헷갈리므로,
    시작할 때 단계별 거리를 미리 찍어 둔다.
    """
    reach = 0.90
    warn = 0
    for i, wp in enumerate(fsm.waypoints):
        # 0~5 는 픽업존 베이스, 6~10 은 순간이동 후 로더 베이스 기준이다
        base = PICK_BASE_POS if i <= 5 else PLACE_BASE_POS
        d = float(np.linalg.norm(np.array(wp) - base))
        if d >= reach:
            mark, warn = "!! 반경 밖", warn + 1
        elif d >= reach * 0.95:
            mark = "?  아슬아슬"
        else:
            mark = "ok"
        print(f"   [{i:2d}] {fsm.NAMES[i]:9s} {vec(wp)}  베이스에서 {d:.3f} m  {mark}")

    if warn:
        print(f"   -> {warn}개가 반경 밖이다. _CLEAR 값을 줄이거나 "
              f"베이스를 목표 쪽으로 당기자")
    else:
        print(f"   -> 전 구간 반경 {reach:.2f} m 안쪽")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = PnPTask(name="pnp_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)

    held = HeldObject(TARGET_PATH)
    held.initialize()

    # 물체가 면에 앉을 시간을 준다. 실측은 이 뒤에 해야 맞다
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    section("TARGET")
    describe_target()

    section("SOLVER")
    ik_solver, lula = create_ik_solver(robot)
    gripper = SurfaceGripperCtl(GRIPPER_NODE)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )

    section("PLAN")
    fsm = PnPFSM(robot, gripper, lula, held)

    section("REACH")
    check_reach(fsm)

    section("RUN")
    print("   press Play in the viewport")

    was_playing = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간 처음부터 다시 한다
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            # 베이스를 픽업존으로 되돌린다. 앞 시행에서 순간이동해 뒀을 수 있다
            try:
                robot.set_world_pose(position=PICK_BASE_POS,
                                     orientation=yaw_quat(PICK_BASE_YAW))
            except Exception:
                pass
            lula.set_robot_base_pose(robot_position=PICK_BASE_POS,
                                     robot_orientation=yaw_quat(PICK_BASE_YAW))
            set_ready_pose(robot)
            held.initialize()
            gripper.reinit()
            gripper.open()
            for _ in range(SETTLE_STEPS):
                world.step(render=True)
            fsm.reset()
            step = 0

        if is_playing:
            target_tcp = fsm.current_target()
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)

            fsm.advance()

            if step % LOG_INTERVAL == 0:
                print_status(robot, solved, fsm, target_tcp)
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()

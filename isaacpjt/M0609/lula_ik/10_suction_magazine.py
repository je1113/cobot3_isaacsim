"""
흡착 확인 — 매거진 하나를 집고 놓기 (+ 실패 시 더 내려가서 재시도)

    isaac_python 10_suction_magazine.py

로봇만 있는 USD(m0609_isaac_sim.usd)에서 시작해 씬을 직접 만든다.

  1. 순수 M0609 를 /World/m0609 로 올린다 (그리퍼도 바닥도 없는 파일)
  2. 바닥 평면을 깐다 (이미 있으면 건너뛴다)
  3. 흡착 그리퍼를 link_6 에 FixedJoint 로 붙인다
  4. 우리가 만든 매거진 USD 를 올린다
  5. 집고 놓는다.  APPROACH -> DESCEND -> GRIP -> LIFT -> MOVE -> LOWER
                   -> RELEASE -> RETREAT

핵심은 GRIP 재시도다.
  흡착이 안 걸리면 흡착면을 GRIP_GAPS 의 다음 값까지 더 내려서 다시 시도한다.
  기본은 5 mm -> 2 mm -> 0 mm -> -3 mm(살짝 누름) 네 번이다.
  어느 간격에서 붙었는지 콘솔에 남으므로, 다음부터는 그 값을 첫 시도로 쓰면 된다.

큐브(25 mm, 20 g)는 간격 5 mm 에서 잘 붙는데 매거진은 안 붙었다.
둘의 차이를 하나씩 없애면서 확인하는 중이라 진단 출력을 많이 넣어 두었다.
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
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

ROBOT_USD        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim/m0609_isaac_sim.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

GRIPPER_USD = (get_assets_root_path()
               + "/Isaac/Robots/UniversalRobots/ur10/grippers/short_gripper.usd")

# 우리가 만든 캐리어 에셋 (isaacpjt/assets). 루트가 곧 RigidBody, 질량 1.0 kg
ASSETS_DIR     = M0609_DIR.parent / "assets"
MAGAZINE_USD   = str(ASSETS_DIR / "magazine_small.usda")    # 250 x 140 x 142 mm
TRAY_STACK_USD = str(ASSETS_DIR / "tray_stack_6.usda")      # 329 x 136 x  78 mm

# 집을 물체. 트레이 스택으로 바꿔 보려면 이 줄만 바꾼다
TARGET_USD   = MAGAZINE_USD
TARGET_NAME  = "magazine"
TARGET_PATH  = f"/World/{TARGET_NAME}"
TARGET_SPAWN = Gf.Vec3d(0.45,  0.30, 0.005)
TARGET_PLACE = np.array([0.45, -0.30])


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

ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정
# ══════════════════════════════════════════════════════════════
GRIPPER_PRIM = f"{ROBOT_PRIM_PATH}/surface_gripper"
GRIPPER_NODE = f"{GRIPPER_PRIM}/SurfaceGripper"

# short_gripper 는 로컬 +X 로 뻗는다. 툴축은 link_6 로컬 +Z 이므로 Y축 -90도.
MOUNT_QUAT   = Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0))
MOUNT_OFFSET = Gf.Vec3f(0.0, 0.0, 0.0)

# 파지 한계는 시작할 때 한 번만 넣는다.
#   시뮬 중에 이 값을 바꾸면 그리퍼 내부 상태가 어떻게 되는지 확실하지 않아,
#   변수를 줄이려고 물체별 변경을 없앴다.
#   매거진 1.0 kg = 9.81 N 이라 60 N 이면 6배 여유다.
COAXIAL_FORCE_LIMIT = 60.0    # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 30.0    # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.03    # m, 이 안에 들어오면 붙는다 (기본 0.02 에서 올림)

# link_6 로컬 +Z 방향으로 흡착면(gripper_tip 바깥면)까지의 거리
SUCTION_FACE_Z = 0.161
# 실제 흡착점(suction_cup/Suction_Joint)은 팁 바깥면보다 이만큼 안쪽에 있다.
#   에셋 실측: 조인트 원점 로컬 x = 158.5 mm, 팁 바깥면 = 161 mm
SUCTION_INSET = 0.0025
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
# GRIP 재시도 사다리. 흡착면을 물체 윗면보다 이만큼 위에 둔다.
#   음수면 물체를 살짝 누른다. 앞에서 실패하면 다음 값으로 더 내려간다.
GRIP_GAPS = [0.005, 0.002, 0.000, -0.003]

PLACE_DROP      = 0.005    # 놓을 때 이만큼 높게 두어 물체가 튀지 않게 한다
APPROACH_HEIGHT = 0.25     # 접근 대기 높이 (흡착면 기준)
LIFT_HEIGHT     = 0.23     # 들고 이동할 높이

GRIP_WAIT    = 90          # 흡착 명령 후 붙었는지 보기까지 기다리는 스텝
RELEASE_WAIT = 90
LIFT_OK_MIN  = 0.03        # 물체가 이만큼 올라가면 흡착 성공

TCP_SPEED = 0.004          # 스텝당 TCP 이동 거리(m)
MIN_STEPS = 60
MAX_STEPS = 600

SETTLE_STEPS = 60          # 물체가 바닥에 앉을 때까지 기다리는 스텝

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


def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    return np.concatenate([[np.cos(half)], (a / np.linalg.norm(a)) * np.sin(half)])


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
    print(f"   mass         {mass:.3f} kg = {mass*9.81:.2f} N   "
          f"(coaxial {COAXIAL_FORCE_LIMIT:.0f} N = {COAXIAL_FORCE_LIMIT/(mass*9.81):.1f}배)")
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


class PickFSM:
    """
      0 APPROACH   물체 위로 접근
      1 DESCEND    이번 시도의 흡착 높이까지 하강
      2 GRIP       흡착하고 붙었는지 확인
                     붙었으면 -> LIFT
                     아니면   -> 간격을 낮춰 DESCEND 로 되돌아간다
                     사다리를 다 쓰면 -> 포기하고 DONE
      3 LIFT       들어올리기          ← 여기서 최종 판정
      4 MOVE       놓을 곳 위로 이동
      5 LOWER      놓을 높이까지 하강
      6 RELEASE    해제
      7 RETREAT    위로 빠지기
      8 DONE
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "RETREAT", "DONE"]
    DONE_STATE = 8

    def __init__(self, robot, gripper):
        self._robot = robot
        self._gripper = gripper
        self.reset()

    def reset(self):
        center_xy, t_z, height = measure()
        self.center_xy = center_xy
        self.obj_top = t_z
        self.height = height
        self.attempt = 0
        self.z_at_grip = None
        self.grip_ok = False
        self.done = False

        cx, cy = center_xy
        gx, gy = TARGET_PLACE
        self.place_z = height + GRIP_GAPS[0] + PLACE_DROP

        print(f"   target       {TARGET_NAME}  center ({cx:+.3f}, {cy:+.3f})  "
              f"top z {t_z:.4f}  height {height*1000:.1f} mm")
        print(f"   place        ({gx:+.3f}, {gy:+.3f})  release z {self.place_z:.4f}")
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
        gx, gy = TARGET_PLACE
        grip_z = self.obj_top + GRIP_GAPS[self.attempt]
        self.grip_z = grip_z
        self.waypoints = [
            np.array([cx, cy, APPROACH_HEIGHT]),   # 0 APPROACH
            np.array([cx, cy, grip_z]),            # 1 DESCEND
            np.array([cx, cy, grip_z]),            # 2 GRIP
            np.array([cx, cy, LIFT_HEIGHT]),       # 3 LIFT
            np.array([gx, gy, LIFT_HEIGHT]),       # 4 MOVE
            np.array([gx, gy, self.place_z]),      # 5 LOWER
            np.array([gx, gy, self.place_z]),      # 6 RELEASE
            np.array([gx, gy, APPROACH_HEIGHT]),   # 7 RETREAT
        ]

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
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]

            if self.state == 2:                       # GRIP
                self.z_at_grip = top_z()
                self._gripper.close()
                self.gripper = "close"
                self.n_steps, dist = GRIP_WAIT, 0.0
            elif self.state == 6:                     # RELEASE
                self._gripper.open()
                self.gripper = "open"
                self.n_steps, dist = RELEASE_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}  {dist:.4f} m  {self.n_steps} steps"
                  f"  gripper {self.gripper}")

        self.step += 1
        if self.step < self.n_steps:
            return

        if self.state == 2:                           # GRIP 끝 → 붙었나
            if not self._check_grip():
                return                                # 재시도로 되돌아갔다
        elif self.state == 3:                         # LIFT 끝 → 최종 판정
            self._judge()

        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            print(f"   [{self.DONE_STATE}] DONE")

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

    def _judge(self):
        """물체가 실제로 딸려 올라왔는지 높이 차로 판정한다"""
        now = top_z()
        rise = now - self.z_at_grip
        ok = rise >= LIFT_OK_MIN
        gripped = self._gripper.gripped()

        print()
        print(f"   {'─' * 56}")
        print(f"   결과        {'성공' if ok else '실패'}   "
              f"(간격 {GRIP_GAPS[self.attempt]*1000:+.0f} mm 에서 붙음)")
        print(f"   물체 윗면   {self.z_at_grip:.4f} -> {now:.4f}  ({rise*1000:+.1f} mm)")
        print(f"   붙은 물체   {gripped}")
        if not ok:
            print(f"   붙긴 했는데 들다가 놓쳤다 -> COAXIAL_FORCE_LIMIT 을 올리거나 "
                  f"TCP_SPEED 를 낮춘다")
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


class MagazineTask(BaseTask):
    """로봇만 있는 USD 에서 시작해 바닥, 그리퍼, 매거진을 얹는다"""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_robot()

        if has_ground_plane():
            print("   ground       already present, skip")
        else:
            scene.add_default_ground_plane()
            print("   ground       added")

        self._attach_surface_gripper()
        self._load_target()
        self._setup_arm_drives()
        self._register_robot(scene)
        print("   scene        ready")

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
        for _ in range(15):
            simulation_app.update()
        print(f"   robot        {ROBOT_PRIM_PATH}")

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
        xf = UsdGeom.Xformable(xform.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(TARGET_SPAWN)
        simulation_app.update()
        print(f"   target       {TARGET_PATH}  <- {Path(TARGET_USD).name}  "
              f"spawn {tuple(round(v, 3) for v in TARGET_SPAWN)}")

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
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )
    lula.set_robot_base_pose(
        robot_position=ROBOT_BASE_POS,
        robot_orientation=ROBOT_BASE_QUAT,
    )
    print(f"   controlled   {', '.join(lula.get_joint_names())}")
    return ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )


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


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = MagazineTask(name="magazine_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)

    # 물체가 바닥에 앉을 시간을 준다. 실측은 이 뒤에 해야 맞다
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    section("TARGET")
    describe_target()

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    gripper = SurfaceGripperCtl(GRIPPER_NODE)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )

    section("PLAN")
    fsm = PickFSM(robot, gripper)

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
            set_ready_pose(robot)
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

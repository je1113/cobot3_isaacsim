"""
흡착 확인 — 물체를 차례로 집고 놓기

    isaac_python 10_suction_blue_cube.py

로봇만 있는 USD(m0609_isaac_sim.usd)에서 시작해 씬을 직접 만든다.

  1. 순수 M0609 를 /World/m0609 로 올린다 (그리퍼도 바닥도 없는 파일)
  2. 바닥 평면을 깐다 (이미 있으면 건너뛴다)
  3. 흡착 그리퍼를 link_6 에 FixedJoint 로 붙인다
  4. 대상 물체를 만든다. 두 종류를 섞어 쓸 수 있다
       - 코드로 만드는 큐브   ("scale" 키)
       - 만들어 둔 USD 에셋   ("usd" 키, 예: 매거진 / 트레이 스택)
  5. 하나씩 차례로 집고 놓고, 매번 흡착에 성공했는지 높이로 판정한다

     물체마다:  APPROACH -> DESCEND -> GRIP -> LIFT -> MOVE
                -> LOWER -> RELEASE -> RETREAT
     끝나면 결과를 한 번에 요약한다

물체 크기는 코드에 박지 않고 물리가 안정된 뒤 월드 바운딩박스를 실측한다.
잡는 높이와 놓는 높이가 거기서 자동으로 나오므로, 큐브든 매거진이든 같은 코드로 다룬다.
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

# 로봇만 들어 있는 USD. 그리퍼도 바닥도 블록도 없다
ROBOT_USD        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim/m0609_isaac_sim.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

GRIPPER_USD = (get_assets_root_path()
               + "/Isaac/Robots/UniversalRobots/ur10/grippers/short_gripper.usd")

# 우리가 만든 캐리어 에셋 (isaacpjt/assets)
#   둘 다 원점이 바닥면 중심이고 RigidBody + 질량 1.0 kg 이 들어 있다.
#   TARGETS 의 "usd" 키에 넣으면 큐브 대신 이 에셋을 집는다.
ASSETS_DIR     = M0609_DIR.parent / "assets"
MAGAZINE_USD   = str(ASSETS_DIR / "magazine_small.usda")    # 250 x 140 x 142 mm
TRAY_STACK_USD = str(ASSETS_DIR / "tray_stack_6.usda")      # 329 x 136 x  78 mm


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
#  대상 물체
# ══════════════════════════════════════════════════════════════
# 항목 하나가 물체 하나다. 순서대로 집고 놓는다.
#   name   프림 이름
#   spawn  만들 때 넣을 위치
#            큐브   -> 중심 좌표
#            USD    -> 에셋 원점 (우리 캐리어는 바닥면 중심이라 z=0 이면 바닥에 딱 붙는다)
#   place  놓을 곳 (xy). 높이는 실측한 물체 높이로 자동 계산한다
#   usd    있으면 이 USD 를 참조한다. 없으면 scale/color 로 큐브를 만든다
CUBE_SCALE = Gf.Vec3f(0.025, 0.025, 0.025)
CUBE_MASS  = 0.02                                # kg

TARGETS = [
    {"name": "blue_cube",
     "spawn": Gf.Vec3d(0.35,  0.10,  0.05),
     "place": np.array([0.50,  0.15]),
     "color": Gf.Vec3f(0.15, 0.35, 0.85)},
    {"name": "green_cube",
     "spawn": Gf.Vec3d(0.30,  0.30,  0.05),
     "place": np.array([0.50,  0.00]),
     "color": Gf.Vec3f(0.20, 0.65, 0.30)},
    {"name": "orange_cube",
     "spawn": Gf.Vec3d(0.30, -0.30, -0.05),      # 바닥 아래 → 아래에서 올려 준다
     "place": np.array([0.50, -0.15]),
     "color": Gf.Vec3f(0.90, 0.50, 0.15)},

    # 매거진 — 상부 플랜지 판(80 x 80 mm)을 흡착한다.
    #   1.0 kg = 9.81 N 이라 기본 한계(20/10 N)로는 여유가 없다.
    #   지름 80 mm 흡착판이면 실제로 수백 N 이 나오므로 60/30 N 은 보수적인 값이다.
    {"name": "magazine",
     "usd":     MAGAZINE_USD,
     "spawn":   Gf.Vec3d(0.45,  0.30,  0.005),
     "place":   np.array([0.45, -0.30]),
     "coaxial": 60.0,
     "shear":   30.0},

    # 트레이 스택 — 캐리어 덮개판 위 플랜지를 흡착한다.
    #   329 mm 로 길어서 놓을 자리를 넉넉히 잡아야 한다. 쓰려면 주석을 푼다.
    # {"name": "tray_stack",
    #  "usd":     TRAY_STACK_USD,
    #  "spawn":   Gf.Vec3d(0.30,  0.42,  0.005),
    #  "place":   np.array([0.30, -0.42]),
    #  "coaxial": 60.0,
    #  "shear":   30.0},
]

CUBE_ROOT = "/World/targets"

# 스폰 높이 하한. 바닥(z=0) 아래에 만들면 물리가 큐브를 튕겨 올린다.
SPAWN_CLEARANCE = 0.002

# 대상 콜라이더의 contactOffset 하한.
#   흡착 그리퍼는 "접촉"이 생긴 물체를 잡는다. 흡착면이 물체에서
#   GRIP_GAP + 2.5 mm(흡착점이 팁보다 안쪽) 만큼 떨어져 있으므로,
#   contactOffset 이 그보다 작으면 접촉 자체가 안 생겨 아무것도 못 잡는다.
#   PhysX 기본값이 0.02 라 보통은 문제없지만, 에셋이 낮춰 놨을 수 있어 확인한다.
MIN_CONTACT_OFFSET = 0.02


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정
# ══════════════════════════════════════════════════════════════
GRIPPER_PRIM = f"{ROBOT_PRIM_PATH}/surface_gripper"
GRIPPER_NODE = f"{GRIPPER_PRIM}/SurfaceGripper"

# short_gripper 는 로컬 +X 로 뻗는다. 툴축은 link_6 로컬 +Z 이므로 Y축 -90도.
MOUNT_QUAT   = Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0))
MOUNT_OFFSET = Gf.Vec3f(0.0, 0.0, 0.0)

# 기본 파지 한계. 물체마다 TARGETS 에서 "coaxial" / "shear" 로 덮어쓸 수 있다.
#   물체 무게의 몇 배로 잡을지가 기준이다. 가감속을 넣어도 정지/출발 때
#   정적 무게에 여유가 없으면 놓아 버린다.
COAXIAL_FORCE_LIMIT = 20.0    # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 10.0    # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.02    # m, 이 안에 들어오면 붙는다

# link_6 로컬 +Z 방향으로 흡착면까지의 거리 (gripper_tip 바깥면)
SUCTION_FACE_Z = 0.161
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
GRIP_GAP        = 0.005    # 흡착면을 물체 윗면보다 이만큼 위에 세운다
PLACE_DROP      = 0.005    # 놓을 때 이만큼 높게 두어 물체가 튀지 않게 한다
APPROACH_HEIGHT = 0.25     # 접근 대기 높이 (흡착면 기준)
LIFT_HEIGHT     = 0.23     # 들고 이동할 높이

GRIP_WAIT    = 120         # 흡착 명령 후 기다리는 스텝
RELEASE_WAIT = 90          # 해제 명령 후 기다리는 스텝
LIFT_OK_MIN  = 0.03        # 물체가 이만큼 올라가면 흡착 성공

TCP_SPEED = 0.004          # 스텝당 TCP 이동 거리(m)
MIN_STEPS = 60
MAX_STEPS = 600

SETTLE_STEPS = 60          # 큐브가 바닥에 앉을 때까지 기다리는 스텝

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
def cube_path(name):
    return f"{CUBE_ROOT}/{name}"


def measure(name):
    """
    물체의 현재 월드 바운딩박스를 잰다.

    캐시를 매번 새로 만드는 이유는 물리가 물체를 움직인 뒤의 값이 필요해서다.
    반환값은 (중심 xy, 윗면 z, 높이).
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(cube_path(name))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        raise RuntimeError(f"{cube_path(name)} 의 바운딩박스가 비어 있다")
    lo, hi = rng.GetMin(), rng.GetMax()
    center_xy = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
    return center_xy, float(hi[2]), float(hi[2] - lo[2])


def top_z(name):
    """물체 윗면 높이만 빠르게"""
    return measure(name)[1]


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

    def set_limits(self, coaxial, shear):
        """물체마다 파지 한계를 바꾼다. USD 속성이라 런타임에 써도 먹는다"""
        stage = omni.usd.get_context().get_stage()
        node = stage.GetPrimAtPath(self._path)
        node.GetAttribute("isaac:coaxialForceLimit").Set(float(coaxial))
        node.GetAttribute("isaac:shearForceLimit").Set(float(shear))

    def gripped(self):
        """붙어 있는 물체 목록. 실패해도 루프가 죽지 않게 한다"""
        try:
            if self._view is not None:
                return self._view.get_gripped_objects()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripped_objects(self._path)
        except Exception:
            return None


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


class SequenceFSM:
    """
    물체 하나당 여덟 단계를 돌고, 끝나면 다음 물체로 넘어간다.

      0 APPROACH   물체 위로 접근
      1 DESCEND    흡착 거리까지 하강
      2 GRIP       흡착 (제자리)
      3 LIFT       들어올리기          ← 여기서 흡착 성공 판정
      4 MOVE       놓을 곳 위로 이동
      5 LOWER      놓을 높이까지 하강
      6 RELEASE    해제 (제자리)
      7 RETREAT    위로 빠지기

    웨이포인트는 물체를 잡기 직전에 실측해서 만든다.
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "RETREAT"]
    N_STATES = 8

    def __init__(self, robot, gripper, targets):
        self._robot = robot
        self._gripper = gripper
        self._targets = targets
        self.reset()

    # ── 진행 ────────────────────────────────────────────
    def reset(self):
        self.index = 0          # 몇 번째 물체인지
        self.state = 0          # 그 물체의 몇 번째 단계인지
        self.step = 0
        self.start = None
        self.gripper = "open"
        self.results = []
        self.waypoints = []
        self.z_at_grip = None
        self.done = False
        self._begin_target()

    @property
    def target(self):
        return self._targets[self.index]

    def _begin_target(self):
        """이번 물체를 실측해서 웨이포인트 여덟 개를 만든다"""
        if self.index >= len(self._targets):
            self.done = True
            self._summary()
            return

        name = self.target["name"]
        center_xy, t_z, height = measure(name)
        cx, cy = center_xy
        gx, gy = self.target["place"]

        grip_z  = t_z + GRIP_GAP
        place_z = height + GRIP_GAP + PLACE_DROP     # 바닥에 앉은 높이 + 여유

        self.waypoints = [
            np.array([cx, cy, APPROACH_HEIGHT]),   # 0 APPROACH
            np.array([cx, cy, grip_z]),            # 1 DESCEND
            np.array([cx, cy, grip_z]),            # 2 GRIP
            np.array([cx, cy, LIFT_HEIGHT]),       # 3 LIFT
            np.array([gx, gy, LIFT_HEIGHT]),       # 4 MOVE
            np.array([gx, gy, place_z]),           # 5 LOWER
            np.array([gx, gy, place_z]),           # 6 RELEASE
            np.array([gx, gy, APPROACH_HEIGHT]),   # 7 RETREAT
        ]

        # 물체마다 파지 한계를 바꾼다. 무거운 것은 기본값으로 못 든다
        coaxial = self.target.get("coaxial", COAXIAL_FORCE_LIMIT)
        shear = self.target.get("shear", SHEAR_FORCE_LIMIT)
        self._gripper.set_limits(coaxial, shear)

        stage = omni.usd.get_context().get_stage()
        mass = total_mass(stage.GetPrimAtPath(cube_path(name)))
        weight = mass * 9.81 if mass else None

        print()
        print(f"   ═══ [{self.index + 1}/{len(self._targets)}] {name} ═══")
        print(f"   pick         ({cx:+.3f}, {cy:+.3f})  top z {t_z:.4f}  "
              f"height {height*1000:.1f} mm")
        if weight is not None:
            print(f"   mass         {mass:.3f} kg = {weight:.2f} N   "
                  f"한계 coaxial {coaxial:.0f} N ({coaxial/weight:.1f}배)  "
                  f"shear {shear:.0f} N")
        print(f"   place        ({gx:+.3f}, {gy:+.3f})  release z {place_z:.4f}")
        print(f"   grip z       {grip_z:.4f}   (윗면 위 {GRIP_GAP*1000:.0f} mm)")

        self.state = 0
        self.step = 0
        self.start = None
        self.z_at_grip = None

    def current_target(self):
        """이번 스텝의 TCP 목표"""
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            return self.waypoints[self.state]
        alpha = ease(self.step / float(self.n_steps))
        return self.start + alpha * (self.goal - self.start)

    def advance(self):
        if self.done:
            return

        # 단계에 처음 들어온 순간 시작점과 스텝 수를 정한다
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]

            if self.state == 2:                       # GRIP
                self.z_at_grip = top_z(self.target["name"])
                self._gripper.close()                 # 한 번만 보내면 된다
                self.gripper = "close"
                self.n_steps, dist = GRIP_WAIT, 0.0
            elif self.state == 6:                     # RELEASE
                self._gripper.open()
                self.gripper = "open"
                self.n_steps, dist = RELEASE_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}  {dist:.4f} m"
                  f"  {self.n_steps} steps  gripper {self.gripper}")

        self.step += 1
        if self.step < self.n_steps:
            return

        if self.state == 2:                           # GRIP 끝 → 부착 확인
            held = self._gripper.gripped()
            print(f"   ── 부착 확인  {held if held else '없음 — 흡착이 안 걸렸다'}")
        if self.state == 3:                           # LIFT 끝 → 판정
            self._judge()

        self.state += 1
        self.step = 0
        self.start = None

        if self.state >= self.N_STATES:
            self.index += 1
            self._begin_target()

    # ── 판정 ────────────────────────────────────────────
    def _judge(self):
        """물체가 실제로 딸려 올라왔는지 높이 차로 판정한다"""
        name = self.target["name"]
        now = top_z(name)
        rise = now - self.z_at_grip
        ok = rise >= LIFT_OK_MIN
        self.results.append((name, ok, rise, self._gripper.gripped()))

        print(f"   ── 흡착 {'성공' if ok else '실패'}   "
              f"{self.z_at_grip:.4f} -> {now:.4f}  ({rise*1000:+.1f} mm)")
        if not ok:
            if held:
                print(f"      붙긴 했는데 놓쳤다 -> 이 물체의 coaxial/shear 를 올리거나 "
                      f"TCP_SPEED 를 낮춘다")
            else:
                print(f"      아예 안 붙었다 -> GRIP_GAP({GRIP_GAP*1000:.0f}mm) 가 "
                      f"MAX_GRIP_DISTANCE({MAX_GRIP_DISTANCE*1000:.0f}mm) 보다 작은지, "
                      f"물체에 RigidBody/Collider 가 있는지 확인")

    def _summary(self):
        print()
        print(f"   {'─' * 56}")
        print(f"   전체 결과   {sum(1 for r in self.results if r[1])}"
              f" / {len(self.results)} 성공")
        for name, ok, rise, held in self.results:
            print(f"     {name:14s} {'성공' if ok else '실패'}"
                  f"   상승 {rise*1000:+7.1f} mm   붙은 물체 {held}")
        print(f"   {'─' * 56}")
        print()


# ══════════════════════════════════════════════════════════════
#  씬 구성
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def total_mass(prim):
    """서브트리에 적힌 질량을 모두 더한다. 없으면 None"""
    found = [p.GetAttribute("physics:mass").Get()
             for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.MassAPI)]
    found = [m for m in found if m]
    return sum(found) if found else None


def ensure_contact_offset(prim, min_offset=MIN_CONTACT_OFFSET):
    """
    콜라이더의 contactOffset 이 너무 작으면 올린다.

    이 값이 흡착면과 물체 사이 거리보다 작으면 PhysX 가 접촉을 만들지 않고,
    서피스 그리퍼는 붙을 대상을 못 찾는다. 물체는 미동도 하지 않는다.
    """
    fixed = []
    for p in Usd.PrimRange(prim):
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        attr = p.GetAttribute("physxCollision:contactOffset")
        cur = attr.Get() if attr else None
        # 속성이 없으면 PhysX 기본값(0.02)이라 건드릴 필요가 없다
        if cur is not None and cur < min_offset:
            attr.Set(float(min_offset))
            fixed.append(p.GetName())
    return fixed


def has_rigid_body(prim):
    """서브트리 어딘가에 RigidBody 가 있는지 본다"""
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return True
    return False


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


class PickPlaceTask(BaseTask):
    """로봇만 있는 USD 에서 시작해 바닥, 그리퍼, 큐브 세 개를 얹는다"""

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
        self._create_targets()
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

        # 1) 참조. short_gripper.usd 의 defaultPrim(/Root)에 RigidBody 와
        #    질량 1 kg 이 이미 붙어 있어 이 prim 이 그대로 강체가 된다
        grip = UsdGeom.Xform.Define(stage, GRIPPER_PRIM)
        grip.GetPrim().GetReferences().AddReference(GRIPPER_USD)
        simulation_app.update()

        # 2) 물리가 스냅하기 전에 시각 위치를 맞춰 둔다 (첫 프레임 튐 방지)
        cache = UsdGeom.XformCache()
        link6_world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(EE_LINK_PATH))
        local = Gf.Matrix4d().SetRotate(Gf.Quatd(MOUNT_QUAT))
        local.SetTranslateOnly(Gf.Vec3d(MOUNT_OFFSET))

        xf = UsdGeom.Xformable(grip.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local * link6_world)

        # 3) FixedJoint. localRot0 에 마운트 회전을 넣으면
        #    link6_frame * MOUNT_QUAT == gripper_frame 이 강제된다
        joint = UsdPhysics.FixedJoint.Define(
            stage, f"{EE_LINK_PATH}/surface_gripper_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(EE_LINK_PATH)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(GRIPPER_PRIM)])
        joint.CreateLocalPos0Attr().Set(MOUNT_OFFSET)
        joint.CreateLocalRot0Attr().Set(MOUNT_QUAT)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))

        # 4) 파지 한계. 에셋 기본값 0 은 무제한이라 반드시 넣는다
        node = stage.GetPrimAtPath(GRIPPER_NODE)
        node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
        node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
        node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)

        print(f"   gripper      {GRIPPER_PRIM} -> {EE_LINK_PATH}")
        print(f"   grip limits  coaxial {COAXIAL_FORCE_LIMIT} N  "
              f"shear {SHEAR_FORCE_LIMIT} N  dist {MAX_GRIP_DISTANCE*1000:.0f} mm")

    def _create_targets(self):
        """
        대상 물체들. 흡착은 강체가 아니면 붙어도 딸려 오지 않는다.

        "usd" 가 있으면 그 에셋을 참조하고 물리는 건드리지 않는다.
        우리 캐리어 에셋에는 RigidBody 와 질량이 이미 들어 있기 때문이다.
        없으면 큐브를 만들고 물리를 직접 붙인다.
        """
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, CUBE_ROOT)

        for t in TARGETS:
            path = cube_path(t["name"])
            pos = Gf.Vec3d(t["spawn"])

            if "usd" in t:
                # 에셋 원점이 바닥면이라 z 하한은 0 근처면 된다
                if pos[2] < SPAWN_CLEARANCE:
                    print(f"   [주의] {t['name']} spawn z {pos[2]:+.3f} 가 바닥 아래다. "
                          f"{SPAWN_CLEARANCE:.3f} 로 올려 놓는다")
                    pos = Gf.Vec3d(pos[0], pos[1], SPAWN_CLEARANCE)

                xform = UsdGeom.Xform.Define(stage, path)
                xform.GetPrim().GetReferences().AddReference(t["usd"])
                xf = UsdGeom.Xformable(xform.GetPrim())
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(pos)
                simulation_app.update()

                target_prim = stage.GetPrimAtPath(path)
                if not has_rigid_body(target_prim):
                    print(f"   [주의] {t['name']} 에 RigidBody 가 없다. "
                          f"흡착해도 딸려 오지 않는다")
                raised = ensure_contact_offset(target_prim)
                if raised:
                    print(f"   [보정] {t['name']} 콜라이더 {len(raised)}개의 "
                          f"contactOffset 을 {MIN_CONTACT_OFFSET*1000:.0f} mm 로 올렸다 "
                          f"(너무 작으면 흡착이 안 걸린다)")
                kind = f"usd {Path(t['usd']).name}"
            else:
                # Cube 는 size 1 이 한 변 1 m 라 스케일이 곧 한 변이 된다
                min_z = 0.5 * CUBE_SCALE[2] + SPAWN_CLEARANCE
                if pos[2] < min_z:
                    print(f"   [주의] {t['name']} spawn z {pos[2]:+.3f} 는 바닥 아래다. "
                          f"{min_z:.3f} 로 올려 만든다")
                    pos = Gf.Vec3d(pos[0], pos[1], min_z)

                cube = UsdGeom.Cube.Define(stage, path)
                cube.CreateSizeAttr(1.0)
                cube.CreateDisplayColorAttr([t["color"]])
                xf = UsdGeom.Xformable(cube.GetPrim())
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(pos)
                xf.AddScaleOp().Set(CUBE_SCALE)

                prim = cube.GetPrim()
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(CUBE_MASS)
                kind = f"cube {CUBE_SCALE[0]*1000:.0f} mm"

            m = total_mass(stage.GetPrimAtPath(path))
            print(f"   target       {t['name']:12s} {kind:24s} "
                  f"mass {m if m else '?'} kg  spawn "
                  f"({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.3f})  "
                  f"place ({t['place'][0]:+.2f}, {t['place'][1]:+.2f})")

        simulation_app.update()

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
    """시작 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


def create_ik_solver(robot):
    """Lula 계산기를 만들고 로봇과 연결한다"""
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
    print(f"   {name:9s} tcp {vec(tcp)}   "
          f"{fsm.target['name']} top {top_z(fsm.target['name']):.4f}   "
          f"gripper {fsm.gripper}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = PickPlaceTask(name="suction_pick_place_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)

    # 큐브가 바닥에 앉을 시간을 준다. 실측은 이 뒤에 해야 맞다
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    gripper = SurfaceGripperCtl(GRIPPER_NODE)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )

    section("PLAN")
    print(f"   targets      {len(TARGETS)} 개, 하나씩 집고 놓는다")
    for t in TARGETS:
        print(f"                 - {t['name']}")
    print(f"   lift z       {LIFT_HEIGHT}")
    print(f"   tcp offset   {SUCTION_FACE_Z} m  (흡착면)")
    print(f"   성공 기준     물체가 {LIFT_OK_MIN*1000:.0f} mm 이상 상승")
    fsm = SequenceFSM(robot, gripper, TARGETS)

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
            # 팔 — 이번 스텝의 목표를 보간으로 구해 IK 로 푼다
            target_tcp = fsm.current_target()
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)

            # 그리퍼는 상태가 바뀌는 순간 FSM 이 한 번만 명령한다
            fsm.advance()

            if step % LOG_INTERVAL == 0:
                print_status(robot, solved, fsm, target_tcp)
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()

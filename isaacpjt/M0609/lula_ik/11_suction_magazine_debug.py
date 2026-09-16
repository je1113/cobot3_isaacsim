"""
흡착 실패 진단 — 매거진 한 개만 집중 테스트

    isaac_python 11_suction_magazine_debug.py

10_suction_blue_cube.py 와 씬 구성은 같고, 매거진 하나만 다룬다.
큐브 세 개를 먼저 통과시키지 않아도 되므로 반복 실행이 빠르고,
아래 세 가지를 단계별로 직접 눈으로 확인할 수 있게 로그를 촘촘히 찍는다.

  1. DESCEND 끝 — IK 가 매번 풀렸는가, 흡착면이 실제로 플랜지 중심
     (80 x 80 mm, 중심 기준 ±40 mm) 안에 들어왔는가
  2. GRIP 대기 중 — 몇 스텝만에 gripped() 가 채워지는가, 끝까지 안 채워지는가
  3. LIFT 중 — 붙었다가 도중에 놓치는가 (실제 상승량을 스텝마다 찍는다)

10_suction_blue_cube.py 에 있던 버그도 같이 고쳤다:
  _judge() 가 advance() 지역변수 held 를 그대로 참조해서, 흡착 실패
  판정이 나는 순간(정확히 지금 우리가 보려는 상황) NameError 로 죽었다.
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

ASSETS_DIR   = M0609_DIR.parent / "assets"
MAGAZINE_USD = str(ASSETS_DIR / "magazine_small.usda")   # 250 x 140 x 142 mm


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
#  대상 물체 — 매거진 하나뿐
# ══════════════════════════════════════════════════════════════
#   flange_plate 는 magazine_small.usda 안에서 (0,0) 중심, 80 x 80 mm,
#   몸체 로컬 z 0.139~0.142 (윗면). 몸체가 좌우 대칭이라 bbox 중심 xy 는
#   곧 스폰 위치의 xy 와 같다 — 즉, 얼라인이 안 맞을 이유가 코드상으론 없다.
#   여기서 그게 실제로도 맞는지 로그로 확인한다.
TARGET = {
    "name":    "magazine",
    "usd":     MAGAZINE_USD,
    "spawn":   Gf.Vec3d(0.45,  0.30,  0.005),
    "place":   np.array([0.45, -0.30]),
    # 진단용: 힘 한계 자체가 원인인지 가려내려고 터무니없이 높여 둔 값.
    # 이래도 LIFT 시작 직후 즉시 떨어지면 힘 한계 문제가 아니라
    # 무게중심이 흡착점(윗면)보다 한참 아래에 있어 생기는 토크/회전 문제다.
    "coaxial": 100.0,
    "shear":   100.0,
}
TARGETS = [TARGET]

CUBE_ROOT = "/World/targets"
SPAWN_CLEARANCE = 0.002
MIN_CONTACT_OFFSET = 0.02


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정
# ══════════════════════════════════════════════════════════════
GRIPPER_PRIM = f"{ROBOT_PRIM_PATH}/surface_gripper"
GRIPPER_NODE = f"{GRIPPER_PRIM}/SurfaceGripper"

MOUNT_QUAT   = Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0))
MOUNT_OFFSET = Gf.Vec3f(0.0, 0.0, 0.0)

COAXIAL_FORCE_LIMIT = 20.0    # N — TARGET 의 "coaxial" 로 덮어씀
SHEAR_FORCE_LIMIT   = 10.0    # N — TARGET 의 "shear" 로 덮어씀
MAX_GRIP_DISTANCE   = 0.02    # m

SUCTION_FACE_Z = 0.161
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])

# 흡착 대상 플랜지 판의 실제 크기 (magazine_small.usda 의 flange_plate) — 로그 비교용
FLANGE_HALF_WIDTH = 0.04       # m, 80 mm 판의 절반


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
GRIP_GAP        = 0.005
PLACE_DROP      = 0.005
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT     = 0.23

GRIP_WAIT    = 120
RELEASE_WAIT = 90
LIFT_OK_MIN  = 0.03

# GRIP / LIFT 동안 몇 스텝마다 상태를 찍을지 — 여기서 진단 해상도를 올린다
POLL_INTERVAL = 10

TCP_SPEED = 0.004
MIN_STEPS = 60
MAX_STEPS = 600

SETTLE_STEPS = 60

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


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


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  물체 측정
# ══════════════════════════════════════════════════════════════
def cube_path(name):
    return f"{CUBE_ROOT}/{name}"


def measure(name):
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
    return measure(name)[1]


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 제어
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

    def set_limits(self, coaxial, shear):
        stage = omni.usd.get_context().get_stage()
        node = stage.GetPrimAtPath(self._path)
        node.GetAttribute("isaac:coaxialForceLimit").Set(float(coaxial))
        node.GetAttribute("isaac:shearForceLimit").Set(float(shear))

    def gripped(self):
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
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def ease(alpha):
    a = float(np.clip(alpha, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


class SequenceFSM:
    NAMES = ["APPROACH", "DESCEND", "GRIP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "RETREAT"]
    N_STATES = 8

    def __init__(self, robot, gripper, targets):
        self._robot = robot
        self._gripper = gripper
        self._targets = targets
        self.reset()

    def reset(self):
        self.index = 0
        self.state = 0
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
        if self.index >= len(self._targets):
            self.done = True
            self._summary()
            return

        name = self.target["name"]
        center_xy, t_z, height = measure(name)
        cx, cy = center_xy
        gx, gy = self.target["place"]

        grip_z  = t_z + GRIP_GAP
        place_z = height + GRIP_GAP + PLACE_DROP

        self.waypoints = [
            np.array([cx, cy, APPROACH_HEIGHT]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, grip_z]),
            np.array([cx, cy, LIFT_HEIGHT]),
            np.array([gx, gy, LIFT_HEIGHT]),
            np.array([gx, gy, place_z]),
            np.array([gx, gy, place_z]),
            np.array([gx, gy, APPROACH_HEIGHT]),
        ]

        coaxial = self.target.get("coaxial", COAXIAL_FORCE_LIMIT)
        shear = self.target.get("shear", SHEAR_FORCE_LIMIT)
        self._gripper.set_limits(coaxial, shear)

        stage = omni.usd.get_context().get_stage()
        mass = total_mass(stage.GetPrimAtPath(cube_path(name)))
        weight = mass * 9.81 if mass else None

        print()
        print(f"   ═══ {name} ═══")
        print(f"   pick center  ({cx:+.4f}, {cy:+.4f})  top z {t_z:.4f}  "
              f"height {height*1000:.1f} mm")
        print(f"   flange span  x [{cx-FLANGE_HALF_WIDTH:+.3f}, {cx+FLANGE_HALF_WIDTH:+.3f}]"
              f"  y [{cy-FLANGE_HALF_WIDTH:+.3f}, {cy+FLANGE_HALF_WIDTH:+.3f}]"
              f"  (80x80mm 플랜지 가정)")
        if weight is not None:
            print(f"   mass         {mass:.3f} kg = {weight:.2f} N   "
                  f"한계 coaxial {coaxial:.0f} N ({coaxial/weight:.1f}배)  "
                  f"shear {shear:.0f} N")
        print(f"   place        ({gx:+.3f}, {gy:+.3f})  release z {place_z:.4f}")
        print(f"   grip z       {grip_z:.4f}   (윗면 위 {GRIP_GAP*1000:.0f} mm, "
              f"max grip dist {MAX_GRIP_DISTANCE*1000:.0f} mm)")

        self.state = 0
        self.step = 0
        self.start = None
        self.z_at_grip = None

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            return self.waypoints[self.state]
        alpha = ease(self.step / float(self.n_steps))
        return self.start + alpha * (self.goal - self.start)

    def advance(self):
        if self.done:
            return

        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]

            if self.state == 1:                       # DESCEND 시작 — 얼라인 확인
                cx, cy = self.waypoints[0][0], self.waypoints[0][1]
                err = np.linalg.norm(self.start[:2] - np.array([cx, cy]))
                print(f"   [align] tcp xy 시작 오차 {err*1000:.1f} mm  "
                      f"(플랜지 반폭 {FLANGE_HALF_WIDTH*1000:.0f} mm)")

            if self.state == 2:                       # GRIP
                self.z_at_grip = top_z(self.target["name"])
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
                  f" goal {vec(self.goal)}  {dist:.4f} m"
                  f"  {self.n_steps} steps  gripper {self.gripper}")

        self.step += 1

        # GRIP / LIFT 는 매 POLL_INTERVAL 스텝마다 실시간 상태를 찍는다.
        # 여기서 "몇 스텝만에 붙는지" / "붙었다가 언제 놓치는지" 가 보인다.
        if self.state in (2, 3) and self.step % POLL_INTERVAL == 0:
            held = self._gripper.gripped()
            now = top_z(self.target["name"])
            rise = now - self.z_at_grip if self.z_at_grip is not None else 0.0
            tcp = get_tcp_pose(self._robot)
            print(f"       step {self.step:3d}/{self.n_steps}  tcp {vec(tcp)}  "
                  f"top_z {now:.4f} ({rise*1000:+.1f} mm)  held {held}")

        if self.step < self.n_steps:
            return

        if self.state == 2:
            held = self._gripper.gripped()
            print(f"   ── 부착 확인  {held if held else '없음 — 흡착이 안 걸렸다'}")
        if self.state == 3:
            self._judge()

        self.state += 1
        self.step = 0
        self.start = None

        if self.state >= self.N_STATES:
            self.index += 1
            self._begin_target()

    def _judge(self):
        """물체가 실제로 딸려 올라왔는지 높이 차로 판정한다"""
        name = self.target["name"]
        now = top_z(name)
        rise = now - self.z_at_grip
        ok = rise >= LIFT_OK_MIN
        held = self._gripper.gripped()
        self.results.append((name, ok, rise, held))

        print(f"   ── 흡착 {'성공' if ok else '실패'}   "
              f"{self.z_at_grip:.4f} -> {now:.4f}  ({rise*1000:+.1f} mm)")
        if not ok:
            if held:
                print(f"      붙긴 했는데 놓쳤다 -> 이 물체의 coaxial/shear 를 올리거나 "
                      f"TCP_SPEED 를 낮춘다")
            else:
                print(f"      아예 안 붙었다 -> 위 [align] 오차가 플랜지 반폭보다 컸는지, "
                      f"GRIP 단계 로그에서 held 가 끝까지 None/빈 값이었는지 확인")

    def _summary(self):
        print()
        print(f"   {'─' * 56}")
        for name, ok, rise, held in self.results:
            print(f"     {name:14s} {'성공' if ok else '실패'}"
                  f"   상승 {rise*1000:+7.1f} mm   붙은 물체 {held}")
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


def total_mass(prim):
    found = [p.GetAttribute("physics:mass").Get()
             for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.MassAPI)]
    found = [m for m in found if m]
    return sum(found) if found else None


def ensure_contact_offset(prim, min_offset=MIN_CONTACT_OFFSET):
    fixed = []
    for p in Usd.PrimRange(prim):
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        attr = p.GetAttribute("physxCollision:contactOffset")
        cur = attr.Get() if attr else None
        if cur is not None and cur < min_offset:
            attr.Set(float(min_offset))
            fixed.append(p.GetName())
    return fixed


def has_rigid_body(prim):
    for p in Usd.PrimRange(prim):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return True
    return False


def has_ground_plane():
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if "groundplane" in name or "ground_plane" in name:
            return True
        if prim.IsA(UsdGeom.Plane) and prim.HasAPI(UsdPhysics.CollisionAPI):
            return True
    return False


class PickPlaceTask(BaseTask):
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
        stage = omni.usd.get_context().get_stage()
        if not stage.GetPrimAtPath("/World").IsValid():
            UsdGeom.Xform.Define(stage, "/World")
        robot = UsdGeom.Xform.Define(stage, ROBOT_PRIM_PATH)
        robot.GetPrim().GetReferences().AddReference(ROBOT_USD)
        for _ in range(15):
            simulation_app.update()
        print(f"   robot        {ROBOT_PRIM_PATH}")

    def _attach_surface_gripper(self):
        stage = omni.usd.get_context().get_stage()

        grip = UsdGeom.Xform.Define(stage, GRIPPER_PRIM)
        grip.GetPrim().GetReferences().AddReference(GRIPPER_USD)
        simulation_app.update()

        cache = UsdGeom.XformCache()
        link6_world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(EE_LINK_PATH))
        local = Gf.Matrix4d().SetRotate(Gf.Quatd(MOUNT_QUAT))
        local.SetTranslateOnly(Gf.Vec3d(MOUNT_OFFSET))

        xf = UsdGeom.Xformable(grip.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(local * link6_world)

        joint = UsdPhysics.FixedJoint.Define(
            stage, f"{EE_LINK_PATH}/surface_gripper_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(EE_LINK_PATH)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(GRIPPER_PRIM)])
        joint.CreateLocalPos0Attr().Set(MOUNT_OFFSET)
        joint.CreateLocalRot0Attr().Set(MOUNT_QUAT)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))

        node = stage.GetPrimAtPath(GRIPPER_NODE)
        node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
        node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
        node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)

        print(f"   gripper      {GRIPPER_PRIM} -> {EE_LINK_PATH}")
        print(f"   grip limits  coaxial {COAXIAL_FORCE_LIMIT} N  "
              f"shear {SHEAR_FORCE_LIMIT} N  dist {MAX_GRIP_DISTANCE*1000:.0f} mm")

    def _create_targets(self):
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, CUBE_ROOT)

        for t in TARGETS:
            path = cube_path(t["name"])
            pos = Gf.Vec3d(t["spawn"])

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
                      f"contactOffset 을 {MIN_CONTACT_OFFSET*1000:.0f} mm 로 올렸다")

            m = total_mass(stage.GetPrimAtPath(path))
            print(f"   target       {t['name']:12s} usd {Path(t['usd']).name:24s} "
                  f"mass {m if m else '?'} kg  spawn "
                  f"({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.3f})  "
                  f"place ({t['place'][0]:+.2f}, {t['place'][1]:+.2f})")

        simulation_app.update()

    def _setup_arm_drives(self):
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
    print(f"   {name:9s} tcp {vec(tcp)}   "
          f"{fsm.target['name']} top {top_z(fsm.target['name']):.4f}   "
          f"gripper {fsm.gripper}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 30   # IK/자세 로그는 매거진 하나뿐이라 촘촘히 본다


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = PickPlaceTask(name="magazine_debug_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)

    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    gripper = SurfaceGripperCtl(GRIPPER_NODE)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )

    section("PLAN")
    print(f"   lift z       {LIFT_HEIGHT}")
    print(f"   tcp offset   {SUCTION_FACE_Z} m  (흡착면)")
    print(f"   성공 기준     물체가 {LIFT_OK_MIN*1000:.0f} mm 이상 상승")
    fsm = SequenceFSM(robot, gripper, TARGETS)

    section("RUN")
    print("   press Play in the viewport")

    was_playing = False
    step = 0
    ik_fail_streak = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

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
            ik_fail_streak = 0

        if is_playing:
            target_tcp = fsm.current_target()
            flange_target = tcp_to_flange(target_tcp, target_quat)

            action, solved = ik_solver.compute_inverse_kinematics(
                target_position=flange_target,
                target_orientation=target_quat,
            )
            if solved:
                robot.apply_action(action)
                ik_fail_streak = 0
            else:
                ik_fail_streak += 1
                if ik_fail_streak in (1, 30, 100):
                    print(f"   [!] IK FAILED {ik_fail_streak} 연속  "
                          f"target {vec(target_tcp)}  flange {vec(flange_target)}")

            fsm.advance()

            if step % LOG_INTERVAL == 0:
                print_status(robot, solved, fsm, target_tcp)
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()

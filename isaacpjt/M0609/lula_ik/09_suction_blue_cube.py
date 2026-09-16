"""
흡착 확인 — 파란 큐브를 붙여서 들어올리기

    isaac_python 09_suction_blue_cube.py

로봇만 있는 USD(m0609_isaac_sim.usd)에서 시작해 씬을 직접 만든다.

  1. 순수 M0609 를 /World/m0609 로 올린다 (그리퍼도 바닥도 없는 파일)
  2. 바닥 평면을 깐다
  3. 흡착 그리퍼를 link_6 에 FixedJoint 로 붙인다
  4. 파란 큐브를 [0.35, 0.10, 0.05] 에 만든다 (scale 0.025)
  5. 로봇을 움직여 흡착하고 들어올린 뒤, 큐브가 딸려 왔는지 높이로 판정한다

     APPROACH -> DESCEND -> GRIP -> LIFT -> CHECK

큐브 크기는 코드에 박지 않고 물리가 안정된 뒤 월드 바운딩박스를 실측한다.
Cube 의 size 속성이 1 인지 2 인지에 따라 실제 크기가 두 배 달라지기 때문이다.
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
#  대상 — 파란 큐브
# ══════════════════════════════════════════════════════════════
CUBE_PATH  = "/World/blue_cube"
CUBE_POS   = Gf.Vec3d(0.35, 0.10, 0.05)
CUBE_SCALE = Gf.Vec3f(0.025, 0.025, 0.025)
CUBE_MASS  = 0.02                               # kg
CUBE_COLOR = Gf.Vec3f(0.15, 0.35, 0.85)


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 설정
# ══════════════════════════════════════════════════════════════
GRIPPER_PRIM = f"{ROBOT_PRIM_PATH}/surface_gripper"
GRIPPER_NODE = f"{GRIPPER_PRIM}/SurfaceGripper"

# short_gripper 는 로컬 +X 로 뻗는다. 툴축은 link_6 로컬 +Z 이므로 Y축 -90도.
MOUNT_QUAT   = Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0))
MOUNT_OFFSET = Gf.Vec3f(0.0, 0.0, 0.0)

COAXIAL_FORCE_LIMIT = 20.0    # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 10.0    # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.02    # m, 이 안에 들어오면 붙는다

# link_6 로컬 +Z 방향으로 흡착면까지의 거리 (gripper_tip 바깥면)
SUCTION_FACE_Z = 0.161
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])


# ══════════════════════════════════════════════════════════════
#  동작 파라미터
# ══════════════════════════════════════════════════════════════
GRIP_GAP        = 0.005    # 흡착면을 큐브 윗면보다 이만큼 위에 세운다
APPROACH_HEIGHT = 0.25     # 접근 대기 높이 (흡착면 기준)
LIFT_HEIGHT     = 0.23     # 들어올릴 높이

GRIP_WAIT   = 120          # 흡착 명령 후 기다리는 스텝
CHECK_WAIT  = 180          # 들고 버티는 스텝
LIFT_OK_MIN = 0.03         # 큐브가 이만큼 올라가면 흡착 성공

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
#  큐브 측정
# ══════════════════════════════════════════════════════════════
def measure_cube():
    """
    큐브의 현재 월드 바운딩박스를 잰다.

    캐시를 매번 새로 만드는 이유는 물리가 큐브를 움직인 뒤의 값이 필요해서다.
    반환값은 (중심 xy, 윗면 z, 한 변 길이).
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(CUBE_PATH)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        raise RuntimeError(f"{CUBE_PATH} 의 바운딩박스가 비어 있다")
    lo, hi = rng.GetMin(), rng.GetMax()
    center_xy = np.array([(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0])
    return center_xy, float(hi[2]), float(hi[2] - lo[2])


def cube_top_z():
    """큐브 윗면 높이만 빠르게"""
    return measure_cube()[1]


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


# ══════════════════════════════════════════════════════════════
#  궤적 + 상태 기계
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


class SuctionCheckFSM:
    """
      0 APPROACH   큐브 위로 접근
      1 DESCEND    흡착 거리까지 하강
      2 GRIP       흡착 (제자리)
      3 LIFT       들어올리기
      4 CHECK      들고 버티며 높이 확인
      5 DONE

    큐브 위치는 Play 를 누른 시점에 실측해서 웨이포인트를 만든다.
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "LIFT", "CHECK", "DONE"]
    DONE_STATE = 5

    def __init__(self, robot, gripper):
        self._robot = robot
        self._gripper = gripper
        self.waypoints = []
        self.cube_z_at_grip = None
        self.result = None
        self.reset()

    def reset(self):
        """큐브를 다시 재고 웨이포인트를 만든다"""
        center_xy, top_z, side = measure_cube()
        cx, cy = center_xy
        grip_z = top_z + GRIP_GAP

        self.waypoints = [
            np.array([cx, cy, APPROACH_HEIGHT]),   # 0 APPROACH
            np.array([cx, cy, grip_z]),            # 1 DESCEND
            np.array([cx, cy, grip_z]),            # 2 GRIP
            np.array([cx, cy, LIFT_HEIGHT]),       # 3 LIFT
            np.array([cx, cy, LIFT_HEIGHT]),       # 4 CHECK
        ]

        print(f"   cube         center ({cx:+.3f}, {cy:+.3f})  "
              f"top z {top_z:.4f}  side {side*1000:.1f} mm")
        print(f"   grip z       {grip_z:.4f}   (윗면 위 {GRIP_GAP*1000:.0f} mm)")

        self.state = 0
        self.step = 0
        self.start = None
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.gripper = "open"
        self.cube_z_at_grip = None
        self.result = None

    def current_target(self):
        """이번 스텝의 TCP 목표"""
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return self.start + alpha * (self.goal - self.start)

    def advance(self):
        if self.state >= self.DONE_STATE:
            return

        # 단계에 처음 들어온 순간 시작점과 스텝 수를 정한다
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]

            if self.state == 2:                      # GRIP
                self.cube_z_at_grip = cube_top_z()
                self._gripper.close()                # 한 번만 보내면 된다
                self.gripper = "close"
                self.n_steps, dist = GRIP_WAIT, 0.0
            elif self.state == 4:                    # CHECK
                self.n_steps, dist = CHECK_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}  {dist:.4f} m"
                  f"  {self.n_steps} steps  gripper {self.gripper}")

        self.step += 1
        if self.step >= self.n_steps:
            if self.state == 4:
                self._judge()
            self.state += 1
            self.step = 0
            self.start = None
            if self.state >= self.DONE_STATE:
                print(f"   [{self.DONE_STATE}] DONE")

    def _judge(self):
        """큐브가 실제로 딸려 올라왔는지 높이 차로 판정한다"""
        now = cube_top_z()
        rise = now - self.cube_z_at_grip
        held = self._gripper.gripped()
        self.result = rise >= LIFT_OK_MIN

        print()
        print(f"   {'─' * 56}")
        print(f"   흡착 결과   {'성공' if self.result else '실패'}")
        print(f"   큐브 윗면   {self.cube_z_at_grip:.4f} -> {now:.4f}"
              f"   ({rise * 1000:+.1f} mm)")
        print(f"   붙은 물체   {held}")
        if not self.result:
            print(f"   판정 기준   {LIFT_OK_MIN*1000:.0f} mm 이상 상승")
            print(f"   확인할 것   GRIP_GAP({GRIP_GAP*1000:.0f}mm) 이 "
                  f"MAX_GRIP_DISTANCE({MAX_GRIP_DISTANCE*1000:.0f}mm) 보다 작은지,")
            print(f"               COAXIAL_FORCE_LIMIT({COAXIAL_FORCE_LIMIT}N) 이 "
                  f"큐브 무게({CUBE_MASS*9.81:.2f}N) 보다 큰지")
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


class SuctionCheckTask(BaseTask):
    """로봇만 있는 USD 에서 시작해 바닥, 그리퍼, 큐브를 얹는다"""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_robot()
        scene.add_default_ground_plane()
        print("   ground       added")
        self._attach_surface_gripper()
        self._create_cube()
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

    def _create_cube(self):
        """파란 큐브. 흡착은 강체가 아니면 붙어도 딸려 오지 않는다"""
        stage = omni.usd.get_context().get_stage()

        cube = UsdGeom.Cube.Define(stage, CUBE_PATH)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([CUBE_COLOR])
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(CUBE_POS)
        xf.AddScaleOp().Set(CUBE_SCALE)

        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(CUBE_MASS)
        simulation_app.update()

        print(f"   cube         {CUBE_PATH} at {tuple(CUBE_POS)} "
              f"scale {CUBE_SCALE[0]}  mass {CUBE_MASS} kg")

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
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
    if not solved:
        print(f"   {name:9s} IK FAILED  target {vec(target_tcp)}")
        return
    tcp = get_tcp_pose(robot)
    print(f"   {name:9s} tcp {vec(tcp)}   cube top {cube_top_z():.4f}"
          f"   gripper {fsm.gripper}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = SuctionCheckTask(name="suction_check_task")
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
    fsm = SuctionCheckFSM(robot, gripper)
    print(f"   lift z       {LIFT_HEIGHT}")
    print(f"   tcp offset   {SUCTION_FACE_Z} m  (흡착면)")
    print(f"   성공 기준     큐브가 {LIFT_OK_MIN*1000:.0f} mm 이상 상승")

    section("RUN")
    print("   press Play in the viewport\n")

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
            print()

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

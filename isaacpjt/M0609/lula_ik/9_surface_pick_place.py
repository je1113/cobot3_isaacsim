"""
Pick & Place — 흡착(서피스) 그리퍼로 블록 들었다 놓기

    isaac_python 9_surface_pick_place.py

6_pick_place.py 와 순서는 같고 그리퍼만 바뀐다.

  평행 그리퍼(RG2)          흡착 그리퍼(SurfaceGripper)
  ────────────────────      ─────────────────────────────
  관절을 닫아 옆면을 문다    윗면에 닿으면 조인트로 붙인다
  TCP = 손가락 끝 0.19671   TCP = 흡착면 0.161
  블록 옆면 높이로 내려감    블록 윗면 위로 내려감
  ParallelGripper.forward()  GripperView.apply_gripper_action()

씬에는 RG2 가 이미 붙어 있으므로, 흡착 그리퍼를 달기 전에 RG2 를 비활성화한다.
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

USD_PATH         = str(M0609_DIR / "gripper_test.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")

GRIPPER_USD = (get_assets_root_path()
               + "/Isaac/Robots/UniversalRobots/ur10/grippers/short_gripper.usd")


# ══════════════════════════════════════════════════════════════
#  로봇 설정 — 6_pick_place.py 와 동일
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
EE_LINK_PATH    = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"
RG2_PRIM_PATH   = f"{ROBOT_PRIM_PATH}/onrobot_rg2ft"

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

# 파지 한계. 기본값 0 은 무제한이라 반드시 넣어 준다.
COAXIAL_FORCE_LIMIT = 20.0    # N, 흡착면 수직
SHEAR_FORCE_LIMIT   = 10.0    # N, 흡착면 평행
MAX_GRIP_DISTANCE   = 0.02    # m, 이 안에 들어오면 붙는다


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
# link_6 로컬 +Z 방향으로 흡착면까지의 거리
#   gripper_tip 바깥면이 그리퍼 로컬 x = 161 mm
SUCTION_FACE_Z = 0.161
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])


# ══════════════════════════════════════════════════════════════
#  목표
# ══════════════════════════════════════════════════════════════
# red_block — 47 mm 정육면체, 20 g, (0.25, 0.10) 에 스폰되어 바닥에 앉는다
#   바닥에 앉으면 윗면이 z = 0.047
BLOCK_TOP_Z = 0.047

PICK_XY  = np.array([0.25,  0.10])
PLACE_XY = np.array([0.45, -0.10])

# 흡착은 윗면을 잡으므로 평행 그리퍼와 높이 기준이 다르다
#   PICK_Z   흡착면을 블록 윗면보다 5 mm 위에 둔다 (파지 거리 20 mm 안)
#   PLACE_Z  놓을 때는 조금 높게 두어 블록이 튀지 않게 한다
PICK_Z          = BLOCK_TOP_Z + 0.005      # 0.052
PLACE_Z         = BLOCK_TOP_Z + 0.008      # 0.055
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT     = 0.23

# 흡착 / 해제 후 기다리는 스텝 수
GRIPPER_WAIT = 120

# 보간 파라미터
TCP_SPEED  = 0.004     # 스텝당 TCP 이동 거리(m)
MIN_STEPS  = 60
MAX_STEPS  = 600

# 접근 방향 — 툴(link_6 로컬 +Z)이 바닥을 향하게
APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


# ══════════════════════════════════════════════════════════════
#  회전 유틸 — 6_pick_place.py 와 동일
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
    """회전축과 각도(도)로 쿼터니언을 만든다"""
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


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
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    """현재 플랜지 pose 로부터 흡착면의 월드 위치를 구한다"""
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 제어
# ══════════════════════════════════════════════════════════════
class SurfaceGripperCtl:
    """
    서피스 그리퍼는 관절이 없어서 열기와 닫기 두 가지뿐이다.

    Isaac 버전에 따라 GripperView 와 커맨드 인터페이스 중 하나만 있을 수 있어
    가능한 쪽을 골라 쓴다.
    """

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
#  궤적 보간
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    return start + alpha * (goal - start)


class SurfacePickPlaceFSM:
    """
      0 APPROACH   블록 위로 접근
      1 DESCEND    흡착 거리까지 하강
      2 GRIP       흡착 (제자리)
      3 LIFT       들어올리기
      4 MOVE       놓을 곳 위로 이동
      5 LOWER      놓을 높이까지 하강
      6 RELEASE    해제 (제자리)
      7 RETREAT    그리퍼만 위로 빼기
      8 DONE
    """

    NAMES = ["APPROACH", "DESCEND", "GRIP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "RETREAT", "DONE"]
    GRIPPER_STATES = {2: "close", 6: "open"}
    DONE_STATE = 8

    def __init__(self, robot, gripper):
        self._robot = robot
        self._gripper = gripper
        self._build_waypoints()
        self.reset()

    def _build_waypoints(self):
        px, py = PICK_XY
        gx, gy = PLACE_XY
        self.waypoints = [
            np.array([px, py, APPROACH_HEIGHT]),   # 0 APPROACH
            np.array([px, py, PICK_Z]),            # 1 DESCEND
            np.array([px, py, PICK_Z]),            # 2 GRIP
            np.array([px, py, LIFT_HEIGHT]),       # 3 LIFT
            np.array([gx, gy, LIFT_HEIGHT]),       # 4 MOVE
            np.array([gx, gy, PLACE_Z]),           # 5 LOWER
            np.array([gx, gy, PLACE_Z]),           # 6 RELEASE
            np.array([gx, gy, APPROACH_HEIGHT]),   # 7 RETREAT
        ]

    def reset(self):
        self.state = 0
        self.step = 0
        self.start = None
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.gripper = "open"

    def current_target(self):
        """이번 스텝의 TCP 목표"""
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        if self.state >= self.DONE_STATE:
            return

        # 단계에 처음 들어온 순간 시작점과 스텝 수를 정하고 그리퍼를 한 번 움직인다
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]

            command = self.GRIPPER_STATES.get(self.state)
            if command is not None:
                self.gripper = command
                # 평행 그리퍼와 달리 매 스텝 유지할 필요 없이 한 번만 보낸다
                if command == "close":
                    self._gripper.close()
                else:
                    self._gripper.open()
                self.n_steps = GRIPPER_WAIT
                dist = 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)

            print(f"   [{self.state}] {self.NAMES[self.state]:9s}"
                  f" goal {vec(self.goal)}"
                  f"  {dist:.4f} m  {self.n_steps} steps  gripper {self.gripper}")

        self.step += 1
        if self.step >= self.n_steps:
            self._next()

    def _next(self):
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            print(f"   [{self.DONE_STATE}] DONE")


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
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


class M0609SurfaceTask(BaseTask):
    """M0609 + 흡착 그리퍼 씬"""

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._disable_rg2()
        self._attach_surface_gripper()
        self._setup_arm_drives()
        self._register_robot(scene)
        print("   scene        ready")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print("   USD          loaded")

    def _disable_rg2(self):
        """
        씬 USD 에는 RG2 가 이미 link_6 에 붙어 있다.
        그대로 두면 그리퍼 두 개가 겹치므로 비활성화한다.
        서브트리가 통째로 빠지면서 AssemblerFixedJoint 도 같이 사라진다.
        """
        stage = omni.usd.get_context().get_stage()
        rg2 = stage.GetPrimAtPath(RG2_PRIM_PATH)
        if rg2.IsValid():
            rg2.SetActive(False)
            simulation_app.update()
            print(f"   rg2          disabled ({RG2_PRIM_PATH})")
        else:
            print(f"   rg2          not found, skip")

    def _attach_surface_gripper(self):
        """흡착 그리퍼를 참조로 올리고 link_6 에 FixedJoint 로 묶는다"""
        stage = omni.usd.get_context().get_stage()

        # 1) 참조. short_gripper.usd 의 defaultPrim(/Root)에 RigidBody 와 질량 1 kg 이
        #    이미 붙어 있어 이 prim 이 그대로 강체가 된다
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

        # 4) 파지 한계. 에셋 기본값 0 은 무제한이다
        node = stage.GetPrimAtPath(GRIPPER_NODE)
        node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
        node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
        node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)

        print(f"   gripper      {GRIPPER_PRIM} -> {EE_LINK_PATH}")
        print(f"   grip limits  coaxial {COAXIAL_FORCE_LIMIT} N  "
              f"shear {SHEAR_FORCE_LIMIT} N  dist {MAX_GRIP_DISTANCE*1000:.0f} mm")

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
        """그리퍼 인자 없이 등록한다. 서피스 그리퍼는 관절이 없어 별도로 다룬다"""
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


# ══════════════════════════════════════════════════════════════
#  IK 솔버
# ══════════════════════════════════════════════════════════════
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


def print_plan(target_quat):
    R = quat_to_matrix(target_quat)
    section("PLAN")
    print(f"   pick xy      {vec(PICK_XY)}")
    print(f"   place xy     {vec(PLACE_XY)}")
    print()
    print(f"   block top z  {BLOCK_TOP_Z}   (47 mm cube, 20 g)")
    print(f"   pick z       {PICK_Z}   흡착면이 블록 위 "
          f"{(PICK_Z - BLOCK_TOP_Z)*1000:.0f} mm")
    print(f"   place z      {PLACE_Z}")
    print(f"   lift z       {LIFT_HEIGHT}")
    print(f"   approach z   {APPROACH_HEIGHT}")
    print()
    print(f"   tcp offset   {SUCTION_FACE_Z} m  (흡착면)")
    print(f"   tcp speed    {TCP_SPEED} m/step")
    print(f"   grip wait    {GRIPPER_WAIT} steps")
    print()
    print(f"   tool   +Z    {vec(R @ np.array([0, 0, 1]))}   approach direction")


def print_status(robot, solved, fsm, target_tcp, gripper):
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
    if not solved:
        print(f"   {name:9s} IK FAILED  target {vec(target_tcp)}")
        return
    tcp = get_tcp_pose(robot)
    held = gripper.gripped()
    print(f"   {name:9s} tcp {vec(tcp)}   gripper {fsm.gripper:5s}  held {held}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = M0609SurfaceTask(name="m0609_surface_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    gripper = SurfaceGripperCtl(GRIPPER_NODE)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    print_plan(target_quat)

    section("RUN")
    print("   press Play in the viewport\n")

    fsm = SurfacePickPlaceFSM(robot, gripper)
    was_playing = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간 시작 자세로 되돌린다
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            set_ready_pose(robot)
            gripper.reinit()
            gripper.open()
            fsm.reset()
            step = 0
            print()

        if is_playing:
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
                print_status(robot, solved, fsm, target_tcp, gripper)
            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()

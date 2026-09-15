"""
색상 기반 Pick & Place — /color_id 구독으로 PC B 와 연동하기

    isaac_python 7_pick_place_color.py

6단계와 골격은 같다. 달라진 것은 "어디서 집어 어디에 놓을지" 를
더 이상 좌표로 하드코딩하지 않고, 색상 하나(1=파랑 2=초록)로 정한다는 점이다.

  ① IK / 그리퍼 확인          — 1_ik_single_pose.py, 3_gripper.py 에서 이미 확인함
  ② 위치 정보 정리            — 관찰 자세, 마커 위치는 상수. 큐브 위치는 get_world_pose() 로 조회
  ③ ROS 없이 완성             — FORCE_COLOR_ID 로 색을 코드에 고정해 한 사이클 확인
  ④ /color_id 구독으로 교체   — 관찰 자세일 때만 값을 받고, 작업 중 값은 무시
  ⑤ 단위 테스트               — 아래처럼 확인한다

    터미널 1)  isaac_python 7_pick_place_color.py     # 이 스크립트, Play
    터미널 2)  ros2 topic echo /color_id               # 도착 확인
    터미널 3)  ros2 topic pub --once /color_id \\
                   std_msgs/msg/Int32 "{data: 1}"      # 1=파랑 2=초록
               (작업 중에 같은 값을 여러 번 pub 해도 무시되는지 확인)
    터미널 2)  ros2 topic echo /rgb --no-arr            # 손목 카메라 발행 확인 (2단계 요구사항)

구독은 파이썬 rclpy 가 아니라 OmniGraph 의 ROS2Subscriber 노드로 한다.
이 환경은 시스템 ROS2(Jazzy, Python 3.12)가 PYTHONPATH 에 먼저 잡혀 있어
Isaac Sim 내장 python(3.11)에서 rclpy 를 직접 import 하면 죽는다 — 이미 USD 에
구성돼 있는 손목 카메라 발행 그래프와 같은 방식(OmniGraph)을 쓰면 이 문제가 없다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import sys
from pathlib import Path
import random
import time

# Kit 은 python stdout 을 자체 버퍼링하기 때문에 print() 가 뷰포트 실행 중에는
# 안 보이다가 프로세스가 끝나야 한꺼번에 나오는 경우가 있다. 즉시 흘려보내도록 고정한다.
sys.stdout.reconfigure(line_buffering=True)

import numpy as np
import omni.usd
import omni.graph.core as og
from pxr import Usd, UsdGeom, UsdPhysics, Gf

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_hw/m0609_camera_cube_hw.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


# ══════════════════════════════════════════════════════════════
#  로봇 설정  (6단계와 동일 — 같은 로봇)
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

# 관찰 자세 — 손목 카메라로 큐브 두 개와 마커 두 개가 보이는 자세.
# 지금은 4~6단계의 "그리퍼 아래를 향하는" 자세를 그대로 쓴다.
# 카메라 화각에 안 들어오면 이 각도만 조정하면 된다.
OBSERVE_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8

FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  큐브 / 마커 배치
# ══════════════════════════════════════════════════════════════
# color_id -> 큐브 색 이름   1=파랑 2=초록
COLOR_ID_TO_NAME = {1: "blue", 2: "green"}

RED_BLOCK_PATH = "/World/red_block"   # 이번 단계에서는 쓰지 않아 비활성화한다

CUBE_PRIM_PATH = {
    "blue":  "/World/blue_block",
    "green": "/World/green_block",
}
DEFAULT_XY = {
    "blue":  np.array([0.35,  0.10]),
    "green": np.array([0.45,  0.10]),
}
CUBE_Z = 0.05   # 큐브가 테이블에 놓인 높이

# 파란/초록 큐브 중 하나를 이 범위 안 랜덤 위치로 옮긴다. 나머지 하나는 기본 위치 그대로 둔다
RANDOM_X_RANGE = (0.15, 0.55)
RANDOM_Y_RANGE = (-0.10, 0.10)
MIN_SEPARATION = 0.12   # 움직이지 않는 큐브와 최소 이 거리는 떨어뜨린다

# 놓을 마커 위치 — 색마다 고정. 큐브가 어디서 오든 이 자리에 놓는다
MARKER_XY = {
    "blue":  np.array([0.45, -0.45]),
    "green": np.array([0.45,  0.20]),
}
MARKER_RGB = {
    "blue":  (0.10, 0.35, 0.95),
    "green": (0.15, 0.80, 0.20),
}
MARKER_RADIUS = 0.05
MARKER_HEIGHT = 0.004

# ③ 단계 테스트용 — 여기 1 또는 2를 넣으면 ROS 구독 없이 그 색으로 한 번 동작한다.
# 확인이 끝나면 다시 None 으로 돌려 ④ 처럼 /color_id 구독을 쓴다.
FORCE_COLOR_ID = None


# ══════════════════════════════════════════════════════════════
#  목표 높이  (6단계와 동일)
# ══════════════════════════════════════════════════════════════
PICK_Z          = 0.05
PLACE_Z         = 0.055
APPROACH_HEIGHT = 0.25
LIFT_HEIGHT     = 0.23

GRIPPER_WAIT = 120

# IK 가 이만큼 연속으로 안 풀리면 그 자리는 못 가는 곳으로 보고 포기한다.
# 로봇 밑동에 너무 가까운 자리 등, 반경(SPEC_REACH) 안이어도 이 자세로는 못 가는 곳이 있다.
IK_FAIL_LIMIT = 90

TCP_SPEED  = 0.004
MIN_STEPS  = 60
MAX_STEPS  = 600
HOLD_STEPS = 60

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


# ══════════════════════════════════════════════════════════════
#  회전 유틸  (6단계와 동일)
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
    """각도 세 개로 목표 자세를 만든다"""
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """쿼터니언을 회전행렬로 바꾼다"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환  (6단계와 동일)
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  궤적 보간  (6단계와 동일)
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    return start + alpha * (goal - start)


class PickPlaceFSM:
    """
    Pick & Place 상태 기계. pick 좌표는 큐브의 실시간 get_world_pose(), 놓을 좌표는
    색이 고정된 마커 좌표를 생성자에서 받는다. 마지막에 관찰 자세로 돌아온다.

      0 APPROACH   큐브 위로 접근
      1 DESCEND    큐브까지 하강
      2 GRASP      그리퍼 닫기 (제자리)
      3 LIFT       들어올리기
      4 MOVE       마커 위로 이동
      5 LOWER      마커 높이까지 하강
      6 RELEASE    그리퍼 열기 (제자리)
      7 RISE       마커 위로 다시 상승 (복귀 중 충돌 방지)
      8 RETURN     관찰 자세로 복귀
      9 DONE
    """

    NAMES = ["APPROACH", "DESCEND", "GRASP", "LIFT",
             "MOVE", "LOWER", "RELEASE", "RISE", "RETURN", "DONE"]
    GRIPPER_STATES = {2: "close", 6: "open"}
    DONE_STATE = 9

    def __init__(self, robot, pick_xy, place_xy, observe_tcp):
        self._robot = robot
        self._build_waypoints(pick_xy, place_xy, observe_tcp)
        self.reset()

    def _build_waypoints(self, pick_xy, place_xy, observe_tcp):
        px, py = pick_xy
        gx, gy = place_xy
        self.waypoints = [
            np.array([px, py, APPROACH_HEIGHT]),   # 0 APPROACH
            np.array([px, py, PICK_Z]),            # 1 DESCEND
            np.array([px, py, PICK_Z]),            # 2 GRASP
            np.array([px, py, LIFT_HEIGHT]),       # 3 LIFT
            np.array([gx, gy, LIFT_HEIGHT]),       # 4 MOVE
            np.array([gx, gy, PLACE_Z]),           # 5 LOWER
            np.array([gx, gy, PLACE_Z]),           # 6 RELEASE
            np.array([gx, gy, LIFT_HEIGHT]),       # 7 RISE
            np.array(observe_tcp),                 # 8 RETURN
        ]

    def reset(self):
        self.state = 0
        self.step = 0
        self.start = None
        self.goal = self.waypoints[0]
        self.n_steps = MIN_STEPS
        self.gripper = "open"

    def current_target(self):
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        if self.state >= self.DONE_STATE:
            return

        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            self.gripper = self.GRIPPER_STATES.get(self.state, self.gripper)

            if self.state in self.GRIPPER_STATES:
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
#  USD 마커 생성 / 랜덤 위치 뽑기
# ══════════════════════════════════════════════════════════════
def set_prim_xy(prim, x, y, z):
    """새 prim 에 translate op 를 추가한다 (마커 전용 — 기존 op 가 없는 prim)"""
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(z)))


def sample_random_xy(avoid_xy, x_range, y_range, min_sep, tries=50):
    """avoid_xy 와 min_sep 이상 떨어진 점을 뽑는다"""
    xy = None
    for _ in range(tries):
        xy = np.array([random.uniform(*x_range), random.uniform(*y_range)])
        if np.linalg.norm(xy - avoid_xy) >= min_sep:
            return xy
    return xy   # tries 를 다 써도 못 찾으면 마지막 값을 쓴다


# ══════════════════════════════════════════════════════════════
#  ROS2 — /color_id 구독 그래프 (OmniGraph, rclpy 안 씀)
# ══════════════════════════════════════════════════════════════
COLOR_ID_GRAPH_PATH = "/World/Graph/ROS_ColorId"
COLOR_ID_TOPIC = "color_id"


def create_color_id_subscriber():
    """
    /color_id (std_msgs/Int32) 를 구독하는 OmniGraph 를 만든다.
    손목 카메라 발행 그래프(USD 에 이미 있음)와 같은 방식 — python rclpy 를 쓰지 않는다.
    """
    _, nodes, _, _ = og.Controller.edit(
        {"graph_path": COLOR_ID_GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Subscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Subscriber.inputs:topicName", COLOR_ID_TOPIC),
                ("Subscriber.inputs:messagePackage", "std_msgs"),
                ("Subscriber.inputs:messageSubfolder", "msg"),
                ("Subscriber.inputs:messageName", "Int32"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "Subscriber.inputs:execIn"),
                ("Context.outputs:context", "Subscriber.inputs:context"),
            ],
        },
    )
    print(f"   /color_id    subscriber ready ({nodes[-1].get_prim_path()})")
    return nodes[-1]


def read_color_id(subscriber_node):
    """지금까지 받은 값을 읽는다. 아직 하나도 안 왔으면 0"""
    try:
        value = og.Controller.attribute("outputs:data", subscriber_node).get()
    except Exception:
        return 0
    return int(value) if value is not None else 0


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
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


class M0609Task(BaseTask):
    """
    set_up_scene 은 BaseTask 가 정한 이름이다. World 가 이 이름으로 부른다.
    _ 로 시작하는 메서드는 우리가 나눈 것이라 이름을 바꿔도 된다.
    """

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        self.cubes = {}   # color -> SingleRigidPrim, get_world_pose() 로 실시간 위치 조회
        self.spawned_color = None   # 이번 실행에서 보이게 스폰한 색

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        stage = omni.usd.get_context().get_stage()
        self._load_usd(stage)
        self._setup_arm_drives(stage)
        self._register_robot(scene)
        self._register_cubes()
        self._spawn_one_cube(stage)
        self._disable_red_block(stage)
        self._create_markers(stage)
        print("   scene        ready")

    def _load_usd(self, stage):
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

        print("   USD          loaded")

    def _setup_arm_drives(self, stage):
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

        gripper = ParallelGripper(
            end_effector_prim_path=ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )

        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=ee_path,
                gripper=gripper,
            )
        )
        print(f"   EE frame     {ee_path}")

    def _register_cubes(self):
        """
        색상 -> 큐브 prim 매핑. get_world_pose()/set_world_pose() 로 다룬다.

        RigidBody 물리 시뮬레이션이 이미 도는 중에 (재스폰 때) 위치를 바꿔야 하므로
        XFormPrim 이 아니라 SingleRigidPrim 을 쓴다 — 얘만 물리 텐서 API 로 실제 텔레포트한다.
        일반 XFormPrim 은 USD 속성만 바꿔서, 물리가 이미 켜진 뒤엔 다음 스텝에 원래
        시뮬레이션 값으로 덮여 버려 스폰 위치가 반영되지 않는다.
        """
        for color, path in CUBE_PRIM_PATH.items():
            self.cubes[color] = SingleRigidPrim(prim_path=path, name=f"{color}_cube")

    def _spawn_one_cube(self, stage):
        """
        파란/초록 큐브 중 하나만 랜덤 위치에 보이게 하고 나머지는 숨긴다.
        이 USD 는 둘 다 invisible 로 저장되어 있어, 고른 것만 MakeVisible() 한다.
        """
        spawned = random.choice(["blue", "green"])
        hidden = "green" if spawned == "blue" else "blue"
        self.spawned_color = spawned

        new_xy = sample_random_xy(
            DEFAULT_XY[hidden], RANDOM_X_RANGE, RANDOM_Y_RANGE, MIN_SEPARATION
        )
        self.cubes[spawned].set_world_pose(
            position=np.array([new_xy[0], new_xy[1], CUBE_Z])
        )

        UsdGeom.Imageable(stage.GetPrimAtPath(CUBE_PRIM_PATH[spawned])).MakeVisible()
        UsdGeom.Imageable(stage.GetPrimAtPath(CUBE_PRIM_PATH[hidden])).MakeInvisible()

        print(f"   spawned cube {spawned:5s} -> {vec(new_xy)}   ({hidden} hidden)")

    def _disable_red_block(self, stage):
        """빨간 큐브는 이번 단계에서 쓰지 않으므로 비활성화한다"""
        prim = stage.GetPrimAtPath(RED_BLOCK_PATH)
        if prim.IsValid():
            prim.SetActive(False)
            print(f"   red_block    disabled")

    def _create_markers(self, stage):
        """놓을 자리를 눈으로 보이는 원판으로 표시한다"""
        for color, xy in MARKER_XY.items():
            path = f"/World/markers/{color}_marker"
            marker = UsdGeom.Cylinder.Define(stage, path)
            marker.GetRadiusAttr().Set(MARKER_RADIUS)
            marker.GetHeightAttr().Set(MARKER_HEIGHT)
            marker.GetDisplayColorAttr().Set([Gf.Vec3f(*MARKER_RGB[color])])
            set_prim_xy(marker.GetPrim(), xy[0], xy[1], MARKER_HEIGHT / 2.0)
        print(f"   markers      blue {vec(MARKER_XY['blue'])}"
              f"   green {vec(MARKER_XY['green'])}")

    @property
    def robot(self):
        return self._robot


def init_gripper(robot, world):
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def init_cubes(task):
    """큐브도 그리퍼처럼 물리 뷰가 준비된 뒤 초기화해야 set_world_pose() 가 텔레포트로 먹힌다"""
    for cube in task.cubes.values():
        cube.initialize()


def despawn_and_respawn(task, color_name):
    """다루던 큐브는 숨기고, 둘 중 하나를 새 랜덤 위치에 다시 스폰한다"""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Imageable(stage.GetPrimAtPath(CUBE_PRIM_PATH[color_name])).MakeInvisible()
    task._spawn_one_cube(stage)


def set_observe_pose(robot):
    """관찰 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(OBSERVE_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  IK 솔버  (6단계와 동일)
# ══════════════════════════════════════════════════════════════
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


def print_dof_info(robot):
    section("DOF")
    for i, name in enumerate(robot.dof_names):
        tag = "arm" if name in ARM_JOINTS else "gripper"
        print(f"   [{i:2d}] {name:28s} {tag}")
    print()
    print(f"   finger index {robot.get_dof_index('finger_joint')}")
    print(f"   num_dof      {robot.num_dof}")


def print_status(robot, solved, fsm, target_tcp, color_name):
    name = fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]
    tag = f"{color_name:5s} {name:9s}"

    if not solved:
        print(f"   {tag} IK FAILED  target {vec(target_tcp)}")
        return

    tcp = get_tcp_pose(robot)
    finger = robot.get_joint_positions()[robot.get_dof_index("finger_joint")]
    print(f"   {tag} tcp {vec(tcp)}   finger {finger:+.4f}")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = M0609Task(name="m0609_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    init_cubes(task)
    set_observe_pose(robot)
    for _ in range(30):
        world.step(render=True)

    print_dof_info(robot)

    section("SOLVER")
    ik_solver = create_ik_solver(robot)
    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    observe_tcp = get_tcp_pose(robot)   # 관찰 자세의 실제 TCP 위치 — RETURN 목표로 쓴다

    section("ROS2")
    color_sub = create_color_id_subscriber()
    last_color_id = read_color_id(color_sub)   # 시작 시점 값으로 기준을 잡는다 (보통 0)

    section("RUN")
    print("   press Play in the viewport")
    print(f"   observe tcp  {vec(observe_tcp)}")
    print("   waiting for /color_id  (1=blue, 2=green)")
    print("   test: ros2 topic pub --once /color_id"
          " std_msgs/msg/Int32 \"{data: 1}\"\n")

    fsm = None
    color_name = None
    forced_used = False
    was_playing = False
    step = 0
    ik_fail_count = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            init_gripper(robot, world)
            init_cubes(task)
            set_observe_pose(robot)
            last_color_id = read_color_id(color_sub)
            fsm = None
            color_name = None
            forced_used = False
            step = 0
            ik_fail_count = 0
            print()

        if is_playing:
            if fsm is None:
                # 관찰 자세 유지 — IK 로 계속 붙잡아 둔다
                flange_target = tcp_to_flange(observe_tcp, target_quat)
                action, solved = ik_solver.compute_inverse_kinematics(
                    target_position=flange_target, target_orientation=target_quat
                )
                if solved:
                    robot.apply_action(action)
                robot.apply_action(robot.gripper.forward(action="open"))

                # ③ 단계 테스트 — FORCE_COLOR_ID 가 있으면 한 번만 그 색으로 강제 실행
                if FORCE_COLOR_ID is not None and not forced_used:
                    color_name = COLOR_ID_TO_NAME.get(FORCE_COLOR_ID)
                    forced_used = True
                else:
                    # 관찰 자세일 때만 값을 읽는다. 값이 "바뀌었을 때"만 새 명령으로 본다
                    color_name = None
                    current_id = read_color_id(color_sub)
                    if current_id != last_color_id and current_id in COLOR_ID_TO_NAME:
                        color_name = COLOR_ID_TO_NAME[current_id]
                        last_color_id = current_id

                # 실제로 보이게 스폰된 색이 아니면 무시한다 — 안 그러면 숨겨진(투명한)
                # 큐브의 기본 위치로 집으러 가버린다
                if color_name is not None and color_name != task.spawned_color:
                    print(f"   IGNORE       color_id={color_name}"
                          f"   (spawned={task.spawned_color}, not visible)")
                    color_name = None

                if color_name is not None:
                    cube_pos, _ = task.cubes[color_name].get_world_pose()
                    pick_xy = cube_pos[:2]
                    place_xy = MARKER_XY[color_name]
                    print(f"   target       {color_name}"
                          f"   pick {vec(pick_xy)} -> place {vec(place_xy)}")
                    fsm = PickPlaceFSM(robot, pick_xy, place_xy, observe_tcp)
                elif step % LOG_INTERVAL == 0:
                    print("   OBSERVE      waiting for /color_id")
            else:
                target_tcp = fsm.current_target()
                flange_target = tcp_to_flange(target_tcp, target_quat)

                action, solved = ik_solver.compute_inverse_kinematics(
                    target_position=flange_target,
                    target_orientation=target_quat,
                )
                if solved:
                    robot.apply_action(action)
                    ik_fail_count = 0
                else:
                    ik_fail_count += 1

                robot.apply_action(robot.gripper.forward(action=fsm.gripper))

                if step % LOG_INTERVAL == 0:
                    print_status(robot, solved, fsm, target_tcp, color_name)

                if ik_fail_count >= IK_FAIL_LIMIT:
                    # 반경 안이어도 이 자세로는 IK 가 안 풀리는 자리 — 포기하고 새로 스폰한다
                    print(f"   UNREACHABLE  {color_name} at {vec(target_tcp)}"
                          f"   — despawn & respawn")
                    despawn_and_respawn(task, color_name)
                    fsm = None
                    color_name = None
                    ik_fail_count = 0
                    last_color_id = read_color_id(color_sub)
                else:
                    fsm.advance()
                    if fsm.state >= fsm.DONE_STATE:
                        # 놓은 큐브는 치우고, 둘 중 하나를 새 랜덤 위치에 다시 스폰한다
                        despawn_and_respawn(task, color_name)
                        fsm = None
                        color_name = None
                        # 작업 중에 들어온 값은 여기서 흡수해 버린다 — 무시하고 새 기준으로 삼는다
                        last_color_id = read_color_id(color_sub)

            step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()

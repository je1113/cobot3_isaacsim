"""
Isaac Sim 쪽 실행 백엔드 — 진짜 rclpy 노드들이 이 파일을 로컬 소켓으로 부린다.

왜 이런 구조인가:
  Isaac 5.1 의 kit 파이썬은 3.11 인데 이 서버의 ROS2 Jazzy(apt)는 3.12 용
  바이너리만 있다. Isaac 이 자체 rclpy(3.11 용)를 번들하긴 하지만
  ament_cmake/rosidl 빌드 툴체인은 없어서, 우리가 만든 cobot3_interfaces
  를 그 환경에 맞게 다시 빌드할 수 없다(확인함). 그래서 isaac_python 안에서
  직접 rclpy 노드를 못 띄운다.

  대신 이 프로세스(isaac_python)는 지금까지 검증한 FSM/비전 코드를 그대로
  들고 있고, 로컬 JSON-RPC 서버 하나만 연다. 진짜 ROS2 노드(시스템 python
  3.12, cobot3_interfaces)는 이 소켓을 통해서만 명령을 보낸다.

  실물 이관 시: pick_place_server/nav_server 는 이 파일 대신 실제 로봇
  드라이버(액션·서비스)를 부르게만 바꾸면 된다. ROS 쪽 노드는 안 바뀐다.

프로토콜: TCP, 줄 단위 JSON. 요청 {"id":.., "method":.., "params":{..}}
         응답 {"id":.., "result":..} 또는 {"id":.., "error":..}

실행:
    isaac_python isaacpjt/ros_bridge/sim_backend.py
    SIM_BACKEND_PORT=8765 isaac_python isaacpjt/ros_bridge/sim_backend.py

머신이 둘일 때 (ROS 는 일반 PC, Isaac 은 GPU PC):
    GPU PC   isaac_python isaacpjt/ros_bridge/sim_backend.py     # 0.0.0.0 에 바인드한다
             hostname -I                                        # 이 IP 를
    ROS PC   export SIM_BACKEND_HOST=<그 IP>                      # 여기에 준다
             nc -vz <그 IP> 8765                                 # succeeded 면 연결 OK
"""

import json
import math
import os
import queue
import signal
import socket
import sys
import threading
import time
from pathlib import Path

from isaacsim import SimulationApp

HEADLESS = os.environ.get("SIM_HEADLESS", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

# 씬(simple_factory_layout.usda)에 이미 OmniGraph 로 박혀 있는 ROS2 브릿지
# 노드들(ROS2PublishClock, nova_carter1/2 의 odom·lidar 퍼블리셔)은 이 확장이
# 꺼져 있으면 그냥 안 돈다 — SimulationApp 기본 구성에는 안 들어 있다. 이
# 백엔드는 원래 JSON-RPC(팔·그리퍼·카메라)만 썼어서 필요 없었는데, Nav2 가
# /clock·/robot1/chassis/odom·/robot1/front_3d_lidar/lidar_points 를 그
# 노드들에서 받아야 해서 필요해졌다. LD_LIBRARY_PATH(isaac_ros 함수)는
# 라이브러리를 "찾을 수 있게" 만들 뿐, 확장을 "켜는" 건 아니다 — 둘 다 필요하다.
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
# ★ 다른 확장(특히 isaacsim.ros2.bridge 의뢰존성 트리에 걸려 나중에 자동으로
# 끌려오는 omni.graph.image.core 등)보다 먼저 켠다. 그렇게 안 하면 로딩 순서에
# 따라 omni.graph.core 가 "Found duplicate of category 'Replicator' - was
# 'Annotators', adding 'Fabric Reader'" / "Category 'Replicator' not accepted
# on node type 'omni.replicator.core.FabricReader'" 경고를 내며 카테고리
# 등록이 꼬인다(실측: sim_backend.py 콘솔에서 매번 재현). 이 카테고리 등록이
# 꼬인 상태에서 이어지는 stage 로딩이 omni.graph.image.core.plugin.so 안에서
# 세그폴트로 죽거나(REACHABILITY 재현됨), 죽지 않고 넘어가더라도 이후
# rep.create.render_product() 로 새로 만드는 render_product 의 rgb annotator
# 가 계속 빈 프레임(shape=(0,))만 주는 것으로 보인다 — observe_pose/scan_qr/
# debug_capture 전부, 심지어 이미 잘 동작하던 front_hawk 카메라로 대조군을
# 만들어도 똑같이 재현됐다. omni.replicator.core 를 여기서 제일 먼저 등록해
# 그 확장이 자기 카테고리를 스스로 정상 선점하게 만들어 경합을 피해본다.
enable_extension("omni.replicator.core")
enable_extension("isaacsim.ros2.bridge")
# SurfaceGripper 는 USD 프림이 아니라 이 익스텐션이 등록하는 OmniGraph 노드
# 타입(isaacsim.robot.surface_gripper.SurfaceGripper)이다 — 스테이지를 열기
# 전에 켜두지 않으면 short_gripper payload 가 로드돼도 OmniGraph 가 그 노드
# 타입을 몰라서 인스턴스화하지 못하고, configure_gripper_limits() 가
# "SurfaceGripper node not found" 로 죽는다(재시도 프레임을 늘려도 안 됨 —
# 익스텐션이 그 전에는 아예 등록을 안 하기 때문).
enable_extension("isaacsim.robot.surface_gripper")

import ctypes

import numpy as np
import omni.usd
import yaml
from pxr import Usd, UsdGeom, UsdPhysics


def _pycapsule_to_bytes(capsule, size):
    """omni.kit.renderer_capture 의 *_callback 계열이 buffer 로 주는 건
    실제 바이트가 아니라 PyCapsule(C 포인터 래퍼)이다 — np.frombuffer 에
    바로 못 넣는다(실측: "TypeError: a bytes-like object is required, not
    'PyCapsule'"). ctypes 의 PyCapsule C-API 로 직접 포인터를 꺼내 읽는다.
    이름을 미리 알 필요는 없다 — PyCapsule_GetName 으로 그 캡슐이 실제로
    갖고 있는 이름을 먼저 읽어서 그대로 PyCapsule_GetPointer 에 되돌려준다
    (PyCapsule_GetPointer 는 이름이 정확히 일치해야만 포인터를 내준다)."""
    ctypes.pythonapi.PyCapsule_GetName.restype = ctypes.c_char_p
    ctypes.pythonapi.PyCapsule_GetName.argtypes = [ctypes.py_object]
    name = ctypes.pythonapi.PyCapsule_GetName(capsule)
    ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.c_void_p
    ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    ptr = ctypes.pythonapi.PyCapsule_GetPointer(capsule, name)
    if not ptr:
        raise RuntimeError("PyCapsule 에서 포인터를 못 가져왔다")
    return bytes((ctypes.c_uint8 * size).from_address(ptr))

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver, ArticulationKinematicsSolver,
)

sys.stdout.reconfigure(line_buffering=True)

THIS_DIR = Path(__file__).resolve().parent
ISAACPJT = THIS_DIR.parent
WS_ROOT = ISAACPJT.parent

sys.path.insert(0, str(WS_ROOT / "src/cobot3_perception"))
from cobot3_perception.qr_pose import estimate_qr_pose, aggregate_qr_poses  # noqa: E402
from cobot3_perception.flange_topview import detect_flange  # noqa: E402

# 어느 씬을 열지. 기본은 simple_factory_layout.usda 이고, 순찰 없는 고정
# 시나리오(task_manager scenario:=static_test)는 매거진 둘·스택 하나만 놓인
#     SIM_WORLD_USD=isaacpjt/worlds/simple_factory_layout_test.usda
# 로 띄운다. 두 씬은 prim 경로가 같다(nova_carter1, magazine_1_orange 등) —
# 아래 하드코딩된 경로가 그대로 맞아야 하므로 씬을 새로 만들 때 이름을 지켜라.
WORLD_USD = os.environ.get("SIM_WORLD_USD") or str(ISAACPJT / "worlds/simple_factory_layout.usda")
URDF_PATH = str(ISAACPJT / "M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESC_PATH = str(ISAACPJT / "M0609/descriptor/m0609_description.yaml")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
GRASP_YAML = WS_ROOT / "src/cobot3_bringup/config/grasp.yaml"
CARRIERS_YAML = WS_ROOT / "src/cobot3_bringup/config/carriers.yaml"
MEASURED = ISAACPJT / "tools/out/layout_measured.yaml"

# ★ 정지(SIGINT/SIGTERM) 시 save_state() 가 쓰고, 다음 기동에서 __init__ 의
#   _restore_state() 가 읽는다. 소스 관리 대상이 아니라(.gitignore) 세션마다
#   갈아치우는 런타임 파일이다 — 씬을 SIM_WORLD_USD 로 바꿔도 경로가 안
#   섞이게, 그 파일 이름을 스냅샷 파일명에 넣는다.
STATE_SNAPSHOT_PATH = ISAACPJT / f"ros_bridge/.state_snapshot.{Path(WORLD_USD).stem}.json"

EE_LINK_NAME = "link_6"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# 로봇 두 대 — simple_factory_layout.usda 에 nova_carter1/nova_carter2 가
# 완전히 같은 구조(m0609 팔 + short_gripper + rsd455)로 미러링돼 있다
# (isaac:namespace = "robot1"/"robot2"). carter prim 이름만 바꾸면
# 그대로 재사용된다 — ROBOT_CARTER_NAME 이 그 매핑이다.
ROBOT_CARTER_NAME = {"robot1": "nova_carter1", "robot2": "nova_carter2"}
DEFAULT_ROBOT_ID = "robot1"   # robot_id 를 안 주는 옛 호출(단일 로봇 시절)의 폴백


def _robot_paths(carter_name):
    base_xform = f"/World/Robots/{carter_name}"
    robot_prim = f"{base_xform}/m0609"
    gripper_prim = f"{robot_prim}/short_gripper"
    return {
        "base_xform": base_xform,
        "robot_prim": robot_prim,
        "base_link": f"{robot_prim}/base_link",       # m0609 자체 base — Lula IK 전용
        "chassis_link": f"{base_xform}/chassis_link",  # ROS 규약의 base_link(AMR 섀시).
                                                        # CarrierScan/PickCarrier 의
                                                        # frame_id="base_link" 는 이쪽이다 —
                                                        # 06 §1 TF 트리: map -> base_link(Carter)
                                                        # -> m0609_base -> tool0. 헷갈리지 말 것.
        "ee_link": f"{robot_prim}/{EE_LINK_NAME}",
        "gripper_prim": gripper_prim,
        "camera_prim": f"{gripper_prim}/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
        "articulation_root_candidates": [f"{base_xform}/chassis_link", base_xform],
    }


# robot1 기본값 — 리팩터 전부터 있던 모듈 상수 이름을 그대로 유지한다
# (debug_* 메서드 기본 인자·주석에서 여전히 참조한다). 실제 RPC 경로는
# 로봇별로 Backend.rigs[robot_id](RobotRig)를 쓴다.
_ROBOT1_PATHS = _robot_paths(ROBOT_CARTER_NAME["robot1"])
ROBOT_PRIM_PATH = _ROBOT1_PATHS["robot_prim"]
BASE_XFORM_PATH = _ROBOT1_PATHS["base_xform"]
BASE_LINK_PATH = _ROBOT1_PATHS["base_link"]
CHASSIS_LINK_PATH = _ROBOT1_PATHS["chassis_link"]
EE_LINK_PATH = _ROBOT1_PATHS["ee_link"]
GRIPPER_PRIM = _ROBOT1_PATHS["gripper_prim"]
CAMERA_PRIM = _ROBOT1_PATHS["camera_prim"]
ARTICULATION_ROOT_CANDIDATES = _ROBOT1_PATHS["articulation_root_candidates"]

CARTER_Z = 0.07963398335074101
RESOLUTION = (1280, 720)
# _capture_frame() 이 raw AOV 캡쳐로 요청하는 depth 채널의 실제 이름.
# Replicator 의 annotator 이름("distance_to_image_plane")과 다르다 — 실측
# 확인: omni.replicator.core.scripts.annotators 의 AnnotatorParams 테이블에
# 찍힌 raw 이름이 이거다("SD" 접미사). "DistanceToImagePlane"(SD 없이)으로
# 등록하면 aov_map 에 이름은 잡히는데 텍스처가 끝까지 (0,0) 해상도로 안 채워진다.
DEPTH_AOV_NAME = "DistanceToImagePlaneSD"
# 12_pick_test.py 는 흡착 중 견고함을 위해 1e8 을 쓰지만, 그 값으로는 관측
# 자세에서 미세 진동이 남아 QR 디코드가 깨졌다(실측 확인). eval_qr_pose_depth.py
# 가 검증한 값(1e5)으로 낮췄다 — SCAN 도 PICK 도 이 값 하나로 돌려 봤더니,
# 흡착 견고성이 부족했다(실측: LIFT 중 rise=88mm·tilt<0.1도로 정상 상승했는데도
# HOLD_WAIT 끝에 gripped=False — SLIP). 위 주석이 예고한 대로 phase 별로
# 나눈다 — observe_pose() 는 QR 을 위해 이 값을 쓰고, pick_phase1_approach()
# 는 12_pick_test.py 검증값(DRIVE_STIFFNESS_PICK)으로 다시 올린다.
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e5, 1e4, 2700.0
DRIVE_STIFFNESS_PICK = 1e8
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
# 부팅 직후 팔의 초기 자세. 예전엔 READY_JOINTS_DEG(위, STOW 이송 자세와
# 같은 값)로 세웠는데, task_manager.py 의 START_DETECTED_FOR_TEST 때문에
# 노드가 뜨자마자 SCAN 이 바로 도는 지금 배선에서는 그 사이 "팔이 아직
# READY 자세인데 SCAN 은 이미 관측 자세인 줄 알고 진행" 하는 과도기가
# 혼선을 줬다(실측: PICK 위치 오차/QR 미인식 재현 이력). 아예 부팅 시점
# 부터 SCAN 관측 자세로 세운다 — simple_factory_layout.usda 의 nova_carter1
# 팔 관절 초기값도 이 자세로 맞춰 놨다(둘 다 일치해야 한다: USD 쪽 값은
# Backend.__init__ 이 아래에서 다시 명시적으로 덮어쓰므로 실제 동작을
# 좌우하는 건 이 상수 쪽이고, USD 값은 "씬만 열었을 때"도 같은 자세로
# 보이게 하기 위한 것).
BOOT_POSE_NAME = "shelf_1_top_close_centered"

# 12_pick_test.py / grasp.yaml 검증값 — 새로 지어내지 않는다.
SUCTION_FACE_Z = 0.161
TCP_OFFSET = np.array([0.0, 0.0, SUCTION_FACE_Z])
APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG = 180.0, 0.0, 0.0
GRIP_WAIT, HOLD_WAIT, SETTLE_STEPS, BASE_MOVE_SETTLE_STEPS = 90, 120, 90, 60
TCP_SPEED, MIN_STEPS, MAX_STEPS = 0.002, 90, 600
MAX_IK_FAIL_STEPS = 30
LIFT_OK_MIN_M, TILT_MAX_DEG = 0.005, 5.0
LIFT_HEIGHT_OFFSET = 0.10

MAGAZINE_XFORM_PATH = "/World/Magazines/shelf_1_magaines/top_magazines/magazine_1_orange"
FLANGE_PATH = f"{MAGAZINE_XFORM_PATH}/flange_plate"
CONVEYOR_FRAME_PATH = "/World/Environment/PackagingZone/ConveyorFrame"

# 12_place_test.py 검증값 — ConveyorFrame(x>=4.2) 바로 앞. pkg_loader 슬롯
# 자체(포트 지오메트리)는 아직 씬에 없어서, 이 지점을 그대로 place 목표로
# 쓴다 (다음 범위: 실제 로더 슬롯).
# ★ 원래 바닥(z=0, FLOOR_Z)에 내려놓게 돼 있었는데, 팔 마운트 높이(~0.63m)
# 에서 거의 전체를 아래로 뻗어야 해서 IK 한계 근처였다 — 실측 재현: PLACE
# 가 NO_IK 로 멈췄다(task_manager.py QR/PICK 디버깅 이력 이어서 발견).
# 12_place_test.py 가 이미 벨트 높이(CONVEYOR_BELT_Z)로 바꿔서 검증해
# 뒀는데 이 서빙 코드(sim_backend.py)에는 그 수정이 반영이 안 돼 있었다 —
# 그대로 옮겨온다. PLACE_TARGET_XY 도 4.0 → 4.1 로 같이 맞춘다(그쪽 값이
# 실측 스윕으로 재검증된 값).
PLACE_TARGET_XY = np.array([4.1, 0.0])
CONVEYOR_BELT_Z = 0.6
PLACE_APPROACH_HEIGHT_OFFSET = 0.15
PLACE_DROP = 0.005
RELEASE_WAIT = 90


# ══════════════════════════════════════════════════════════════
#  기하 유틸 (12_pick_test.py 와 동일)
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([
        w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def quat_from_axis(axis, deg):
    half = np.radians(deg)/2.0
    a = np.array(axis, dtype=float)
    return np.concatenate([[np.cos(half)], (a/np.linalg.norm(a))*np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    q = quat_mul(quat_from_axis([1,0,0], roll_deg), quat_from_axis([0,1,0], pitch_deg))
    return quat_mul(q, quat_from_axis([0,0,1], yaw_deg)) / np.linalg.norm(
        quat_mul(q, quat_from_axis([0,0,1], yaw_deg)))


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x,y,z,w]); x,y,z,w = x/n,y/n,z/n,w/n
    return quat_to_matrix([w,x,y,z])


def yaw_of_quat(quat_wxyz):
    R = quat_to_matrix(quat_wxyz)
    return math.atan2(R[1, 0], R[0, 0])


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose_from_ee(pos, quat):
    return np.asarray(pos) + quat_to_matrix(quat) @ TCP_OFFSET


def ease(alpha):
    a = float(np.clip(alpha, 0.0, 1.0))
    return a*a*(3.0-2.0*a)


def steps_for(start, goal):
    dist = float(np.linalg.norm(goal-start))
    return int(np.clip(dist/TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def tilt_deg_from_quat(quat_wxyz):
    R = quat_to_matrix(quat_wxyz)
    up = R @ np.array([0.0,0.0,1.0])
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def get_world_pose(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    imag = q.GetImaginary()
    return np.array([t[0],t[1],t[2]]), np.array([q.GetReal(), imag[0], imag[1], imag[2]])


def measure_prim(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    return np.array([(lo[0]+hi[0])/2.0, (lo[1]+hi[1])/2.0]), float(hi[2]), float(hi[2]-lo[2])


def resolve_articulation_root(candidates, fallback):
    stage = omni.usd.get_context().get_stage()
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    return fallback


def configure_drives(stage, robot_prim_path):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim_path)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        for dt in ["angular", "linear"]:
            d = UsdPhysics.DriveAPI.Get(prim, dt)
            if d:
                d.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                d.GetDampingAttr().Set(DRIVE_DAMPING)
                d.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)


def filter_collision(a, b):
    stage = omni.usd.get_context().get_stage()
    from pxr import Sdf
    rel = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel()
    rel.AddTarget(Sdf.Path(b))


def find_prim_path(root_path, name):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def wait_for_stage_load(ctx, min_frames=60, max_frames=600):
    """open_stage()/Load() 뒤에 고정 프레임만 돌리면 외부 payload(short_gripper 등)가
    아직 안 붙은 상태에서 다음 단계로 넘어갈 수 있다 — SurfaceGripper not found 로
    재현됨. get_stage_loading_status()[2](대기 중인 로드 개수)가 0이 될 때까지
    돈다. min_frames 는 상태가 바로 0으로 보고되는 첫 프레임들을 건너뛰기 위한
    최소 대기."""
    for _ in range(min_frames):
        simulation_app.update()
    for _ in range(max_frames - min_frames):
        if ctx.get_stage_loading_status()[2] == 0:
            break
        simulation_app.update()


# 12_pick_test.py 검증값. 에셋 기본값(coaxial/shear=0)으로 두면 아무것도 못
# 든다 — frames.yaml suction_gripper.asset_defaults 주석 참고.
COAXIAL_FORCE_LIMIT = 200.0
SHEAR_FORCE_LIMIT = 100.0
MAX_GRIP_DISTANCE = 0.03


def configure_gripper_limits(stage, gripper_prim, max_wait_frames=180):
    """씬에 붙어 있는 SurfaceGripper 노드의 한계값을 12_pick_test.py 값으로
    덮어쓴다. 이걸 빼먹으면 위치가 완벽해도 흡착이 전혀 안 붙는다 — 실제로
    한 번 이 실수를 했다(NO_ATTACH 4/4, GT 좌표로 줘도 재현됨).

    반환값(SurfaceGripper 노드 경로 자체)이 중요하다 — SurfaceGripperCtl 은
    부모 prim(short_gripper)이 아니라 이 노드 경로를 받아야 한다. 처음에
    부모 경로를 넘겼다가 또 한 번 NO_ATTACH 를 재현했다.

    short_gripper 는 외부 payload(omniverse-content-production S3)라서
    stage.Load() 가 "끝났다"고 리턴한 뒤에도 실제 프림이 몇 프레임 늦게
    붙는 경우가 있었다(RuntimeError: SurfaceGripper node not found 로 재현됨).
    바로 죽이지 말고 max_wait_frames 만큼 재시도한다.
    """
    root = stage.GetPrimAtPath(gripper_prim)
    if root.IsValid() and root.HasPayload() and not root.IsLoaded():
        print(f"   !! {gripper_prim} payload 가 unloaded 상태 — root.Load() 직접 호출")
        root.Load()
        simulation_app.update()

    node_path = find_prim_path(gripper_prim, "SurfaceGripper")
    waited = 0
    while node_path is None and waited < max_wait_frames:
        simulation_app.update()
        waited += 1
        node_path = find_prim_path(gripper_prim, "SurfaceGripper")
    if node_path is None:
        root = stage.GetPrimAtPath(gripper_prim)
        if not root.IsValid():
            diag = f"{gripper_prim} 프림 자체가 없음 (root invalid)"
        else:
            names = [str(p.GetPath()) for p in Usd.PrimRange(root)]
            diag = (
                f"{gripper_prim} 은 있음: hasPayload={root.HasPayload()} "
                f"isLoaded={root.IsLoaded()} isActive={root.IsActive()} "
                f"loadRules={stage.GetLoadRules()} "
                f"하위 프림 {len(names) - 1}개: {names[1:]}"
            )
        raise RuntimeError(
            f"SurfaceGripper node not found under {gripper_prim} "
            f"(waited {waited} extra frames). {diag}"
        )
    node = stage.GetPrimAtPath(node_path)
    node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
    node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
    node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)
    print(f"   grip limits  {node_path}  coaxial {COAXIAL_FORCE_LIMIT:.0f} N  "
          f"shear {SHEAR_FORCE_LIMIT:.0f} N  maxGripDistance {MAX_GRIP_DISTANCE*1000:.0f} mm")
    return node_path


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼
# ══════════════════════════════════════════════════════════════
class SurfaceGripperCtl:
    def __init__(self, path):
        self._path = path
        self._view = None
        self.reinit()

    def reinit(self):
        self._view = None
        try:
            from isaacsim.robot.surface_gripper import GripperView
            self._view = GripperView(paths=self._path)
        except Exception:
            pass

    def close(self):
        if self._view is not None:
            self._view.apply_gripper_action(np.array([1.0]))
        else:
            import isaacsim.robot.surface_gripper as sg
            sg.close_gripper(self._path)

    def open(self):
        if self._view is not None:
            self._view.apply_gripper_action(np.array([-1.0]))
        else:
            import isaacsim.robot.surface_gripper as sg
            sg.open_gripper(self._path)

    def gripped(self):
        try:
            if self._view is not None:
                return self._view.get_gripped_objects()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripped_objects(self._path)
        except Exception:
            return None


def _flatten_gripped_paths(gripped):
    if not gripped:
        return []
    flat = []
    for item in gripped:
        flat.extend(item) if isinstance(item, (list, tuple)) else flat.append(item)
    return [str(x) for x in flat if str(x).strip() not in ("", "None")]


def holding(gripped):
    return bool(_flatten_gripped_paths(gripped))


# ══════════════════════════════════════════════════════════════
#  RPC 서버 — 별도 스레드. 실제 Isaac API 호출은 전부 메인 루프(월드 스텝
#  스레드)에서만 한다. PhysX/USD 는 스레드 세이프하지 않다.
# ══════════════════════════════════════════════════════════════
class _Call:
    __slots__ = ("method", "params", "result", "error", "done")
    def __init__(self, method, params):
        self.method, self.params = method, params
        self.result, self.error = None, None
        self.done = threading.Event()


_inbox = queue.Queue()
_status_lock = threading.Lock()
# 로봇별로 따로 둔다 — pick_place_server 두 인스턴스(robot1/robot2 네임스페이스)
# 가 각자 _poll_until 에서 get_status 를 폴링하는데, 하나로 합쳐 두면 로봇1이
# PICK 중일 때 로봇2의 액션 feedback 에 로봇1의 phase 가 찍힌다.
_status = {robot_id: {"phase": "IDLE", "gripped": False, "gap_m": 0.0, "message": ""}
           for robot_id in ROBOT_CARTER_NAME}


def _set_status(robot_id, **kv):
    with _status_lock:
        _status[robot_id].update(kv)


def _get_status(robot_id):
    with _status_lock:
        return dict(_status[robot_id])


def _rpc_serve(host, port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(8)
    print(f"   RPC server  {host}:{port}")
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=_handle_conn, args=(conn,), daemon=True).start()


def _handle_conn(conn):
    f = conn.makefile("rwb")
    try:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except Exception as e:
                f.write((json.dumps({"error": f"bad json: {e}"}) + "\n").encode()); f.flush()
                continue
            method = req.get("method")
            if method == "get_status":         # 폴링용 — 큐를 거치지 않고 바로 답한다
                robot_id = req.get("params", {}).get("robot_id", DEFAULT_ROBOT_ID)
                resp = {"id": req.get("id"), "result": _get_status(robot_id)}
            else:
                call = _Call(method, req.get("params", {}))
                _inbox.put(call)
                ok = call.done.wait(timeout=float(req.get("timeout_s", 60.0)))
                if not ok:
                    resp = {"id": req.get("id"), "error": "timeout"}
                elif call.error is not None:
                    resp = {"id": req.get("id"), "error": call.error}
                else:
                    resp = {"id": req.get("id"), "result": call.result}
            f.write((json.dumps(resp) + "\n").encode())
            f.flush()
    except Exception as e:
        print(f"   !! RPC 연결 오류: {e}")
    finally:
        conn.close()


# ══════════════════════════════════════════════════════════════
#  로봇 한 대의 핸들 + 진행 중인 PICK/PLACE 상태
# ══════════════════════════════════════════════════════════════
class RobotRig:
    """로봇 하나(robot1/robot2)의 프림 경로·IK·그리퍼 핸들과, OBSERVE ->
    APPROACH -> FINISH 여러 RPC 호출에 걸쳐 들고 있어야 하는 중간 상태
    (current_magazine_path/flange_world/slot_world)를 담는다.

    이 상태를 Backend 인스턴스 전체에 하나만 두면(리팩터 전처럼) 안 된다 —
    로봇 두 대의 pick_place_server 가 같은 소켓(sim_backend)에 붙어서, 로봇1의
    APPROACH 호출과 로봇2의 APPROACH 호출이 큐에서 번갈아 처리될 수 있다.
    공유 상태였다면 로봇1이 APPROACH 에서 저장해 둔 flange_world 를 로봇2의
    호출이 그새 덮어써, 로봇1의 FINISH 가 엉뚱한 위치로 내려가게 된다."""

    def __init__(self, robot_id, carter_name):
        self.robot_id = robot_id
        paths = _robot_paths(carter_name)
        self.base_xform_path = paths["base_xform"]
        self.robot_prim_path = paths["robot_prim"]
        self.base_link_path = paths["base_link"]
        self.chassis_link_path = paths["chassis_link"]
        self.ee_link_path = paths["ee_link"]
        self.gripper_prim = paths["gripper_prim"]
        self.camera_prim = paths["camera_prim"]
        self.articulation_root_candidates = paths["articulation_root_candidates"]

        self.robot = None              # SingleManipulator — Backend.__init__ 이 채운다
        self.lula = None               # LulaKinematicsSolver
        self.solver = None             # ArticulationKinematicsSolver
        self.gripper_node_path = None  # SurfaceGripper OmniGraph 노드 경로
        self.gripper = None            # SurfaceGripperCtl
        self.capture_ready = False     # _ensure_camera_warm() 이 한 번 세팅하면 True
        self.pending_capture = None    # _capture_frame() 이 진행 중인 캡쳐를 GC 로부터 붙잡아두는 자리
        self.viewport = None           # 이 로봇 전담 뷰포트(ViewportAPI) — _get_or_create_viewport() 가 채운다
        self.viewport_window = None    # 로봇용으로 새로 만든 창이면 그 객체(GC 방지). 기존 활성 뷰포트를
                                        # 쓰는 로봇(보통 robot1)은 만든 창이 없으니 None 으로 둔다.

        self.current_magazine_path = MAGAZINE_XFORM_PATH  # PICK 전 기본값(레거시 메서드용)
        self.flange_world = None       # pick APPROACH -> FINISH 로 넘기는 목표
        self.slot_world = None         # place APPROACH -> FINISH
        self.place_approach_dist_m = PLACE_APPROACH_HEIGHT_OFFSET


# ══════════════════════════════════════════════════════════════
#  씬 / 로봇
# ══════════════════════════════════════════════════════════════
class Backend:
    def __init__(self):
        self.frames = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
        self.grasp = yaml.safe_load(GRASP_YAML.read_text(encoding="utf-8"))
        self.carriers = yaml.safe_load(CARRIERS_YAML.read_text(encoding="utf-8"))

        ctx = omni.usd.get_context()
        ctx.open_stage(WORLD_USD)
        wait_for_stage_load(ctx)
        self.stage = ctx.get_stage()
        # open_stage() 의 load_set 기본값(LOAD_ALL)과 무관하게, 이 앱 프로필에서는
        # short_gripper payload 가 로드 안 된 채로 남는 걸 확인했다(실패 시 진단
        # 로그가 "프림은 있음, 하위 0개" 를 찍음 — Usd.PrimRange 의 기본 predicate 는
        # unloaded 프림을 root 조차 스킵한다). 그래서 명시적으로 Load() 가 필요하다.
        self.stage.Load()
        wait_for_stage_load(ctx)

        self.rigs = {robot_id: RobotRig(robot_id, carter_name)
                     for robot_id, carter_name in ROBOT_CARTER_NAME.items()}

        for rig in self.rigs.values():
            configure_drives(self.stage, rig.robot_prim_path)
            rig.gripper_node_path = configure_gripper_limits(self.stage, rig.gripper_prim)
        # pkg_loader 정지 지점에서 _carry_gripped_object_through_teleport 가
        # 매거진을 tcp 바로 아래(ConveyorFrame 충돌체와 살짝 겹치는 위치)로
        # 순간이동시킨다 — 실측: 필터링 없이는 그 겹침을 PhysX 가 몇 m 밖으로
        # 튕겨내는 걸로 "해결"했다. 이 우회 자체가 실제 접촉을 흉내 낼 필요가
        # 없으므로 걸러낸다. 이 우회는 robot1 하드코딩 경로(teleport_base ->
        # 재흡착)에서만 쓰였고 지금 배선(실제 Nav2)에서는 아무도 안 부른다 —
        # robot2 용 필터 쌍은 그 경로를 실제로 쓰게 되면 추가한다.
        filter_collision(self.rigs["robot1"].gripper_prim, MAGAZINE_XFORM_PATH)
        filter_collision(MAGAZINE_XFORM_PATH, CONVEYOR_FRAME_PATH)

        self.world = World(stage_units_in_meters=1.0)
        for rig in self.rigs.values():
            root_path = resolve_articulation_root(
                rig.articulation_root_candidates, rig.robot_prim_path)
            rig.robot = self.world.scene.add(SingleManipulator(
                prim_path=root_path, name=f"m0609_robot_{rig.robot_id}",
                end_effector_prim_path=rig.ee_link_path))
        self.magazine = self.world.scene.add(SingleRigidPrim(
            prim_path=MAGAZINE_XFORM_PATH, name="magazine"))
        self.world.reset()
        for rig in self.rigs.values():
            rig.robot.initialize()
        # ★ 2단계 부팅 — 1) 먼저 안전하다고 검증된 READY_JOINTS_DEG 로
        # 즉시 스냅하고 충분히 세워 안정시킨다. 2) 안정된 뒤에야
        # BOOT_POSE_NAME(SCAN 관측 자세, joint_3=150° 근처로 훨씬 더
        # 뻗은 자세)로 _servo_joint_deg(부드러운 보간)로 옮긴다.
        # 순서를 바꿔서 reset() 직후 곧바로 SCAN 자세로 순간 스냅해봤더니
        # (또는 USD 의 state:angular:physics:position 자체를 그 값으로
        # 박아봤더니) 베이스가 물리 충격으로 넘어지는 게 실측 재현됐다 —
        # 이 씬에서 READY_JOINTS_DEG 는 오래 써 온 안전한 시작 자세라
        # 그대로 두고, 거기서 SCAN 자세까지는 반드시 부드럽게 옮긴다.
        # 로봇 두 대 모두 같은 절차를 거친다.
        for rig in self.rigs.values():
            q = np.zeros(rig.robot.num_dof)
            for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
                q[rig.robot.get_dof_index(name)] = np.deg2rad(deg)
            rig.robot.set_joint_positions(q)
        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)

        taught = yaml.safe_load((ISAACPJT / "tools/out/taught_poses.yaml").read_text(encoding="utf-8"))
        for rig in self.rigs.values():
            self._servo_joint_deg(rig.robot_id, taught[BOOT_POSE_NAME]["joints_deg"],
                                  n_steps=SETTLE_STEPS)

        self.magazine_spawn_pos, self.magazine_spawn_quat = self.magazine.get_world_pose()

        # ★ PICK 판정(rise/tilt)이 "지금 실제로 집은 매거진"이 아니라 항상
        # MAGAZINE_XFORM_PATH(magazine_1_orange) 하나만 쟀던 버그의 수정.
        # 이 씬은 magazine_2_blue 같은 variant 가 선반마다 여러 인스턴스로
        # 있어서(layout_measured.yaml 참고 — shelf_1/2 x top/bottom x
        # orange/blue x 2개씩, 총 16개) 이름만으로는 "지금 집은 그것"을 못
        # 가른다. QR 은 종류만 담아서 인스턴스 ID 도 없다(task_manager.py
        # 상단 "알려진 갭" 참고). 그래서 이름이 아니라 위치로 가른다 —
        # pick_phase1_approach 가 이미 아는 실제 목표 flange_world 좌표에
        # 가장 가까운 인스턴스를 찾는다. 그 후보 목록을 여기서 한 번만
        # 읽어둔다(매 PICK 마다 yaml 다시 읽을 필요 없음). 두 로봇이
        # 같은 매거진 후보 목록을 공유한다 — 어느 선반 것이든 좌표로 가른다.
        meas_layout = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
        self._all_magazine_prims = [m["prim"] for m in meas_layout["magazines"].values()]
        # ★ yaml 목록은 기본 씬의 매거진 16 개다. 다른 씬(SIM_WORLD_USD)이나
        #   스택(F3_STK*, 플랜지가 있는 캐리어면 전부)도 같은 방식으로 "실제로
        #   집은 그것" 을 찾을 수 있게, 스테이지에서 flange_plate 자식을 가진
        #   prim 을 전부 후보에 더한다. 없는 prim 은 _find_nearest_magazine 이
        #   measure_prim 예외로 건너뛰므로 yaml 쪽 목록은 그대로 둔다.
        try:
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath("/World")):
                if prim.GetChild("flange_plate").IsValid():
                    path = str(prim.GetPath())
                    if path not in self._all_magazine_prims:
                        self._all_magazine_prims.append(path)
        except Exception as e:      # noqa: BLE001 — 후보 탐색 실패는 치명적이지 않다
            print(f"   !! 플랜지 후보 탐색 실패({e}) — layout_measured.yaml 목록만 쓴다")
        print(f"   플랜지 후보 {len(self._all_magazine_prims)} 개")
        # ★ rig.current_magazine_path 기본값은 RobotRig.__init__ 이 이미
        #   MAGAZINE_XFORM_PATH 로 세워 둔다(로봇별로 갖는 상태라 여기서
        #   전역으로 다시 세울 self._current_magazine_path 는 없다).

        for rig in self.rigs.values():
            base_pos0, base_quat0 = get_world_pose(rig.base_link_path)
            rig.lula = LulaKinematicsSolver(robot_description_path=DESC_PATH, urdf_path=URDF_PATH)
            rig.lula.set_robot_base_pose(robot_position=base_pos0, robot_orientation=base_quat0)
            rig.solver = ArticulationKinematicsSolver(
                robot_articulation=rig.robot, kinematics_solver=rig.lula,
                end_effector_frame_name=EE_LINK_NAME)
            rig.gripper = SurfaceGripperCtl(rig.gripper_node_path)
        # ★ 공용 기본값. 실제 IK 에는 로봇별 _target_quat(robot_id) 를 쓴다 —
        #   이 값은 차체 yaw 0 인 경우와 같고, 남겨 두는 건 참고용이다.
        self.target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)

        # 이전 종료 시점의 로봇/씬 상태가 있으면 위 기본 스폰 배치를 덮어쓴다.
        self._restore_state()

        st = self.frames["static_transforms"]
        self.R_l6_cam = quat_xyzw_to_mat(st["m0609_tool0__camera_link"]["quat_xyzw"])
        self.t_l6_cam = np.array(st["m0609_tool0__camera_link"]["xyz"])
        self.R_cam_opt = quat_xyzw_to_mat(st["camera_link__camera_color_optical_frame"]["quat_xyzw"])
        ci = self.frames["wrist_camera"]["camera_info_observed"]
        self.K = np.array(ci["k"], dtype=float).reshape(3, 3)
        self.dist = np.array(ci["d"], dtype=float)

        print(f"   scene ready — robots: {list(self.rigs.keys())}")

    def _set_arm_stiffness(self, robot_id, stiffness):
        """observe_pose() 는 QR 이 깨지지 않게 DRIVE_STIFFNESS(1e5)로,
        pick_phase1_approach() 는 흡착 유지력이 충분한 DRIVE_STIFFNESS_PICK
        (1e8, 12_pick_test.py 검증값)으로 되돌린다. 위 DRIVE_STIFFNESS_PICK
        정의부 주석 참고 — 실측: 1e5 로 두면 LIFT 중 rise=88mm·tilt<0.1도로
        정상 상승해도 HOLD_WAIT 끝에 gripped=False (SLIP) 였다."""
        rig = self.rigs[robot_id]
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(rig.robot_prim_path)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for dt in ["angular", "linear"]:
                d = UsdPhysics.DriveAPI.Get(prim, dt)
                if d:
                    d.GetStiffnessAttr().Set(stiffness)

    def _sync_ik_base(self, robot_id):
        rig = self.rigs[robot_id]
        pos, quat = get_world_pose(rig.base_link_path)
        rig.lula.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
        return pos, quat

    def _require_playing(self):
        """Stop 상태에서는 articulation view 가 무효라 get_joint_positions() 가
        None 을 반환한다 — 그 자리에서 바로 TypeError('NoneType' object does
        not support item assignment) 로 죽어서 원인을 알기 어려웠다(실측).
        자동으로 다시 play() 하지는 않는다 — 사용자가 일부러 Stop 을 누른
        경우와 구분이 안 되기 때문이다. 대신 여기서 명확한 이유를 알려준다."""
        if not self.world.is_playing():
            raise RuntimeError(
                "시뮬레이션이 Play 상태가 아니다 — Isaac Sim 뷰포트에서 Play 를 "
                "누른 뒤 다시 시도해라 (Stop 상태에서는 로봇 articulation 을 "
                "읽거나 움직일 수 없다)")

    def _set_joint_deg(self, robot_id, joints_deg):
        self._require_playing()
        rig = self.rigs[robot_id]
        idx = np.array([rig.robot.get_dof_index(j) for j in ARM_JOINTS])
        q = rig.robot.get_joint_positions()
        q[idx] = np.deg2rad(joints_deg)
        rig.robot.set_joint_positions(q)
        rig.robot.set_joint_velocities(np.zeros_like(q))
        rig.robot.apply_action(ArticulationAction(joint_positions=np.deg2rad(joints_deg),
                                                   joint_indices=idx))

    def _servo_joint_deg(self, robot_id, target_joints_deg, n_steps=SETTLE_STEPS):
        """_set_joint_deg 는 set_joint_positions() 로 관절을 즉시 스냅한다 —
        아무것도 안 들고 있을 때(observe_pose)는 문제없지만, STOW 처럼 흡착
        중에 쓰면 그 순간 가속으로 접합이 끊긴다(실측: LIFT 판정은 통과했는데
        STOW 이후 최종 gripped=False). 대신 매 스텝 목표를 다시 명령해 부드럽게
        움직인다 — _servo_tcp 와 같은 방식."""
        self._require_playing()
        rig = self.rigs[robot_id]
        idx = np.array([rig.robot.get_dof_index(j) for j in ARM_JOINTS])
        start_deg = np.degrees(rig.robot.get_joint_positions()[idx])
        target_deg = np.array(target_joints_deg, dtype=float)
        for i in range(1, n_steps + 1):
            self.world.step(render=not HEADLESS)
            cur_deg = start_deg + ease(i / float(n_steps)) * (target_deg - start_deg)
            rig.robot.apply_action(ArticulationAction(
                joint_positions=np.deg2rad(cur_deg), joint_indices=idx))

    def _target_quat(self, robot_id):
        """이 로봇의 접근 자세(그리퍼가 수직 아래를 보는 자세).

        ★ 차체 yaw 를 90도 단위로 반올림해서 그만큼 같이 돌린다.
          예전에는 self.target_quat 하나를 시작할 때 만들어 두고 모든 로봇이
          그대로 썼다. 그건 **월드 기준 고정** 자세라(GRIPPER_YAW_DEG = 0),
          차체가 어느 쪽을 보든 그리퍼는 항상 월드 +x 에 맞춰야 했다.
          robot1 은 yaw 0 이라 우연히 맞았지만 robot2 는 yaw 180 이라
          팔이 180도를 더 비틀어야 했고, 관절 한계에 걸려 IK 가 안 풀렸다
          (실측: robot1 은 OBSERVE 통과, robot2 는 "관측 자세 IK 실패").

        ★ 실제 yaw 를 그대로 안 쓰고 90도로 스냅하는 이유
          잡을 대상의 방향은 선반이 정한다 — 매거진은 선반에 맞춰 놓여 있고
          선반은 축에 정렬돼 있다(shelf_2 는 shelf_1 에서 180도 돌아 있다).
          로봇의 주행 yaw 에는 정차 오차가 섞이는데, 그 오차까지 그리퍼에
          그대로 옮기면 잡는 면이 그만큼 틀어진다. 90도 스냅은 "어느 선반
          앞이냐" 만 뽑아내고 정차 오차는 버린다.
        """
        rig = self.rigs[robot_id]
        _, base_q = get_world_pose(rig.chassis_link_path)
        yaw_deg = math.degrees(yaw_of_quat(base_q))
        snapped = round(yaw_deg / 90.0) * 90.0
        return make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG,
                                GRIPPER_YAW_DEG + snapped)

    def _get_tcp_pose(self, robot_id):
        rig = self.rigs[robot_id]
        pos, quat = rig.robot.end_effector.get_world_pose()
        return get_tcp_pose_from_ee(pos, quat)

    def _servo_tcp(self, robot_id, goal_tcp, phase_name):
        rig = self.rigs[robot_id]
        # ★ IK 를 풀기 전에 Lula 에게 팔 베이스가 지금 어디인지 알려준다.
        #
        #   왜 여기서 매번 하는가 — goal_tcp 도 _get_tcp_pose 도 월드 좌표인데,
        #   Lula 솔버는 "팔 베이스가 어디 있는지" 를 따로 들고 있다. 베이스가
        #   움직였는데 그걸 안 갱신하면 솔버는 옛 자리 기준으로 풀어서 목표가
        #   작업공간 밖으로 나간다 — 증상은 언제나 "IK 실패" 다.
        #
        #   예전에는 주행이 teleport_base() 였고 거기서 _sync_ik_base() 를
        #   불러 줬다. 그런데 주행이 실제 Nav2 로 바뀌면서 아무도 teleport_base
        #   를 부르지 않게 됐고(그 메서드 독스트링 ★ 참고), 그 순간부터 이
        #   동기화가 통째로 빠졌다. 솔버는 로봇이 뜬 자리(도크)를 계속 믿었다.
        #   실측: 순찰로 선반 앞까지 간 로봇 두 대가 OBSERVE 에서 100% "관측
        #   자세 IK 실패". prior 거리를 아무리 줄여도(0.795 -> 0.712) 그대로였다.
        #
        #   12_place_test2.py 는 베이스를 옮길 때마다 sync_ik_base_pose() 를
        #   부른다(7 군데). 같은 일을 여기 한 곳에서 한다 — IK 를 쓰는 모든
        #   경로가 _servo_tcp 를 지나므로, 부르는 쪽이 잊어버릴 수 없다.
        #   비용은 prim pose 읽기 한 번이라 매 호출마다 해도 무해하다.
        self._sync_ik_base(robot_id)
        # 이 로봇의 접근 자세. 차체 방향에 맞춰 돈다 — _target_quat 주석 참고.
        target_quat = self._target_quat(robot_id)
        start = self._get_tcp_pose(robot_id)
        n_steps, dist = steps_for(start, goal_tcp)
        fail = 0
        for i in range(1, n_steps + 1):
            self.world.step(render=not HEADLESS)
            tcp = start + ease(i / float(n_steps)) * (goal_tcp - start)
            action, solved = rig.solver.compute_inverse_kinematics(
                target_position=tcp_to_flange(tcp, target_quat),
                target_orientation=target_quat)
            if solved:
                rig.robot.apply_action(action)
                fail = 0
            else:
                fail += 1
                if fail >= MAX_IK_FAIL_STEPS:
                    return False
        return True

    # ── RPC 메서드 ──────────────────────────────────────────
    # 아래 대부분은 robot_id="robot1"/"robot2" 를 받는다 — pick_place_server/
    # carrier_code_reader 가 자기 ROS 네임스페이스를 그대로 실어 보낸다
    # (robot_id 를 안 주면 DEFAULT_ROBOT_ID="robot1" 폴백 — 옛 호출과 호환).
    def teleport_base(self, x, y, yaw_deg, robot_id=DEFAULT_ROBOT_ID):
        """임시 nav_server 가 부르는 것 — "가짜 주행". SLAM 준비되면 이
        메서드는 그대로 두고, 임시 nav_server 만 실제 Nav2 클라이언트로
        바뀐다(이 백엔드는 안 바뀐다). ★ 지금은 아무 ROS 노드도 이 메서드를
        안 부른다 — 실제 Nav2(multi_navigation.launch.py)로 이미 대체됐다.

        ★★ 이 메서드가 안 불리면서 _sync_ik_base() 도 같이 죽어 있었다.
        그래서 지금은 _servo_tcp() 가 IK 마다 직접 동기화한다(그쪽 주석 참고).
        여기 호출은 남겨 둔다 — 텔레포트는 IK 경로를 안 지나므로 필요하다.

        ★ 한 번에 점프하지 않는다 — 목표까지 여러 스텝에 걸쳐 조금씩
        set_world_pose() 를 다시 부른다(_servo_base). 12_place_test.py 실측:
        "set_world_pose() 를 부르면 거리와 무관하게 흡착이 즉시 풀린다" —
        즉 한 걸음(스텝)만 움직여도 깨지는 성질이라, 잘게 쪼개도 결과는
        같다. 그래도 "정말 몇 스텝째 놓치는지" 를 실측으로 보여주려고 이
        방식을 쓴다 — dropped_at_step 로 보고한다(gripped=False 로 시작하면
        None)."""
        rig = self.rigs[robot_id]
        was_gripped = holding(rig.gripper.gripped())
        dropped_at_step = self._servo_base(robot_id, x, y, yaw_deg)
        self._sync_ik_base(robot_id)
        # rig.base_link_path(m0609/base_link) 는 팔의 마운트 기준점이지 AMR
        # 섀시가 아니다 — 카터 위에 약 0.2 m 앞으로 얹혀 있다(taught_poses.yaml
        # 의 base_link_world vs m0609_base_world 차이와 일치). 여기서는 실제로
        # 텔레포트한 섀시 pose 를 보고한다. Lula 동기화(_sync_ik_base)는
        # 그대로 m0609/base_link 를 쓴다 — IK 에는 그게 맞는 프레임이다.
        pos, quat_w = get_world_pose(rig.chassis_link_path)

        carried_ok = True
        if was_gripped:
            carried_ok = self._carry_gripped_object_through_teleport(robot_id)

        return {"base_x": float(pos[0]), "base_y": float(pos[1]),
               "base_yaw_deg": math.degrees(yaw_of_quat(quat_w)),
               "carried_ok": carried_ok, "dropped_at_step": dropped_at_step}

    def _servo_base(self, robot_id, target_x, target_y, target_yaw_deg):
        """베이스를 목표 pose 까지 여러 스텝에 걸쳐 보간 이동한다("가짜
        주행"). 실제 바퀴 속도 제어가 아니라 매 스텝 set_world_pose() 를
        다시 부르는 것뿐이다 — Nav2 전까지의 임시 근사."""
        rig = self.rigs[robot_id]
        start_p, start_q = get_world_pose(rig.base_xform_path)
        start_yaw = math.degrees(yaw_of_quat(start_q))
        dist = float(np.linalg.norm([target_x - start_p[0], target_y - start_p[1]]))
        n_steps = int(np.clip(dist / 0.03, MIN_STEPS, MAX_STEPS * 2))

        was_gripped = holding(rig.gripper.gripped())
        dropped_at_step = None
        for i in range(1, n_steps + 1):
            a = ease(i / float(n_steps))
            x = start_p[0] + a * (target_x - start_p[0])
            y = start_p[1] + a * (target_y - start_p[1])
            yaw = start_yaw + a * (target_yaw_deg - start_yaw)
            quat = quat_from_axis([0, 0, 1], yaw)
            rig.robot.set_world_pose(position=np.array([x, y, CARTER_Z]), orientation=quat)
            try:
                rig.robot.set_linear_velocity(np.zeros(3))
                rig.robot.set_angular_velocity(np.zeros(3))
            except Exception:
                pass
            self.world.step(render=not HEADLESS)
            if was_gripped and dropped_at_step is None and not holding(rig.gripper.gripped()):
                dropped_at_step = i
                print(f"   !! [{robot_id}] 주행 중 {i}/{n_steps} 스텝에서 흡착이 끊겼다")
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            self.world.step(render=not HEADLESS)
        return dropped_at_step

    def _carry_gripped_object_through_teleport(self, robot_id):
        """★ 이 시뮬레이션의 한계 우회: 베이스 아티큘레이션에 set_world_pose()
        를 부르면 거리와 무관하게 흡착이 즉시 풀린다(12_place_test.py /
        14_place_test_pse.py 실측 — 상대 위치를 그대로 들고 옮겨도 마찬가지였다).
        그래서 "이송 중 계속 붙들고 있다"처럼 보이게 하려고, 텔레포트가 끝난
        자리에서 매거진을 그리퍼 바로 아래로 다시 옮겨 재흡착한다. 실물
        로봇은 이 메서드를 안 타므로(진짜로 붙든 채 이동하니) 이관 시 자연히
        빠진다 — nav_server/pick_place_server 쪽 코드는 안 바뀐다.

        ★ self.magazine(MAGAZINE_XFORM_PATH, shelf_1 orange 고정 하나)만
        재배치한다 — teleport_base 를 아무도 안 부르는 지금 배선에서는
        도달하지 않는 경로라 robot2/다른 매거진 인스턴스로 일반화하지
        않았다. 실제로 이 경로를 다시 쓰게 되면 rig.current_magazine_path
        인스턴스를 재배치하도록 고쳐야 한다.

        재배치는 tcp_now 를 그대로 쓴다(gripper.close() 가 maxGripDistance
        30mm 안에서만 붙으니 tcp 에 최대한 가까워야 한다) — pkg_loader 정지
        지점에서는 m0609 팔 베이스가 섀시보다 ~0.2m 앞으로 얹혀 있어(teleport_base
        위 주석) 이 위치가 ConveyorFrame 충돌체(x>=4.2)와 살짝 겹친다. 실측:
        필터링 없이는 그 순간 PhysX 가 매거진을 몇 m 밖으로 튕겨냈다 — __init__
        에서 magazine ↔ ConveyorFrame 충돌을 걸러(filter_collision) 근본
        해결했다(어차피 이 메서드는 순간이동 흡착 유지용 우회라 그 둘의 충돌
        자체가 의미 없다).

        ★ READY_JOINTS_DEG(STOW 자세)에서 그대로 재흡착을 시도하지 않는다.
        그 자세는 (a) 그리퍼가 아래를 보지 않고(축 방향이 pick 때와 달라
        월드 -Z 오프셋이 안 맞는다), (b) pkg_loader 에서는 ConveyorFrame
        충돌체와 가까워 위치 계산이 조금만 틀려도 PhysX 가 매거진을 몇 m
        밖으로 튕겨냈다(실측 재현 2회). 대신 pick_phase2_finish 의 SUCTION
        이 이미 검증한 자세(target_quat, 수직 하강 방향)로 팔을 먼저 옮긴
        뒤 그 자세에서 재흡착한다 — 좌표 공식도 grip_z 와 동일하게 월드 -Z
        오프셋을 쓸 수 있다. 호버 지점(섀시 앞 0.3m, 위 1.0m)은 바닥·선반·
        컨베이어 전부와 충분히 떨어져 있어 충돌 걱정이 없다."""
        rig = self.rigs[robot_id]
        rig.gripper.open()
        rig.gripper.reinit()

        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        safe_hover = base_p + R_base @ np.array([0.3, 0.0, 1.0])
        self._servo_tcp(robot_id, safe_hover, "TRANSIT_HOVER")
        for _ in range(10):
            self.world.step(render=not HEADLESS)

        tcp_now = self._get_tcp_pose(robot_id)   # target_quat 자세라 월드 -Z 오프셋이 맞다
        mag_height = measure_prim(MAGAZINE_XFORM_PATH)[2]
        # 직전까지 떨어져 있던 자세(mag_quat)를 그대로 쓰면 기울어진 채로
        # 재배치되어 바운딩박스가 커지고 주변과 걸릴 수 있다 — reset_magazine
        # 과 같은 깨끗한 직립 자세(magazine_spawn_quat)로 되돌린다.
        origin = tcp_now - np.array([0.0, 0.0, mag_height])
        self.magazine.set_world_pose(position=origin, orientation=self.magazine_spawn_quat)
        try:
            self.magazine.set_linear_velocity(np.zeros(3))
            self.magazine.set_angular_velocity(np.zeros(3))
        except Exception:
            pass
        # reset_magazine 과 같은 이유로, 접촉을 바로 시도하지 않고 먼저
        # 안정화시킨다 — 텔레포트 직후 첫 스텝에서 생기는 과도 반응(있다면)이
        # gripper.close() 전에 가라앉게 한다.
        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)

        rig.gripper.close()
        for _ in range(GRIP_WAIT):
            self.world.step(render=not HEADLESS)
        ok = holding(rig.gripper.gripped())
        if not ok:
            print(f"   !! [{robot_id}] 이송 후 재흡착 실패 — 이송 중 놓친 것으로 처리")
        return ok

    def reset_magazine(self, robot_id=DEFAULT_ROBOT_ID):
        """반복 테스트용. carrier_code_reader/pick 결과에 영향받은 매거진을
        원위치로 되돌린다. robot_id 는 그리퍼를 열어 둘 로봇 — 매거진
        자체(MAGAZINE_XFORM_PATH)는 로봇과 무관한 전역 픽스처다."""
        rig = self.rigs[robot_id]
        rig.gripper.open()
        for _ in range(20):
            self.world.step(render=not HEADLESS)
        self.magazine.set_world_pose(position=self.magazine_spawn_pos,
                                     orientation=self.magazine_spawn_quat)
        self.magazine.set_linear_velocity(np.zeros(3))
        self.magazine.set_angular_velocity(np.zeros(3))
        for _ in range(30):
            self.world.step(render=not HEADLESS)
        rig.gripper.reinit()
        return {"ok": True}

    def save_state(self):
        """정지 시(main() 의 SIGINT/SIGTERM 처리) 로봇 베이스 pose·팔 관절값·
        그리퍼 흡착 대상과, 매거진/캐리어(스택) 배치를 스냅샷으로 남긴다.
        다음 기동에서 __init__ 이 _restore_state() 로 그대로 되돌린다.

        ★ 왜 필요한가: 이 프로세스가 재시작되면 stage 를 처음부터 다시 열어
        USD 에 박힌 스폰 배치로 돌아간다. 그런데 Nav2/AMCL 은 이 프로세스와
        별개로 죽지 않고 계속 도는 게 보통이라(운영 편의상 sim_backend 만
        재기동하는 경우), 시뮬레이터 쪽 로봇 위치가 스폰으로 되돌아가면
        AMCL 이 믿는 위치와 실제 위치가 어긋난다 — 이 스냅샷이 그 어긋남을
        없앤다.

        ★ SIGKILL(-9) 은 어떤 프로세스도 못 잡는다 — 그 경우엔 마지막
        정상 종료 시점의 스냅샷으로 복원된다(그 뒤 상태는 유실)."""
        try:
            robots = {}
            for robot_id, rig in self.rigs.items():
                base_p, base_q = get_world_pose(rig.base_xform_path)
                idx = [rig.robot.get_dof_index(j) for j in ARM_JOINTS]
                arm_deg = np.degrees(rig.robot.get_joint_positions()[idx]).tolist()
                robots[robot_id] = {
                    "base_pos": base_p.tolist(),
                    "base_quat_wxyz": base_q.tolist(),
                    "arm_joints_deg": arm_deg,
                    "gripped_paths": _flatten_gripped_paths(rig.gripper.gripped()),
                }

            magazines = {}
            for path in self._all_magazine_prims:
                pos, quat = get_world_pose(path)
                magazines[path] = {"pos": pos.tolist(), "quat_wxyz": quat.tolist()}

            snapshot = {
                "world_usd": WORLD_USD,
                "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "robots": robots,
                "magazines": magazines,
            }
            tmp = STATE_SNAPSHOT_PATH.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(snapshot, indent=2), encoding="utf-8")
            tmp.replace(STATE_SNAPSHOT_PATH)   # 원자적 교체 — 쓰다 죽어도 이전 스냅샷이 남는다
            print(f"   상태 스냅샷 저장: {STATE_SNAPSHOT_PATH}")
        except Exception as e:      # noqa: BLE001 — 저장 실패해도 종료 자체는 막지 않는다
            print(f"   !! 상태 스냅샷 저장 실패({e}) — 다음 기동은 기본 배치로 시작한다")

    def _restore_state(self):
        """save_state() 스냅샷이 있으면 __init__ 의 기본 스폰 배치 위에 덮어
        씌운다. rig.lula/rig.gripper 가 이미 만들어진 뒤(그 바로 다음 줄)
        불러야 한다 — 재흡착엔 gripper 가, 베이스 이동 뒤 IK 보정엔 lula 가
        필요하다.

        ★ world_usd 가 지금 WORLD_USD 와 다르면(씬을 바꿔 실행) 통째로
        건너뛴다 — 다른 씬의 프림 경로/좌표를 이 씬에 그대로 적용하면
        엉뚱한 곳으로 텔레포트하거나 존재하지 않는 prim 경로로 조용히
        아무 일도 안 하게 된다."""
        if not STATE_SNAPSHOT_PATH.exists():
            return
        try:
            snapshot = json.loads(STATE_SNAPSHOT_PATH.read_text(encoding="utf-8"))
        except Exception as e:
            print(f"   !! 상태 스냅샷을 못 읽었다({e}) — 기본 배치로 시작한다")
            return
        if snapshot.get("world_usd") != WORLD_USD:
            print(f"   상태 스냅샷이 다른 씬 것이다({snapshot.get('world_usd')}) — 건너뛴다")
            return

        for path, m in (snapshot.get("magazines") or {}).items():
            if not self.stage.GetPrimAtPath(path).IsValid():
                continue    # 이 씬에는 이제 없는 매거진/스택 — 조용히 건너뛴다
            SingleRigidPrim(prim_path=path).set_world_pose(
                position=np.array(m["pos"]), orientation=np.array(m["quat_wxyz"]))

        for robot_id, r in (snapshot.get("robots") or {}).items():
            rig = self.rigs.get(robot_id)
            if rig is None:
                continue
            rig.robot.set_world_pose(
                position=np.array(r["base_pos"]), orientation=np.array(r["base_quat_wxyz"]))
            self._sync_ik_base(robot_id)   # 베이스를 옮겼으니 lula 도 그 pose 를 다시 알아야 한다
            self._set_joint_deg(robot_id, r["arm_joints_deg"])

        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)

        for robot_id, r in (snapshot.get("robots") or {}).items():
            rig = self.rigs.get(robot_id)
            if rig is None or not r.get("gripped_paths"):
                continue
            rig.gripper.close()
            for _ in range(GRIP_WAIT):
                self.world.step(render=not HEADLESS)
            if not holding(rig.gripper.gripped()):
                print(f"   !! [{robot_id}] 재흡착 복원 실패 — 스냅샷엔 흡착 중이었는데 지금은 안 붙는다")

        print(f"   상태 스냅샷 복원 완료 ({snapshot.get('saved_at', '?')} 저장분)")

    def observe_pose(self, pose_name=None, joints_deg=None, robot_id=DEFAULT_ROBOT_ID):
        """관측 자세로 이동한다. 두 가지 중 하나로 목표를 준다:
          pose_name    taught_poses.yaml 에 있는 키 (예: shelf_1_top_close_centered)
          joints_deg   J1..J6 목록 (도). shelves.yaml 의
                       scan_passes.arm_teach_pose 는 rad 단위(meta.units)니
                       호출하는 쪽(carrier_code_reader)이 도로 바꿔 넘긴다.
        둘 다 주어지면 joints_deg 가 우선한다. 둘 다 없으면 ValueError.
        carrier_code_reader.scan_qr() 전에 부른다. robot_id 는 어느 로봇의
        팔/손목캠을 움직일지 고른다(네임스페이스 그대로: "robot1"/"robot2")."""
        self._set_arm_stiffness(robot_id, DRIVE_STIFFNESS)   # QR 디코드가 깨지지 않는 값으로
        if joints_deg is not None:
            self._set_joint_deg(robot_id, list(joints_deg))
        elif pose_name is not None:
            taught = yaml.safe_load((ISAACPJT / "tools/out/taught_poses.yaml").read_text(encoding="utf-8"))
            pose = taught[pose_name]
            self._set_joint_deg(robot_id, pose["joints_deg"])
        else:
            raise ValueError("observe_pose: pose_name 또는 joints_deg 가 필요하다")
        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)
        self._ensure_camera_warm(robot_id)
        return {"ok": True, "pose": pose_name or "joints(deg): %s" % joints_deg}

    def _get_or_create_viewport(self, robot_id):
        """이 로봇 전담 뷰포트(ViewportAPI). 첫 로봇(ROBOT_CARTER_NAME 의
        첫 항목 — 지금은 robot1)은 기존 GUI 메인 뷰포트를 그대로 쓰고,
        나머지 로봇은 각자 독립된 뷰포트 창을 새로 만든다.

        ★ 예전 판은 뷰포트가 하나뿐이라 카메라를 로봇마다 돌려썼다 —
        patrol 중 로봇 둘이 동시에 scan_qr 을 폴링하면(carrier_code_reader
        decode_hz) 서로 밀어냈다. debug_dual_viewport_capture() 로 실측
        확인: create_viewport_window() 로 만든 두 번째 뷰포트에서도 같은
        omni.kit.widget.viewport.capture 방식(annotator/FabricReader 를
        안 거침, _ensure_camera_warm 독스트링 참고)이 그대로 통하고, 두
        뷰포트가 같은 world.step() 루프 안에서 독립적으로(같은 프레임에)
        완료된다 — 그래서 로봇마다 하나씩 전담시킬 수 있다.

        ★ 실측 함정: create_viewport_window() 는 창을 만들기만 하고
        visible 을 안 켜 준다. omni.kit.viewport.window.ViewportWindow 의
        `self.viewport_api.updates_enabled = self.visible` 때문에, 안 보이는
        창은 렌더 자체가 안 돌아 캡쳐가 전부 검은 화면(버퍼 크기는 맞는데
        내용이 0)으로 나온다 — 그래서 여기서 명시적으로 visible/updates_enabled
        를 켠다."""
        rig = self.rigs[robot_id]
        if rig.viewport is not None:
            return rig.viewport

        first_robot_id = next(iter(self.rigs))
        if robot_id == first_robot_id:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport is None:
                raise RuntimeError(
                    "활성 뷰포트를 찾을 수 없다 — headless 모드에서는 이 우회가 안 통한다")
            rig.viewport = viewport
            return rig.viewport

        from omni.kit.viewport.utility import create_viewport_window
        window = create_viewport_window(
            name=f"Viewport-{robot_id}", width=1280, height=720,
            camera_path=rig.camera_prim)
        if window is None:
            raise RuntimeError(
                f"{robot_id} 전담 뷰포트 생성 실패 — create_viewport_window() 가 None 을 반환했다")
        window.visible = True
        window.viewport_api.updates_enabled = True
        rig.viewport_window = window   # GC 방지 — 창 객체를 안 붙잡으면 사라진다
        rig.viewport = window.viewport_api
        return rig.viewport

    def _ensure_camera_warm(self, robot_id, force=False):
        """관측 자세에 이미 도착한 뒤, 이 로봇 전담 뷰포트를 손목 카메라로
        돌리고 depth AOV 를 등록한다.

        ★★★ 이전 판은 rep.create.render_product() + AnnotatorRegistry 로
        새 render_product 를 만들었는데, 이 세션(Isaac Sim 5.1.0 rc.19)에서
        그 경로 자체가 원인 불명으로 항상 빈 프레임만 줬다 — 카메라 종류·
        동시 부하·확장 로드 순서·multi_gpu·Kit 사용자 설정을 전부 바꿔봐도
        재현됐고, 순정 재설치판에서도 재현됐다(스테이지 로드 중
        omni.graph.core 쪽 세그폴트까지 같이 남 — 업스트림
        Replicator/FabricReader 버그로 보인다. task_manager.py QR 인식
        디버깅 이력 참고). 심지어 이미 정상 동작 중인 메인 뷰포트의
        render_product 에 새 annotator 를 "붙이기만" 해도 똑같이
        빈 프레임이었다 — 문제가 render_product 가 아니라 annotator
        (FabricReader) 파이프라인 자체에 있다는 뜻이다.

        그래서 Replicator/AnnotatorRegistry 를 아예 안 거치는
        omni.kit.widget.viewport.capture(Kit 자체 스크린샷/뷰포트 캡쳐가
        쓰는 것과 같은 omni.renderer_capture 백엔드)로 바꿨다 — 뷰포트
        카메라를 손목 카메라로 돌리고, 그 뷰포트의 render_product 에
        raw AOV 캡쳐(_capture_frame)로 RGB+depth 를 직접 받는다.
        depth 는 omni.kit.viewport.utility.add_aov_to_viewport() 로 등록하는데
        (Replicator 를 안 거치고 RenderProduct prim 의 orderedVars 에 USD
        레벨로만 RenderVar 를 추가하는 함수라 FabricReader 버그를 피해간다),
        이 함수 자체에도 버그가 있다 — `/app/hydra/renderSettings/
        saveUsdAttributes` 가 True 일 때 타는 분기가
        `for render_var_prims in render_var_prims:` 로 루프 변수를 자기
        자신에 덮어써서 그 안의 `render_var_prim`(단수)이 UnboundLocalError
        로 죽는다(실측 재현). 그 설정을 미리 꺼서 우회한다.

        raw AOV 이름은 Replicator 주석("distance_to_image_plane")과 다르다 —
        omni.replicator.core.scripts.annotators 의 AnnotatorParams 테이블에
        찍힌 실제 이름은 "DistanceToImagePlaneSD"(SD 접미사, 실측 확인:
        "DistanceToImagePlane"으로는 aov_map 에 등록만 되고 텍스처 해상도가
        (0,0)으로 끝까지 안 채워짐 — SD 이름이라야 R32_SFLOAT/1280x720 로
        실제 채워진다).

        ★ 로봇마다 전담 뷰포트를 쓴다(_get_or_create_viewport) — 첫
        로봇(robot1)은 GUI 메인 화면을 그대로 보여주고, 나머지는 각자
        독립 창이라 두 로봇이 동시에 관측해도 서로 안 밀어낸다(실측:
        debug_dual_viewport_capture). capture_ready 는 로봇별로 따로
        추적한다 — 그 로봇 뷰포트에서 처음 한 번만 AOV 를 등록하면 된다.
        """
        rig = self.rigs[robot_id]
        from omni.kit.viewport.utility import add_aov_to_viewport
        import carb.settings

        viewport = self._get_or_create_viewport(robot_id)
        viewport.camera_path = rig.camera_prim
        if force or not rig.capture_ready:
            carb.settings.get_settings().set(
                "/app/hydra/renderSettings/saveUsdAttributes", False)
            add_aov_to_viewport(viewport, DEPTH_AOV_NAME)
            rig.capture_ready = False
        for _ in range(30):
            self.world.step(render=True)
        # 실제로 유효한 프레임이 나오는지 한 번 확인한다 — 이전 판의
        # "워밍업 검증" 과 같은 취지다. 실패하면 그대로 예외를 올린다.
        self._capture_frame(robot_id, timeout_frames=240)
        rig.capture_ready = True

    def _capture_frame(self, robot_id, timeout_frames=180):
        """이 로봇 전담 뷰포트의 render_product 에서 RGB(BGR 로 변환해서
        반환)+depth 를 raw 바이트 콜백으로 한 프레임 받는다.
        _ensure_camera_warm() 독스트링 참고 — Replicator/AnnotatorRegistry
        (FabricReader)를 아예 안 거친다. 호출 전에 _ensure_camera_warm(robot_id)
        로 그 뷰포트가 이미 이 로봇의 카메라를 보고 있어야 한다."""
        rig = self.rigs[robot_id]
        from omni.kit.widget.viewport.capture import MultiAOVByteCapture

        viewport = self._get_or_create_viewport(robot_id)

        result = {}

        def _on_rgb(buffer, buffer_size, width, height, byte_format):
            raw = _pycapsule_to_bytes(buffer, buffer_size)
            arr = np.frombuffer(raw, dtype=np.uint8, count=buffer_size)
            result["rgb"] = arr.reshape(height, width, 4)[:, :, :3][:, :, ::-1].copy()

        def _on_depth(buffer, buffer_size, width, height, byte_format):
            raw = _pycapsule_to_bytes(buffer, buffer_size)
            arr = np.frombuffer(raw, dtype=np.float32, count=width * height)
            result["depth"] = arr.reshape(height, width).astype(np.float64).copy()

        cap = MultiAOVByteCapture(["", DEPTH_AOV_NAME], [_on_rgb, _on_depth])
        rig.pending_capture = cap  # GC 되면 콜백이 안 온다 — 끝날 때까지 붙잡아둔다
        viewport.schedule_capture(cap)
        for _ in range(timeout_frames):
            self.world.step(render=True)
            if "rgb" in result and "depth" in result:
                rig.pending_capture = None
                return result["rgb"], result["depth"]
        rig.pending_capture = None
        raise RuntimeError("프레임 캡쳐 타임아웃 — rgb/depth 콜백이 오지 않았다")

    def scan_qr(self, expected_id=None, n_frames=3, robot_id=DEFAULT_ROBOT_ID):
        """cobot3_perception.qr_pose 로 QR 자세를 재고, base_link 프레임으로
        변환해 돌려준다. carrier_code_reader 노드가 그대로 CarrierScan.qr_pose
        에 옮겨 담을 수 있는 형태다. observe_pose(robot_id=...) 로 이미 그
        로봇의 관측 자세/카메라가 준비돼 있어야 한다."""
        rig = self.rigs[robot_id]
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)

        obs_list = []
        decoded = ""
        for _ in range(n_frames):
            rgb, depth = self._capture_frame(robot_id)
            l6_p, l6_q = get_world_pose(rig.ee_link_path)
            R_l6 = quat_to_matrix(l6_q)
            R_opt = R_l6 @ self.R_l6_cam @ self.R_cam_opt
            p_opt = l6_p + R_l6 @ self.t_l6_cam
            obs = estimate_qr_pose(rgb, depth, self.K, self.dist, R_opt, p_opt,
                                   expected_id=expected_id)
            obs_list.append(obs)
            if obs.ok:
                decoded = obs.decoded

        agg = aggregate_qr_poses(obs_list)
        if not agg.ok:
            return {"ok": False, "reason": agg.reason}

        # world -> base_link (CarrierScan.qr_pose 의 frame_id 규약)
        p_rel = R_base.T @ (agg.center_world - base_p)
        # qr_pose.py 의 QR 프레임(x=오른쪽, y=아래, z=안쪽)을 yaw 하나로 재구성한다.
        # 매거진은 항상 upright 라 y_qr(이미지 아래)는 세계 -Z 로 고정이다
        # (qr_pose.py 의 estimate_qr_pose 와 동일 가정).
        yaw = agg.yaw_rad
        x_qr_world = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        y_qr_world = np.array([0.0, 0.0, -1.0])
        z_qr_world = np.cross(x_qr_world, y_qr_world)
        R_qr_world = np.column_stack([x_qr_world, y_qr_world, z_qr_world])
        R_qr_base = R_base.T @ R_qr_world

        return {
            "ok": True, "decoded": agg.decoded or decoded,
            "n_used": agg.n_used, "n_total": agg.n_total,
            "pos_std_mm": agg.pos_std_mm.tolist(), "yaw_std_deg": agg.yaw_std_deg,
            "qr_pose_base_link": {
                "position": p_rel.tolist(),
                "quat_wxyz": self._mat_to_quat(R_qr_base).tolist(),
            },
            "qr_pose_world": {
                "position": agg.center_world.tolist(),
                "yaw_deg": math.degrees(yaw),
            },
        }

    @staticmethod
    def _mat_to_quat(R):
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R[2,1]-R[1,2])/S; y = (R[0,2]-R[2,0])/S; z = (R[1,0]-R[0,1])/S
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = math.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
            w = (R[2,1]-R[1,2])/S; x = 0.25*S; y=(R[0,1]+R[1,0])/S; z=(R[0,2]+R[2,0])/S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2
            w = (R[0,2]-R[2,0])/S; x=(R[0,1]+R[1,0])/S; y=0.25*S; z=(R[1,2]+R[2,1])/S
        else:
            S = math.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2
            w = (R[1,0]-R[0,1])/S; x=(R[0,2]+R[2,0])/S; y=(R[1,2]+R[2,1])/S; z=0.25*S
        return np.array([w,x,y,z])

    def _find_nearest_magazine(self, flange_world):
        """실제로 지금 집으려는 매거진이 씬의 몇 번째 인스턴스인지는 이름
        만으로 못 가른다(위 __init__ 의 self._all_magazine_prims 주석 참고).
        pick_phase1_approach 가 이미 아는 실제 목표 flange_world(QR pose 로
        역산한 3D 좌표)에 flange_plate 가 가장 가까운 인스턴스를 찾는다."""
        best_path, best_d = None, None
        for path in self._all_magazine_prims:
            try:
                cx, top_z, _h = measure_prim(f"{path}/flange_plate")
            except Exception:
                continue
            d = float(np.linalg.norm(np.array([cx[0], cx[1], top_z]) - flange_world))
            if best_d is None or d < best_d:
                best_path, best_d = path, d
        return best_path or MAGAZINE_XFORM_PATH

    def pick_observe_flange(self, flange_pose_base_link, variant, robot_id=DEFAULT_ROBOT_ID):
        """PickCarrier 의 OBSERVE — qr_pose(prior) 위로 손목캠을 가져가
        flange_topview.detect_flange() 로 진짜 파지점(중심 xy·윗면 z·yaw)을
        잰다. PickCarrier.action ★ 블록과 pick_place_server.py 독스트링이
        "알려진 갭"으로 적어 두던 손목캠 재관측을 채운다 — 12_pick_test.py
        의 FlangeVision(observe_tcp/capture)과 같은 수식을 그대로 옮겼다.

        입력 flange_pose_base_link 는 _flange_pose_from_qr()(pick_place_server.py)
        가 QR 대략값으로 계산한 prior — 탐색 창 중심으로만 쓰고, 반환값이
        진짜 파지점이다(pick_phase1_approach 에 그대로 넘기면 된다).

        프레임 캡쳐는 scan_qr 과 같은 이유로 _capture_frame() 을 쓴다 —
        Replicator/AnnotatorRegistry 는 이 세션(Isaac Sim 5.1.0 rc.19)에서
        rgb annotator 가 항상 빈 프레임(shape=(0,))만 줘서(_ensure_camera_warm
        독스트링 참고), 뷰포트 캡쳐 우회로 이미 바꿔 둔 경로를 그대로 쓴다."""
        rig = self.rigs[robot_id]
        entry = self.grasp["magazines"].get(variant)
        if entry is None:
            return {"ok": False, "reason": f"grasp.yaml 에 없는 variant: {variant}"}
        fv = self.grasp["flange_vision"]

        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        p_rel = np.array(flange_pose_base_link["position"])
        prior_world = base_p + R_base @ p_rel
        R_prior_rel = quat_to_matrix(np.array(flange_pose_base_link["quat_wxyz"]))
        yaw_prior_world = yaw_of_quat(base_q) + math.atan2(
            (R_base @ R_prior_rel)[1, 0], (R_base @ R_prior_rel)[0, 0])

        # observe_tcp — FlangeVision.observe_tcp 와 동일 수식. target_quat(고정
        # top-down 접근 자세)로 팔이 도착했을 때 손목캠이 prior_world 를
        # observe_image_offset_px 위치(흡착 컵이 화면 아래를 가리므로 중앙보다
        # 위)에 담도록 카메라/TCP 위치를 역산한다.
        h = float(fv["observe_cam_height_m"])
        du, dv = fv["observe_image_offset_px"]
        R_tool = quat_to_matrix(self._target_quat(robot_id))
        R_wo = R_tool @ self.R_l6_cam @ self.R_cam_opt
        p_c = np.array([du / self.K[0, 0] * h, dv / self.K[1, 1] * h, h])
        cam_pos = prior_world - R_wo @ p_c
        tool0_pos = cam_pos - R_tool @ self.t_l6_cam
        observe_tcp = tool0_pos + R_tool @ TCP_OFFSET

        _set_status(robot_id, phase="OBSERVE_FLANGE", message="")
        if not self._servo_tcp(robot_id, observe_tcp, "OBSERVE_FLANGE"):
            return {"ok": False, "reason": "관측 자세 IK 실패"}

        self._ensure_camera_warm(robot_id)
        try:
            rgb, depth = self._capture_frame(robot_id)
        except RuntimeError as e:
            return {"ok": False, "reason": str(e)}

        l6_p, l6_q = get_world_pose(rig.ee_link_path)
        R_l6 = quat_to_matrix(l6_q)
        R_opt = R_l6 @ self.R_l6_cam @ self.R_cam_opt
        p_opt = l6_p + R_l6 @ self.t_l6_cam

        obs = detect_flange(
            rgb, self.K, self.dist, R_opt, p_opt,
            expected_center_world=prior_world, top_z_prior=float(prior_world[2]),
            yaw_prior_rad=yaw_prior_world, color=entry.get("color"),
            flange_size_m=tuple(entry["flange_size"][:2]),
            depth=depth if fv.get("use_depth", True) else None,
            depth_band_m=float(fv["depth_band_m"]),
            search_radius_m=float(fv["search_radius_m"]),
            size_tol=float(fv["size_tol"]), min_fill=float(fv["min_fill"]),
            max_depth_correction_m=float(fv["max_depth_correction_m"]))

        if not obs.ok:
            _set_status(robot_id, phase="OBSERVE_FLANGE", message=obs.reason)
            return {"ok": False, "reason": obs.reason}

        p_rel_out = R_base.T @ (obs.center_world - base_p)
        yaw_rel_out = obs.yaw_rad - yaw_of_quat(base_q)
        half = yaw_rel_out / 2.0
        print(f"   OBSERVE_FLANGE  [{robot_id}] {variant}  depth_corr {obs.depth_correction_m*1000:+.1f} mm  "
              f"fill {obs.fill_ratio:.2f}  edge_rms {obs.edge_rms_mm:.2f} mm")
        return {"ok": True,
               "position": p_rel_out.tolist(),
               "quat_wxyz": [float(math.cos(half)), 0.0, 0.0, float(math.sin(half))]}

    def pick_phase1_approach(self, flange_pose_base_link, approach_dist_m, robot_id=DEFAULT_ROBOT_ID):
        """PickCarrier 의 APPROACH. flange_pose 는 base_link 프레임
        {position:[x,y,z], quat_wxyz:[..]} (Pose 규약과 동일, position=판
        윗면 중심, +Z=법선 위)."""
        rig = self.rigs[robot_id]
        self._set_arm_stiffness(robot_id, DRIVE_STIFFNESS_PICK)   # 흡착 유지력 검증값으로
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        p_rel = np.array(flange_pose_base_link["position"])
        flange_world = base_p + R_base @ p_rel
        rig.flange_world = flange_world   # DESCEND 단계에서 재사용
        # ★ pick_phase2_finish 의 rise/tilt 판정이 엉뚱한(하드코딩된
        # magazine_1_orange) 매거진을 재던 버그의 수정 — 실제 목표 위치에
        # 가장 가까운 인스턴스를 여기서 미리 찾아둔다.
        rig.current_magazine_path = self._find_nearest_magazine(flange_world)

        _set_status(robot_id, phase="APPROACH", gripped=False, gap_m=0.0, message="")
        goal = flange_world + np.array([0, 0, approach_dist_m])
        ok = self._servo_tcp(robot_id, goal, "APPROACH")
        if not ok:
            _set_status(robot_id, phase="FAILED", message="APPROACH IK 실패")
            return {"success": False, "fail_reason": "NO_IK", "phase": "APPROACH"}
        return {"success": True}

    def pick_phase2_finish(self, grip_gaps_m, offset_limit_m, lift_height_m=None,
                           robot_id=DEFAULT_ROBOT_ID):
        """DESCEND -> SUCTION -> LIFT -> STOW. offset_limit_m 은 이제
        pick_place_server 의 declare_parameter 값을 그대로 받는다 (PickCarrier.action
        리팩터 이후 flange_short_side_m 이 goal 에서 빠졌다 — offset_limit_m
        을 서버 파라미터로 직접 갖는 쪽이 계약과 맞는다)."""
        rig = self.rigs[robot_id]
        if lift_height_m is None:
            lift_height_m = LIFT_HEIGHT_OFFSET
        flange_world = rig.flange_world

        for attempt, gap in enumerate(grip_gaps_m):
            _set_status(robot_id, phase="DESCEND", gap_m=gap)
            grip_z = flange_world + np.array([0, 0, gap])
            if not self._servo_tcp(robot_id, grip_z, "DESCEND"):
                _set_status(robot_id, phase="FAILED", message="DESCEND IK 실패")
                return {"success": False, "fail_reason": "NO_IK", "phase": "DESCEND"}

            _set_status(robot_id, phase="SUCTION", gap_m=gap)
            rig.gripper.close()
            for _ in range(GRIP_WAIT):
                self.world.step(render=not HEADLESS)
            ok = holding(rig.gripper.gripped())
            _set_status(robot_id, gripped=ok)
            print(f"   [{robot_id}] 시도 {attempt+1}/{len(grip_gaps_m)}  간격 {gap*1000:+.0f} mm  "
                  f"-> {'붙었다' if ok else '안 붙었다'}")
            if ok:
                break
            rig.gripper.open()
        else:
            _set_status(robot_id, phase="FAILED", message="흡착 실패 — grip_gaps 전부 소진")
            return {"success": False, "fail_reason": "NO_ATTACH", "phase": "SUCTION",
                   "used_gap_m": 0.0}

        # 흡착 순간 TCP 횡오차 (OFF_FLANGE 판정 — PickCarrier.action 주석 그대로)
        tcp_now = self._get_tcp_pose(robot_id)
        final_offset_m = float(np.linalg.norm((tcp_now - flange_world)[:2]))

        # ★ 하드코딩된 FLANGE_PATH/self.magazine(magazine_1_orange) 대신,
        # pick_phase1_approach 가 찾아둔 "실제로 지금 집는 그 인스턴스"를
        # 잰다 — 안 그러면 아무도 안 건드리는 magazine_1_orange 만 계속
        # 재서 rise 가 항상 0mm 으로 나오고 실제로는 성공한 PICK 이
        # SLIP 으로 오판정된다(실측 재현 — task_manager.py QR 인식
        # 디버깅 이력 참고).
        target_flange_path = f"{rig.current_magazine_path}/flange_plate"
        _set_status(robot_id, phase="LIFT")
        top_z0 = measure_prim(target_flange_path)[1]
        lift_goal = flange_world + np.array([0, 0, lift_height_m])
        self._servo_tcp(robot_id, lift_goal, "LIFT")
        for _ in range(HOLD_WAIT):
            self.world.step(render=not HEADLESS)
        gripped_after_lift = holding(rig.gripper.gripped())
        _set_status(robot_id, gripped=gripped_after_lift)
        top_z1 = measure_prim(target_flange_path)[1]
        rise_m = top_z1 - top_z0
        _, mag_q = get_world_pose(rig.current_magazine_path)
        tilt_deg = tilt_deg_from_quat(mag_q)

        if not gripped_after_lift:
            _set_status(robot_id, phase="FAILED", message="리프트 중 놓쳤다")
            return {"success": False, "fail_reason": "SLIP", "phase": "LIFT",
                   "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m}

        if final_offset_m > offset_limit_m:
            _set_status(robot_id, phase="FAILED", message="OFF_FLANGE")
            return {"success": False, "fail_reason": "OFF_FLANGE", "phase": "LIFT",
                   "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m}

        _set_status(robot_id, phase="STOW")
        # STOW: 이송 자세로. READY_JOINTS_DEG 로 되돌리는 것으로 대신한다 —
        # _set_joint_deg(즉시 스냅) 대신 _servo_joint_deg(부드러운 보간)를
        # 쓴다. 실측: 즉시 스냅은 LIFT 판정(rise·tilt·gripped 전부 정상)을
        # 통과한 뒤에도 그 스냅 가속으로 흡착이 끊겼다.
        self._servo_joint_deg(robot_id, READY_JOINTS_DEG, n_steps=SETTLE_STEPS)
        for _ in range(HOLD_WAIT):
            self.world.step(render=not HEADLESS)

        ok = (rise_m >= LIFT_OK_MIN_M) and (tilt_deg <= TILT_MAX_DEG) and holding(rig.gripper.gripped())
        _set_status(robot_id, phase="DONE" if ok else "FAILED",
                    message="" if ok else f"판정 실패 rise={rise_m*1000:.1f}mm tilt={tilt_deg:.2f}")
        return {"success": bool(ok), "fail_reason": "NONE" if ok else "SLIP",
               "gripped": bool(holding(rig.gripper.gripped())),
               "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m,
               "used_gap_m": float(gap), "rise_mm": rise_m*1000, "tilt_deg": tilt_deg}

    def get_place_slot_pose_base_link(self, robot_id=DEFAULT_ROBOT_ID):
        """편의 메서드 — get_flange_pose_world 와 같은 목적, place 쪽 GT.
        포트/슬롯 지오메트리가 아직 씬에 없어서(다음 범위), 12_place_test.py 가
        검증한 컨베이어 벨트 위 스테이징 지점(PLACE_TARGET_XY, CONVEYOR_BELT_Z)을
        그대로 현재 base_link(=chassis) 프레임으로 돌려준다. pkg_loader 정지
        지점에 도착한 뒤(NavigateTo 완료 후) 호출해야 값이 맞다.

        z 는 PICK 때와 같은 관례를 쓴다 — "판 윗면"(flange_plate, 물체
        바닥에서 물체 높이만큼 위)을 옮긴다. 그래서 벨트 높이에 물체 자체를
        얹었을 때의 바닥은 CONVEYOR_BELT_Z 지만, 흡착해서 들고 있는
        flange_plate 는 거기서 물체 높이(magazine_height)만큼 더 위에
        있어야 물체 바닥이 실제로 벨트에 닿는다(12_place_test.py 의
        place_ref_z = CONVEYOR_BELT_Z + magazine_height 와 같은 식이다).
        지금 들고 있는 실제 인스턴스(rig.current_magazine_path, PICK
        때 pick_phase1_approach 가 찾아둔 것)의 실측 높이를 그대로 쓴다."""
        rig = self.rigs[robot_id]
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        magazine_height = measure_prim(rig.current_magazine_path)[2]
        target_z = CONVEYOR_BELT_Z + magazine_height
        world_target = np.array([PLACE_TARGET_XY[0], PLACE_TARGET_XY[1], target_z])
        p_rel = R_base.T @ (world_target - base_p)
        return {"position": p_rel.tolist(), "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}

    def place_phase1_approach(self, slot_pose_base_link, approach_dist_m=None,
                              robot_id=DEFAULT_ROBOT_ID):
        """PlaceCarrier 의 APPROACH. slot_pose 는 base_link 프레임
        {position:[x,y,z], quat_wxyz:[..]}, z=슬롯 바닥. 이송(TRANSIT)은
        여기서 안 한다 — task_manager 가 이 호출 전에 NavigateTo 로 이미
        pkg_loader 에 도착해 있어야 한다(재흡착은 teleport_base 안에서
        처리됨, _carry_gripped_object_through_teleport 참고)."""
        rig = self.rigs[robot_id]
        if not holding(rig.gripper.gripped()):
            _set_status(robot_id, phase="FAILED", message="APPROACH 시작 시점에 이미 안 붙어 있다")
            return {"success": False, "fail_reason": "NOT_GRIPPED"}

        if approach_dist_m is None:
            approach_dist_m = PLACE_APPROACH_HEIGHT_OFFSET
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        p_rel = np.array(slot_pose_base_link["position"])
        slot_world = base_p + R_base @ p_rel
        rig.slot_world = slot_world   # DESCEND 단계에서 재사용
        rig.place_approach_dist_m = float(approach_dist_m)

        _set_status(robot_id, phase="APPROACH", gripped=True, gap_m=0.0, message="")
        goal = slot_world + np.array([0, 0, approach_dist_m])
        ok = self._servo_tcp(robot_id, goal, "APPROACH")
        if not ok:
            _set_status(robot_id, phase="FAILED", message="APPROACH IK 실패")
            return {"success": False, "fail_reason": "NO_IK"}
        return {"success": True}

    def place_phase2_finish(self, release_height_m, robot_id=DEFAULT_ROBOT_ID):
        """DESCEND -> RELEASE -> RETRACT.

        ★ 알려진 갭: PORT_OCCUPIED(슬롯 점유 감지)는 아직 구현하지 않았다 —
        이 씬에 슬롯 자체가 없어서(place_phase1_approach 독스트링 참고)
        항상 비어 있다고 본다."""
        rig = self.rigs[robot_id]
        slot_world = rig.slot_world
        approach_dist_m = rig.place_approach_dist_m

        _set_status(robot_id, phase="DESCEND")
        descend_goal = slot_world + np.array([0, 0, release_height_m])
        if not self._servo_tcp(robot_id, descend_goal, "DESCEND"):
            _set_status(robot_id, phase="FAILED", message="DESCEND IK 실패")
            return {"success": False, "fail_reason": "NO_IK"}

        _set_status(robot_id, phase="RELEASE")
        rig.gripper.open()
        for _ in range(RELEASE_WAIT):
            self.world.step(render=not HEADLESS)
        released = not holding(rig.gripper.gripped())
        _set_status(robot_id, gripped=not released)

        _set_status(robot_id, phase="RETRACT")
        retract_goal = slot_world + np.array([0, 0, approach_dist_m])
        self._servo_tcp(robot_id, retract_goal, "RETRACT")
        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)

        _set_status(robot_id, phase="DONE" if released else "FAILED",
                    message="" if released else "흡착이 안 풀렸다")
        return {"success": bool(released), "gripped": bool(not released),
               "fail_reason": "NONE" if released else "COLLISION"}

    def get_flange_pose_world(self, magazine_key, robot_id=DEFAULT_ROBOT_ID):
        """편의 메서드 — task_manager 개발/테스트용. 실측 GT 플랜지 pose 를
        base_link 프레임으로 돌려준다 (QR 대신 GT 로 파이프라인만 먼저
        확인하고 싶을 때)."""
        rig = self.rigs[robot_id]
        meas = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
        m = meas["magazines"][magazine_key]
        fc = np.array(m["flange_center"]); fc[2] = m["flange_top_z"]
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        p_rel = R_base.T @ (fc - base_p)
        return {"position": p_rel.tolist(), "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}

    def debug_capture_via_widget(self, path, camera_prim=None, settle_frames=30, wait_frames=120):
        """디버그 전용 — omni.replicator.core(AnnotatorRegistry/FabricReader)를
        완전히 안 거치는 별도 경로로 캡쳐해본다. omni.kit.widget.viewport.capture
        가 쓰는 것과 같은 네이티브 Kit 캡쳐(omni.renderer_capture)라서, 지금까지
        재현된 "annotator 가 항상 빈 프레임" 버그가 여기도 재현되는지가
        Replicator/FabricReader 쪽 문제인지 아니면 렌더러 자체 문제인지를
        가른다."""
        import os
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file

        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("활성 뷰포트를 찾을 수 없다 — headless 모드에서는 안 통한다")
        if camera_prim:
            viewport.camera_path = camera_prim
        for _ in range(settle_frames):
            self.world.step(render=True)

        try:
            if os.path.exists(path):
                os.remove(path)
        except OSError:
            pass

        capture_viewport_to_file(viewport, file_path=path)
        for _ in range(wait_frames):
            self.world.step(render=True)
            if os.path.exists(path) and os.path.getsize(path) > 0:
                return {"ok": True, "saved": path, "size": os.path.getsize(path)}
        return {"ok": False, "saved": None}

    def debug_capture_aov(self, aov_name, camera_prim=None, settle_frames=30, wait_frames=120):
        """디버그 전용 — Replicator/AnnotatorRegistry 를 거치지 않고
        (omni.kit.widget.viewport.capture 의 MultiAOVByteCapture 로) 임의의
        AOV 하나를 raw 바이트로 받아본다. depth(distance_to_image_plane)가
        이 경로로도 되는지 확인하는 용도 — 정확한 raw AOV 이름을 모르니
        여러 후보를 넣어보고 aov_map 에 뭐가 실제로 들어있는지도 로그로
        남긴다."""
        import carb.settings
        from omni.kit.viewport.utility import get_active_viewport, add_aov_to_viewport
        from omni.kit.widget.viewport.capture import MultiAOVByteCapture

        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("활성 뷰포트를 찾을 수 없다")
        if camera_prim:
            viewport.camera_path = camera_prim
        # ★ add_aov_to_viewport() 자체에 버그가 있다 —
        # /app/hydra/renderSettings/saveUsdAttributes 가 True 일 때 타는
        # 분기가 `for render_var_prims in render_var_prims:` 로 루프
        # 변수를 자기 자신에 덮어써서 그 안의 `render_var_prim`(단수)이
        # UnboundLocalError 로 죽는다(실측 재현). 이 세션 설정값이 True 라
        # 매번 그 분기를 탄다 — False 분기(버그 없음)를 강제로 타게 만든다.
        carb.settings.get_settings().set("/app/hydra/renderSettings/saveUsdAttributes", False)
        add_aov_to_viewport(viewport, aov_name)
        for _ in range(settle_frames):
            self.world.step(render=True)

        result = {}
        seen_aovs = []

        def _on_capture(buffer, buffer_size, width, height, byte_format):
            result["got"] = True
            result["width"] = width
            result["height"] = height
            result["byte_format"] = str(byte_format)
            result["buffer_size"] = buffer_size

        class _Probe(MultiAOVByteCapture):
            def capture(self, aov_map, frame_info, hydra_texture, result_handle):
                seen_aovs.extend(list(aov_map.keys()))
                aov_data = aov_map.get(aov_name)
                if aov_data:
                    tex = aov_data.get("texture", {})
                    result["aov_data_keys"] = list(aov_data.keys())
                    result["texture_keys"] = list(tex.keys())
                    result["texture_info"] = {k: str(v) for k, v in tex.items()
                                               if k != "rp_resource"}
                return super().capture(aov_map, frame_info, hydra_texture, result_handle)

        cap = _Probe([aov_name], [_on_capture])
        self._debug_cap_ref = cap  # GC 방지 — 콜백 끝날 때까지 붙잡아둔다
        viewport.schedule_capture(cap)
        for _ in range(wait_frames):
            self.world.step(render=True)
            if result.get("got"):
                break
        result["requested_aov"] = aov_name
        result["available_aovs"] = seen_aovs
        return result

    def debug_state(self, robot_id=DEFAULT_ROBOT_ID):
        """디버그 전용 — tcp/매거진 world pose 와 간격을 바로 본다.

        rig.current_magazine_path(PICK 시도 때 pick_phase1_approach 가
        찾아둔 실제 인스턴스)를 쓴다 — 하드코딩된 MAGAZINE_XFORM_PATH
        (magazine_1_orange)를 그대로 뒀더니 실제로 집은 게 magazine_2_blue
        여도 엉뚱한 매거진 위치가 찍혀서 디버깅에 혼선을 줬다."""
        rig = self.rigs[robot_id]
        tcp = self._get_tcp_pose(robot_id)
        mag_p, mag_q = get_world_pose(rig.current_magazine_path)
        cx, top_z, height = measure_prim(rig.current_magazine_path)
        return {"tcp_world": tcp.tolist(), "magazine_world_pos": mag_p.tolist(),
               "magazine_path": rig.current_magazine_path,
               "magazine_top_z": top_z, "magazine_height": height,
               "magazine_center_xy": cx.tolist(),
               "gripped": holding(rig.gripper.gripped()),
               "dist_tcp_to_mag_top": float(np.linalg.norm(tcp - np.array([cx[0], cx[1], top_z])))}

    def debug_list_cameras(self, root_path="/World/Robots/nova_carter1"):
        """디버그 전용 — root 아래 Camera 타입 프림 경로를 전부 나열한다.
        대조군으로 쓸 다른 카메라(front_hawk 등)를 찾을 때 쓴다."""
        root = self.stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            return {"root": root_path, "valid": False, "cameras": []}
        cams = [str(p.GetPath()) for p in Usd.PrimRange(root)
                if p.GetTypeName() == "Camera"]
        return {"root": root_path, "valid": True, "cameras": cams}

    def debug_capture_prim(self, camera_prim, path, width=640, height=480):
        """디버그 전용 — 임의의 카메라 prim 하나로 새 render_product 를 만들어
        한 번 찍어본다. self._rgb/self._depth(손목 카메라 전용)는 건드리지
        않는다 — CAMERA_PRIM 이외의 카메라로 렌더 파이프라인 자체가 이
        세션에서 살아있는지 대조군으로 볼 때 쓴다."""
        import cv2
        import omni.replicator.core as rep
        prim = self.stage.GetPrimAtPath(camera_prim)
        if not prim.IsValid():
            raise RuntimeError(f"{camera_prim} 프림이 없다")
        rp = rep.create.render_product(camera_prim, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])
        for _ in range(60):
            self.world.step(render=True)
        ok = False
        raw = np.asarray(rgb.get_data())
        for _ in range(240):
            raw = np.asarray(rgb.get_data())
            if raw.ndim == 3:
                ok = True
                break
            self.world.step(render=True)
        result = {"camera_prim": camera_prim, "ok": ok}
        if ok:
            img = raw[:, :, :3][:, :, ::-1].copy()
            cv2.imwrite(path, img)
            result["saved"] = path
            result["shape"] = list(img.shape)
        else:
            result["last_shape"] = list(raw.shape)
        rgb.detach()
        return result

    def debug_check_camera_prim(self, robot_id=DEFAULT_ROBOT_ID):
        """디버그 전용 — 이 로봇의 카메라 prim 이 지금 스테이지에 실제로
        존재/로드돼 있는지 확인한다. rgb annotator 가 계속 shape=(0,) 을 줄
        때 render_product 가 애초에 존재하지 않는 prim 을 가리키고 있는
        건 아닌지 가른다."""
        rig = self.rigs[robot_id]
        prim = self.stage.GetPrimAtPath(rig.camera_prim)
        info = {"path": rig.camera_prim, "valid": prim.IsValid()}
        if prim.IsValid():
            info["type"] = prim.GetTypeName()
            info["active"] = prim.IsActive()
        # 조상 중 payload 가 unloaded 인 게 있는지 위로 훑는다 — 자식 경로가
        # 안 보이는 가장 흔한 이유다(configure_gripper_limits 의 short_gripper
        # 사례와 같은 종류).
        chain = []
        p = self.stage.GetPrimAtPath(rig.gripper_prim)
        for name in ["", "rsd455", "RSD455", "Camera_OmniVision_OV9782_Color"]:
            if name:
                p = p.GetChild(name) if p.IsValid() else p
            chain.append({
                "path": str(p.GetPath()) if p.IsValid() else f"<invalid after {name!r}>",
                "valid": p.IsValid(),
                "hasPayload": p.HasPayload() if p.IsValid() else None,
                "isLoaded": p.IsLoaded() if p.IsValid() else None,
            })
        info["chain"] = chain
        return info

    def debug_capture(self, path, force=False, robot_id=DEFAULT_ROBOT_ID):
        """디버그 전용 — 지금 이 로봇의 손목 카메라가 보는 그림을 저장한다."""
        import cv2
        rig = self.rigs[robot_id]
        self._ensure_camera_warm(robot_id, force=force)
        rgb, _depth = self._capture_frame(robot_id)
        cv2.imwrite(path, rgb)
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        l6_p, l6_q = get_world_pose(rig.ee_link_path)
        return {"saved": path, "chassis_world": base_p.tolist(),
               "link6_world": l6_p.tolist()}

    def debug_dual_viewport_capture(self, timeout_frames=240):
        """실험 전용 — 뷰포트를 하나 더 만들어(create_viewport_window) 로봇
        둘의 손목캠을 각자 전담시키고, 두 캡쳐를 **같은 프레임 루프에서
        동시에 스케줄**해서 둘 다 독립적으로 완료되는지 본다.

        목적: annotator(FabricReader) 경로 없이(_ensure_camera_warm 독스트링
        참고 — 그 경로는 이 세션에서 항상 빈 프레임을 준다) 뷰포트 캡쳐
        방식을 두 개로 늘릴 수 있는지 확인 — patrol 중 로봇 둘이 동시에
        scan_qr 을 폴링하면 뷰포트 하나로는 서로 밀어낸다(carrier_code_reader
        decode_hz 폴링 참고)."""
        from omni.kit.viewport.utility import (
            get_active_viewport, create_viewport_window, add_aov_to_viewport,
        )
        from omni.kit.widget.viewport.capture import MultiAOVByteCapture
        import carb.settings

        vp1 = get_active_viewport()
        vp1.camera_path = self.rigs["robot1"].camera_prim

        if getattr(self, "_second_viewport_window", None) is None:
            self._second_viewport_window = create_viewport_window(
                name="SecondCam", width=640, height=480,
                camera_path=self.rigs["robot2"].camera_prim)
            if self._second_viewport_window is None:
                return {"ok": False, "reason": "create_viewport_window() 이 None 을 반환했다"}
        vp2 = self._second_viewport_window.viewport_api
        vp2.camera_path = self.rigs["robot2"].camera_prim

        carb.settings.get_settings().set("/app/hydra/renderSettings/saveUsdAttributes", False)
        add_aov_to_viewport(vp1, DEPTH_AOV_NAME)
        add_aov_to_viewport(vp2, DEPTH_AOV_NAME)
        for _ in range(30):
            self.world.step(render=True)

        result = {"vp1": {}, "vp2": {}}

        def _make_cb(tag, key):
            def _cb(buffer, buffer_size, width, height, byte_format):
                result[tag][key] = [height, width]
            return _cb

        cap1 = MultiAOVByteCapture(
            ["", DEPTH_AOV_NAME], [_make_cb("vp1", "rgb"), _make_cb("vp1", "depth")])
        cap2 = MultiAOVByteCapture(
            ["", DEPTH_AOV_NAME], [_make_cb("vp2", "rgb"), _make_cb("vp2", "depth")])
        self._pending_dual_capture = (cap1, cap2)   # GC 방지
        vp1.schedule_capture(cap1)
        vp2.schedule_capture(cap2)

        frames_to_vp1 = None
        frames_to_vp2 = None
        for i in range(1, timeout_frames + 1):
            self.world.step(render=True)
            if frames_to_vp1 is None and "rgb" in result["vp1"] and "depth" in result["vp1"]:
                frames_to_vp1 = i
            if frames_to_vp2 is None and "rgb" in result["vp2"] and "depth" in result["vp2"]:
                frames_to_vp2 = i
            if frames_to_vp1 is not None and frames_to_vp2 is not None:
                break
        self._pending_dual_capture = None
        return {
            "vp1": result["vp1"], "vp2": result["vp2"],
            "frames_to_vp1": frames_to_vp1, "frames_to_vp2": frames_to_vp2,
            "both_ok": bool(
                result["vp1"].get("rgb") and result["vp1"].get("depth") and
                result["vp2"].get("rgb") and result["vp2"].get("depth")),
        }


def _request_shutdown(signum, frame):
    # ★ 여기서 바로 save_state() 를 부르지 않는다 — 시그널 핸들러는 메인
    # 스레드가 어느 bytecode 를 실행 중이든 끼어들어 실행되므로, world.step()
    # 이나 PhysX 호출 도중일 수도 있다(스레드 세이프하지 않음). 플래그만
    # 세워 두고 main() 의 루프가 다음 반복 시작 지점(안전한 지점)에서 보고
    # 빠져나가게 한다.
    global _shutdown_requested
    _shutdown_requested = True


_shutdown_requested = False


def main():
    port = int(os.environ.get("SIM_BACKEND_PORT", "8765"))
    backend = Backend()
    # SIGKILL(-9) 은 못 잡는다 — 그 경우 상태 저장 없이 죽고, 다음 기동은
    # 마지막으로 저장된(또는 아예 없는) 스냅샷으로 시작한다.
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)
    # 0.0.0.0 = 다른 머신에서도 받는다. ROS 노드를 일반 PC 에서, Isaac 을 GPU PC 에서
    # 돌리는 구성이라 127.0.0.1 이면 sim_client 가 붙지 못한다(Connection refused).
    # 클라이언트 쪽은 SIM_BACKEND_HOST 로 이 머신의 IP 를 준다(sim_client.py:24).
    threading.Thread(target=_rpc_serve, args=("0.0.0.0", port), daemon=True).start()
    for robot_id in backend.rigs:
        _set_status(robot_id, phase="IDLE", message="ready")

    print("=" * 70)
    print("  sim_backend ready — RPC 대기 중")
    print("=" * 70)

    METHODS = {
        "teleport_base": backend.teleport_base,
        "reset_magazine": backend.reset_magazine,
        "observe_pose": backend.observe_pose,
        "scan_qr": backend.scan_qr,
        "pick_observe_flange": backend.pick_observe_flange,
        "pick_phase1_approach": backend.pick_phase1_approach,
        "pick_phase2_finish": backend.pick_phase2_finish,
        "place_phase1_approach": backend.place_phase1_approach,
        "place_phase2_finish": backend.place_phase2_finish,
        "get_place_slot_pose_base_link": backend.get_place_slot_pose_base_link,
        "get_flange_pose_world": backend.get_flange_pose_world,
        "debug_capture": backend.debug_capture,
        "debug_dual_viewport_capture": backend.debug_dual_viewport_capture,
        "debug_state": backend.debug_state,
        "debug_check_camera_prim": backend.debug_check_camera_prim,
        "debug_list_cameras": backend.debug_list_cameras,
        "debug_capture_prim": backend.debug_capture_prim,
        "debug_capture_via_widget": backend.debug_capture_via_widget,
        "debug_capture_aov": backend.debug_capture_aov,
    }

    while simulation_app.is_running() and not _shutdown_requested:
        backend.world.step(render=not HEADLESS)
        try:
            while True:
                call = _inbox.get_nowait()
                fn = METHODS.get(call.method)
                if fn is None:
                    call.error = f"알 수 없는 method: {call.method}"
                else:
                    try:
                        call.result = fn(**call.params)
                    except Exception as e:
                        import traceback
                        traceback.print_exc()
                        call.error = f"{type(e).__name__}: {e}"
                call.done.set()
        except queue.Empty:
            pass

    if _shutdown_requested:
        print("   종료 신호 수신 — 상태 스냅샷 저장 후 닫는다")
    backend.save_state()
    simulation_app.close()


if __name__ == "__main__":
    main()

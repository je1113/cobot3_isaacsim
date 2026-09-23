"""
Isaac Sim 실행 백엔드 — 진짜 rclpy 노드들이 이 파일을 로컬 소켓(JSON-RPC)으로 부린다.

Isaac 5.1 의 kit 파이썬(3.11)에서는 시스템 ROS2 Jazzy(3.12용)의 rclpy 를 못 쓴다.
그래서 이 프로세스는 JSON-RPC 서버만 열고, 실제 ROS2 노드(시스템 python 3.12)가
이 소켓을 통해서만 명령을 보낸다.

프로토콜: TCP, 줄 단위 JSON. {"id","method","params"} -> {"id","result"} / {"id","error"}

실행 (워크스페이스 루트에서):
    ./isaacpjt/ros_bridge/run_sim_backend.sh
    ./isaacpjt/ros_bridge/run_sim_backend.sh --check    # Isaac 안 띄우고 환경만 검사

★ isaac_python 으로 직접 띄우지 마라 — ROS 를 source 한 셸의 환경을 물려받아
  SimulationApp(...) 생성자에서 세그폴트로 죽는다. 격리 이유는 run_sim_backend.sh 참고.

머신이 둘일 때 (ROS 는 일반 PC, Isaac 은 GPU PC):
    GPU PC   ./isaacpjt/ros_bridge/run_sim_backend.sh   # 0.0.0.0 에 바인드
    ROS PC   export SIM_BACKEND_HOST=<GPU PC IP>
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
# ★ 2026-09-23 시도했다가 기각: 표준 Replicator 캡쳐(rep.create.render_product
#   + AnnotatorRegistry)가 이 세션에서 항상 빈 프레임(shape=(0,))만 준다.
#   Isaac Sim 5.1.0 의 알려진 이슈(멀티 GPU 렌더링에서 annotator 가 깨짐,
#   GitHub isaac-sim/IsaacSim#507)와 정황이 비슷해서 SIM_MULTI_GPU=0 으로
#   꺼 보고 debug_capture_prim 으로 재확인했지만, scene ready 까지 정상
#   부팅된 세션에서도 여전히 빈 프레임이었다 — 원인이 아니다. 토글은
#   남겨두되(기본값 True, 동작 그대로) 이 가설은 더 안 판다.
MULTI_GPU = os.environ.get("SIM_MULTI_GPU", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS, "multi_gpu": MULTI_GPU})

# 씬에 박혀 있는 ROS2 브릿지 OmniGraph(odom·lidar·clock 퍼블리셔)는 이 확장이
# 꺼져 있으면 안 돈다 — Nav2 가 그 토픽들을 받아야 해서 필요하다.
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
# omni.replicator.core 를 먼저 켜야 카테고리 등록 경합(세그폴트·빈 프레임)을 피한다.
enable_extension("omni.replicator.core")
enable_extension("isaacsim.ros2.bridge")
# SurfaceGripper 는 이 익스텐션이 등록하는 OmniGraph 노드 타입이다 — 스테이지를
# 열기 전에 켜야 한다.
enable_extension("isaacsim.robot.surface_gripper")
# 매거진 스포너(isaacpjt/scripts/magazine_spawner.py)는 Script API(BehaviorScript)
# 라서 이 확장이 켜져 있어야 on_init/on_play 콜백이 불린다.
try:
    enable_extension("omni.kit.scripting")  # Isaac Sim 5.1
except Exception:
    enable_extension("omni.behavior.scripting.core")  # 최신 Kit
# 스크립트 실행 보안 확인 팝업을 끈다 — 헤드리스/자동 실행엔 응답할 사람이 없어
# 팝업이 뜨면 스크립트가 영원히 안 실행된다.
import carb.settings

carb.settings.get_settings().set_bool("/app/scripting/ignoreWarningDialog", True)

import ctypes

import numpy as np
import omni.usd
import yaml
from pxr import Usd, UsdGeom, UsdPhysics


def _pycapsule_to_bytes(capsule, size):
    """렌더 캡쳐 콜백이 주는 PyCapsule 에서 실제 바이트를 꺼낸다."""
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

# 기본 씬. SIM_WORLD_USD 로 다른 씬(예: static_test 용 simple_factory_layout_test.usda)을
# 줄 수 있다 — 두 씬은 prim 경로가 같아야 한다(nova_carter1, magazine_1_orange 등).
WORLD_USD = os.environ.get("SIM_WORLD_USD") or str(ISAACPJT / "worlds/simple_factory_layout.usda")
URDF_PATH = str(ISAACPJT / "M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESC_PATH = str(ISAACPJT / "M0609/descriptor/m0609_description.yaml")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
GRASP_YAML = WS_ROOT / "src/cobot3_bringup/config/grasp.yaml"
CARRIERS_YAML = WS_ROOT / "src/cobot3_bringup/config/carriers.yaml"
MEASURED = ISAACPJT / "tools/out/layout_measured.yaml"

# 정지(SIGINT/SIGTERM) 시 save_state() 가 쓰고, 다음 기동에서 _restore_state() 가
# 읽는 런타임 스냅샷(.gitignore 대상). 씬마다 다른 파일을 쓰도록 이름에 스템을 넣는다.
STATE_SNAPSHOT_PATH = ISAACPJT / f"ros_bridge/.state_snapshot.{Path(WORLD_USD).stem}.json"

EE_LINK_NAME = "link_6"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# 로봇 두 대 — 씬의 nova_carter1/nova_carter2 가 완전히 같은 구조로 미러링돼 있다.
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
        "chassis_link": f"{base_xform}/chassis_link",  # ROS 규약의 base_link(AMR 섀시)
        "ee_link": f"{robot_prim}/{EE_LINK_NAME}",
        "gripper_prim": gripper_prim,
        "camera_prim": f"{gripper_prim}/rsd455/RSD455/Camera_OmniVision_OV9782_Color",
        "articulation_root_candidates": [f"{base_xform}/chassis_link", base_xform],
    }


# 로봇별 실제 경로는 Backend.rigs[robot_id](RobotRig)가 들고 있다.
# _capture_frame() 이 raw AOV 로 요청하는 depth 채널의 실제 이름 — Replicator 의
# annotator 이름("distance_to_image_plane")과 다르다.
DEPTH_AOV_NAME = "DistanceToImagePlaneSD"
# observe_pose(QR 용)와 pick(흡착 유지력용)의 강성을 분리한다 — 하나로 쓰면
# QR 디코드가 깨지거나 흡착이 SLIP 난다.
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e5, 1e4, 2700.0
DRIVE_STIFFNESS_PICK = 1e8
# ★ 2026-09-23: 12_pick_test.py(검증판)는 DRIVE_MAX_FORCE 도 stiffness 와
#   같이 1e8 이다. 여기는 그동안 부팅 기본값(2700.0, DRIVE_MAX_FORCE)에
#   고정돼 있었다 — _set_arm_stiffness() 가 stiffness 만 바꾸고 max force
#   는 안 건드렸다. stiffness 는 1e8 인데 낼 수 있는 힘은 2700N 로 묶여
#   있으면, 필요한 힘이 그 한계를 넘을 때마다 잘리는(saturation) 게 매
#   스텝 반복돼 흔들림(limit-cycle 진동)으로 나타난다 — pick 중 6D 자세가
#   떨리던 원인. stiffness 와 짝을 맞춰 pick 때만 이 값도 같이 올린다.
DRIVE_MAX_FORCE_PICK = 1e8
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
# 부팅 시 팔을 이 자세로 옮길지("" 면 안 옮기고 홈에서 시작). 로봇은 항상 홈
# 자세로 시작해야 한다 — 뻗은 채로 주행하면 Nav2 코스트맵이 팔을 못 본다.
BOOT_POSE_NAME = ""

# 팔은 항상 홈(READY_JOINTS_DEG)에서 시작한다 — _restore_state() 가 이전 종료
# 시점의 팔 관절값을 복원해도 여기서 덮어쓴다.
BOOT_ARM_HOME = True

# 매거진/스택도 매 기동마다 씬 원위치에서 시작한다(시험 재현성). 베이스 pose 는
# 복원한 그대로 둔다 — 어차피 Nav2 가 다시 몰고 간다.
BOOT_RESET_MAGAZINES = True

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

# 12_place_test.py 검증값 — ConveyorFrame 바로 앞. 실제 로더 슬롯 지오메트리가
# 아직 씬에 없어서 이 지점을 place 목표로 쓴다.
PLACE_TARGET_XY = np.array([4.1, 0.0])
CONVEYOR_BELT_Z = 0.6
PLACE_APPROACH_HEIGHT_OFFSET = 0.15
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


def prim_live(stage, path):
    """prim 이 있고 활성(active)인가. IsValid() 만으로는 부족하다 — 매거진
    스포너 도입 이후 선반 슬롯의 매거진은 전부 active=false 인 스폰 틀이다."""
    prim = stage.GetPrimAtPath(path)
    return bool(prim) and prim.IsValid() and prim.IsActive()


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
    """open_stage()/Load() 뒤 외부 payload 가 아직 안 붙었을 수 있으니,
    로드 대기 개수가 0이 될 때까지 프레임을 돌린다."""
    for _ in range(min_frames):
        simulation_app.update()
    for _ in range(max_frames - min_frames):
        if ctx.get_stage_loading_status()[2] == 0:
            break
        simulation_app.update()


# 12_pick_test.py 검증값. 에셋 기본값(coaxial/shear=0)으로 두면 아무것도 못 든다.
COAXIAL_FORCE_LIMIT = 200.0
SHEAR_FORCE_LIMIT = 100.0
MAX_GRIP_DISTANCE = 0.03


def configure_gripper_limits(stage, gripper_prim, max_wait_frames=180):
    """씬의 SurfaceGripper 노드 한계값을 12_pick_test.py 검증값으로 덮어쓴다.
    short_gripper 는 외부 payload 라 로드가 몇 프레임 늦을 수 있어 재시도한다.
    반환값(SurfaceGripper 노드 경로)이 중요하다 — SurfaceGripperCtl 은 부모
    prim 이 아니라 이 노드 경로를 받아야 한다."""
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
# 로봇별로 따로 둔다 — 하나로 합치면 로봇1이 PICK 중일 때 로봇2의 상태 조회에
# 로봇1의 phase 가 찍힌다.
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

    이 상태를 Backend 인스턴스 전체에 하나만 두면 안 된다 — 로봇 두 대의
    호출이 같은 소켓에서 번갈아 처리되면서 서로의 중간 상태를 덮어쓴다."""

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
        self.pending_capture = None    # 진행 중인 캡쳐를 GC 로부터 붙잡아두는 자리
        self.viewport = None           # 이 로봇 전담 뷰포트 — _get_or_create_viewport() 가 채운다
        self.viewport_window = None    # 새로 만든 창이면 그 객체(GC 방지)

        self.current_magazine_path = MAGAZINE_XFORM_PATH  # PICK 전 기본값(레거시 메서드용)
        self.flange_world = None       # pick APPROACH -> FINISH 로 넘기는 목표
        self.slot_world = None         # place APPROACH -> FINISH
        self.place_approach_dist_m = PLACE_APPROACH_HEIGHT_OFFSET


# ══════════════════════════════════════════════════════════════
#  씬 / 로봇
# ══════════════════════════════════════════════════════════════
class Backend:
    def __init__(self):
        # Stop → Play 복구 표시(_ensure_live). 맨 앞에 둔다 — 아래 부팅 중에
        # _restore_state → _set_joint_deg → _require_playing 이 이미 읽는다.
        self._needs_reinit = False
        self.frames = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
        self.grasp = yaml.safe_load(GRASP_YAML.read_text(encoding="utf-8"))
        self.carriers = yaml.safe_load(CARRIERS_YAML.read_text(encoding="utf-8"))

        ctx = omni.usd.get_context()
        ctx.open_stage(WORLD_USD)
        wait_for_stage_load(ctx)
        self.stage = ctx.get_stage()
        # short_gripper payload 가 open_stage() 뒤에도 로드 안 된 채로 남을 수
        # 있어 명시적으로 Load() 한다.
        self.stage.Load()
        wait_for_stage_load(ctx)

        self.rigs = {robot_id: RobotRig(robot_id, carter_name)
                     for robot_id, carter_name in ROBOT_CARTER_NAME.items()}

        for rig in self.rigs.values():
            configure_drives(self.stage, rig.robot_prim_path)
            rig.gripper_node_path = configure_gripper_limits(self.stage, rig.gripper_prim)
        # 고정 매거진이 스포너의 비활성 틀이면 self.magazine 을 None 으로 둔다(아래).
        self._fixed_magazine = prim_live(self.stage, MAGAZINE_XFORM_PATH)
        if not self._fixed_magazine:
            print(f"   고정 매거진 {MAGAZINE_XFORM_PATH} 은 비활성 스폰 틀이다 — "
                  f"매거진은 magazine_spawner 가 런타임에 만든다")

        self.world = World(stage_units_in_meters=1.0)
        for rig in self.rigs.values():
            root_path = resolve_articulation_root(
                rig.articulation_root_candidates, rig.robot_prim_path)
            rig.robot = self.world.scene.add(SingleManipulator(
                prim_path=root_path, name=f"m0609_robot_{rig.robot_id}",
                end_effector_prim_path=rig.ee_link_path))
        # 고정 매거진은 스포너 없는 옛 씬에서만 물리 객체다 — 지금 씬에서는 None.
        self.magazine = (self.world.scene.add(SingleRigidPrim(
            prim_path=MAGAZINE_XFORM_PATH, name="magazine"))
            if self._fixed_magazine else None)
        self.world.reset()
        for rig in self.rigs.values():
            rig.robot.initialize()
        # 2단계 부팅 — 먼저 안전한 READY_JOINTS_DEG 로 즉시 스냅해 안정시킨 뒤,
        # 안정된 상태에서만 BOOT_POSE_NAME 으로 보간 이동한다. reset() 직후
        # 곧바로 뻗은 자세로 스냅하면 베이스가 물리 충격으로 넘어진다.
        for rig in self.rigs.values():
            q = np.zeros(rig.robot.num_dof)
            for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
                q[rig.robot.get_dof_index(name)] = np.deg2rad(deg)
            rig.robot.set_joint_positions(q)
        self._settle(SETTLE_STEPS)

        if BOOT_POSE_NAME:
            taught = yaml.safe_load((ISAACPJT / "tools/out/taught_poses.yaml").read_text(encoding="utf-8"))
            for rig in self.rigs.values():
                self._servo_joint_deg(rig.robot_id, taught[BOOT_POSE_NAME]["joints_deg"],
                                      n_steps=SETTLE_STEPS)
            print(f"  팔 부팅 자세: {BOOT_POSE_NAME}")
        else:
            print(f"  팔 부팅 자세: READY(홈) {READY_JOINTS_DEG}")

        if self.magazine is not None:
            self.magazine_spawn_pos, self.magazine_spawn_quat = self.magazine.get_world_pose()
        else:
            self.magazine_spawn_pos = self.magazine_spawn_quat = None

        # 이 씬은 같은 종류(variant) 매거진이 선반마다 여러 인스턴스로 있어서
        # 이름만으로는 "지금 집은 그것"을 못 가른다 — 좌표로 가른다
        # (_find_nearest_magazine). 그 후보 목록을 여기서 한 번만 읽어둔다.
        meas_layout = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
        # 스포너의 비활성 틀은 뺀다 — 물리 객체가 아니라 잴 수도 옮길 수도 없다.
        self._all_magazine_prims = [m["prim"] for m in meas_layout["magazines"].values()
                                    if prim_live(self.stage, m["prim"])]
        # yaml 목록은 기본 씬 것뿐이다 — 다른 씬/스택도 찾을 수 있게 flange_plate
        # 자식을 가진 prim 을 스테이지에서 훑어 후보에 더한다.
        try:
            for prim in Usd.PrimRange(self.stage.GetPrimAtPath("/World")):
                if prim.GetChild("flange_plate").IsValid():
                    path = str(prim.GetPath())
                    if path not in self._all_magazine_prims:
                        self._all_magazine_prims.append(path)
        except Exception as e:      # noqa: BLE001 — 후보 탐색 실패는 치명적이지 않다
            print(f"   !! 플랜지 후보 탐색 실패({e}) — layout_measured.yaml 목록만 쓴다")
        print(f"   플랜지 후보 {len(self._all_magazine_prims)} 개")

        for rig in self.rigs.values():
            base_pos0, base_quat0 = get_world_pose(rig.base_link_path)
            rig.lula = LulaKinematicsSolver(robot_description_path=DESC_PATH, urdf_path=URDF_PATH)
            rig.lula.set_robot_base_pose(robot_position=base_pos0, robot_orientation=base_quat0)
            rig.solver = ArticulationKinematicsSolver(
                robot_articulation=rig.robot, kinematics_solver=rig.lula,
                end_effector_frame_name=EE_LINK_NAME)
            rig.gripper = SurfaceGripperCtl(rig.gripper_node_path)
        # 공용 기본값(차체 yaw 0 인 경우) — 실제 IK 에는 로봇별 _target_quat 를 쓴다.
        self.target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)

        # 이전 종료 시점의 로봇/씬 상태가 있으면 위 기본 스폰 배치를 덮어쓴다.
        self._restore_state()

        # 팔만 다시 홈으로 — _restore_state() 가 되살린 뻗은 자세를 덮어쓴다.
        # 베이스 pose 와 씬 오브젝트는 복원된 그대로 둔다.
        if BOOT_ARM_HOME:
            # 순간 스냅이 아니라 보간으로 옮긴다 — 한 프레임에 꺾으면 물리
            # 충격이 생겨 흡착이 SLIP/NO_ATTACH 로 깨진다.
            for rig in self.rigs.values():
                self._servo_joint_deg(rig.robot_id, READY_JOINTS_DEG,
                                      n_steps=SETTLE_STEPS)
            self._settle(SETTLE_STEPS)
            print(f"  팔 시작 자세: 홈 {READY_JOINTS_DEG} (로봇 {len(self.rigs)}대, 보간 이동)")

        st = self.frames["static_transforms"]
        self.R_l6_cam = quat_xyzw_to_mat(st["m0609_tool0__camera_link"]["quat_xyzw"])
        self.t_l6_cam = np.array(st["m0609_tool0__camera_link"]["xyz"])
        self.R_cam_opt = quat_xyzw_to_mat(st["camera_link__camera_color_optical_frame"]["quat_xyzw"])
        ci = self.frames["wrist_camera"]["camera_info_observed"]
        self.K = np.array(ci["k"], dtype=float).reshape(3, 3)
        self.dist = np.array(ci["d"], dtype=float)

        # ── Stop → Play 복구 ────────────────────────────────────────────
        # Isaac Sim 뷰포트에서 Stop 하면 physics view 가 무효화되는데, scene
        # 에 넣은 객체는 옛 view 핸들을 그대로 들고 있어 get_joint_positions()
        # 가 None 을 돌려준다. STOP 이벤트를 표시해 두고, 다시 Play 된 첫
        # 순간에 scene 을 새 view 로 다시 묶는다(_ensure_live).
        # `import omni.timeline` 을 함수 안에서 쓰면 omni 가 지역 이름이 되어
        # 위쪽 omni.usd.get_context() 가 UnboundLocalError 로 죽으므로 별칭을 쓴다.
        import omni.timeline as omni_timeline
        self._stop_sub = (omni_timeline.get_timeline_interface()
                          .get_timeline_event_stream()
                          .create_subscription_to_pop_by_type(
                              int(omni_timeline.TimelineEventType.STOP),
                              self._on_timeline_stop))

        print(f"   scene ready — robots: {list(self.rigs.keys())}")

    def _on_timeline_stop(self, _event):
        if not self._needs_reinit:
            print("   ■ 시뮬레이션 Stop — 다시 Play 하면 로봇 핸들을 새로 잡는다")
        self._needs_reinit = True

    def _ensure_live(self):
        """Stop 뒤 다시 Play 됐으면 scene 객체를 새 physics view 에 다시 묶는다.
        메인 루프와 관절을 만지는 경로(_require_playing)가 매번 부른다."""
        if not self._needs_reinit or not self.world.is_playing():
            return
        self.world.initialize_physics()
        for rig in self.rigs.values():
            rig.gripper.reinit()
            # 진행 중이던 pick/place 목표는 Stop 이 USD 를 되돌리면서 의미를
            # 잃는다 — 옛 목표로 FINISH 가 내려가지 않게 비운다.
            rig.flange_world = None
            rig.slot_world = None
        self._needs_reinit = False
        ok = all(r.robot.get_joint_positions() is not None for r in self.rigs.values())
        print(f"   ▶ 시뮬레이션 다시 Play — 로봇 핸들 재초기화 "
              f"{'완료' if ok else '실패(관절값을 여전히 못 읽는다)'}")

    def _set_arm_stiffness(self, robot_id, stiffness, max_force):
        """observe_pose() 는 QR 이 안 깨지는 (DRIVE_STIFFNESS, DRIVE_MAX_FORCE) 로,
        pick_phase1_approach() 는 (DRIVE_STIFFNESS_PICK, DRIVE_MAX_FORCE_PICK) 으로
        되돌린다 — 둘은 항상 짝으로 바뀌어야 한다(DRIVE_MAX_FORCE_PICK 주석 참고,
        stiffness 만 올리고 max force 를 그대로 두면 흔들린다)."""
        rig = self.rigs[robot_id]
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(rig.robot_prim_path)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for dt in ["angular", "linear"]:
                d = UsdPhysics.DriveAPI.Get(prim, dt)
                if d:
                    d.GetStiffnessAttr().Set(stiffness)
                    d.GetMaxForceAttr().Set(max_force)

    def _sync_ik_base(self, robot_id):
        rig = self.rigs[robot_id]
        pos, quat = get_world_pose(rig.base_link_path)
        rig.lula.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
        return pos, quat

    def _require_playing(self):
        """Stop 상태에서는 articulation view 가 무효라 관절 조작이 바로
        TypeError 로 죽는다 — 명확한 이유를 알려준다. 자동으로 다시 play()
        하지는 않는다(사용자가 일부러 Stop 한 경우와 구분이 안 된다)."""
        if not self.world.is_playing():
            raise RuntimeError(
                "시뮬레이션이 Play 상태가 아니다 — Isaac Sim 뷰포트에서 Play 를 "
                "누른 뒤 다시 시도해라 (Stop 상태에서는 로봇 articulation 을 "
                "읽거나 움직일 수 없다)")
        self._ensure_live()

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
        """_set_joint_deg 는 관절을 즉시 스냅한다 — 흡착 중에 쓰면 그 가속으로
        접합이 끊긴다. 대신 매 스텝 목표를 다시 명령해 부드럽게 움직인다."""
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

        차체 yaw 를 90도 단위로 반올림해서 그만큼 같이 돌린다 — 월드 기준
        고정 자세를 쓰면 차체 yaw 에 따라 팔이 관절 한계에 걸려 IK 가 안
        풀리는 로봇이 생긴다. 실제 yaw 를 그대로 안 쓰고 90도로 스냅하는
        이유는, 잡을 대상의 방향은 축에 정렬된 선반이 정하고 로봇의 정차
        오차까지 그리퍼에 옮기면 안 되기 때문이다."""
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

    def _settle(self, n_steps, render=None):
        """물리를 n_steps 프레임 진행시킨다. render 를 안 주면 기본 동작
        (not HEADLESS)을, 디버그 캡처처럼 항상 렌더가 필요한 곳은
        render=True 로 강제한다."""
        do_render = (not HEADLESS) if render is None else render
        for _ in range(n_steps):
            self.world.step(render=do_render)

    def _servo_tcp(self, robot_id, goal_tcp, phase_name):
        rig = self.rigs[robot_id]
        # IK 를 풀기 전에 Lula 에게 팔 베이스가 지금 어디인지 매번 알려준다 —
        # 안 하면 베이스가 움직인 뒤 솔버가 옛 자리 기준으로 풀어 IK 가 실패한다.
        self._sync_ik_base(robot_id)
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
    # 아래 대부분은 robot_id="robot1"/"robot2" 를 받는다 — 안 주면 DEFAULT_ROBOT_ID 폴백.
    def _require_fixed_magazine(self, what):
        if self.magazine is None:
            raise RuntimeError(
                f"{what}: 고정 매거진 {MAGAZINE_XFORM_PATH} 이 이 씬에서는 비활성 스폰 "
                f"틀이다 — 매거진은 magazine_spawner 가 만들고 되돌린다. Stop → Play 로 "
                f"스포너를 리셋해라")

    def reset_magazine(self, robot_id=DEFAULT_ROBOT_ID):
        """반복 테스트용. 매거진을 원위치로 되돌린다."""
        self._require_fixed_magazine("reset_magazine")
        rig = self.rigs[robot_id]
        rig.gripper.open()
        self._settle(20)
        self.magazine.set_world_pose(position=self.magazine_spawn_pos,
                                     orientation=self.magazine_spawn_quat)
        self.magazine.set_linear_velocity(np.zeros(3))
        self.magazine.set_angular_velocity(np.zeros(3))
        self._settle(30)
        rig.gripper.reinit()
        return {"ok": True}

    def save_state(self):
        """정지 시 로봇 베이스 pose·팔 관절값·그리퍼 흡착 대상과 매거진/캐리어
        배치를 스냅샷으로 남긴다. 다음 기동에서 _restore_state() 가 되돌린다
        — sim_backend 만 재기동돼도 Nav2/AMCL 이 믿는 위치와 어긋나지 않게."""
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

            # 스포너가 만든 매거진(.../Spawned/...)은 남기지 않는다 — 스포너가
            # 재기동마다 번호를 다시 매기므로, 저장해 두면 다음 판의 다른
            # 매거진이 같은 이름으로 순간이동한다. 비활성 틀도 뺀다.
            magazines = {}
            for path in self._all_magazine_prims:
                if "/Spawned/" in path or not prim_live(self.stage, path):
                    continue
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
        씌운다. world_usd 가 지금 씬과 다르면 통째로 건너뛴다 — 다른 씬의
        좌표를 그대로 적용하면 엉뚱한 곳으로 텔레포트한다."""
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

        if BOOT_RESET_MAGAZINES:
            n = len(snapshot.get("magazines") or {})
            print(f"  매거진 {n}개: 스냅샷을 건너뛰고 씬 원위치에서 시작한다")
        else:
            for path, m in (snapshot.get("magazines") or {}).items():
                if "/Spawned/" in path or not prim_live(self.stage, path):
                    continue    # 이 씬에는 없거나 비활성(스폰 틀)인 매거진 — 조용히 건너뛴다
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

        self._settle(SETTLE_STEPS)

        for robot_id, r in (snapshot.get("robots") or {}).items():
            rig = self.rigs.get(robot_id)
            if rig is None or not r.get("gripped_paths"):
                continue
            rig.gripper.close()
            self._settle(GRIP_WAIT)
            if not holding(rig.gripper.gripped()):
                print(f"   !! [{robot_id}] 재흡착 복원 실패 — 스냅샷엔 흡착 중이었는데 지금은 안 붙는다")

        print(f"   상태 스냅샷 복원 완료 ({snapshot.get('saved_at', '?')} 저장분)")

    def observe_pose(self, pose_name=None, joints_deg=None, robot_id=DEFAULT_ROBOT_ID):
        """관측 자세로 이동한다. pose_name(taught_poses.yaml 키) 또는
        joints_deg(J1..J6, 도) 중 하나로 목표를 준다 — 둘 다 있으면 joints_deg
        우선. carrier_code_reader.scan_qr() 전에 부른다."""
        self._set_arm_stiffness(robot_id, DRIVE_STIFFNESS, DRIVE_MAX_FORCE)   # QR 디코드가 깨지지 않는 값으로
        if joints_deg is not None:
            self._servo_joint_deg(robot_id, list(joints_deg))
        elif pose_name is not None:
            taught = yaml.safe_load((ISAACPJT / "tools/out/taught_poses.yaml").read_text(encoding="utf-8"))
            pose = taught[pose_name]
            self._servo_joint_deg(robot_id, pose["joints_deg"])
        else:
            raise ValueError("observe_pose: pose_name 또는 joints_deg 가 필요하다")
        self._settle(SETTLE_STEPS)
        self._ensure_camera_warm(robot_id)
        return {"ok": True, "pose": pose_name or "joints(deg): %s" % joints_deg}

    def _get_or_create_viewport(self, robot_id):
        """이 로봇 전담 뷰포트. 첫 로봇(robot1)은 기존 GUI 메인 뷰포트를
        그대로 쓰고, 나머지는 각자 독립 뷰포트 창을 새로 만든다 — 하나만
        쓰면 두 로봇이 동시에 scan_qr 을 폴링할 때 서로 밀어낸다.

        create_viewport_window() 는 창을 만들기만 하고 visible 을 안 켜줘서,
        안 보이는 창은 렌더가 안 돌아 캡쳐가 전부 검은 화면으로 나온다 —
        여기서 visible/updates_enabled 를 명시적으로 켠다."""
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
        """관측 자세 도착 후, 이 로봇 전담 뷰포트를 손목 카메라로 돌리고
        depth AOV 를 등록한다.

        Replicator/AnnotatorRegistry 경로는 이 세션에서 rgb annotator 가
        항상 빈 프레임만 줘서, 그걸 아예 안 거치는 omni.kit.widget.viewport.capture
        (Kit 자체 스크린샷이 쓰는 것과 같은 백엔드)로 캡쳐한다. depth 는
        add_aov_to_viewport() 로 등록하는데, 이 함수 자체에
        `/app/hydra/renderSettings/saveUsdAttributes` 가 True 일 때 죽는
        버그가 있어 미리 꺼서 우회한다.

        raw AOV 이름은 Replicator 표기("distance_to_image_plane")와 달리
        "DistanceToImagePlaneSD"(SD 접미사)라야 실제로 채워진다."""
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
        self._settle(30, render=True)
        # 실제로 유효한 프레임이 나오는지 한 번 확인한다. 실패하면 예외를 올린다.
        self._capture_frame(robot_id, timeout_frames=240)
        rig.capture_ready = True

    def _capture_frame(self, robot_id, timeout_frames=180):
        """이 로봇 전담 뷰포트의 render_product 에서 RGB(BGR 변환)+depth 를
        raw 바이트 콜백으로 한 프레임 받는다. 호출 전에 _ensure_camera_warm()
        으로 그 뷰포트가 이미 이 로봇의 카메라를 보고 있어야 한다."""
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
        변환해 돌려준다. observe_pose(robot_id=...) 로 이미 그 로봇의 관측
        자세/카메라가 준비돼 있어야 한다."""
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
        # QR 프레임(x=오른쪽, y=아래, z=안쪽)을 yaw 하나로 재구성한다 — 매거진은
        # 항상 upright 라 y_qr(이미지 아래)은 세계 -Z 로 고정이다.
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

    def _magazine_candidates(self):
        """지금 씬에 살아 있는 플랜지 캐리어 전부. 스포너가 매거진을
        런타임에 만들고 집어 가면 새 이름으로 다시 만들기 때문에, 부팅 때
        모은 목록만으론 모자라 매번 /World/Magazines 를 다시 훑는다."""
        out = [p for p in self._all_magazine_prims if prim_live(self.stage, p)]
        root = self.stage.GetPrimAtPath("/World/Magazines")
        if root:
            for prim in Usd.PrimRange(root):
                if prim.GetChild("flange_plate").IsValid():
                    path = str(prim.GetPath())
                    if path not in out:
                        out.append(path)
        return out

    def _find_nearest_magazine(self, flange_world):
        """지금 집으려는 매거진이 씬의 몇 번째 인스턴스인지 이름만으로 못
        가르므로, pick_phase1_approach 가 이미 아는 실제 목표 flange_world
        에 flange_plate 가 가장 가까운 인스턴스를 찾는다. 후보가 하나도
        없으면 None — 호출자가 명시적으로 처리한다."""
        best_path, best_d = None, None
        for path in self._magazine_candidates():
            try:
                cx, top_z, _h = measure_prim(f"{path}/flange_plate")
            except Exception:
                continue
            d = float(np.linalg.norm(np.array([cx[0], cx[1], top_z]) - flange_world))
            if best_d is None or d < best_d:
                best_path, best_d = path, d
        return best_path

    def pick_observe_flange(self, flange_pose_base_link, variant, robot_id=DEFAULT_ROBOT_ID):
        """PickCarrier 의 OBSERVE — qr_pose(prior) 위로 손목캠을 가져가
        flange_topview.detect_flange() 로 진짜 파지점(중심 xy·윗면 z·yaw)을
        잰다. flange_pose_base_link 는 QR 대략값으로 계산한 prior(탐색 창
        중심)이고, 반환값이 진짜 파지점(pick_phase1_approach 에 그대로
        넘긴다)이다."""
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

        # observe_tcp — target_quat(고정 top-down 접근 자세)로 팔이 도착했을 때
        # 손목캠이 prior_world 를 observe_image_offset_px 위치에 담도록
        # 카메라/TCP 위치를 역산한다.
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
        {position:[x,y,z], quat_wxyz:[..]} (position=판 윗면 중심, +Z=법선 위)."""
        rig = self.rigs[robot_id]
        self._set_arm_stiffness(robot_id, DRIVE_STIFFNESS_PICK, DRIVE_MAX_FORCE_PICK)   # 흡착 유지력 검증값으로
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        p_rel = np.array(flange_pose_base_link["position"])
        flange_world = base_p + R_base @ p_rel
        rig.flange_world = flange_world   # DESCEND 단계에서 재사용
        # 실제 목표 위치에 가장 가까운 매거진 인스턴스를 여기서 미리 찾아둔다
        # — pick_phase2_finish 의 rise/tilt 판정이 이걸 쓴다.
        nearest = self._find_nearest_magazine(flange_world)
        if nearest is None:
            _set_status(robot_id, phase="FAILED", message="씬에 살아있는 매거진이 없다")
            return {"success": False, "fail_reason": "NO_MAGAZINE", "phase": "APPROACH"}
        rig.current_magazine_path = nearest

        _set_status(robot_id, phase="APPROACH", gripped=False, gap_m=0.0, message="")
        goal = flange_world + np.array([0, 0, approach_dist_m])
        ok = self._servo_tcp(robot_id, goal, "APPROACH")
        if not ok:
            _set_status(robot_id, phase="FAILED", message="APPROACH IK 실패")
            return {"success": False, "fail_reason": "NO_IK", "phase": "APPROACH"}
        return {"success": True}

    def pick_phase2_finish(self, grip_gaps_m, offset_limit_m, lift_height_m=None,
                           robot_id=DEFAULT_ROBOT_ID):
        """DESCEND -> SUCTION -> LIFT -> STOW."""
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
            self._settle(GRIP_WAIT)
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

        # 흡착 순간 TCP 횡오차 (OFF_FLANGE 판정)
        tcp_now = self._get_tcp_pose(robot_id)
        final_offset_m = float(np.linalg.norm((tcp_now - flange_world)[:2]))

        # pick_phase1_approach 가 찾아둔 실제 인스턴스를 잰다 — 하드코딩된
        # 고정 매거진만 재면 rise 가 항상 0mm 로 나와 성공한 PICK 이 SLIP 으로
        # 오판정된다.
        target_flange_path = f"{rig.current_magazine_path}/flange_plate"
        _set_status(robot_id, phase="LIFT")
        top_z0 = measure_prim(target_flange_path)[1]
        lift_goal = flange_world + np.array([0, 0, lift_height_m])
        self._servo_tcp(robot_id, lift_goal, "LIFT")
        self._settle(HOLD_WAIT)
        gripped_after_lift = holding(rig.gripper.gripped())
        _set_status(robot_id, gripped=gripped_after_lift)
        top_z1 = measure_prim(target_flange_path)[1]
        rise_m = top_z1 - top_z0
        _, mag_q = get_world_pose(rig.current_magazine_path)
        tilt_deg = tilt_deg_from_quat(mag_q)

        if not gripped_after_lift:
            _set_status(robot_id, phase="FAILED", message="리프트 중 놓쳤다")
            return {"success": False, "fail_reason": "SLIP", "phase": "LIFT",
                   "gripped": False, "used_gap_m": float(gap),
                   "rise_mm": rise_m*1000, "tilt_deg": tilt_deg,
                   "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m}

        if final_offset_m > offset_limit_m:
            _set_status(robot_id, phase="FAILED", message="OFF_FLANGE")
            return {"success": False, "fail_reason": "OFF_FLANGE", "phase": "LIFT",
                   "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m}

        _set_status(robot_id, phase="STOW")
        # STOW: 이송 자세로. 즉시 스냅 대신 보간으로 옮긴다 — 스냅 가속으로
        # LIFT 판정 통과 뒤에도 흡착이 끊길 수 있다.
        self._servo_joint_deg(robot_id, READY_JOINTS_DEG, n_steps=SETTLE_STEPS)
        self._settle(HOLD_WAIT)

        ok = (rise_m >= LIFT_OK_MIN_M) and (tilt_deg <= TILT_MAX_DEG) and holding(rig.gripper.gripped())
        _set_status(robot_id, phase="DONE" if ok else "FAILED",
                    message="" if ok else f"판정 실패 rise={rise_m*1000:.1f}mm tilt={tilt_deg:.2f}")
        return {"success": bool(ok), "fail_reason": "NONE" if ok else "SLIP",
               "phase": "STOW",
               "gripped": bool(holding(rig.gripper.gripped())),
               "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m,
               "used_gap_m": float(gap), "rise_mm": rise_m*1000, "tilt_deg": tilt_deg}

    def gripper_state(self, robot_id=DEFAULT_ROBOT_ID):
        """지금 그리퍼가 뭔가 들고 있나 — 실제 SurfaceGripper 값.
        get_status 의 gripped 는 마지막으로 기록한 값이라 묵을 수 있어서
        (떨어뜨린 뒤에도 True 로 남는다) 판정에는 이걸 쓴다."""
        rig = self.rigs[robot_id]
        return {"gripped": bool(holding(rig.gripper.gripped()))}

    def move_joints(self, joints_deg=None, n_steps=None, robot_id=DEFAULT_ROBOT_ID):
        """팔을 관절 목표로 보간 이동한다. 흡착 중이어도 쓴다 — pick 뒤 이송 자세,
        place 앞 READY 복귀(pick_place_server 의 carry_joints_deg 참고).

        joints_deg 가 없으면 READY_JOINTS_DEG(STOW 자세). stiffness 는 건드리지
        않는다 — pick 이 올려 둔 DRIVE_STIFFNESS_PICK 을 그대로 써야 들고 옮길 때
        안 처진다(observe_pose 처럼 1e5 로 내리면 안 된다).
        기본 스텝은 STOW 의 두 배다. 이송 자세는 STOW 보다 멀리 움직여서, 같은
        스텝이면 가속이 커져 흡착이 끊길 수 있다."""
        self._require_playing()
        rig = self.rigs[robot_id]
        target = list(joints_deg) if joints_deg is not None else list(READY_JOINTS_DEG)
        self._servo_joint_deg(robot_id, target, n_steps=int(n_steps or 2 * SETTLE_STEPS))
        self._settle(HOLD_WAIT)
        gripped = bool(holding(rig.gripper.gripped()))
        _set_status(robot_id, gripped=gripped)
        return {"success": True, "joints_deg": target, "gripped": gripped}

    def get_place_slot_pose_base_link(self, robot_id=DEFAULT_ROBOT_ID):
        """편의 메서드 — place 쪽 GT. 슬롯 지오메트리가 아직 씬에 없어서,
        검증된 컨베이어 벨트 위 스테이징 지점을 base_link 프레임으로 돌려준다.
        pkg_loader 정지 지점 도착 후 호출해야 값이 맞다.

        z 는 PICK 과 같은 관례(판 윗면)를 쓴다 — 벨트 높이에 물체 바닥이
        닿으려면 flange_plate 는 물체 높이만큼 더 위여야 한다. 지금 들고 있는
        실제 인스턴스의 실측 높이를 그대로 쓴다."""
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
        pkg_loader 에 도착해 있어야 한다(실제 Nav2 이송 중에는 그리퍼가
        물체를 계속 물리적으로 붙든 채 이동하므로 재흡착이 필요 없다)."""
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

        알려진 갭: PORT_OCCUPIED(슬롯 점유 감지)는 아직 구현하지 않았다 —
        이 씬에 슬롯 자체가 없어서 항상 비어 있다고 본다."""
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
        self._settle(RELEASE_WAIT)
        released = not holding(rig.gripper.gripped())
        _set_status(robot_id, gripped=not released)

        _set_status(robot_id, phase="RETRACT")
        retract_goal = slot_world + np.array([0, 0, approach_dist_m])
        self._servo_tcp(robot_id, retract_goal, "RETRACT")
        self._settle(SETTLE_STEPS)

        _set_status(robot_id, phase="DONE" if released else "FAILED",
                    message="" if released else "흡착이 안 풀렸다")
        return {"success": bool(released), "gripped": bool(not released),
               "fail_reason": "NONE" if released else "COLLISION"}

    def get_flange_pose_world(self, magazine_key, robot_id=DEFAULT_ROBOT_ID):
        """편의 메서드 — task_manager 개발/테스트용. 실측 GT 플랜지 pose 를
        base_link 프레임으로 돌려준다(QR 대신 GT 로 파이프라인만 먼저 확인할 때)."""
        rig = self.rigs[robot_id]
        meas = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
        m = meas["magazines"][magazine_key]
        fc = np.array(m["flange_center"]); fc[2] = m["flange_top_z"]
        base_p, base_q = get_world_pose(rig.chassis_link_path)
        R_base = quat_to_matrix(base_q)
        p_rel = R_base.T @ (fc - base_p)
        return {"position": p_rel.tolist(), "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}

    def debug_capture_via_widget(self, path, camera_prim=None, settle_frames=30, wait_frames=120):
        """디버그 전용 — Replicator(AnnotatorRegistry/FabricReader)를 완전히
        안 거치는 네이티브 Kit 캡쳐로 찍어본다."""
        import os
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file

        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("활성 뷰포트를 찾을 수 없다 — headless 모드에서는 안 통한다")
        if camera_prim:
            viewport.camera_path = camera_prim
        self._settle(settle_frames, render=True)

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
        """디버그 전용 — Replicator/AnnotatorRegistry 를 거치지 않고 임의의
        AOV 하나를 raw 바이트로 받아본다."""
        import carb.settings
        from omni.kit.viewport.utility import get_active_viewport, add_aov_to_viewport
        from omni.kit.widget.viewport.capture import MultiAOVByteCapture

        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("활성 뷰포트를 찾을 수 없다")
        if camera_prim:
            viewport.camera_path = camera_prim
        # add_aov_to_viewport() 자체 버그(saveUsdAttributes=True 분기에서
        # UnboundLocalError)를 피하려고 미리 끈다.
        carb.settings.get_settings().set("/app/hydra/renderSettings/saveUsdAttributes", False)
        add_aov_to_viewport(viewport, aov_name)
        self._settle(settle_frames, render=True)

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
        """디버그 전용 — tcp/매거진 world pose 와 간격을 바로 본다. 하드코딩된
        고정 매거진 대신 pick_phase1_approach 가 찾아둔 실제 인스턴스를 쓴다."""
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
        """디버그 전용 — root 아래 Camera 타입 프림 경로를 전부 나열한다."""
        root = self.stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            return {"root": root_path, "valid": False, "cameras": []}
        cams = [str(p.GetPath()) for p in Usd.PrimRange(root)
                if p.GetTypeName() == "Camera"]
        return {"root": root_path, "valid": True, "cameras": cams}

    def debug_capture_prim(self, camera_prim, path, width=640, height=480):
        """디버그 전용 — 임의의 카메라 prim 하나로 새 render_product 를 만들어
        한 번 찍어본다(다른 카메라로 렌더 파이프라인이 살아있는지 대조군 확인용)."""
        import cv2
        import omni.replicator.core as rep
        prim = self.stage.GetPrimAtPath(camera_prim)
        if not prim.IsValid():
            raise RuntimeError(f"{camera_prim} 프림이 없다")
        rp = rep.create.render_product(camera_prim, (width, height))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([rp])
        self._settle(60, render=True)
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
        존재/로드돼 있는지 확인한다."""
        rig = self.rigs[robot_id]
        prim = self.stage.GetPrimAtPath(rig.camera_prim)
        info = {"path": rig.camera_prim, "valid": prim.IsValid()}
        if prim.IsValid():
            info["type"] = prim.GetTypeName()
            info["active"] = prim.IsActive()
        # 조상 중 payload 가 unloaded 인 게 있는지 위로 훑는다 — 자식 경로가
        # 안 보이는 가장 흔한 이유다.
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
        """실험 전용 — 뷰포트를 하나 더 만들어 로봇 둘의 손목캠을 각자
        전담시키고, 두 캡쳐를 같은 프레임 루프에서 동시에 스케줄해서 둘 다
        독립적으로 완료되는지 본다(patrol 중 동시 scan_qr 폴링 검증용)."""
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
        self._settle(30, render=True)

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
    # 시그널 핸들러에서 바로 save_state() 를 부르지 않는다 — world.step()/PhysX
    # 호출 도중일 수 있어 스레드 세이프하지 않다. 플래그만 세우고 main() 루프의
    # 안전한 지점에서 빠져나가게 한다.
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
    # 0.0.0.0 = 다른 머신에서도 받는다 — ROS 를 일반 PC, Isaac 을 GPU PC 에서
    # 돌리는 구성 지원.
    threading.Thread(target=_rpc_serve, args=("0.0.0.0", port), daemon=True).start()
    for robot_id in backend.rigs:
        _set_status(robot_id, phase="IDLE", message="ready")

    print("=" * 70)
    print("  sim_backend ready — RPC 대기 중")
    print("=" * 70)

    METHODS = {
        "reset_magazine": backend.reset_magazine,
        "observe_pose": backend.observe_pose,
        "scan_qr": backend.scan_qr,
        "pick_observe_flange": backend.pick_observe_flange,
        "pick_phase1_approach": backend.pick_phase1_approach,
        "pick_phase2_finish": backend.pick_phase2_finish,
        "move_joints": backend.move_joints,
        "gripper_state": backend.gripper_state,
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
        # Stop → Play 를 RPC 가 오기 전에 복구해 둔다.
        backend._ensure_live()
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

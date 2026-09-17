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
"""

import json
import math
import os
import queue
import socket
import sys
import threading
import time
from pathlib import Path

from isaacsim import SimulationApp

HEADLESS = os.environ.get("SIM_HEADLESS", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.usd
import yaml
from pxr import Usd, UsdGeom, UsdPhysics

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

WORLD_USD = str(ISAACPJT / "worlds/simple_factory_layout.usda")
URDF_PATH = str(ISAACPJT / "M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESC_PATH = str(ISAACPJT / "M0609/descriptor/m0609_description.yaml")
FRAMES_YAML = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
GRASP_YAML = WS_ROOT / "src/cobot3_bringup/config/grasp.yaml"
CARRIERS_YAML = WS_ROOT / "src/cobot3_bringup/config/carriers.yaml"
MEASURED = ISAACPJT / "tools/out/layout_measured.yaml"

ROBOT_PRIM_PATH = "/World/Robots/nova_carter1/m0609"
BASE_XFORM_PATH = "/World/Robots/nova_carter1"
BASE_LINK_PATH = f"{ROBOT_PRIM_PATH}/base_link"     # m0609 자체 base — Lula IK 전용
CHASSIS_LINK_PATH = f"{BASE_XFORM_PATH}/chassis_link"   # ROS 규약의 base_link(AMR 섀시).
                                                        # CarrierScan/PickCarrier 의
                                                        # frame_id="base_link" 는 이쪽이다 —
                                                        # 06 §1 TF 트리: map -> base_link(Carter)
                                                        # -> m0609_base -> tool0. 헷갈리지 말 것.
EE_LINK_NAME = "link_6"
EE_LINK_PATH = f"{ROBOT_PRIM_PATH}/{EE_LINK_NAME}"
GRIPPER_PRIM = f"{ROBOT_PRIM_PATH}/short_gripper"
CAMERA_PRIM = f"{GRIPPER_PRIM}/rsd455/RSD455/Camera_OmniVision_OV9782_Color"
ARTICULATION_ROOT_CANDIDATES = [f"{BASE_XFORM_PATH}/chassis_link", BASE_XFORM_PATH]
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

CARTER_Z = 0.07963398335074101
RESOLUTION = (1280, 720)
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

# 12_place_test.py / 14_place_test_pse.py 검증값 — ConveyorFrame(x>=4.2) 바로 앞
# 바닥(z=0) 스테이징 지점. pkg_loader 슬롯 자체(포트 지오메트리)는 아직 씬에
# 없어서, 이 바닥 지점을 그대로 place 목표로 쓴다 (다음 범위: 실제 로더 슬롯).
PLACE_TARGET_XY = np.array([4.0, 0.0])
FLOOR_Z = 0.0
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


def configure_drives(stage):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
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


# 12_pick_test.py 검증값. 에셋 기본값(coaxial/shear=0)으로 두면 아무것도 못
# 든다 — frames.yaml suction_gripper.asset_defaults 주석 참고.
COAXIAL_FORCE_LIMIT = 200.0
SHEAR_FORCE_LIMIT = 100.0
MAX_GRIP_DISTANCE = 0.03


def configure_gripper_limits(stage, gripper_prim):
    """씬에 붙어 있는 SurfaceGripper 노드의 한계값을 12_pick_test.py 값으로
    덮어쓴다. 이걸 빼먹으면 위치가 완벽해도 흡착이 전혀 안 붙는다 — 실제로
    한 번 이 실수를 했다(NO_ATTACH 4/4, GT 좌표로 줘도 재현됨).

    반환값(SurfaceGripper 노드 경로 자체)이 중요하다 — SurfaceGripperCtl 은
    부모 prim(short_gripper)이 아니라 이 노드 경로를 받아야 한다. 처음에
    부모 경로를 넘겼다가 또 한 번 NO_ATTACH 를 재현했다.
    """
    node_path = find_prim_path(gripper_prim, "SurfaceGripper")
    if node_path is None:
        raise RuntimeError(f"SurfaceGripper node not found under {gripper_prim}")
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


def holding(gripped):
    if not gripped:
        return False
    flat = []
    for item in gripped:
        flat.extend(item) if isinstance(item, (list, tuple)) else flat.append(item)
    return any(str(x).strip() not in ("", "None") for x in flat)


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
_status = {"phase": "IDLE", "gripped": False, "gap_m": 0.0, "message": ""}


def _set_status(**kv):
    with _status_lock:
        _status.update(kv)


def _get_status():
    with _status_lock:
        return dict(_status)


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
                resp = {"id": req.get("id"), "result": _get_status()}
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
#  씬 / 로봇
# ══════════════════════════════════════════════════════════════
class Backend:
    def __init__(self):
        self.frames = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
        self.grasp = yaml.safe_load(GRASP_YAML.read_text(encoding="utf-8"))
        self.carriers = yaml.safe_load(CARRIERS_YAML.read_text(encoding="utf-8"))

        ctx = omni.usd.get_context()
        ctx.open_stage(WORLD_USD)
        for _ in range(60):
            simulation_app.update()
        self.stage = ctx.get_stage()
        self.stage.Load()
        for _ in range(60):
            simulation_app.update()
        configure_drives(self.stage)
        self._gripper_node_path = configure_gripper_limits(self.stage, GRIPPER_PRIM)
        filter_collision(GRIPPER_PRIM, MAGAZINE_XFORM_PATH)
        # pkg_loader 정지 지점에서 _carry_gripped_object_through_teleport 가
        # 매거진을 tcp 바로 아래(ConveyorFrame 충돌체와 살짝 겹치는 위치)로
        # 순간이동시킨다 — 실측: 필터링 없이는 그 겹침을 PhysX 가 몇 m 밖으로
        # 튕겨내는 걸로 "해결"했다. 이 우회 자체가 실제 접촉을 흉내 낼 필요가
        # 없으므로 걸러낸다.
        filter_collision(MAGAZINE_XFORM_PATH, CONVEYOR_FRAME_PATH)

        self.world = World(stage_units_in_meters=1.0)
        root_path = resolve_articulation_root(ARTICULATION_ROOT_CANDIDATES, ROBOT_PRIM_PATH)
        self.robot = self.world.scene.add(SingleManipulator(
            prim_path=root_path, name="m0609_robot", end_effector_prim_path=EE_LINK_PATH))
        self.magazine = self.world.scene.add(SingleRigidPrim(
            prim_path=MAGAZINE_XFORM_PATH, name="magazine"))
        self.world.reset()
        self.robot.initialize()
        self._set_ready_pose()
        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)

        self.magazine_spawn_pos, self.magazine_spawn_quat = self.magazine.get_world_pose()

        base_pos0, base_quat0 = get_world_pose(BASE_LINK_PATH)
        self.lula = LulaKinematicsSolver(robot_description_path=DESC_PATH, urdf_path=URDF_PATH)
        self.lula.set_robot_base_pose(robot_position=base_pos0, robot_orientation=base_quat0)
        self.solver = ArticulationKinematicsSolver(
            robot_articulation=self.robot, kinematics_solver=self.lula,
            end_effector_frame_name=EE_LINK_NAME)
        self.gripper = SurfaceGripperCtl(self._gripper_node_path)
        self.target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
        self._rgb = None
        self._depth = None

        st = self.frames["static_transforms"]
        self.R_l6_cam = quat_xyzw_to_mat(st["m0609_tool0__camera_link"]["quat_xyzw"])
        self.t_l6_cam = np.array(st["m0609_tool0__camera_link"]["xyz"])
        self.R_cam_opt = quat_xyzw_to_mat(st["camera_link__camera_color_optical_frame"]["quat_xyzw"])
        ci = self.frames["wrist_camera"]["camera_info_observed"]
        self.K = np.array(ci["k"], dtype=float).reshape(3, 3)
        self.dist = np.array(ci["d"], dtype=float)

        print("   scene ready")

    def _set_arm_stiffness(self, stiffness):
        """observe_pose() 는 QR 이 깨지지 않게 DRIVE_STIFFNESS(1e5)로,
        pick_phase1_approach() 는 흡착 유지력이 충분한 DRIVE_STIFFNESS_PICK
        (1e8, 12_pick_test.py 검증값)으로 되돌린다. 위 DRIVE_STIFFNESS_PICK
        정의부 주석 참고 — 실측: 1e5 로 두면 LIFT 중 rise=88mm·tilt<0.1도로
        정상 상승해도 HOLD_WAIT 끝에 gripped=False (SLIP) 였다."""
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for dt in ["angular", "linear"]:
                d = UsdPhysics.DriveAPI.Get(prim, dt)
                if d:
                    d.GetStiffnessAttr().Set(stiffness)

    def _sync_ik_base(self):
        pos, quat = get_world_pose(BASE_LINK_PATH)
        self.lula.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
        return pos, quat

    def _set_ready_pose(self):
        q = np.zeros(self.robot.num_dof)
        for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
            q[self.robot.get_dof_index(name)] = np.deg2rad(deg)
        self.robot.set_joint_positions(q)

    def _set_joint_deg(self, joints_deg):
        idx = np.array([self.robot.get_dof_index(j) for j in ARM_JOINTS])
        q = self.robot.get_joint_positions()
        q[idx] = np.deg2rad(joints_deg)
        self.robot.set_joint_positions(q)
        self.robot.set_joint_velocities(np.zeros_like(q))
        self.robot.apply_action(ArticulationAction(joint_positions=np.deg2rad(joints_deg),
                                                    joint_indices=idx))

    def _servo_joint_deg(self, target_joints_deg, n_steps=SETTLE_STEPS):
        """_set_joint_deg 는 set_joint_positions() 로 관절을 즉시 스냅한다 —
        아무것도 안 들고 있을 때(observe_pose)는 문제없지만, STOW 처럼 흡착
        중에 쓰면 그 순간 가속으로 접합이 끊긴다(실측: LIFT 판정은 통과했는데
        STOW 이후 최종 gripped=False). 대신 매 스텝 목표를 다시 명령해 부드럽게
        움직인다 — _servo_tcp 와 같은 방식."""
        idx = np.array([self.robot.get_dof_index(j) for j in ARM_JOINTS])
        start_deg = np.degrees(self.robot.get_joint_positions()[idx])
        target_deg = np.array(target_joints_deg, dtype=float)
        for i in range(1, n_steps + 1):
            self.world.step(render=not HEADLESS)
            cur_deg = start_deg + ease(i / float(n_steps)) * (target_deg - start_deg)
            self.robot.apply_action(ArticulationAction(
                joint_positions=np.deg2rad(cur_deg), joint_indices=idx))

    def _get_tcp_pose(self):
        pos, quat = self.robot.end_effector.get_world_pose()
        return get_tcp_pose_from_ee(pos, quat)

    def _servo_tcp(self, goal_tcp, phase_name):
        start = self._get_tcp_pose()
        n_steps, dist = steps_for(start, goal_tcp)
        fail = 0
        for i in range(1, n_steps + 1):
            self.world.step(render=not HEADLESS)
            tcp = start + ease(i / float(n_steps)) * (goal_tcp - start)
            action, solved = self.solver.compute_inverse_kinematics(
                target_position=tcp_to_flange(tcp, self.target_quat),
                target_orientation=self.target_quat)
            if solved:
                self.robot.apply_action(action)
                fail = 0
            else:
                fail += 1
                if fail >= MAX_IK_FAIL_STEPS:
                    return False
        return True

    # ── RPC 메서드 ──────────────────────────────────────────
    def teleport_base(self, x, y, yaw_deg):
        """임시 nav_server 가 부르는 것 — "가짜 주행". SLAM 준비되면 이
        메서드는 그대로 두고, 임시 nav_server 만 실제 Nav2 클라이언트로
        바뀐다(이 백엔드는 안 바뀐다).

        ★ 한 번에 점프하지 않는다 — 목표까지 여러 스텝에 걸쳐 조금씩
        set_world_pose() 를 다시 부른다(_servo_base). 12_place_test.py 실측:
        "set_world_pose() 를 부르면 거리와 무관하게 흡착이 즉시 풀린다" —
        즉 한 걸음(스텝)만 움직여도 깨지는 성질이라, 잘게 쪼개도 결과는
        같다. 그래도 "정말 몇 스텝째 놓치는지" 를 실측으로 보여주려고 이
        방식을 쓴다 — dropped_at_step 로 보고한다(gripped=False 로 시작하면
        None)."""
        was_gripped = holding(self.gripper.gripped())
        dropped_at_step = self._servo_base(x, y, yaw_deg)
        self._sync_ik_base()
        # BASE_LINK_PATH(m0609/base_link) 는 팔의 마운트 기준점이지 AMR 섀시가
        # 아니다 — 카터 위에 약 0.2 m 앞으로 얹혀 있다(taught_poses.yaml 의
        # base_link_world vs m0609_base_world 차이와 일치). 여기서는 실제로
        # 텔레포트한 섀시 pose 를 보고한다. Lula 동기화(_sync_ik_base)는
        # 그대로 m0609/base_link 를 쓴다 — IK 에는 그게 맞는 프레임이다.
        pos, quat_w = get_world_pose(f"{BASE_XFORM_PATH}/chassis_link")

        carried_ok = True
        if was_gripped:
            carried_ok = self._carry_gripped_object_through_teleport()

        return {"base_x": float(pos[0]), "base_y": float(pos[1]),
               "base_yaw_deg": math.degrees(yaw_of_quat(quat_w)),
               "carried_ok": carried_ok, "dropped_at_step": dropped_at_step}

    def _servo_base(self, target_x, target_y, target_yaw_deg):
        """베이스를 목표 pose 까지 여러 스텝에 걸쳐 보간 이동한다("가짜
        주행"). 실제 바퀴 속도 제어가 아니라 매 스텝 set_world_pose() 를
        다시 부르는 것뿐이다 — Nav2 전까지의 임시 근사."""
        start_p, start_q = get_world_pose(BASE_XFORM_PATH)
        start_yaw = math.degrees(yaw_of_quat(start_q))
        dist = float(np.linalg.norm([target_x - start_p[0], target_y - start_p[1]]))
        n_steps = int(np.clip(dist / 0.03, MIN_STEPS, MAX_STEPS * 2))

        was_gripped = holding(self.gripper.gripped())
        dropped_at_step = None
        for i in range(1, n_steps + 1):
            a = ease(i / float(n_steps))
            x = start_p[0] + a * (target_x - start_p[0])
            y = start_p[1] + a * (target_y - start_p[1])
            yaw = start_yaw + a * (target_yaw_deg - start_yaw)
            quat = quat_from_axis([0, 0, 1], yaw)
            self.robot.set_world_pose(position=np.array([x, y, CARTER_Z]), orientation=quat)
            try:
                self.robot.set_linear_velocity(np.zeros(3))
                self.robot.set_angular_velocity(np.zeros(3))
            except Exception:
                pass
            self.world.step(render=not HEADLESS)
            if was_gripped and dropped_at_step is None and not holding(self.gripper.gripped()):
                dropped_at_step = i
                print(f"   !! 주행 중 {i}/{n_steps} 스텝에서 흡착이 끊겼다")
        for _ in range(BASE_MOVE_SETTLE_STEPS):
            self.world.step(render=not HEADLESS)
        return dropped_at_step

    def _carry_gripped_object_through_teleport(self):
        """★ 이 시뮬레이션의 한계 우회: 베이스 아티큘레이션에 set_world_pose()
        를 부르면 거리와 무관하게 흡착이 즉시 풀린다(12_place_test.py /
        14_place_test_pse.py 실측 — 상대 위치를 그대로 들고 옮겨도 마찬가지였다).
        그래서 "이송 중 계속 붙들고 있다"처럼 보이게 하려고, 텔레포트가 끝난
        자리에서 매거진을 그리퍼 바로 아래로 다시 옮겨 재흡착한다. 실물
        로봇은 이 메서드를 안 타므로(진짜로 붙든 채 이동하니) 이관 시 자연히
        빠진다 — nav_server/pick_place_server 쪽 코드는 안 바뀐다.

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
        self.gripper.open()
        self.gripper.reinit()

        base_p, base_q = get_world_pose(CHASSIS_LINK_PATH)
        R_base = quat_to_matrix(base_q)
        safe_hover = base_p + R_base @ np.array([0.3, 0.0, 1.0])
        self._servo_tcp(safe_hover, "TRANSIT_HOVER")
        for _ in range(10):
            self.world.step(render=not HEADLESS)

        tcp_now = self._get_tcp_pose()   # target_quat 자세라 월드 -Z 오프셋이 맞다
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

        self.gripper.close()
        for _ in range(GRIP_WAIT):
            self.world.step(render=not HEADLESS)
        ok = holding(self.gripper.gripped())
        if not ok:
            print("   !! 이송 후 재흡착 실패 — 이송 중 놓친 것으로 처리")
        return ok

    def reset_magazine(self):
        """반복 테스트용. carrier_code_reader/pick 결과에 영향받은 매거진을
        원위치로 되돌린다."""
        self.gripper.open()
        for _ in range(20):
            self.world.step(render=not HEADLESS)
        self.magazine.set_world_pose(position=self.magazine_spawn_pos,
                                     orientation=self.magazine_spawn_quat)
        self.magazine.set_linear_velocity(np.zeros(3))
        self.magazine.set_angular_velocity(np.zeros(3))
        for _ in range(30):
            self.world.step(render=not HEADLESS)
        self.gripper.reinit()
        return {"ok": True}

    def observe_pose(self):
        """관측 자세로 이동한다 (일단 2층만 정의 — taught_poses.yaml
        shelf_1_top_close_centered). carrier_code_reader.scan_qr() 전에 부른다."""
        self._set_arm_stiffness(DRIVE_STIFFNESS)   # QR 디코드가 깨지지 않는 값으로
        taught = yaml.safe_load((ISAACPJT / "tools/out/taught_poses.yaml").read_text(encoding="utf-8"))
        pose = taught["shelf_1_top_close_centered"]
        self._set_joint_deg(pose["joints_deg"])
        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)
        self._ensure_camera_warm()
        return {"ok": True, "pose": "shelf_1_top_close_centered"}

    def _ensure_camera_warm(self):
        """render_product 를 여기서(관측 자세에 이미 도착한 뒤) 처음 만든다.

        __init__ 시점(팔이 READY 자세 — QR 과 무관한 방향을 봄)에 미리
        만들어 뒀더니 RTX 가 그 첫 시야 기준으로 밉맵/텍스처 스트리밍 상태를
        고정해 버려서, 나중에 관측 자세로 옮겨 렌더 스텝을 아무리 밟아도
        QR 디코드가 계속 깨졌다(실측으로 원인 확정 — eval_qr_pose_depth.py
        는 이미 관측 자세에 있는 상태에서 render_product 를 만들어서 이
        문제가 없었다). 그래서 '진짜로 볼 것을 보고 있는 상태'에서 처음
        만들고, 그 뒤로는 재사용한다.
        """
        if self._rgb is None:
            import omni.replicator.core as rep
            rp = rep.create.render_product(CAMERA_PRIM, RESOLUTION)
            self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            self._rgb.attach([rp])
            self._depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
            self._depth.attach([rp])
        for _ in range(60):
            self.world.step(render=True)

    def scan_qr(self, expected_id=None, n_frames=3):
        """cobot3_perception.qr_pose 로 QR 자세를 재고, base_link 프레임으로
        변환해 돌려준다. carrier_code_reader 노드가 그대로 CarrierScan.qr_pose
        에 옮겨 담을 수 있는 형태다."""
        base_p, base_q = get_world_pose(CHASSIS_LINK_PATH)
        R_base = quat_to_matrix(base_q)

        obs_list = []
        decoded = ""
        for _ in range(n_frames):
            for _ in range(3):
                self.world.step(render=True)
            rgb_raw = np.asarray(self._rgb.get_data())
            if rgb_raw.ndim != 3:
                raise RuntimeError(
                    f"rgb annotator 가 빈 프레임을 줬다 (shape={rgb_raw.shape}) — "
                    "render_product 워밍업이 부족했을 수 있다")
            rgb = rgb_raw[:, :, :3][:, :, ::-1].copy()
            depth = np.asarray(self._depth.get_data(), dtype=np.float64).reshape(rgb.shape[:2])
            l6_p, l6_q = get_world_pose(EE_LINK_PATH)
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

    def pick_phase1_approach(self, flange_pose_base_link, approach_dist_m):
        """PickCarrier 의 APPROACH. flange_pose 는 base_link 프레임
        {position:[x,y,z], quat_wxyz:[..]} (Pose 규약과 동일, position=판
        윗면 중심, +Z=법선 위)."""
        self._set_arm_stiffness(DRIVE_STIFFNESS_PICK)   # 흡착 유지력 검증값으로
        base_p, base_q = get_world_pose(CHASSIS_LINK_PATH)
        R_base = quat_to_matrix(base_q)
        p_rel = np.array(flange_pose_base_link["position"])
        flange_world = base_p + R_base @ p_rel
        self._flange_world = flange_world   # DESCEND 단계에서 재사용

        _set_status(phase="APPROACH", gripped=False, gap_m=0.0, message="")
        goal = flange_world + np.array([0, 0, approach_dist_m])
        ok = self._servo_tcp(goal, "APPROACH")
        if not ok:
            _set_status(phase="FAILED", message="APPROACH IK 실패")
            return {"success": False, "fail_reason": "NO_IK", "phase": "APPROACH"}
        return {"success": True}

    def pick_phase2_finish(self, grip_gaps_m, offset_limit_m, lift_height_m=None):
        """DESCEND -> SUCTION -> LIFT -> STOW. offset_limit_m 은 이제
        pick_place_server 의 declare_parameter 값을 그대로 받는다 (PickCarrier.action
        리팩터 이후 flange_short_side_m 이 goal 에서 빠졌다 — offset_limit_m
        을 서버 파라미터로 직접 갖는 쪽이 계약과 맞는다)."""
        if lift_height_m is None:
            lift_height_m = LIFT_HEIGHT_OFFSET
        flange_world = self._flange_world

        for attempt, gap in enumerate(grip_gaps_m):
            _set_status(phase="DESCEND", gap_m=gap)
            grip_z = flange_world + np.array([0, 0, gap])
            if not self._servo_tcp(grip_z, "DESCEND"):
                _set_status(phase="FAILED", message="DESCEND IK 실패")
                return {"success": False, "fail_reason": "NO_IK", "phase": "DESCEND"}

            _set_status(phase="SUCTION", gap_m=gap)
            self.gripper.close()
            for _ in range(GRIP_WAIT):
                self.world.step(render=not HEADLESS)
            ok = holding(self.gripper.gripped())
            _set_status(gripped=ok)
            print(f"   시도 {attempt+1}/{len(grip_gaps_m)}  간격 {gap*1000:+.0f} mm  "
                  f"-> {'붙었다' if ok else '안 붙었다'}")
            if ok:
                break
            self.gripper.open()
        else:
            _set_status(phase="FAILED", message="흡착 실패 — grip_gaps 전부 소진")
            return {"success": False, "fail_reason": "NO_ATTACH", "phase": "SUCTION",
                   "used_gap_m": 0.0}

        # 흡착 순간 TCP 횡오차 (OFF_FLANGE 판정 — PickCarrier.action 주석 그대로)
        tcp_now = self._get_tcp_pose()
        final_offset_m = float(np.linalg.norm((tcp_now - flange_world)[:2]))

        _set_status(phase="LIFT")
        top_z0 = measure_prim(FLANGE_PATH)[1]
        lift_goal = flange_world + np.array([0, 0, lift_height_m])
        self._servo_tcp(lift_goal, "LIFT")
        for _ in range(HOLD_WAIT):
            self.world.step(render=not HEADLESS)
        gripped_after_lift = holding(self.gripper.gripped())
        _set_status(gripped=gripped_after_lift)
        top_z1 = measure_prim(FLANGE_PATH)[1]
        rise_m = top_z1 - top_z0
        _, mag_q = self.magazine.get_world_pose()
        tilt_deg = tilt_deg_from_quat(mag_q)

        if not gripped_after_lift:
            _set_status(phase="FAILED", message="리프트 중 놓쳤다")
            return {"success": False, "fail_reason": "SLIP", "phase": "LIFT",
                   "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m}

        if final_offset_m > offset_limit_m:
            _set_status(phase="FAILED", message="OFF_FLANGE")
            return {"success": False, "fail_reason": "OFF_FLANGE", "phase": "LIFT",
                   "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m}

        _set_status(phase="STOW")
        # STOW: 이송 자세로. READY_JOINTS_DEG 로 되돌리는 것으로 대신한다 —
        # _set_joint_deg(즉시 스냅) 대신 _servo_joint_deg(부드러운 보간)를
        # 쓴다. 실측: 즉시 스냅은 LIFT 판정(rise·tilt·gripped 전부 정상)을
        # 통과한 뒤에도 그 스냅 가속으로 흡착이 끊겼다.
        self._servo_joint_deg(READY_JOINTS_DEG, n_steps=SETTLE_STEPS)
        for _ in range(HOLD_WAIT):
            self.world.step(render=not HEADLESS)

        ok = (rise_m >= LIFT_OK_MIN_M) and (tilt_deg <= TILT_MAX_DEG) and holding(self.gripper.gripped())
        _set_status(phase="DONE" if ok else "FAILED",
                    message="" if ok else f"판정 실패 rise={rise_m*1000:.1f}mm tilt={tilt_deg:.2f}")
        return {"success": bool(ok), "fail_reason": "NONE" if ok else "SLIP",
               "gripped": bool(holding(self.gripper.gripped())),
               "final_offset_m": final_offset_m, "offset_limit_m": offset_limit_m,
               "used_gap_m": float(gap), "rise_mm": rise_m*1000, "tilt_deg": tilt_deg}

    def get_place_slot_pose_base_link(self):
        """편의 메서드 — get_flange_pose_world 와 같은 목적, place 쪽 GT.
        포트/슬롯 지오메트리가 아직 씬에 없어서(다음 범위), 12_place_test.py 가
        검증한 바닥 스테이징 지점(PLACE_TARGET_XY, FLOOR_Z)을 그대로 현재
        base_link(=chassis) 프레임으로 돌려준다. pkg_loader 정지 지점에
        도착한 뒤(NavigateTo 완료 후) 호출해야 값이 맞다."""
        base_p, base_q = get_world_pose(CHASSIS_LINK_PATH)
        R_base = quat_to_matrix(base_q)
        world_target = np.array([PLACE_TARGET_XY[0], PLACE_TARGET_XY[1], FLOOR_Z])
        p_rel = R_base.T @ (world_target - base_p)
        return {"position": p_rel.tolist(), "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}

    def place_phase1_approach(self, slot_pose_base_link, approach_dist_m=None):
        """PlaceCarrier 의 APPROACH. slot_pose 는 base_link 프레임
        {position:[x,y,z], quat_wxyz:[..]}, z=슬롯 바닥. 이송(TRANSIT)은
        여기서 안 한다 — task_manager 가 이 호출 전에 NavigateTo 로 이미
        pkg_loader 에 도착해 있어야 한다(재흡착은 teleport_base 안에서
        처리됨, _carry_gripped_object_through_teleport 참고)."""
        if not holding(self.gripper.gripped()):
            _set_status(phase="FAILED", message="APPROACH 시작 시점에 이미 안 붙어 있다")
            return {"success": False, "fail_reason": "NOT_GRIPPED"}

        if approach_dist_m is None:
            approach_dist_m = PLACE_APPROACH_HEIGHT_OFFSET
        base_p, base_q = get_world_pose(CHASSIS_LINK_PATH)
        R_base = quat_to_matrix(base_q)
        p_rel = np.array(slot_pose_base_link["position"])
        slot_world = base_p + R_base @ p_rel
        self._slot_world = slot_world   # DESCEND 단계에서 재사용
        self._place_approach_dist_m = float(approach_dist_m)

        _set_status(phase="APPROACH", gripped=True, gap_m=0.0, message="")
        goal = slot_world + np.array([0, 0, approach_dist_m])
        ok = self._servo_tcp(goal, "APPROACH")
        if not ok:
            _set_status(phase="FAILED", message="APPROACH IK 실패")
            return {"success": False, "fail_reason": "NO_IK"}
        return {"success": True}

    def place_phase2_finish(self, release_height_m):
        """DESCEND -> RELEASE -> RETRACT.

        ★ 알려진 갭: PORT_OCCUPIED(슬롯 점유 감지)는 아직 구현하지 않았다 —
        이 씬에 슬롯 자체가 없어서(place_phase1_approach 독스트링 참고)
        항상 비어 있다고 본다."""
        slot_world = self._slot_world
        approach_dist_m = getattr(self, "_place_approach_dist_m", PLACE_APPROACH_HEIGHT_OFFSET)

        _set_status(phase="DESCEND")
        descend_goal = slot_world + np.array([0, 0, release_height_m])
        if not self._servo_tcp(descend_goal, "DESCEND"):
            _set_status(phase="FAILED", message="DESCEND IK 실패")
            return {"success": False, "fail_reason": "NO_IK"}

        _set_status(phase="RELEASE")
        self.gripper.open()
        for _ in range(RELEASE_WAIT):
            self.world.step(render=not HEADLESS)
        released = not holding(self.gripper.gripped())
        _set_status(gripped=not released)

        _set_status(phase="RETRACT")
        retract_goal = slot_world + np.array([0, 0, approach_dist_m])
        self._servo_tcp(retract_goal, "RETRACT")
        for _ in range(SETTLE_STEPS):
            self.world.step(render=not HEADLESS)

        _set_status(phase="DONE" if released else "FAILED",
                    message="" if released else "흡착이 안 풀렸다")
        return {"success": bool(released), "gripped": bool(not released),
               "fail_reason": "NONE" if released else "COLLISION"}

    def get_flange_pose_world(self, magazine_key):
        """편의 메서드 — task_manager 개발/테스트용. 실측 GT 플랜지 pose 를
        base_link 프레임으로 돌려준다 (QR 대신 GT 로 파이프라인만 먼저
        확인하고 싶을 때)."""
        meas = yaml.safe_load(MEASURED.read_text(encoding="utf-8"))
        m = meas["magazines"][magazine_key]
        fc = np.array(m["flange_center"]); fc[2] = m["flange_top_z"]
        base_p, base_q = get_world_pose(CHASSIS_LINK_PATH)
        R_base = quat_to_matrix(base_q)
        p_rel = R_base.T @ (fc - base_p)
        return {"position": p_rel.tolist(), "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}

    def debug_state(self):
        """디버그 전용 — tcp/매거진 world pose 와 간격을 바로 본다."""
        tcp = self._get_tcp_pose()
        mag_p, mag_q = self.magazine.get_world_pose()
        cx, top_z, height = measure_prim(MAGAZINE_XFORM_PATH)
        return {"tcp_world": tcp.tolist(), "magazine_world_pos": mag_p.tolist(),
               "magazine_top_z": top_z, "magazine_height": height,
               "magazine_center_xy": cx.tolist(),
               "gripped": holding(self.gripper.gripped()),
               "dist_tcp_to_mag_top": float(np.linalg.norm(tcp - np.array([cx[0], cx[1], top_z])))}

    def debug_capture(self, path):
        """디버그 전용 — 지금 손목 카메라가 보는 그림을 저장한다."""
        import cv2
        self._ensure_camera_warm()
        rgb = np.asarray(self._rgb.get_data())[:, :, :3][:, :, ::-1].copy()
        cv2.imwrite(path, rgb)
        base_p, base_q = get_world_pose(CHASSIS_LINK_PATH)
        l6_p, l6_q = get_world_pose(EE_LINK_PATH)
        return {"saved": path, "chassis_world": base_p.tolist(),
               "link6_world": l6_p.tolist()}


def main():
    port = int(os.environ.get("SIM_BACKEND_PORT", "8765"))
    backend = Backend()
    threading.Thread(target=_rpc_serve, args=("127.0.0.1", port), daemon=True).start()
    _set_status(phase="IDLE", message="ready")

    print("=" * 70)
    print("  sim_backend ready — RPC 대기 중")
    print("=" * 70)

    METHODS = {
        "teleport_base": backend.teleport_base,
        "reset_magazine": backend.reset_magazine,
        "observe_pose": backend.observe_pose,
        "scan_qr": backend.scan_qr,
        "pick_phase1_approach": backend.pick_phase1_approach,
        "pick_phase2_finish": backend.pick_phase2_finish,
        "place_phase1_approach": backend.place_phase1_approach,
        "place_phase2_finish": backend.place_phase2_finish,
        "get_place_slot_pose_base_link": backend.get_place_slot_pose_base_link,
        "get_flange_pose_world": backend.get_flange_pose_world,
        "debug_capture": backend.debug_capture,
        "debug_state": backend.debug_state,
    }

    while simulation_app.is_running():
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

    simulation_app.close()


if __name__ == "__main__":
    main()

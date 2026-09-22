"""
티칭한 자세를 그대로 떠서 YAML 로 남긴다.

    isaac_python isaacpjt/tools/capture_pose.py                         # shelf_1_a
    CAPTURE_BASE=shelf_1_b isaac_python isaacpjt/tools/capture_pose.py
    CAPTURE_BASE=shelf_2_a isaac_python isaacpjt/tools/capture_pose.py
    CAPTURE_BASE=shelf_2_b isaac_python isaacpjt/tools/capture_pose.py
    # 12_place_test3.py 의 관측 자세(포장 출력 자리 앞)를 티칭할 때:
    CAPTURE_BASE=packaging_output isaac_python isaacpjt/tools/capture_pose.py
    #   자세를 잡은 뒤 이름을 "packaging_output_scan" 으로 Capture 하면
    #   12_place_test3.py 가 taught_poses.yaml 에서 그 이름을 자동으로 찾아 쓴다.

  쓸 수 있는 정차 지점은 BASE_WAYPOINTS 에 있다. 시작하면 그 자리에서
  사거리에 드는 QR 표적 목록을 찍어 준다.

쓰는 법:
  1. 이 스크립트를 돌리면 뷰포트 창과 함께 "Teach Pose" 패널이 뜬다.
     카터는 선반 앞 waypoint 로 옮겨지고 시뮬은 Play 상태로 들어간다.
  2. 패널의 joint_1..6 슬라이더로 자세를 만든다. 이름을 적고 Capture 를 누르면
     그 순간의 상태를 떠서 out/taught_poses.yaml 에 덧붙인다.
  3. 원하는 자세 개수만큼 반복하고 Ctrl-C 로 끝낸다.

  Isaac Sim 메뉴(Tools > Robotics > ...)를 찾지 마라. isaac_python 으로 띄우는
  앱(isaacsim.exp.base.python.kit)에는 메뉴 확장이 아예 안 올라온다. 그래서
  조작 UI 를 이 스크립트 안에 직접 넣었다.

터미널만으로 쓰고 싶거나 UI 가 안 뜰 때 (원격/헤드리스):
      echo "0 -30 90 0 60 0"   > /tmp/capture_jog     # 관절값(deg) 6 개
      echo "shelf_1_top_scan"  > /tmp/capture_pose    # 캡처
  두 파일 모두 스크립트가 읽고 지운다. UI 와 같이 써도 된다.

뜨는 내용:
  - 관절값 (deg / rad)
  - 베이스(base_link) world pose, 팔 베이스 world pose
  - link_6(=m0609_tool0) world pose
  - 카메라 광학 프레임 world pose
  - 그 자세에서 화면에 들어오는 QR 라벨과 각각의 거리 / 예상 픽셀 크기

QR 판정은 frames.yaml 의 camera_info_observed(실제 토픽에서 받은 K)와
out/layout_measured.yaml 의 QR 위치를 쓴다. 눈대중이 아니다.
"""

import os

from isaacsim import SimulationApp

HEADLESS = os.environ.get("CAPTURE_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import sys
import time
from pathlib import Path

import numpy as np
import omni.usd
import yaml
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation

sys.stdout.reconfigure(line_buffering=True)


THIS_DIR     = Path(__file__).resolve().parent
ISAACPJT_DIR = THIS_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD     = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
FRAMES_YAML   = WS_ROOT / "src/cobot3_bringup/config/frames.yaml"
MEASURED_YAML = THIS_DIR / "out/layout_measured.yaml"
OUT_YAML      = THIS_DIR / "out/taught_poses.yaml"
TRIGGER       = Path(os.environ.get("CAPTURE_TRIGGER", "/tmp/capture_pose"))
JOG           = Path(os.environ.get("CAPTURE_JOG", "/tmp/capture_jog"))

JOINT_LIMIT_DEG = 360.0      # URDF 상 ±6.2832 rad. 슬라이더는 보기 좋게 자른다
SLIDER_RANGE_DEG = 180.0

ROBOT       = "/World/Robots/nova_carter1"
CHASSIS     = f"{ROBOT}/chassis_link"
ARM         = f"{ROBOT}/m0609"
ARM_BASE    = f"{ARM}/base_link"
LINK6       = f"{ARM}/link_6"
CAMERA      = (f"{ARM}/short_gripper/rsd455/RSD455/"
               f"Camera_OmniVision_OV9782_Color")
ARM_JOINTS  = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

ARTICULATION_ROOT_CANDIDATES = [CHASSIS, ROBOT]

CARTER_Z = 0.07963398335074101

# 스캔 정차 지점.
#
# QR 이 x 방향으로 1.825 m 에 퍼져 있는데 팔 도달거리가 0.9 m 라 한 자리에서
# 다 못 본다. 그리고 디코드하려면 0.45 m 안쪽이어야 해서(qr_decode_range.py
# 실측) 한 자세에 매거진 하나씩만 들어온다. 그래서 한 선반당 두 번 정차하고,
# 각 정차에서 4 개씩(1층 2 + 2층 2 ... 실제로는 6 개가 사거리 안) 본다.
#
# 아래 x 값은 "표적거리 0.30 m 일 때 팔 베이스~카메라 직선거리가 0.75 m 이내"
# 를 만족하는 최소 정차 조합을 훑어서 고른 것이다. shelf_2 가 shelf_1 보다
# x 가 0.20 m 큰 이유는, 통로를 향한 QR 라벨이 매거진 반대쪽 면(qr_label_py)
# 이라 x 오프셋 부호가 반대이기 때문이다.
#
# base_y 는 선반 전면에서 0.4167 m 떨어진 자리 (티칭으로 검증된 값).
BASE_WAYPOINTS = {
    "shelf_1_a": dict(xy=(-5.75,  1.4500), yaw_deg=0.0),
    "shelf_1_b": dict(xy=(-5.25,  1.4500), yaw_deg=0.0),
    "shelf_2_a": dict(xy=(-5.55, -1.6425), yaw_deg=0.0),
    "shelf_2_b": dict(xy=(-5.05, -1.6425), yaw_deg=0.0),
    # 예전 값 — 12_place_test.py 의 WP_PICK 부근. 파지 테스트용으로 남겨 둔다.
    "shelf_1":   dict(xy=(-6.5,   1.4500), yaw_deg=0.0),
    "shelf_2":   dict(xy=(-6.5,  -1.6425), yaw_deg=0.0),
    # 포장 출력 자리(packaging_flow.py SHELF_X=3.25/SHELF_SLOT_Y[0]=2.25) 앞
    # 0.55 m — 12_place_test3.py 의 WP_STAGE_OUTPUT/OUTPUT_STANDOFF_M 과 같은
    # 값이다. 거기서 관측 자세를 잡아 "packaging_output_scan" 으로 캡처하면
    # 12_place_test3.py 가 그 이름을 자동으로 찾아 쓴다(OBSERVE_POSE_NAME).
    "packaging_output": dict(xy=(3.25, 1.70), yaw_deg=0.0),
    "dock":      None,                    # 씬에 있는 자리 그대로 둔다
}

# 이 정차에서 노려볼 만한 표적을 고르는 기준
TARGET_STANDOFF_M = 0.30    # 디코드 실측 한계 0.456 m 에 여유
ARM_WORK_RADIUS_M = 0.75    # 도달거리 0.9 m 의 83 %

# 티칭 시작 자세 (12_place_test.py 의 READY_JOINTS_DEG 와 동일)
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 12_place_test.py 는 1e8/1e8 을 쓰지만 거기는 IK 가 매 스텝 목표각을 갱신해서
# 오차가 항상 작다. 여기는 사람이 슬라이더를 확 움직이므로 순간 오차가 크고,
# 그 토크가 베이스(카터)를 뒤집어 버린다. 그래서 maxForce 를 URDF 의 관절
# effort limit(2700)으로 묶고, 아래 JOG_RATE 로 목표각 변화율도 제한한다.
DRIVE_STIFFNESS = 1e5
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 2700.0     # m0609_isaac_sim.urdf 의 joint effort limit

JOG_RATE_DEG_PER_STEP = 2.0  # 목표각을 한 스텝에 이만큼씩만 옮긴다


def yaw_quat_wxyz(deg):
    h = np.radians(deg) / 2.0
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)])


def world_pose(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None, None
    m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat().GetNormalized()
    im = q.GetImaginary()
    R = np.array(m).T[:3, :3]
    # 스케일이 1 이라고 가정 (씬 전체가 1)
    return (np.array([t[0], t[1], t[2]]), R,
            np.array([q.GetReal(), im[0], im[1], im[2]]))


def resolve_articulation_root():
    stage = omni.usd.get_context().get_stage()
    for path in ARTICULATION_ROOT_CANDIDATES:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    return ROBOT


def configure_drives():
    """티칭 중 팔이 중력에 주저앉지 않게 드라이브를 세게 걸어 둔다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(ARM)
    n = 0
    for prim in Usd.PrimRange(root):
        if prim.GetName() not in ARM_JOINTS:
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
        drive.CreateDampingAttr().Set(DRIVE_DAMPING)
        drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)
        n += 1
    return n


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    n = np.linalg.norm([x, y, z, w])
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def qr_visibility(cam_pos, R_opt, qrs, fx, fy, cx, cy, W, H, qr_side):
    """광학 프레임 기준으로 각 QR 이 화면에 들어오는지 본다.

    투영해서 픽셀 좌표를 내고, 화면 밖이거나 뒤에 있거나 라벨이 카메라를
    등지고 있으면 뺀다.
    """
    out = []
    for name, center, normal in qrs:
        v = center - cam_pos
        p = R_opt.T @ v                    # 광학 프레임 좌표
        if p[2] <= 0.01:
            continue                       # 카메라 뒤
        u = fx * p[0] / p[2] + cx
        w = fy * p[1] / p[2] + cy
        if not (0 <= u < W and 0 <= w < H):
            continue
        # 라벨이 카메라를 향하고 있나 (법선과 시선이 마주봐야 한다)
        if float(np.dot(normal, v)) >= 0:
            continue
        dist = float(np.linalg.norm(v))
        out.append(dict(name=name, dist_m=round(dist, 4),
                        px=round(float(fx * qr_side / dist), 1),
                        uv=[round(float(u), 1), round(float(w), 1)]))
    return sorted(out, key=lambda d: d["dist_m"])


class TeachPanel:
    """관절 슬라이더 + 캡처 버튼. omni.ui 가 없으면 조용히 비활성화된다.

    isaac_python 으로 띄우는 앱에는 메뉴가 없어서 Articulation Inspector 같은
    기존 패널을 못 쓴다. 그래서 필요한 것만 직접 만든다.
    """

    def __init__(self, joint_names, initial_deg, on_capture):
        self.ok = False
        self._models = []
        self._name_model = None
        self._status = None
        self._on_capture = on_capture
        try:
            import omni.ui as ui
        except Exception as exc:
            print(f"   UI 없음 ({exc}) — 파일 인터페이스만 쓴다")
            return

        self._win = ui.Window("Teach Pose", width=460, height=460)
        with self._win.frame:
            with ui.VStack(spacing=6, height=0):
                ui.Label("관절 (deg)", height=22)
                for name, deg in zip(joint_names, initial_deg):
                    with ui.HStack(height=26, spacing=6):
                        ui.Label(name, width=70)
                        drag = ui.FloatDrag(min=-SLIDER_RANGE_DEG,
                                            max=SLIDER_RANGE_DEG, step=0.5)
                        drag.model.set_value(float(deg))
                        self._models.append(drag.model)
                ui.Spacer(height=8)
                ui.Label("자세 이름", height=22)
                self._name_model = ui.StringField(height=26).model
                self._name_model.set_value("shelf_1_top_scan")
                ui.Spacer(height=6)
                ui.Button("Capture", height=34, clicked_fn=self._clicked)
                ui.Spacer(height=6)
                self._status = ui.Label("", word_wrap=True, height=80)
        self.ok = True

    def _clicked(self):
        name = (self._name_model.get_value_as_string() or "").strip()
        self._on_capture(name)

    def targets_deg(self):
        """슬라이더가 가리키는 관절값. UI 가 없으면 None."""
        if not self.ok:
            return None
        return [m.get_value_as_float() for m in self._models]

    def set_targets_deg(self, values):
        """파일 jog 로 값이 바뀌었을 때 슬라이더를 따라가게 한다"""
        if not self.ok:
            return
        for m, v in zip(self._models, values):
            m.set_value(float(v))

    def say(self, text):
        if self._status is not None:
            self._status.text = text


def targets_for_stop(meas, arm_pos, aisle_sign, shelf_prefix=None):
    """이 정차 자리에서 사거리 안에 드는 QR 표적을 고른다.

    판정은 '표적거리 TARGET_STANDOFF_M 에서 팔 베이스~카메라 직선거리가
    ARM_WORK_RADIUS_M 이내' 다. 도달 가능하다는 뜻이지 충돌이 없다는 뜻은
    아니다 — 실제로 티칭해 봐야 안다.
    """
    rows = []
    for key, m in sorted(meas.get("magazines", {}).items()):
        if shelf_prefix and not key.startswith(shelf_prefix + "/"):
            continue                          # 다른 선반은 볼 일이 없다
        for lab, q in m.get("qr", {}).items():
            n = np.array(q["normal_world"])
            if np.sign(n[1]) != aisle_sign:
                continue                      # 통로 반대쪽 라벨
            c = np.array(q["center_world"])
            want = c + n * TARGET_STANDOFF_M  # 카메라가 있어야 할 자리
            d = float(np.linalg.norm(want - arm_pos))
            rows.append((d <= ARM_WORK_RADIUS_M, d, key, lab, c))
    return sorted(rows, key=lambda r: (not r[0], r[1]))


def read_jog_file():
    """/tmp/capture_jog 에서 관절값 6 개를 읽는다. 없거나 형식이 틀리면 None."""
    if not JOG.exists():
        return None
    try:
        raw = JOG.read_text(encoding="utf-8")
    except OSError:
        return None
    JOG.unlink(missing_ok=True)
    parts = raw.replace(",", " ").split()
    if len(parts) != len(ARM_JOINTS):
        print(f"   jog 무시 — 관절값 {len(ARM_JOINTS)} 개가 필요한데 "
              f"{len(parts)} 개 왔다: {raw.strip()!r}")
        return None
    try:
        vals = [float(v) for v in parts]
    except ValueError:
        print(f"   jog 무시 — 숫자가 아니다: {raw.strip()!r}")
        return None
    if any(abs(v) > JOINT_LIMIT_DEG for v in vals):
        print(f"   jog 무시 — ±{JOINT_LIMIT_DEG:.0f}도 범위를 벗어났다: {vals}")
        return None
    return vals


def main():
    cfg = yaml.safe_load(FRAMES_YAML.read_text(encoding="utf-8"))
    meas = yaml.safe_load(MEASURED_YAML.read_text(encoding="utf-8")) \
        if MEASURED_YAML.exists() else {"magazines": {}}

    K = cfg["wrist_camera"]["camera_info_observed"]
    fx, cx = K["k"][0], K["k"][2]
    fy, cy = K["k"][4], K["k"][5]
    W, H = K["width"], K["height"]
    qr_side = cfg["layout"]["magazine_1_orange"]["qr_side_m"]

    # 모든 QR 라벨을 한 덩어리로 모아 둔다
    qrs = []
    for key, m in meas["magazines"].items():
        for lab, q in m.get("qr", {}).items():
            qrs.append((f"{key}:{lab}",
                        np.array(q["center_world"]),
                        np.array(q["normal_world"])))

    # camera_link -> optical 회전 (광학 프레임을 만들 때 쓴다)
    R_cam_opt = quat_xyzw_to_mat(
        cfg["static_transforms"]["camera_link__camera_color_optical_frame"]["quat_xyzw"])
    R_l6_cam = quat_xyzw_to_mat(
        cfg["static_transforms"]["m0609_tool0__camera_link"]["quat_xyzw"])

    ctx = omni.usd.get_context()
    ctx.open_stage(WORLD_USD)
    for _ in range(60):
        simulation_app.update()
    ctx.get_stage().Load()
    for _ in range(60):
        simulation_app.update()

    n = configure_drives()
    world = World(stage_units_in_meters=1.0)
    root = resolve_articulation_root()
    robot = world.scene.add(SingleArticulation(prim_path=root, name="carter_m0609"))
    world.reset()

    idx = np.array([robot.get_dof_index(j) for j in ARM_JOINTS])

    def send_targets(deg_values):
        """드라이브 목표각을 준다"""
        rad = np.deg2rad(np.asarray(deg_values, dtype=float))
        if hasattr(robot, "set_joint_position_targets"):
            robot.set_joint_position_targets(rad, joint_indices=idx)
        else:
            from isaacsim.core.utils.types import ArticulationAction
            robot.apply_action(ArticulationAction(joint_positions=rad,
                                                  joint_indices=idx))

    def zero_base_velocity():
        """텔레포트 직후 남은 속도를 털어낸다. 안 그러면 카터가 튀어나간다."""
        for attr, arg in (("set_velocities", np.zeros(6)),
                          ("set_linear_velocity", np.zeros(3)),
                          ("set_angular_velocity", np.zeros(3))):
            fn = getattr(robot, attr, None)
            if fn is not None:
                try:
                    fn(arg)
                except Exception:
                    pass

    # 순서가 중요하다. 관절을 ready 로 옮기면서 드라이브 목표각을 같이 주지 않으면
    # 목표각이 0 인 채로 남아, 스티프니스가 ready 자세를 0 으로 되돌리려고 큰
    # 토크를 걸고 그게 카터를 뒤집는다. 실제로 그렇게 뒤집혔다.
    q = robot.get_joint_positions()
    q[idx] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros_like(q))
    send_targets(READY_JOINTS_DEG)
    for _ in range(30):
        world.step(render=not HEADLESS)

    which = os.environ.get("CAPTURE_BASE", "shelf_1_a")
    if which not in BASE_WAYPOINTS:
        sys.exit(f"CAPTURE_BASE='{which}' 를 모르겠다. "
                 f"쓸 수 있는 값: {', '.join(BASE_WAYPOINTS)}")
    wp = BASE_WAYPOINTS[which]
    if wp is not None:
        pos = np.array([wp["xy"][0], wp["xy"][1], CARTER_Z])
        robot.set_world_pose(position=pos, orientation=yaw_quat_wxyz(wp["yaw_deg"]))
        zero_base_velocity()
        for _ in range(90):
            world.step(render=not HEADLESS)

        got, _ = robot.get_world_pose()
        got = np.asarray(got, dtype=float)
        # z 는 바퀴가 바닥에 닿으면서 내려앉는 게 정상이다 (씬의 카터가 0.0796 m
        # 떠 있게 저작돼 있다). 자세 티칭에서 문제가 되는 건 xy 어긋남이다.
        drift_xy = float(np.linalg.norm(got[:2] - pos[:2]))
        print(f"   베이스 안착   xy 오차 {drift_xy*1000:.1f} mm   "
              f"z {pos[2]*1000:.1f} -> {got[2]*1000:.1f} mm (바퀴 접지)")
        if drift_xy > 0.02:
            print(f"   !! xy 가 {drift_xy*1000:.0f} mm 어긋났다 — 자세를 뜨기 전에 확인해라")

    for f in (TRIGGER, JOG):
        if f.exists():
            f.unlink()

    captured = {}
    if OUT_YAML.exists():
        captured = yaml.safe_load(OUT_YAML.read_text(encoding="utf-8")) or {}

    def capture(name):
        """지금 이 순간의 상태를 떠서 YAML 에 덧붙인다"""
        name = (name or "").strip() or f"pose_{len(captured) + 1}"

        jp = robot.get_joint_positions()
        joints_rad = [float(jp[i]) for i in idx]
        joints_deg = [float(np.degrees(v)) for v in joints_rad]

        base_p, _, base_q = world_pose(CHASSIS)
        arm_p, _, _       = world_pose(ARM_BASE)
        l6_p, l6_R, l6_q  = world_pose(LINK6)
        cam_p, _, _       = world_pose(CAMERA)

        # 광학 프레임 = link_6 회전 * (link6->camera_link) * (camera_link->optical)
        R_opt = l6_R @ R_l6_cam @ R_cam_opt
        seen = qr_visibility(cam_p, R_opt, qrs, fx, fy, cx, cy, W, H, qr_side)

        captured[name] = dict(
            base_waypoint=which,
            base_link_world=[round(float(v), 5) for v in base_p],
            base_link_quat_wxyz=[round(float(v), 6) for v in base_q],
            m0609_base_world=[round(float(v), 5) for v in arm_p],
            joints=dict(zip(ARM_JOINTS, [round(v, 4) for v in joints_deg])),
            joints_deg=[round(v, 4) for v in joints_deg],
            joints_rad=[round(v, 6) for v in joints_rad],
            tool0_world=[round(float(v), 5) for v in l6_p],
            tool0_quat_wxyz=[round(float(v), 6) for v in l6_q],
            camera_optical_world=[round(float(v), 5) for v in cam_p],
            qr_in_view=seen,
            captured_at=time.strftime("%Y-%m-%d %H:%M:%S"),
        )
        OUT_YAML.parent.mkdir(parents=True, exist_ok=True)
        OUT_YAML.write_text(
            yaml.safe_dump(captured, allow_unicode=True, sort_keys=False),
            encoding="utf-8")

        print()
        print(f"   [{name}] 캡처")
        print("      관절(deg) " + ", ".join(f"{v:7.2f}" for v in joints_deg))
        print(f"      tool0     {np.round(l6_p, 4).tolist()}")
        print(f"      카메라    {np.round(cam_p, 4).tolist()}")
        if seen:
            for d in seen:
                print(f"      QR  {d['name']:52s} {d['dist_m']:.3f} m  "
                      f"{d['px']:.0f} px  uv {d['uv']}")
        else:
            print("      QR  화면에 들어온 라벨 없음")
        print(f"      -> {OUT_YAML}")

        lines = [f"{name}: QR {len(seen)} 개"]
        for d in seen[:4]:
            lines.append(f"  {d['name'].split('/')[-1]}  "
                         f"{d['dist_m']:.2f} m  {d['px']:.0f} px")
        panel.say("\n".join(lines))
        return captured[name]

    panel = TeachPanel(ARM_JOINTS, READY_JOINTS_DEG, capture)

    print("=" * 72)
    print("  자세 티칭 — 준비됨")
    print("=" * 72)
    print(f"   씬            {WORLD_USD}")
    print(f"   아티큘레이션  {root}   (드라이브 {n} 개 설정)")
    print(f"   베이스        {which}  {wp['xy'] if wp else '씬 그대로'}")
    if panel.ok:
        print('   조작          뷰포트 옆 "Teach Pose" 패널의 슬라이더')
    else:
        print("   조작          UI 를 못 띄웠다 — 아래 파일 인터페이스를 써라")
    print()
    print("   터미널에서도 된다:")
    print(f"       echo \"0 -30 90 0 60 0\"  > {JOG}      # 관절값(deg) 6 개")
    print(f"       echo \"shelf_1_top_scan\" > {TRIGGER}   # 캡처")
    print()
    print(f"   뜬 자세는 {OUT_YAML} 에 쌓인다.  Ctrl-C 로 종료.")

    if wp is not None:
        arm_now, _, _ = world_pose(ARM_BASE)
        aisle = -1.0 if wp["xy"][1] > 0 else 1.0
        # waypoint 이름 앞부분이 곧 선반 이름이다 ("shelf_1_a" -> "shelf_1")
        shelf_prefix = "_".join(which.split("_")[:2]) if which.startswith("shelf_") else None
        rows = targets_for_stop(meas, arm_now, aisle, shelf_prefix)
        inr = [r for r in rows if r[0]]
        print()
        print(f"   이 자리에서 노릴 표적 ({len(inr)} / {len(rows)} 개가 사거리 안)")
        for ok_, d, key, lab, c in rows:
            mark = "O" if ok_ else "-"
            print(f"      {mark} {key:36s} 팔베이스로부터 {d:5.2f} m"
                  f"   QR {np.round(c, 3).tolist()}")
        print(f"      (기준: 표적거리 {TARGET_STANDOFF_M*100:.0f} cm 에서 "
              f"{ARM_WORK_RADIUS_M*100:.0f} cm 이내. 충돌은 안 본다)")
    print("=" * 72)

    # 지금 실제로 나가고 있는 목표각. 슬라이더를 확 움직여도 여기서 한 스텝에
    # JOG_RATE_DEG_PER_STEP 씩만 따라가므로 순간 토크가 튀지 않는다.
    sent = list(READY_JOINTS_DEG)
    try:
        while simulation_app.is_running():
            world.step(render=not HEADLESS)

            # 파일 jog 가 오면 그 값을 슬라이더에도 반영해 둔다
            jog = read_jog_file()
            if jog is not None:
                panel.set_targets_deg(jog)
                wanted = jog
            else:
                wanted = panel.targets_deg()

            if wanted is not None:
                step = JOG_RATE_DEG_PER_STEP
                moved = False
                for i, w in enumerate(wanted):
                    d = w - sent[i]
                    if abs(d) > 1e-4:
                        sent[i] += max(-step, min(step, d))
                        moved = True
                if moved:
                    send_targets(sent)

            if TRIGGER.exists():
                try:
                    name = TRIGGER.read_text(encoding="utf-8").strip()
                except OSError:
                    continue
                TRIGGER.unlink(missing_ok=True)
                capture(name)
    except KeyboardInterrupt:
        print("\n   종료")

try:
    main()
finally:
    sys.stdout.flush()
    os._exit(0)

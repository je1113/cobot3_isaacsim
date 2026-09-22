"""
단위 테스트 2 — **QR 인식**으로 매거진을 집어 포장 라인에 넣고, 거기서 나온
**스택**을 다시 QR 로 찾아 검사 라인으로 옮긴다.

    isaac_python 12_place_test2.py          (GUI 필수 — 아래 ★ 캡쳐 참고)

12_place_test.py 와 무엇이 다른가
──────────────────────────────────
  12_place_test.py : USD 에서 대상의 GT pose 를 직접 읽어 집는다(인식 없음).
  이 스크립트      : **손목 카메라로 QR 을 읽어** 그 결과로 파지점을 계산한다.
                     GT 는 오직 '맞았는지 채점' 에만 쓴다.

시나리오
────────
    ① SHELF-A 앞으로 텔레포트 → QR 인식 → 매거진 pick
    ② 포장 벨트(PackagingZone)로 텔레포트 → 벨트 위에 place
    ③ 벨트가 매거진을 포장 트리거로 실어 감
         → packaging_flow.py 가 매거진을 비활성화하고 **스택을 스폰**
         → 스택이 언로더 벨트를 타고 흘러가 어딘가에 멈춤
    ④ 멈춘 자리 앞으로 텔레포트 → QR 인식 → 스택 pick
    ⑤ 검사 벨트(TestingZone)로 텔레포트 → 벨트 위에 place

★ GUI 가 필요한 이유 (캡쳐 경로)
────────────────────────────────
  Isaac Sim 5.1.0 rc.19 에서 rep.create.render_product() 의 rgb 애노테이터가
  **항상 빈 프레임**(shape=(0,))을 준다는 것이 sim_backend.py 에 기록돼 있다.
  그래서 기본 경로를 뷰포트 캡쳐(omni.kit.widget.viewport.capture)로 잡았고,
  그건 활성 뷰포트가 있어야 해서 헤드리스로는 안 된다.
  애노테이터가 되는 빌드라면 CAPTURE=annotator 로 바꿔 쓸 수 있다(둘 다 구현해 뒀다).

★ 두 캐리어의 QR 이 서로 다르다 — 하나로 뭉뚱그리면 안 된다
────────────────────────────────────────────────────────────
                    라벨 크기   디코드 내용        data_side_m
    매거진          50 mm      "1" / "2"          0.03621  (qr_pose 기본값)
    스택            40 mm      "F3-STKO-1" 등     0.02897  ← 반드시 넘겨야 한다
  스택 라벨이 40 mm 인데 기본값(50 mm 기준)을 쓰면 PnP 교차검증이 어긋난다.
  (위치 자체는 깊이 평면 적합에서 나오므로 치명적이진 않지만, 맞는 값을 준다)

★ 스택의 파지점은 grasp.yaml 에 없다 — 에셋 기하에서 유도했다
──────────────────────────────────────────────────────────────
  grasp.yaml 에는 매거진 둘만 있다. 스택용 T_QR_grasp_xyz 는 에셋의
  qr_label_ny 중심과 flange_plate 윗면 중심의 차이로 계산했고, **같은 계산식이
  매거진 두 종의 grasp.yaml 값을 소수점까지 재현하는 것을 확인**했다
  (CARRIERS 표 아래 주석 참고). 그래서 스택 값도 같은 근거를 갖는다.

★ 이 스크립트가 **검증하지 못한** 것 (돌려봐야 안다)
────────────────────────────────────────────────────
  1. 새 레이아웃에서 QR 이 실제로 읽히는지. 티칭 자세(shelf_1_top_close_centered)
     는 옛 배치에서 잡은 것이고, 여기서는 "로봇↔매거진 상대 위치가 같으면 관절도
     같다" 는 가정으로 재사용한다. shelves.yaml 의 standoff 0.55 m 가 그 가정이다.
  2. 벨트의 surfaceVelocity 가 실제로 물체를 옮기는지. 안 옮기면 ③에서 멈춘다
     (그 경우를 타임아웃으로 잡아 메시지를 낸다).
  3. 스택이 어디에 멈추는지. packaging_flow.py 의 SHELF_X(3.25)/SHELF_Z(0.64)는
     실제 ShelfDeck(x 3.694~4.044, 윗면 0.54)과 어긋나 있어, 스택이 허공에서
     떨어질 수 있다. 그래서 ④는 **좌표를 박지 않고** 멈춘 자리를 찾아간다.
"""

import os
import sys

from isaacsim import SimulationApp

# ★ 뷰포트 캡쳐 경로를 쓰므로 GUI 가 기본이다. headless 로 두면 활성 뷰포트가
#   없어 캡쳐가 실패한다(sim_backend.py 의 _ensure_camera_warm 주석 참고).
HEADLESS = os.environ.get("PICK_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

# ★ 확장 로드 순서가 중요하다. omni.replicator.core 를 먼저 켜지 않으면
#   OmniGraph 의 "Replicator" 카테고리 등록이 omni.graph.image.core 에 덮여
#   스테이지 로드 중 죽거나 애노테이터가 영구히 빈 프레임을 준다(sim_backend.py).
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")
# ★ Nav2 로 주행하려면 반드시 필요하다. 이게 없으면 씬의 nova_carter 에 붙은
#   ROS2 OmniGraph 노드들이 전부 "Could not find node type interface" 로 죽어
#   /clock 도 안 나가고 cmd_vel 도 못 받는다 — nav_server 가 goal 을 받아도
#   로봇은 제자리에 서 있는다.
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.robot.surface_gripper")
# ★ PackagingUnloaderZone 이 OmniScriptingAPI 로 packaging_flow.py 를 달고 있다.
#   이 확장이 없으면 스테이지를 열어도 그 스크립트가 안 돌아 **스택이 영영 안 나온다.**
enable_extension("omni.kit.scripting")

import ctypes
import dataclasses
import re
import shutil
import math
import time
from pathlib import Path
from typing import Optional

import numpy as np
import omni.usd
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    ArticulationKinematicsSolver,
)
from isaacsim.core.utils.types import ArticulationAction


# ══════════════════════════════════════════════════════════════
#  경로 — 인식 코드는 ROS 패키지 안에 있지만 ROS 를 import 하지 않는다
# ══════════════════════════════════════════════════════════════
THIS_DIR     = Path(__file__).resolve().parent
M0609_DIR    = THIS_DIR.parent
ISAACPJT_DIR = M0609_DIR.parent
WS_ROOT      = ISAACPJT_DIR.parent

WORLD_USD        = str(ISAACPJT_DIR / "worlds/simple_factory_layout.usda")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")
FRAMES_YAML      = str(WS_ROOT / "src/cobot3_bringup/config/frames.yaml")
SHELVES_YAML     = str(WS_ROOT / "src/cobot3_bringup/config/shelves.yaml")
TAUGHT_POSES     = str(ISAACPJT_DIR / "tools/out/taught_poses.yaml")
DUMP_DIR         = ISAACPJT_DIR / "tools/out/place_test2_frames"

# qr_pose / flange_topview 는 numpy+cv2 만 쓴다 — rclpy 를 import 하지 않아
# Isaac 의 python(3.11)에서 그대로 불린다. (carrier_code_reader.py 는 ROS 라 못 쓴다)
sys.path.insert(0, str(WS_ROOT / "src/cobot3_perception"))
from cobot3_perception.qr_pose import (  # noqa: E402
    estimate_qr_pose, aggregate_qr_poses, set_external_decoder,
)


# ══════════════════════════════════════════════════════════════
#  씬 prim 경로
# ══════════════════════════════════════════════════════════════
EE_LINK_NAME = "link_6"

SPAWNED_STACKS_PATH = "/World/Environment/PackagingUnloaderZone/SpawnedStacks"


class RobotCtx:
    """로봇 한 대치 prim 경로와 런타임 객체를 한데 묶는다.

    ★ 예전에는 nova_carter1 경로가 모듈 상수(ROBOT_PRIM_PATH 등)로 박혀 있었다.
      두 대를 **동시에** 돌리려면 경로가 로봇마다 달라야 해서 여기로 모았다.
      씬에는 nova_carter1 / nova_carter2 가 둘 다 있고 구성(m0609 +
      short_gripper)도 같다 — USD 에서 확인했다.

    name 은 로그 이름이자 **Nav2 네임스페이스**다(/robot1, /robot2).
    """

    def __init__(self, name, carter, shelf_id, magazine, via=(), staging=None,
                 staging_via=()):
        self.name = name
        self.carter = carter
        self.shelf_id = shelf_id
        self.magazine = magazine
        self.via = via
        # 벨트 자리를 기다리는 동안 미리 가 있을 자리 (x, y, yaw_rad). 로봇마다
        # 다르게 준다 — 같은 곳에서 기다리면 둘이 서로 들이받는다.
        self.staging = staging
        self.staging_via = staging_via
        # setup 에서 채운다
        self.robot = None
        self.lula = None
        self.solver = None
        self.gripper = None
        self.vision = None
        self.teleporter = None
        self.nav = None

    @property
    def arm(self):
        return f"{self.carter}/m0609"

    @property
    def base_link(self):
        return f"{self.arm}/base_link"

    @property
    def ee_link(self):
        return f"{self.arm}/{EE_LINK_NAME}"

    @property
    def gripper_prim(self):
        return f"{self.arm}/short_gripper"

    @property
    def camera_prim(self):
        return f"{self.gripper_prim}/rsd455/RSD455/Camera_OmniVision_OV9782_Color"

    @property
    def articulation_candidates(self):
        return [f"{self.carter}/chassis_link", self.carter]

    def __repr__(self):
        return f"<RobotCtx {self.name} {self.shelf_id}>"


# 집을 매거진. 둘 다 주황이라 packaging_flow 가 ORANGE_STACK_PAYLOAD 를 스폰한다.
#
# ★ SHELF-B 쪽은 원래 QR 을 통로에서 볼 수 없었다. 매거진 에셋의 QR·라벨은
#   로컬 -Y 면에 있는데(assets/magazine_*_qr.usda 의 translate=(0,-0.063,·)),
#   shelf_2 의 매거진이 shelf_1 것을 **회전 없이 복사**해서 orient 가 항등이라
#   -Y 가 선반 안쪽을 향했다. 통로는 shelf_2 기준 +Y 쪽이다. 그래서
#   simple_factory_layout.usda 에서 shelf_2 매거진 4개를 Z축 180° 돌렸다
#   (orient (1,0,0,0) -> (0,0,0,1)). 이제 두 선반이 거울 대칭이다.
MAGAZINE_SHELF_1 = "/World/Magazines/shelf_1_magaines/top_magazines/magazine_1_orange"
MAGAZINE_SHELF_2 = "/World/Magazines/shelf_2_magaines/top_magazines/magazine_1_orange"


# ══════════════════════════════════════════════════════════════
#  캐리어 사양 — QR 이 다르고 파지점도 다르다
# ══════════════════════════════════════════════════════════════
@dataclasses.dataclass
class CarrierSpec:
    """QR 한 장에서 파지점까지 가는 데 필요한 값 전부."""
    name: str
    expected_id: Optional[str]   # 디코드 문자열. None 이면 아무거나 받는다
    qr_side_m: float             # 라벨 한 변 (m)
    t_qr_grasp: np.ndarray       # QR 프레임에서 흡착점까지 (x_qr, y_qr, z_qr)

    @property
    def data_side_m(self) -> float:
        """QR 데이터 영역 한 변. 라벨 29모듈 중 데이터가 21모듈이다
        (gen_carrier_assets.py 의 QR_SCALE 주석: 29모듈 x 16px)."""
        return self.qr_side_m * 21.0 / 29.0


# T_QR_grasp 유도 — QR 프레임은 qr_label_ny(-Y 면) 기준으로
#   x_qr = +X_world,  y_qr = -Z_world,  z_qr = +Y_world
# (광학 프레임 x=right,y=down,z=forward 로 -Y 면을 보면 x = y×z = (-Z)×(+Y) = +X)
# 로컬 좌표 차이 d = (flange 윗면 중심) - (QR 라벨 중심) 을 그 축들로 사영하면
#   T = (d.x, -d.z, d.y)
# ★ 이 식이 grasp.yaml 의 매거진 두 값을 소수점까지 재현한다:
#     magazine_1_orange  QR(-0.09, -0.0705, 0.075)  flange(0,0,0.142)
#                        → [+0.090, -0.067, +0.0705]   = grasp.yaml ✓
#     magazine_2_blue    QR(-0.115,-0.1005, 0.035)  flange(0,0,0.106)
#                        → [+0.115, -0.071, +0.1005]   = grasp.yaml ✓
#   그래서 아래 스택 값도 같은 근거를 갖는다(스택은 grasp.yaml 에 없다).
MAGAZINE_SPECS = {
    "1": CarrierSpec("magazine_1_orange", "1", 0.050, np.array([0.090, -0.067, 0.0705])),
    "2": CarrierSpec("magazine_2_blue",   "2", 0.050, np.array([0.115, -0.071, 0.1005])),
}
STACK_SPECS = {
    # F3_STKO: QR(-0.1373,-0.06845,0.02472)  flange(0,0,0.07772)
    "F3-STKO": CarrierSpec("stack_orange", None, 0.040, np.array([0.1373, -0.0530, 0.06845])),
    # F3_STKB: QR(-0.1373,-0.06845,0.03996)  flange(0,0,0.09696)
    "F3-STKB": CarrierSpec("stack_blue",   None, 0.040, np.array([0.1373, -0.0570, 0.06845])),
}


# ══════════════════════════════════════════════════════════════
#  씬 기하 — agent 가 usda 에서 읽어 확인한 값
# ══════════════════════════════════════════════════════════════
CARTER_Z   = 0.07963398335074101     # 베이스 텔레포트 z (씬의 스폰 z 와 같다)
BELT_TOP_Z = 0.5407278772166425      # 세 벨트 모두 동일 (BeltTop 중심 0.49073 + 0.05)

# 벨트별 중심선 y. x 범위는 셋 다 [4.05, 8.75], 프레임은 [4.2, 8.6].
BELT_Y_PACKAGING = 4.6                    # +X 로 흐른다 → 포장 트리거(7.45)로 실려 간다
BELT_Y_TESTING   = -2.705314596908152     # +X 로 흐른다

# 벨트에 올려놓는 x. 프레임 앞면(4.2)보다 안쪽이라 벨트가 프레임에 받쳐진다.
# 더 앞(4.05~4.2)은 벨트가 프레임 밖으로 내민 구간이라 불안정하다.
BELT_PLACE_X = 4.30
# 벨트 앞에 서는 베이스 x. 12_place_test.py 가 프레임 앞면에서 0.35 m 띄운 것과 같다.
BELT_BASE_X  = 3.85

# 선반에서: 로봇은 매거진 앞 0.55 m (taught_poses 의 shelf_1_top_close_centered 가
# 실제로 쓴 상대 위치). shelves.yaml 에도 같은 값이 들어 있다.
SHELF_STANDOFF_M = 0.55

# 선반 앞에서 벨트로 나갈 때 거치는 통로 경유점.
#
# 왜 필요한가: nav_server 는 회피를 안 하고 목표로 직선으로 민다. 선반 pick
# 자세에서 벨트로 바로 쏘면 그 직선이 Shelf_01 을 관통한다 — 점유맵
# (simple_factory_layout.png, 해상도 0.05) 으로 재 보면 그 구간의 최소여유가
# 0.00 m 고, 실제로도 매번 (-0.16, +2.47) 에서 멈췄다(선반 동쪽 전면 모서리).
# 이 점을 거치면 두 구간 모두 최소여유 0.45 m 로 로봇 반경 0.35 m 를 넘긴다.
# (경유점 후보를 맵 위에서 격자 탐색해 고른 값 중 경로가 가장 짧은 것)
# ══════════════════════════════════════════════════════════════
#  경로 — 로봇 크기를 제대로 넣고 다시 잡은 값
#
#  ★ 처음엔 로봇 반경을 0.35 m 로 잡고 "여유 0.40 m 면 안전" 이라고 했는데
#    **틀렸다.** nav2 파라미터의 실제 풋프린트는
#      [[0.14, 0.25], [0.14, -0.25], [-0.607, -0.25], [-0.607, 0.25]]
#    즉 폭 0.5 m(반폭 0.25), 길이 0.747 m 인데 base_link 가 앞쪽에 치우쳐
#    **뒤로 0.607 m** 나간다. 외접 반경이 0.656 m 다(+패딩 0.03).
#    그래서 기준을 둘로 나눈다:
#      · 직진 통과      여유 >= 0.40  (반폭 0.25 + 여유)
#      · 제자리 회전    여유 >= 0.70  (외접 0.656 + 여유)
#
#  ★ 실제로 robot2 가 SHELF-B 앞에서 **넘어졌다.** 그 자리 여유는 0.40 m 인데
#    yaw=pi 로 서 있다가 북동쪽 대기 지점을 보려고 약 128도 제자리 회전을
#    했고, 뒤로 0.607 m 나간 몸이 Shelf_02 를 쓸었다. robot1 이 무사했던 건
#    회전각이 34도로 작았기 때문이지 경로가 안전해서가 아니었다.
#
#  ★ 그래서 선반에서는 **회전하지 않고 빠져나온다.** 같은 y 를 유지한 채
#    +x 로만 움직이면 된다 — robot1(yaw 0)은 전진, robot2(yaw pi)는 목표가
#    등 뒤라 nav_server 가 조향 없이 후진한다(steer_err = 0). 회전은 여유가
#    0.76~0.79 m 인 탈출 지점에 도착한 뒤에 한다.
# ══════════════════════════════════════════════════════════════
# 통로 동쪽 탈출점 — **Nav2 목표를 두 번에 나누기 위한** 지점이다.
#
# ★ 왜 다시 넣었나: Nav2 의 전역 계획은 선반을 피해 돌아가지만, 지역 계획
#   (DWB)이 지름길을 탄다. 지역 코스트맵을 만드는 라이다가 전방 180도만 보기
#   때문에(angle_min/max = ±90°) 옆에 있는 선반이 안 보인다. 실측: robot1 이
#   pick 자세에서 대기 지점으로 바로 보냈더니 매번 북쪽 Shelf_01 로 파고들어
#   (-2.42, +2.44) — 선반 전면에서 0.05 m — 에 갇혔고 planner 실패 16 건이
#   났다. 같은 실행에서 직선이 통로 안에 머무는 robot2 는 실패 0 건이었다.
#
#   목표를 통로 안 지점으로 한 번 끊어 주면 DWB 의 지역 목표가 통로가 되어
#   코너를 파고들 이유가 없어진다. 선반과 같은 y 로 동쪽으로만 나온다.
#   A->E1 구간 여유 0.45 m, B->E2 0.40 m (로봇 반폭 0.25 m).
#   두 탈출점 모두 여유 ~1.0 m 라 거기서 방향을 바꿔도 된다(외접 0.656 m).
SHELF_EXIT_ROBOT1 = ((0.60, 2.0364),)
SHELF_EXIT_ROBOT2 = ((0.60, -1.0636),)

# 벨트 **정서쪽** 공용 접근점. 여기서만 벨트로 들어간다.
#
# ★ 왜 정서쪽인가: 접근 방향(heading 0)이 벨트 최종 yaw(0)와 같아서 벨트 앞에서
#   제자리 회전을 하지 않는다. 벨트 접안점 여유는 0.20 m 뿐이라 거기서 돌면
#   뒤로 0.607 m 나간 몸이 걸린다. 이 점의 여유 1.80 m, 여기서 벨트까지 0.90 m.
BELT_APPROACH_P = (2.25, 4.60)

# 경유점·주차 지점은 이만큼 안에 들어오면 도달한 것으로 본다.
# nav_server 의 도착 판정(DRIVE_POS_TOL = 8 cm)과 Nav2 의 goal checker 는
# **마지막 접안**에나 필요한 값이다. 통과만 하면 되는 지점에까지 요구하면,
# 조향 오차가 커질수록 속도를 0 으로 줄이는 제어 특성 때문에 목표 근처에서
# 기어가다 타임아웃이 난다(실측: 0.33 m 를 남기고 180 초 소진).
VIA_REACHED_TOL_M = 0.60

# 벨트 자리를 기다리는 동안 미리 가 있을 자리. (x, y, yaw_rad)
#
# ★ 둘 다 벨트 **남쪽**이다. robot1 을 북쪽(3.00, 6.00)에 뒀을 때 Shelf_01 을
#   돌아 넘어가야 했고, 지역 코스트맵을 만드는 라이다가 전방 180도만 보는 탓에
#   왼쪽 선반을 놓쳐 DWB 가 코너를 파고들었다 — 팽창영역에 갇혀 Nav2 가
#   "failed to plan from (-2.11, 2.40)" 로 출발조차 못 했다. 남쪽으로만 움직인
#   robot2 는 같은 실행에서 planner 실패가 0 건이었다.
#
#   robot1 (4.50, 2.75)  벨트까지 1.96 m  여유 1.05 m   탈출점에서 구간 0.90
#   robot2 (3.25, 3.75)  벨트까지 1.04 m  여유 0.81 m   탈출점에서 구간 0.81
#   둘 사이 1.60 m (외접 0.656 두 대분 1.32 m 보다 넓다)
#
# ★ 배정이 교차한다(가까운 선반↔먼 대기 지점). 각자의 탈출점에서 상대편
#   대기 지점으로 가는 구간이 더 넓기 때문이다 — robot2 의 탈출점에서
#   (4.50, 2.75) 로 가는 직선은 여유가 0.21 m 까지 좁아진다.
# yaw 는 접근점 P 를 향하게 준다 — 자리를 얻으면 곧바로 출발한다.
STAGING_ROBOT1 = (4.50, 2.75, 2.4534)
STAGING_ROBOT2 = (3.25, 3.75, 2.4371)


# ══════════════════════════════════════════════════════════════
#  로봇 / 그리퍼
# ══════════════════════════════════════════════════════════════
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# ★ 강성을 두 단계로 쓴다. 관측 자세에서 1e8 이면 잔진동으로 **QR 디코딩이
#   깨진다**(sim_backend.py 실측). 관측은 1e5, 파지/이송은 1e8.
DRIVE_STIFFNESS_SCAN = 1e5
DRIVE_STIFFNESS_PICK = 1e8
DRIVE_DAMPING        = 1e4
DRIVE_MAX_FORCE      = 1e8

READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

COAXIAL_FORCE_LIMIT = 200.0
SHEAR_FORCE_LIMIT   = 100.0
MAX_GRIP_DISTANCE   = 0.03

SUCTION_FACE_Z = 0.161
TCP_OFFSET     = np.array([0.0, 0.0, SUCTION_FACE_Z])

GRIP_GAPS = [0.005, 0.002, 0.000, -0.003]

APPROACH_HEIGHT_OFFSET = 0.15
LIFT_HEIGHT_OFFSET     = 0.10
PLACE_DROP             = 0.005

GRIP_WAIT    = 90
HOLD_WAIT    = 120
RELEASE_WAIT = 90
SETTLE_STEPS = 90
BASE_MOVE_SETTLE_STEPS = 60

TCP_SPEED = 0.002
MIN_STEPS = 90
MAX_STEPS = 600

APPROACH_ROLL_DEG  = 180.0
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0

MAX_IK_FAIL_STEPS = 30
LOG_INTERVAL = 60

LIFT_OK_MIN_M = 0.005
TILT_MAX_DEG  = 5.0

# QR 인식
QR_FRAMES       = 3        # 여러 장 찍어 중앙값으로 모은다(aggregate_qr_poses)
# ★ 캡쳐 해상도. frames.yaml 의 K 는 1280x720 기준이지만, 그 해상도에서는
#   50 mm QR 을 277 mm 에서 보면 **모듈당 3.8 px** 이라 디코딩이 간신히 되거나
#   안 된다(실측: 3프레임 중 1프레임만 성공). 배율을 올리면 모듈당 픽셀이
#   그만큼 늘어난다. FOV 는 그대로이므로 K 는 배율만큼 선형으로 늘리면 된다.
CAPTURE_SCALE   = 2        # 1280x720 -> 2560x1440,  모듈당 3.8 -> 7.6 px
QR_SETTLE_STEPS = 30
# ★ 두 대를 동시에 돌릴 때는 반드시 annotator 다.
#   viewport 캡쳐(MultiAOVByteCapture)는 **활성 뷰포트 하나**를 손목캠으로
#   바꿔서 찍는 방식이라, 두 로봇이 같은 뷰포트를 서로 뺏는다. annotator 는
#   카메라마다 render_product 를 따로 붙이므로 몇 대든 동시에 찍을 수 있고
#   헤드리스에서도 된다. CAPTURE 를 명시하면 그 값을 존중한다.
CAPTURE_MODE    = os.environ.get(
    "CAPTURE", "viewport" if int(os.environ.get("ROBOTS", "2")) < 2 else "annotator")
DEPTH_AOV_NAME  = "DistanceToImagePlaneSD"   # ★ Replicator 이름이 아니다

# 스택 대기
STACK_SPAWN_TIMEOUT_S = 90.0   # 벨트가 매거진을 트리거까지 실어 가는 시간
STACK_SETTLE_TOL_M    = 0.002
STACK_SETTLE_FRAMES   = 60

FAIL_OK, FAIL_SLIP, FAIL_TIMEOUT, FAIL_UNREACHABLE, FAIL_NO_QR = 0, 3, 5, 6, 8


# ══════════════════════════════════════════════════════════════
#  회전 / 기하 유틸 — 12_place_test.py 와 동일
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_from_axis(axis, deg):
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    return np.concatenate([[np.cos(half)], (a / np.linalg.norm(a)) * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg), quat_from_axis([0, 1, 0], pitch_deg))
    return quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg)) / 1.0


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def quat_xyzw_to_mat(q):
    x, y, z, w = q
    return quat_to_matrix(np.array([w, x, y, z]))


def tcp_to_flange(tcp_pos, quat):
    return np.array(tcp_pos) - quat_to_matrix(quat) @ TCP_OFFSET


def get_tcp_pose(robot):
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


def ease(alpha):
    a = float(np.clip(alpha, 0.0, 1.0))
    return a * a * (3.0 - 2.0 * a)


def steps_for(start, goal):
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def tilt_deg_from_quat(q):
    up = quat_to_matrix(q) @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def vec(v, d=3):
    return "[" + " ".join(f"{x:+.{d}f}" for x in v) + "]"


def section(title):
    print(f"\n{'─' * 66}\n {title}\n{'─' * 66}")


# ══════════════════════════════════════════════════════════════
#  USD 유틸
# ══════════════════════════════════════════════════════════════
def stage():
    return omni.usd.get_context().get_stage()


def find_prim_path(root_path, name):
    root = stage().GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def resolve_articulation_root(candidates, fallback):
    for path in candidates:
        prim = stage().GetPrimAtPath(path)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    return fallback


def get_world_pose(prim_path):
    prim = stage().GetPrimAtPath(prim_path)
    m = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return np.array([t[0], t[1], t[2]]), np.array([q.GetReal(), im[0], im[1], im[2]])


def measure_prim(prim_path):
    """(중심 xy, 윗면 z, 높이) — 채점과 배치 높이 계산에만 쓴다(파지에는 안 쓴다)."""
    prim = stage().GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng.IsEmpty():
        raise RuntimeError(f"{prim_path} 의 바운딩박스가 비어 있다")
    lo, hi = rng.GetMin(), rng.GetMax()
    return (np.array([(lo[0]+hi[0])/2.0, (lo[1]+hi[1])/2.0]),
            float(hi[2]), float(hi[2]-lo[2]))


def filter_collision(path_a, path_b):
    a = stage().GetPrimAtPath(path_a)
    UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel().AddTarget(Sdf.Path(path_b))


def set_prim_pose(prim_path, pos, quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    """USD 로 직접 pose 를 다시 쓴다 — 물리 엔진이 rigid body 를 따라온다."""
    xform = UsdGeom.Xformable(stage().GetPrimAtPath(prim_path))
    xform.ClearXformOpOrder()
    m = Gf.Matrix4d().SetRotate(Gf.Quatd(quat_wxyz[0], Gf.Vec3d(*quat_wxyz[1:])))
    m.SetTranslateOnly(Gf.Vec3d(*pos))
    xform.AddTransformOp().Set(m)


def set_arm_stiffness(value, arm_path):
    """관측(낮게) ↔ 파지(높게) 를 오간다. 이유는 위 DRIVE_STIFFNESS_SCAN 주석.

    ★ arm_path 를 받는다 — 두 대를 돌릴 때 한쪽 강성을 바꾸면서 다른 쪽까지
      같이 바꾸면, 관측 중인 로봇의 팔이 파지 강성으로 굳어 버린다."""
    n = 0
    for prim in Usd.PrimRange(stage().GetPrimAtPath(arm_path)):
        if prim.GetName() not in ARM_JOINTS:
            continue
        for kind in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, kind)
            if drive:
                drive.GetStiffnessAttr().Set(value)
                n += 1
    return n


# ══════════════════════════════════════════════════════════════
#  흡착 그리퍼 — 12_place_test.py 와 동일
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
        self._view.apply_gripper_action(np.array([1.0])) if self._view else self._command("close_gripper")

    def open(self):
        self._view.apply_gripper_action(np.array([-1.0])) if self._view else self._command("open_gripper")

    def gripped(self):
        try:
            if self._view is not None:
                return self._view.get_gripped_objects()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripped_objects(self._path)
        except Exception:
            return None

    def status(self):
        try:
            if self._view is not None:
                return self._view.get_surface_gripper_status()
            import isaacsim.robot.surface_gripper as sg
            return sg.get_gripper_status(self._path)
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
#  QR 인식 — 캡쳐 + 자세 추정
# ══════════════════════════════════════════════════════════════
def _pycapsule_to_bytes(capsule, size):
    """뷰포트 캡쳐 콜백은 bytes 가 아니라 PyCapsule 을 준다."""
    ctypes.pythonapi.PyCapsule_GetName.restype = ctypes.c_char_p
    ctypes.pythonapi.PyCapsule_GetName.argtypes = [ctypes.py_object]
    name = ctypes.pythonapi.PyCapsule_GetName(capsule)
    ctypes.pythonapi.PyCapsule_GetPointer.restype = ctypes.c_void_p
    ctypes.pythonapi.PyCapsule_GetPointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    ptr = ctypes.pythonapi.PyCapsule_GetPointer(capsule, name)
    if not ptr:
        raise RuntimeError("PyCapsule 에서 포인터를 못 가져왔다")
    return bytes((ctypes.c_uint8 * size).from_address(ptr))


class QrVision:
    """손목 카메라로 QR 을 읽어 월드 자세를 낸다.

    ★ 카메라 내부파라미터(K)와 link_6→카메라 변환은 **frames.yaml** 에서 읽는다.
      USD 카메라 prim 에서 뽑지 않는 이유는 ROS TF 가 주는 값과 같아야 하기
      때문이다(diag_qr_reproj.py 가 두 경로의 차이를 0.0 mm / 0.0 deg 로 측정했다).

    ★ K 는 해상도 1280x720 기준이다. 뷰포트 캡쳐를 쓰면 **뷰포트 크기가 곧
      해상도**라, 창 크기가 다르면 K 가 틀린 값이 된다. 캡쳐한 프레임 크기를
      확인해서 어긋나면 경고한다.
    """

    BASE_WH = (1280, 720)          # frames.yaml 의 K 가 기준으로 삼은 해상도

    def __init__(self, world, ctx):
        # ctx 로 카메라/링크 경로를 받는다 — 두 대가 각자 손목캠을 쓴다.
        self._world = world
        self._ctx = ctx
        self._pending = None
        self._ready = False

        with open(FRAMES_YAML, "r", encoding="utf-8") as fh:
            frames = yaml.safe_load(fh)
        st = frames["static_transforms"]
        self.R_l6_cam  = quat_xyzw_to_mat(st["m0609_tool0__camera_link"]["quat_xyzw"])
        self.t_l6_cam  = np.array(st["m0609_tool0__camera_link"]["xyz"], dtype=float)
        self.R_cam_opt = quat_xyzw_to_mat(st["camera_link__camera_color_optical_frame"]["quat_xyzw"])
        ci = frames["wrist_camera"]["camera_info_observed"]
        self.K_base = np.array(ci["k"], dtype=float).reshape(3, 3)
        self.K      = self.K_base.copy()
        self.dist   = np.array(ci["d"], dtype=float)
        print(f"   camera K     fx={self.K[0,0]:.1f} cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}"
              f"  (기준 {self.BASE_WH[0]}x{self.BASE_WH[1]})")

    def _scale_K(self, width, height):
        """캡쳐된 실제 해상도에 맞춰 K 를 다시 만든다.

        ★ 해상도만 바뀌고 FOV 는 그대로이므로 fx,fy,cx,cy 가 배율에 선형으로 붙는다.
          예전 판은 해상도가 다르면 경고만 하고 틀린 K 를 그대로 썼는데, 그러면
          QR 위치가 조용히 어긋난다 — 경고보다 고치는 쪽이 맞다.
        """
        sx = width / float(self.BASE_WH[0])
        sy = height / float(self.BASE_WH[1])
        K = self.K_base.copy()
        K[0, 0] *= sx; K[0, 2] *= sx
        K[1, 1] *= sy; K[1, 2] *= sy
        return K

    # ── 카메라 월드 자세 (link_6 × 정적 변환) ────────────────
    def optical_pose(self):
        l6_p, l6_q = get_world_pose(self._ctx.ee_link)
        R_l6 = quat_to_matrix(l6_q)
        return R_l6 @ self.R_l6_cam @ self.R_cam_opt, l6_p + R_l6 @ self.t_l6_cam

    # ── 캡쳐 ─────────────────────────────────────────────────
    def warm(self):
        """제너레이터다 — 렌더가 돌아야 애노테이터가 채워지는데, 그 스텝을
        스스로 밟으면 두 대를 동시에 못 돌린다. run_concurrent 가 밟는다."""
        if CAPTURE_MODE == "annotator":
            yield from self._warm_annotator()
        else:
            yield from self._warm_viewport()
        self._ready = True

    def _warm_annotator(self):
        import omni.replicator.core as rep
        self._rp = rep.create.render_product(
            self._ctx.camera_prim,
            (self.BASE_WH[0] * CAPTURE_SCALE, self.BASE_WH[1] * CAPTURE_SCALE))
        self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self._depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        self._rgb_annot.attach([self._rp])
        self._depth_annot.attach([self._rp])
        for _ in range(30):
            yield
        print(f"   capture      annotator (render_product)  {self._ctx.name}")

    def _warm_viewport(self):
        # ★ add_aov_to_viewport 는 saveUsdAttributes 가 True 면 UnboundLocalError
        #   로 죽는다(업스트림 버그). 먼저 꺼 둔다.
        import carb.settings
        from omni.kit.viewport.utility import get_active_viewport, add_aov_to_viewport

        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError(
                "활성 뷰포트가 없다 — 뷰포트 캡쳐는 GUI 에서만 된다. "
                "헤드리스로 돌리려면 CAPTURE=annotator 로 바꿔 볼 것")
        viewport.camera_path = self._ctx.camera_prim
        # ★ 렌더 해상도를 올린다. QR 모듈이 픽셀로 몇 개냐가 디코딩 성패를 가른다.
        want = (self.BASE_WH[0] * CAPTURE_SCALE, self.BASE_WH[1] * CAPTURE_SCALE)
        try:
            viewport.resolution = want
            print(f"   viewport     해상도 {want[0]}x{want[1]} 로 설정 (x{CAPTURE_SCALE})")
        except Exception as exc:
            print(f"   viewport     해상도 설정 실패 ({exc}) — 기본값으로 진행한다")
        carb.settings.get_settings().set("/app/hydra/renderSettings/saveUsdAttributes", False)
        add_aov_to_viewport(viewport, DEPTH_AOV_NAME)
        for _ in range(30):
            yield
        print("   capture      viewport (MultiAOVByteCapture)  ※ 뷰포트가 손목캠 화면으로 바뀐다")

    def capture(self, timeout_frames=240):
        """(bgr uint8 HxWx3, depth float64 HxW[m]) — depth 는 광학 z(=image plane 거리)."""
        if not self._ready:
            yield from self.warm()
        rgb, depth = (yield from (self._capture_annotator() if CAPTURE_MODE == "annotator"
                                  else self._capture_viewport(timeout_frames)))
        h, w = depth.shape
        self.K = self._scale_K(w, h)     # 경고 대신 실제 해상도에 맞춘다
        return rgb, depth

    def _capture_annotator(self):
        for _ in range(4):
            yield
        img = np.asarray(self._rgb_annot.get_data())
        if img.size == 0:
            raise RuntimeError(
                "rgb 애노테이터가 빈 프레임을 줬다 — Isaac 5.1.0 rc.19 의 알려진 버그다. "
                "CAPTURE=viewport 로 돌릴 것(GUI 필요)")
        bgr = img[:, :, :3][:, :, ::-1].copy()
        depth = np.asarray(self._depth_annot.get_data(), dtype=np.float64).reshape(bgr.shape[:2])
        return bgr, depth

    def _capture_viewport(self, timeout_frames):
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.widget.viewport.capture import MultiAOVByteCapture

        viewport = get_active_viewport()
        out = {}

        def _on_rgb(buffer, size, width, height, fmt):
            arr = np.frombuffer(_pycapsule_to_bytes(buffer, size), dtype=np.uint8, count=size)
            out["rgb"] = arr.reshape(height, width, 4)[:, :, :3][:, :, ::-1].copy()

        def _on_depth(buffer, size, width, height, fmt):
            arr = np.frombuffer(_pycapsule_to_bytes(buffer, size), dtype=np.float32,
                                count=width * height)
            out["depth"] = arr.reshape(height, width).astype(np.float64).copy()

        cap = MultiAOVByteCapture(["", DEPTH_AOV_NAME], [_on_rgb, _on_depth])
        self._pending = cap          # ★ GC 되면 콜백이 영영 안 온다
        viewport.schedule_capture(cap)
        for _ in range(timeout_frames):
            yield
            if "rgb" in out and "depth" in out:
                self._pending = None
                return out["rgb"], out["depth"]
        self._pending = None
        raise RuntimeError("프레임 캡쳐 타임아웃 — rgb/depth 콜백이 오지 않았다")

    # ── QR 자세 ──────────────────────────────────────────────
    def scan(self, spec: CarrierSpec, n_frames=QR_FRAMES, dump_tag=None):
        """여러 장을 찍어 중앙값으로 모은다. 한 장은 잡음에 약하다.

        ★ dump_tag 를 주면 캡쳐한 프레임을 파일로 떨군다. QR 이 안 읽힐 때
          '카메라가 어디를 보고 있나' 는 눈으로 봐야 알 수 있다 — 로그만으로는
          화각 밖인지, 너무 멀어서인지, 어두워서인지 구분이 안 된다.
        """
        import cv2
        obs = []
        for i in range(n_frames):
            bgr, depth = yield from self.capture()
            R_opt, p_opt = self.optical_pose()
            # ★ 씬 조명이 어두워 흰 라벨이 172/255 정도로만 찍힌다(실측).
            #   대비를 펴 주면 같은 프레임이 디코딩된다. 기하는 안 건드린다.
            bgr = cv2.normalize(bgr, None, 0, 255, cv2.NORM_MINMAX)
            o = estimate_qr_pose(
                bgr, depth, self.K, self.dist, R_opt, p_opt,
                expected_id=spec.expected_id,
                data_side_m=spec.data_side_m)   # ★ 스택은 40 mm 라 기본값과 다르다
            obs.append(o)
            if dump_tag:
                DUMP_DIR.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(DUMP_DIR / f"{dump_tag}_{i}_rgb.png"), bgr)
                d = np.nan_to_num(depth, nan=0.0, posinf=0.0)
                near = np.clip(d, 0.0, 2.0) / 2.0 * 255.0
                cv2.imwrite(str(DUMP_DIR / f"{dump_tag}_{i}_depth.png"), near.astype(np.uint8))
        agg = aggregate_qr_poses(obs)
        for i, o in enumerate(obs):
            print(f"      frame {i}  ok={o.ok}  decoded={o.decoded!r}  {o.reason}")
        if dump_tag:
            print(f"      프레임 저장 → {DUMP_DIR}/{dump_tag}_*.png")
        return agg


def grasp_from_qr(agg, spec: CarrierSpec):
    """QR 자세 → 흡착점(월드).

    QR 프레임을 월드에서 다시 세운다:
      x_qr : yaw 로 정해지는 수평 방향(라벨의 '오른쪽')
      y_qr : 월드 -Z (라벨의 '아래'. 캐리어가 똑바로 서 있다는 가정)
      z_qr : x×y (라벨 안쪽)
    sim_backend.scan_qr 이 쓰는 것과 같은 구성이다.
    """
    yaw = float(agg.yaw_rad)
    x_qr = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    y_qr = np.array([0.0, 0.0, -1.0])
    R_qr = np.column_stack([x_qr, y_qr, np.cross(x_qr, y_qr)])
    return np.asarray(agg.center_world, dtype=float) + R_qr @ spec.t_qr_grasp, yaw


# ══════════════════════════════════════════════════════════════
#  씬 구성
# ══════════════════════════════════════════════════════════════
class Task2(BaseTask):
    """씬을 한 번 올리고, ctxs 에 든 로봇을 **전부** 셋업한다.

    ★ 월드 USD 는 한 번만 참조한다 — 로봇마다 붙이면 씬이 겹쳐 올라간다."""

    def __init__(self, name, ctxs):
        super().__init__(name=name, offset=None)
        self._ctxs = ctxs
        self._robots = {}
        self._gripper_paths = {}

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        world_prim = stage().GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage(), "/World").GetPrim()
        world_prim.GetReferences().AddReference(WORLD_USD)
        for _ in range(15):
            simulation_app.update()
        print(f"   USD          loaded  {WORLD_USD}")

        for ctx in self._ctxs:
            print(f"   ── {ctx.name}  ({ctx.shelf_id})")
            print(f"   arm drives   {set_arm_stiffness(DRIVE_STIFFNESS_PICK, ctx.arm)}")
            for prim in Usd.PrimRange(stage().GetPrimAtPath(ctx.arm)):
                if prim.GetName() in ARM_JOINTS:
                    for kind in ("angular", "linear"):
                        d = UsdPhysics.DriveAPI.Get(prim, kind)
                        if d:
                            d.GetDampingAttr().Set(DRIVE_DAMPING)
                            d.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)

            node_path = find_prim_path(ctx.gripper_prim, "SurfaceGripper")
            if node_path is None:
                raise RuntimeError(f"SurfaceGripper node not found under {ctx.gripper_prim}")
            node = stage().GetPrimAtPath(node_path)
            node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
            node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
            node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)
            self._gripper_paths[ctx.name] = node_path
            print(f"   gripper      {node_path}")

            # ★ 그리퍼만이 아니라 **로봇 전체**와 걸러야 한다.
            #   팔을 READY 로 접으면 매거진이 차체 가까이 온다. 주행 진동에
            #   차체나 다른 링크에 스치면 그 충격이 흡착 한계를 넘겨 떨어진다.
            #   실측: 질량 1 kg(무게 ~10 N)인데 한계는 coaxial 200 N /
            #   shear 100 N 이라 하중으로는 절대 안 풀린다. 그런데 열린 공간
            #   직진 중에 두 대 다 놓쳤다(robot1 35s, robot2 24s) — 남는
            #   설명은 충돌 충격뿐이다.
            filter_collision(ctx.gripper_prim, ctx.magazine)
            filter_collision(ctx.carter, ctx.magazine)
            filter_collision(ctx.arm, ctx.magazine)

            ee_path = find_prim_path(ctx.arm, EE_LINK_NAME)
            root_path = resolve_articulation_root(ctx.articulation_candidates, ctx.arm)
            # ★ name 은 씬 안에서 유일해야 한다 — 두 대가 같은 이름이면
            #   scene.add 가 뒤엣것으로 덮어써서 한 대만 움직인다.
            self._robots[ctx.name] = scene.add(SingleManipulator(
                prim_path=root_path, name=f"m0609_{ctx.name}",
                end_effector_prim_path=ee_path))
            print(f"   articulation {root_path}")

    def robot(self, name):
        return self._robots[name]

    def gripper_node_path(self, name):
        return self._gripper_paths[name]


def set_ready_pose(robot):
    q = np.zeros(robot.num_dof)
    for name, deg in zip(ARM_JOINTS, READY_JOINTS_DEG):
        q[robot.get_dof_index(name)] = np.deg2rad(deg)
    robot.set_joint_positions(q)


def set_joints_deg(robot, joints_deg):
    q = robot.get_joint_positions()
    for name, deg in zip(ARM_JOINTS, joints_deg):
        q[robot.get_dof_index(name)] = np.deg2rad(deg)
    robot.set_joint_positions(q)


# 팔을 접는 데 주는 스텝 수. 관절이 실제로 따라가야 하므로 넉넉히 준다 —
# 목표만 걸고 바로 출발하면 접히는 도중에 베이스가 움직인다.
CARRY_RAMP_STEPS = 150


def carry_pose_action(robot):
    """반송 자세(READY) 목표. (관절 인덱스, 목표 라디안, ArticulationAction).

    ★ 이걸 따로 뺀 이유: 접은 **뒤에도 계속 걸어 줘야** 한다. 벨트 자리를
      기다리는 동안 목표를 안 걸었더니 팔이 흘러내려 매거진을 놓쳤다
      (실측: 대기 중 gripped=True -> False). hold_ik 가 "목표를 매 스텝 다시
      건다" 고 적어 둔 것과 같은 문제다 — 한 번만 걸면 안 된다.
    """
    idx = np.array([robot.get_dof_index(name) for name in ARM_JOINTS])
    target = np.deg2rad(np.array(READY_JOINTS_DEG, dtype=float))
    return idx, target, ArticulationAction(joint_positions=target, joint_indices=idx)


def set_carry_pose(robot, gripper=None, n_steps=CARRY_RAMP_STEPS):
    """PICK 을 끝낸 팔을 READY(0, 0, 90, 0, 90, 0)로 접는다. 주행 직전에 부른다.

    왜 필요한가(실측): PICK 을 끝낸 팔은 매거진을 든 채 선반 쪽으로 뻗어 있다.
    그대로 베이스가 출발하면 팔과 캐리어가 선반에 걸려 로봇이 못 빠져나온다 —
    nav_server 는 cmd_vel 로 0.32 m/s 를 계속 내보내는데 amcl_pose 는 2분에
    8 cm 밖에 안 움직였고, y≈2.47(Shelf_01 전면 기둥 y=2.4891 바로 앞)에
    붙어 선 채였다.

    ★ set_joint_positions 로 순간이동시키지 않는다 — 흡착으로 붙어 있는
      캐리어가 관절의 순간 점프를 못 따라가 떨어질 수 있다. 목표 관절값을
      apply_action 으로 걸고 컨트롤러가 따라가게 한다. hold_ik 와 같은
      이유로 **매 스텝 다시 건다** — 한 번만 걸면 팔이 흘러내린다.

    덤: PLACE 의 IK 시드도 좋아진다. PICK 직후 관절은 먼 목표에 나쁜 시드라
    IK 가 안 풀린다는 게 이미 reattach_if_needed 에 적혀 있던 문제다.

    반환: 접고 나서도 흡착이 남아 있으면 True (gripper 를 안 주면 항상 True).
    """
    idx, target, action = carry_pose_action(robot)
    for _ in range(n_steps):
        robot.apply_action(action)
        yield

    err_deg = float(np.rad2deg(np.abs(robot.get_joint_positions()[idx] - target)).max())
    if gripper is None:
        print(f"   반송 자세(READY) 복귀  최대 관절오차 {err_deg:.1f}°")
        return True
    held = holding(gripper.gripped())
    print(f"   반송 자세(READY) 복귀  최대 관절오차 {err_deg:.1f}°  gripped={held}")
    if not held:
        print("   !! 팔을 접는 동안 흡착이 풀렸다 — 캐리어를 놓쳤다")
    return held


def sync_ik_base_pose(ctx):
    pos, quat = get_world_pose(ctx.base_link)
    ctx.lula.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
    return pos, quat


# ══════════════════════════════════════════════════════════════
#  Nav2 주행 — isaac_python 안에서 rclpy 를 import 할 수 없어 ros2 CLI 를
#  자식 프로세스로 띄운다.
#
#  ★ Isaac 5.1 의 kit python 은 3.11 이고 ROS2 Jazzy(apt)는 3.12 용 바이너리만
#    있다. 그래서 이 프로세스 안에서 rclpy.node.Node() 를 만들면 하드 크래시가
#    난다. 자식 프로세스 안에서는 시스템 python 이 처리하니 ABI 문제가 없다.
#
#  ★ 텔레포트를 대신하는 것이라 **재흡착 단계가 통째로 사라진다.**
#    베이스 아티큘레이션에 set_world_pose() 를 부르면 거리와 무관하게 흡착이
#    즉시 풀려서, 예전 판은 팔을 목표로 보낸 뒤 캐리어를 그리퍼 밑에 갖다 놓고
#    다시 붙이는 우회가 필요했다. 실제로 주행하면 그럴 일이 없다.
# ══════════════════════════════════════════════════════════════
# ★ nav_server 는 **상대 이름**을 쓴다 — 노드가 뜬 네임스페이스가 곧 로봇
#   식별자다(nav_server.py 주석: "robot_namespace 파라미터는 없앴다").
#   그래서 /robot1 아래로 띄워야 cmd_vel 이 /robot1/cmd_vel 로 나가고
#   amcl_pose 도 /robot1/amcl_pose 를 듣는다.
#       ros2 run cobot3_navigation nav_server --ros-args -r __ns:=/robot1 \
#            -p use_sim_time:=true
#   네임스페이스 없이 띄우면 /cmd_vel 로 혼자 떠들고 amcl_pose 를 못 받아
#   goal 을 받아도 주행이 시작되지 않는다(실측).
# 네임스페이스는 이제 로봇마다 다르다 — NavDriver 인스턴스가 들고 있고,
# 여기 있는 건 한 대만 돌릴 때의 기본값이다.
NAV_NAMESPACE = os.environ.get("NAV_NS", "robot1")

# ★ 이송 구간은 Nav2 본래 스택(navigate_to_pose)에 맡긴다.
#
#   왜 바꿨나 — cobot3_navigation/nav_server 는 목표를 향해 직선으로 밀고,
#   조향 오차가 45도를 넘으면 전진 속도를 0 으로 만들고 **제자리 회전**만 한다
#   (nav_server.py:223 speed_scale). 그 회전이 어디서 일어날지는 컨트롤러가
#   정하지 경로가 정하지 않는다. 실측: robot2 가 선반 옆 여유 0.39 m 지점에서
#   돌기 시작해(cmd_vel linear.x=-0.0, angular.z=0.15, 위치 4초간 고정)
#   뒤로 0.607 m 나간 몸이 Shelf_02 를 때렸다. 경유점을 아무리 잘 잡아도
#   막을 수 없는 문제다.
#
#   Nav2 는 풋프린트를 알고(코스트맵 footprint) 장애물을 피해 경로를 짜므로
#   경유점을 손으로 계산할 필요도 없어진다. 대신 pick 자리의 정밀 정지는
#   여전히 텔레포트다 — Nav2 의 도착 허용오차는 주행용이지 파지용이 아니다.
NAV2_ACTION_TYPE    = "nav2_msgs/action/NavigateToPose"
PRECISE_ACTION_TYPE = "cobot3_interfaces/action/NavigateTo"
NAV_ACTION_TYPE = NAV2_ACTION_TYPE          # 기동 점검용(하나만 확인하면 된다)
NAV_DRIVE_TIMEOUT_S = 180.0

# Nav2 를 쓸지 텔레포트로 갈지. nav_server 가 안 떠 있으면 NAV=0 으로 돌린다.
USE_NAV = os.environ.get("NAV", "1") == "1"


def _clean_ros2_env():
    """isaac_python(Kit 번들 Python 3.11) 은 자기 stdlib/extension 경로를
    PYTHONPATH/PYTHONHOME 에 심어 두는데, 이게 자식 프로세스로 띄우는 시스템
    `ros2`(별도 python3 바이너리, /opt/ros/jazzy) 에도 그대로 상속된다.
    ros2 스크립트가 자기 stdlib 대신 Isaac 의 순수 파이썬 `re` 모듈을 줍고,
    그 안의 컴파일된 `_sre` 확장(다른 빌드)과 매직 넘버가 안 맞아
    AssertionError: SRE module mismatch 로 죽는다(실측).

    ★ PYTHONPATH 를 통째로 지웠더니 다른 크래시가 났다 — ros2cli 자체가
    /opt/ros/jazzy/.../site-packages 를 PYTHONPATH 로 찾는데(ros_set 이
    source 하는 setup.bash 가 거기다 심어 둔다), 통째로 지우면 그것까지
    같이 날아가 PackageNotFoundError: ros2cli 로 죽었다(실측). "isaacsim"/
    "kit/python" 이 들어간 항목만 걸러내고 나머지(ROS2 site-packages)는
    남긴다. PYTHONHOME 은 ROS2 쪽이 쓸 일이 없어 통째로 지운다."""
    env = dict(os.environ)
    env.pop("PYTHONHOME", None)
    old_path = env.get("PYTHONPATH", "")
    kept = [p for p in old_path.split(os.pathsep)
            if p and "isaacsim" not in p and f"{os.sep}kit{os.sep}python" not in p]
    if kept:
        env["PYTHONPATH"] = os.pathsep.join(kept)
    else:
        env.pop("PYTHONPATH", None)
    return env


def _ros2_spawn(args, env):
    """subprocess.Popen/run 대신 os.posix_spawn 을 직접 쓴다.

    ★ 실측: subprocess 로 ros2 CLI 를 띄운 직후(퍼블리시/goal 요청 모두)
    Kit 프로세스가 매번 똑같은 크래시로 죽었다 — omni.graph.core.plugin 의
    std::recursive_mutex 해시맵을 건드리다 죽는데, 스택 심볼이
    atexit_callfuncs/Py_FinalizeEx 로 찍혀서 처음엔 종료 시점 크래시로
    오인했다. 하지만 스크립트가 아직 한참 남은 시점(NAV 섹션 진입 직후)
    에도 똑같이 났다 — subprocess 호출 자체가 방아쇠였다는 뜻이다.

    Kit 은 carb.tasking.plugin 스레드를 24개 띄워 두는데(커맨드라인
    --/plugins/carb.tasking.plugin/threadCount=24), 그 상태에서
    subprocess.Popen 이 (조건에 따라) fork() 경로를 타면 다른 스레드가
    들고 있던 락이 자식 프로세스에 풀리지 않은 채로 남는 고전적
    fork-안전성 문제가 생긴다 — 정확히 recursive_mutex 해시맵이 걸리는
    이유와 들어맞는다. os.posix_spawn() 은 vfork 기반이라 이 문제를
    피한다. 반환: (pid, stdout+stderr 를 묶은 읽기용 fd)."""
    exe = shutil.which(args[0])
    if exe is None:
        raise FileNotFoundError(args[0])
    r_fd, w_fd = os.pipe()
    try:
        pid = os.posix_spawn(
            exe, args, env,
            file_actions=[
                (os.POSIX_SPAWN_DUP2, w_fd, 1),
                (os.POSIX_SPAWN_DUP2, w_fd, 2),
                (os.POSIX_SPAWN_CLOSE, r_fd),
                (os.POSIX_SPAWN_CLOSE, w_fd),
            ],
        )
    finally:
        os.close(w_fd)
    os.set_blocking(r_fd, False)
    return pid, r_fd


def _ros2_drain(r_fd, chunks):
    while True:
        try:
            data = os.read(r_fd, 65536)
        except BlockingIOError:
            return
        except OSError:
            return
        if not data:
            return
        chunks.append(data)


def _ros2_poll(pid, r_fd, chunks):
    """None 이면 아직 실행 중. 끝났으면 (returncode, 누적 출력 str)."""
    _ros2_drain(r_fd, chunks)
    done_pid, status = os.waitpid(pid, os.WNOHANG)
    if done_pid != pid:
        return None
    _ros2_drain(r_fd, chunks)
    os.close(r_fd)
    rc = os.WEXITSTATUS(status) if os.WIFEXITED(status) else -1
    return rc, b"".join(chunks).decode(errors="replace")


def _ros2_kill(pid, r_fd):
    try:
        os.kill(pid, 9)
        os.waitpid(pid, 0)
    except OSError:
        pass
    os.close(r_fd)


def _ros2_run_blocking(args, env, timeout_s):
    """비대화형 짧은 호출용(NAV 섹션의 사전 점검 등) — world.step() 인터리빙이
    필요 없는 곳에서만 쓴다. 반환: (returncode 또는 None(타임아웃), 출력)."""
    pid, r_fd = _ros2_spawn(args, env)
    chunks = []
    deadline = time.time() + timeout_s
    while True:
        result = _ros2_poll(pid, r_fd, chunks)
        if result is not None:
            return result
        if time.time() > deadline:
            _ros2_kill(pid, r_fd)
            return None, "".join(c.decode(errors="replace") for c in chunks)
        time.sleep(0.02)


# ══════════════════════════════════════════════════════════════
#  QR 디코딩을 바깥 파이썬에 맡긴다
# ══════════════════════════════════════════════════════════════
#  Isaac 이 번들한 cv2 는 contrib 가 아니다 — 4.11.0 인데
#  wechat_qrcode_WeChatQRCode 가 없다(실측:
#  hasattr(cv2, "wechat_qrcode_WeChatQRCode") == False).
#  그래서 Isaac 안에서는 기울어진 라벨을 못 읽는다. 같은 프레임 9장을
#  시스템 파이썬(3.12, opencv-contrib)은 9/9 읽었고 Isaac 안에서는 0/9 였다.
#
#  ROS 노드(carrier_code_reader)는 시스템 파이썬에서 도니까 원래 잘 읽는다.
#  degrade 되는 건 이 Isaac 테스트뿐이라, 여기서만 밖으로 한 번 나갔다 온다.
#
#  ★ subprocess 를 쓰지 않는다 — Kit 의 24 스레드 프로세스에서 fork 하면
#    죽는다(_ros2_spawn 독스트링 참고). 같은 posix_spawn 경로를 쓴다.
QR_DECODE_PYTHON = os.environ.get("QR_DECODE_PYTHON", "/usr/bin/python3")

# ★ 상주 프로세스로 둔다. 호출마다 새로 띄우면 파이썬+OpenCV 임포트에만
#   0.27 초가 든다(실측). 한 번의 SCAN 에서 프레임 3장 x (전체 1회 + 후보
#   최대 6회) = 최대 21회를 부르므로 로봇당 5.7 초, 두 대면 10 초가 넘는다.
#   그동안 _ros2_run_blocking 이 world.step() 을 안 밟아 **시뮬 전체가 멈춘다**
#   — 다른 로봇도, use_sim_time 인 AMCL/nav_server 도 같이 선다.
#   상주시키면 한 번만 임포트하고 이후엔 경로 한 줄에 결과 한 줄이라 수 ms 다.
_QR_SERVER_SRC = (
    "import sys, cv2\n"
    "d = cv2.wechat_qrcode_WeChatQRCode()\n"
    "sys.stdout.write('READY\\n'); sys.stdout.flush()\n"
    "for line in sys.stdin:\n"
    "    p = line.strip()\n"
    "    if not p:\n"
    "        continue\n"
    "    try:\n"
    "        img = cv2.imread(p)\n"
    "        r = d.detectAndDecode(img)\n"
    "        t = [x for x in (r[0] if isinstance(r, tuple) else r) if x]\n"
    "        out = t[0] if t else ''\n"
    "    except Exception:\n"
    "        out = ''\n"
    "    sys.stdout.write(out + '\\n'); sys.stdout.flush()\n"
)

_qr_srv = None          # (pid, 쓰기 fd, 읽기 파일객체)
_qr_srv_failed = False


def _qr_server():
    """상주 디코더를 띄우고 (pid, w_fd, r_file) 을 돌려준다. 실패하면 None."""
    global _qr_srv, _qr_srv_failed
    if _qr_srv is not None or _qr_srv_failed:
        return _qr_srv
    try:
        in_r, in_w = os.pipe()          # 부모 -> 자식 (경로)
        out_r, out_w = os.pipe()        # 자식 -> 부모 (결과)
        exe = shutil.which(QR_DECODE_PYTHON) or QR_DECODE_PYTHON
        pid = os.posix_spawn(
            exe, [exe, "-c", _QR_SERVER_SRC], _clean_ros2_env(),
            file_actions=[
                (os.POSIX_SPAWN_DUP2, in_r, 0),
                (os.POSIX_SPAWN_DUP2, out_w, 1),
                (os.POSIX_SPAWN_CLOSE, in_w),
                (os.POSIX_SPAWN_CLOSE, out_r),
            ],
        )
        os.close(in_r); os.close(out_w)
        rf = os.fdopen(out_r, "r")
        if rf.readline().strip() != "READY":
            raise RuntimeError("디코더가 READY 를 안 보냈다")
        _qr_srv = (pid, in_w, rf)
        print(f"   QR 외부 디코더 상주 시작  pid={pid}  ({QR_DECODE_PYTHON})")
    except Exception as e:
        print(f"   !! 외부 QR 디코더를 못 띄웠다: {type(e).__name__}: {e}")
        print(f"      {QR_DECODE_PYTHON} 에 opencv-contrib 가 있는지 확인할 것")
        _qr_srv_failed = True
        _qr_srv = None
    return _qr_srv


def _external_qr_decode(image):
    """프레임/크롭을 임시 PNG 로 떨구고 상주 디코더에 물어본다. 실패하면 ""."""
    global _qr_srv, _qr_srv_failed
    srv = _qr_server()
    if srv is None:
        return ""
    import cv2
    pid, w_fd, rf = srv
    tmp = DUMP_DIR / "_extdecode.png"
    try:
        DUMP_DIR.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(tmp), image):
            return ""
        os.write(w_fd, (str(tmp) + "\n").encode())
        return rf.readline().strip()
    except Exception as e:
        print(f"   !! 외부 QR 디코더 통신 실패: {type(e).__name__}: {e}")
        _qr_srv, _qr_srv_failed = None, True
        return ""


set_external_decoder(_external_qr_decode)


class NavDriver:
    """nav_server(/<ns>/navigation/navigate_to) 에 좌표를 주고 기다린다.

    ★ 메서드가 전부 **제너레이터**다. world.step() 을 스스로 밟지 않고 yield
      하고, 스텝은 run_concurrent() 가 한 번만 밟는다. 그래야 두 대가 같은
      시뮬레이션 시간 위에서 동시에 움직인다 — 예전처럼 각자 world.step() 을
      돌리면 한 대가 도는 동안 다른 한 대는 코드가 아예 안 돈다.
    """

    def __init__(self, ns):
        self.ns = ns
        # 긴 이송은 Nav2(회피), 마지막 접안은 nav_server(정밀). 이유는 위 주석.
        self.nav2_action    = f"/{ns}/navigate_to_pose" if ns else "/navigate_to_pose"
        self.precise_action = (f"/{ns}/navigation/navigate_to" if ns
                               else "/navigation/navigate_to")
        # go_to_pick 이 sync_localization 결과를 여기 남기고 go_to_place 가 읽는다.
        # 기본 False — 한 번도 동기화한 적 없으면 주행하지 않는다.
        self.localized = False
        # 주행 중 놓쳐서 중단했는지 — 호출부가 원인을 구분할 수 있게 남긴다
        self.dropped_while_driving = False

    def _run_stepping(self, args, timeout_s):
        """ros2 CLI 를 돌리며 기다리는 동안 매 루프 yield 한다.

        ★ _ros2_run_blocking 을 쓰면 안 되는 이유: 그 함수는 time.sleep() 으로
          통째로 막혀서 그동안 world.step() 이 안 불린다. 그러면 /clock 이
          멈추고, use_sim_time 인 AMCL·nav_server 도 같이 멈춘다 — 하필
          initialpose 를 처리시켜야 하는 바로 그 구간에서. 반환은
          _ros2_run_blocking 과 같은 (rc 또는 None, 출력)."""
        try:
            pid, r_fd = _ros2_spawn(args, _clean_ros2_env())
        except Exception as e:
            return None, f"{type(e).__name__}: {e}"
        chunks = []
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            yield
            result = _ros2_poll(pid, r_fd, chunks)
            if result is not None:
                return result
        _ros2_kill(pid, r_fd)
        return None, "".join(c.decode(errors="replace") for c in chunks)

    def _read_amcl_pose(self):
        """지금 AMCL 이 믿는 위치를 (x, y) 로 읽는다. 못 읽으면 None.

        amcl_pose 는 TRANSIENT_LOCAL 이라 로봇이 멈춰 있어도 latch 된 최신값을
        받을 수 있다(nav_server 모듈 독스트링과 같은 이유로 QoS 를 맞춘다)."""
        rc, out = yield from self._run_stepping(
            ["ros2", "topic", "echo", "--once", "--qos-durability", "transient_local",
             "--qos-reliability", "reliable", f"/{self.ns}/amcl_pose"],
            timeout_s=10.0,
        )
        if rc != 0:
            return None
        got = {}
        for key in ("x", "y"):
            # position 이 orientation 보다 먼저 나오고 header 에는 x/y 가 없으므로
            # 첫 매치가 곧 position.x / position.y 다.
            m = re.search(rf"^\s+{key}:\s*([-+0-9.eE]+)\s*$", out, re.M)
            if m is None:
                return None
            got[key] = float(m.group(1))
        return got["x"], got["y"]

    def sync_localization(self, x, y, yaw_deg, tol_m=0.30, tries=3):
        """텔레포트로 베이스를 옮긴 직후 반드시 부른다 — 안 하면 AMCL 은
        텔레포트를 모르고 옛 위치를 계속 추정해서 nav_server 가 엉뚱한 방향으로
        출발한다. 텔레포트라 위치를 정확히 아니 공분산을 작게 준다.

        ★ 쏘고 끝내지 않고 **반영됐는지 확인한다.** 예전엔 퍼블리시만 하고
          넘어갔는데, AMCL 이 아예 안 떠 있어서(구독자 0) 메시지가 통째로
          버려졌는데도 그대로 진행했다 — 그 결과 nav_server 가 amcl_pose 를
          한 번도 못 받아 3분 타임아웃 뒤 BLOCKED 로 죽었다. 여기서 못 맞추면
          주행을 아예 시작하지 않는 게 맞다.

        반환: 동기화 확인되면 True."""
        yaw = math.radians(yaw_deg)
        pose_yaml = (
            "{header: {frame_id: 'map'}, "
            "pose: {pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
            "orientation: {z: %.6f, w: %.6f}}, "
            "covariance: [%s]}}" % (
                float(x), float(y), math.sin(yaw / 2.0), math.cos(yaw / 2.0),
                ",".join("0.01" if i in (0, 7, 35) else "0.0" for i in range(36)),
            )
        )
        for attempt in range(1, tries + 1):
            # --once 는 퍼블리셔를 만들자마자 한 번 쏘고 바로 끝난다 — DDS
            # 디스커버리(이 프로세스의 퍼블리셔 <-> AMCL 구독자 매칭)가 그
            # 찰나에 안 끝나면 메시지가 그냥 유실된다(실측: rviz 에서 위치가
            # 텔레포트 반영 안 된 채 이상하게 보임). 짧게(2초) 반복 퍼블리시
            # 해서 매칭 이후에도 최소 한 번은 확실히 들어가게 한다.
            rc, out = yield from self._run_stepping(
                ["ros2", "topic", "pub", "-r", "10", "-t", "20",
                 f"/{self.ns}/initialpose",
                 "geometry_msgs/msg/PoseWithCovarianceStamped", pose_yaml],
                timeout_s=15.0,
            )
            if rc != 0:
                print(f"   [{self.ns}] !! initialpose 퍼블리시 실패 (rc={rc}): {out.strip()[-400:]}")
            # AMCL 은 initialpose 를 받아도 그 자리에서 amcl_pose 를 내지
            # 않는다 — 다음 스캔이 들어와야 낸다. 그래서 여기서 계속 스텝을
            # 밟아 /<ns>/scan 이 실제로 흐르게 해 줘야 한다.
            for _ in range(60):
                yield

            got = yield from self._read_amcl_pose()
            if got is None:
                print(f"   [{self.ns}] !! amcl_pose 를 못 읽었다 (시도 {attempt}/{tries}) — "
                      f"AMCL 이 떠 있는지 확인: ros2 topic info /{self.ns}/amcl_pose")
                continue
            err = math.hypot(got[0] - x, got[1] - y)
            if err <= tol_m:
                print(f"   [{self.ns}] AMCL 동기화 확인  ({got[0]:+.3f}, {got[1]:+.3f})  "
                      f"오차 {err*1000:.0f} mm")
                return True
            print(f"   [{self.ns}] !! AMCL 이 아직 ({got[0]:+.3f}, {got[1]:+.3f}) 를 믿는다 — "
                  f"목표와 {err:.2f} m 차이 (시도 {attempt}/{tries})")
        print(f"   [{self.ns}] !! AMCL 위치 동기화 실패 — 이 상태로 주행하면 엉뚱한 데로 간다")
        return False

    def drive_to(self, x, y, yaw_deg, timeout_s=NAV_DRIVE_TIMEOUT_S, gripper=None,
                 hold_robot=None, hold_action=None, precise=False, gt_path=None):
        yaw = math.radians(yaw_deg)
        pose_part = (
            "pose: {header: {frame_id: 'map'}, "
            "pose: {position: {x: %.6f, y: %.6f, z: 0.0}, "
            "orientation: {z: %.6f, w: %.6f}}}" % (
                float(x), float(y), math.sin(yaw / 2.0), math.cos(yaw / 2.0),
            )
        )
        if precise:
            # cobot3_interfaces/NavigateTo — goal 은 pose 하나
            action, atype = self.precise_action, PRECISE_ACTION_TYPE
            goal_yaml = "{%s}" % pose_part
        else:
            # nav2_msgs/NavigateToPose — pose + behavior_tree
            action, atype = self.nav2_action, NAV2_ACTION_TYPE
            goal_yaml = "{%s, behavior_tree: ''}" % pose_part
        try:
            pid, r_fd = _ros2_spawn(
                ["ros2", "action", "send_goal", action, atype, goal_yaml],
                _clean_ros2_env(),
            )
        except Exception as e:
            print(f"   [{self.ns}] !! 'ros2 action send_goal' 실행 실패: {type(e).__name__}: {e}")
            return False

        chunks = []
        deadline = time.time() + timeout_s + 30.0   # 액션 서버 대기(최대 30s) 여유
        result = None
        # 주행 중엔 흡착 상태를 전혀 안 지켜봤었다 — TRANSPORT 끝에는
        # gripped=True 였는데 PLACE MOVE 시작 직후 바로 놓친 걸로 나온 실측
        # 이후 추가: 5초마다 찍어서 "주행 중 어디서 떨어졌는지" vs "PLACE
        # 전환 시점 문제인지"를 다음 실행에서 구분할 수 있게 한다.
        last_log = t_start = time.time()
        while time.time() < deadline:
            # ★ 주행하는 동안에도 반송 자세를 매 스텝 다시 건다. 안 걸면 팔이
            #   흔들려 흡착이 풀린다 — 실측: 8 m 를 달린 robot2 가 주행 도중
            #   gripped=False 가 됐다. 한 대만 돌렸을 때는 로봇이 선반에 걸려
            #   거의 안 움직여서 이 문제가 안 보였다.
            if hold_robot is not None and hold_action is not None:
                hold_robot.apply_action(hold_action)
            yield
            result = _ros2_poll(pid, r_fd, chunks)
            if result is not None:
                break
            if gripper is not None and time.time() - last_log >= 2.0:
                last_log = time.time()
                held = holding(gripper.gripped())
                if not held:
                    # ★ 놓친 채로 계속 달릴 이유가 없다. 도착해서 reattach 가
                    #   실패할 때까지 두면 '어디서' 떨어졌는지도 모른다.
                    print(f"   [{self.ns}] !! 주행 중 캐리어를 놓쳤다 "
                          f"(출발 {time.time() - t_start:.0f}s 경과) — 주행 중단")
                    _ros2_kill(pid, r_fd)
                    self.dropped_while_driving = True
                    return False
                # ★ AMCL 좌표만 찍으면 "로봇이 엉뚱한 데 갔다" 와 "AMCL 이
                #   틀렸다" 를 구분할 수 없다. USD 에서 읽은 실제 좌표를 같이
                #   찍어 둔다 — 실측으로 두 대가 목표 반대편 10 m 밖에 있는
                #   것처럼 나온 적이 있는데, 그때 어느 쪽이 거짓인지 몰랐다.
                gt = ""
                if gt_path is not None:
                    try:
                        gp, _ = get_world_pose(gt_path)
                        gt = f"  실제({gp[0]:+.2f},{gp[1]:+.2f})"
                    except Exception:
                        gt = ""
                print(f"   [{self.ns}] ...주행 중  gripped={held}  "
                      f"({time.time() - t_start:.0f}s){gt}")

        if result is None:
            print(f"   [{self.ns}] !! 주행 타임아웃({timeout_s:.0f}s) — send_goal 강제 종료")
            _ros2_kill(pid, r_fd)
            return False

        _, out = result
        ok = "Goal finished with status: SUCCEEDED" in out
        print(f"   [{self.ns}] 주행 -> {'SUCCEEDED' if ok else 'FAILED'}")
        if not ok:
            print(out.strip()[-800:])
        return ok


def reattach_if_needed(robot, solver, gripper, nav, approach_tcp,
                       target_quat, carrier_path, carrier_height):
    """팔을 배치 접근 위치로 보낸다. 텔레포트로 왔다면 흡착을 되살린다.

    ★ Nav2 로 왔으면 흡착이 그대로다 — 팔만 옮기면 끝이다.
    ★ 텔레포트로 왔으면 set_world_pose 가 흡착을 끊었으므로, 팔을 먼저 보내고
      캐리어를 그리퍼 밑에 갖다 놓은 뒤 다시 붙인다. 이 우회는 순전히
      텔레포트 때문에 필요한 것이라, 실주행에서는 타지 않는다.
    """
    if nav is None:
        # 먼 목표에 PICK 직후 관절이 나쁜 시드가 되어 IK 가 안 풀린다 — 초기화
        set_ready_pose(robot)
        for _ in range(SETTLE_STEPS):
            yield

    yield from hold_ik(robot, solver, approach_tcp, target_quat, MAX_STEPS, report=True)

    if nav is not None:
        if holding(gripper.gripped()):
            return True
        print("   !! 주행으로 왔는데 흡착이 풀려 있다 — 이송 중 떨어뜨린 것이다")
        return False

    tcp_now = get_tcp_pose(robot)
    set_prim_pose(carrier_path, [tcp_now[0], tcp_now[1], tcp_now[2] - carrier_height])
    gripper.close()
    for _ in range(GRIP_WAIT):
        yield from hold_ik(robot, solver, approach_tcp, target_quat, 1)
        if holding(gripper.gripped()):
            yield from hold_ik(robot, solver, approach_tcp, target_quat, BASE_MOVE_SETTLE_STEPS)
            print("   재흡착  -> 붙었다")
            return True
    print(f"   재흡착  -> 안 붙었다   tcp={vec(get_tcp_pose(robot), 4)}  "
          f"캐리어={vec(get_world_pose(carrier_path)[0], 4)}  status={gripper.status()}")
    return False


# ══════════════════════════════════════════════════════════════
#  이동 — **빈손이면 텔레포트, 들고 있으면 Nav2**
#
#  왜 나누는가 (둘 다 실측 근거가 있다):
#
#   · pick 자리는 정확해야 한다. 관측 자세(taught_poses)가 "로봇이 매거진
#     앞 0.55 m 에 yaw 0 으로 서 있다" 를 전제로 한 관절값이라, 베이스가
#     조금만 어긋나도 QR 이 화각 밖으로 나간다. Nav2 는 SUCCEEDED 를
#     돌려주고도 y 로 0.76 m 짧고 yaw 가 뒤집힌 채 섰다(실측: 카메라 시선이
#     [-0.74,-0.67] 로 선반 반대편을 봤다). 도착 허용오차가 주행용이지
#     파지용이 아니다.
#
#   · 반대로 캐리어를 들고 있을 때 텔레포트하면 **흡착이 즉시 풀린다**
#     (set_world_pose 는 거리와 무관하게 끊는다 — 실측). 그래서 이송
#     구간은 실제로 주행해야 하고, 그러면 재흡착 우회가 필요 없어진다.
#
#  pick 자리로 갈 때는 언제나 빈손이라 텔레포트로 잃을 흡착이 없다.
# ══════════════════════════════════════════════════════════════
def go_to_pick(ctx, x, y, yaw_rad, label):
    """pick 자리로 — 텔레포트로 정확히 세운다. (빈손일 때만 부른다)"""
    yaw_deg = math.degrees(yaw_rad)
    print(f"   [{ctx.name}] TELEPORT → {label}  ({x:+.3f}, {y:+.3f}, {yaw_deg:+.1f}°)  [정밀 정지]")
    yield from ctx.teleporter.go([x, y, CARTER_Z], yaw_rad, SETTLE_STEPS)
    # ★ AMCL 은 텔레포트를 모른다. 알려주지 않으면 다음 Nav2 이송이 옛 위치를
    #   기준으로 경로를 짜서 엉뚱한 데로 간다.
    #   실패해도 여기서 멈추지 않는다 — pick 자체는 텔레포트로 이미 정확히 서
    #   있어서 AMCL 과 무관하게 할 수 있다. 막아야 하는 건 그 다음 주행이라,
    #   결과만 남겨 두고 go_to_place 가 판단하게 한다.
    if ctx.nav is not None:
        ctx.nav.localized = yield from ctx.nav.sync_localization(x, y, yaw_deg)
    return True


def go_to_place(ctx, x, y, yaw_rad, label, via=None, loose=False, precise=False):
    """place 자리로 — 캐리어를 들고 가므로 실제로 주행한다.

    via: 목표 전에 거쳐 갈 (x, y) 목록. 회피가 없는 nav_server 를 쓰기 때문에
         장애물을 피하는 책임이 호출부에 있다 — 아래 주석 참고."""
    yaw_deg = math.degrees(yaw_rad)
    nav, gripper, robot = ctx.nav, ctx.gripper, ctx.robot
    via = ctx.via if via is None else via
    if nav is None:
        print(f"   [{ctx.name}] TELEPORT → {label}  ({x:+.3f}, {y:+.3f}, {yaw_deg:+.1f}°)  [NAV=0]")
        yield from ctx.teleporter.go([x, y, CARTER_Z], yaw_rad)
        return True
    # 팔은 여기서 접지 않는다 — 미션이 **pick 직후** set_carry_pose 로 접는다.
    # 대기 중에도 접혀 있어야 하고(벨트 자리를 기다리는 동안), 두 번 접으면
    # 그만큼 스텝만 버린다.
    if not nav.localized:
        # AMCL 이 로봇 위치를 모르는 채로 목표를 주면, nav_server 는 amcl_pose 를
        # 기다리다 3분 타임아웃 뒤 BLOCKED 로 죽는다(실측). 그 3분을 기다릴
        # 이유가 없고, 더 나쁘게는 "옛 위치를 믿는" 경우엔 실제로 엉뚱한
        # 방향으로 밀고 나간다 — 캐리어를 든 채로.
        print(f"   [{ctx.name}] !! {label} 주행 취소 — AMCL 이 로봇 위치를 모른다")
        return False
    # ★ 경유점은 이제 기본적으로 비어 있다. Nav2 가 코스트맵에서 풋프린트를
    #   보고 알아서 돌아가기 때문이다. nav_server 를 쓰던 때는 회피가 없어서
    #   선반을 관통하는 직선을 피하려고 여기서 손으로 경유점을 끼워 넣어야
    #   했다 — 그 방식은 결국 실패했다. 컨트롤러가 조향 오차 45도를 넘기면
    #   아무 데서나 제자리 회전을 시작하는데, 그 자리를 경로가 정할 수 없다.
    # 주행 내내 유지할 반송 자세. robot 이 없으면(NAV=0 경로) 유지하지 않는다.
    carry_action = carry_pose_action(robot)[2] if robot is not None else None
    pts = [(float(vx), float(vy)) for vx, vy in via] + [(float(x), float(y))]
    for i, (px, py) in enumerate(pts):
        if i + 1 < len(pts):
            # 경유점의 yaw 는 **다음 지점을 향하는 방향**으로 준다. 최종 yaw 로
            # 맞춰 두면 nav_server 의 _fine_align 이 구간 끝에서 최종 방향으로
            # 돌려놓고, 다음 구간 출발 때 도로 돌아야 한다.
            nx, ny = pts[i + 1]
            leg_yaw = math.degrees(math.atan2(ny - py, nx - px))
            leg_label = f"{label} 경유{i + 1}"
        else:
            leg_yaw = yaw_deg
            leg_label = label
        print(f"   [{ctx.name}] NAV → {leg_label}  ({px:+.3f}, {py:+.3f}, {leg_yaw:+.1f}°)")
        if not (yield from nav.drive_to(px, py, leg_yaw, gripper=gripper,
                                        hold_robot=robot, hold_action=carry_action,
                                        precise=precise, gt_path=ctx.base_link)):
            # ★ 경유점은 **통과점이지 도킹 자세가 아니다.** nav_server 의
            #   도착 판정(8 cm)은 마지막 목표에나 필요한 값이고, 경유점에서는
            #   그 정밀도를 못 맞춰 타임아웃이 난다 — 실측: robot2 가 후진으로
            #   탈출 지점까지 갔다가 0.33 m 를 남기고 180 초를 소진했다.
            #   충분히 가까우면 다음 구간으로 넘어간다. 마지막 목표는 예외 없다.
            # loose=True 면 마지막 목표도 느슨하게 받는다 — 대기/후퇴 자리는
            # '주차' 지점이지 도킹 자세가 아니다. 벨트 접안만 엄격하다.
            if i + 1 < len(pts) or loose:
                here = yield from nav._read_amcl_pose()
                if here is not None:
                    gap = math.hypot(here[0] - px, here[1] - py)
                    if gap <= VIA_REACHED_TOL_M:
                        print(f"   [{ctx.name}] {leg_label} 못 붙었지만 {gap:.2f} m 까지 왔다 "
                              f"— 통과점이므로 다음 구간으로 간다")
                        continue
                    try:
                        gp, _ = get_world_pose(ctx.base_link)
                        gts = f"  실제({gp[0]:+.2f},{gp[1]:+.2f})"
                    except Exception:
                        gts = ""
                    print(f"   [{ctx.name}] !! {leg_label} 에서 {gap:.2f} m 떨어져 멈췄다  "
                          f"AMCL({here[0]:+.2f},{here[1]:+.2f}){gts}")
            return False
    for _ in range(BASE_MOVE_SETTLE_STEPS):
        yield
    return True


# ══════════════════════════════════════════════════════════════
#  두 대를 한 스텝 루프에 얹는다
# ══════════════════════════════════════════════════════════════
def run_concurrent(world, missions):
    """미션 제너레이터들을 한 world.step() 루프 위에서 번갈아 진행시킨다.

    ★ 왜 제너레이터인가: 원래 이 스크립트의 동작 블록은 저마다 안에서
      world.step() 을 돌리는 **블로킹** 함수였다. 그 구조로는 두 대를 동시에
      못 돌린다 — 한 대가 주행 루프를 도는 동안 다른 한 대의 코드는 아예
      실행되지 않기 때문이다. 그래서 world.step() 자리를 전부 yield 로 바꾸고,
      스텝은 여기서 **한 번만** 밟는다. 두 로봇이 같은 시뮬레이션 시간 위에
      올라간다.

    ★ 스텝은 미션 수와 무관하게 루프당 한 번이다. 미션마다 밟으면 한 스텝에
      물리가 두 번 진행돼 시간이 두 배로 흐른다.

    반환: {미션 이름: 반환값}
    """
    alive = list(missions)
    results = {}
    while alive:
        still = []
        for m in alive:
            try:
                next(m.gen)
                still.append(m)
            except StopIteration as stop:
                results[m.name] = stop.value
                print(f"   ── {m.name} 미션 종료 (결과 {stop.value})")
            except Exception as e:      # 한 대가 죽어도 나머지는 계속 간다
                results[m.name] = f"EXC {type(e).__name__}: {e}"
                print(f"   !! {m.name} 미션 예외 {type(e).__name__}: {e}")
                import traceback
                traceback.print_exc()
        alive = still
        world.step(render=not HEADLESS)
    return results


class Mission:
    def __init__(self, name, gen):
        self.name = name
        self.gen = gen


class BeltStation:
    """포장 벨트 접안 자리는 **하나**다 — 한 번에 한 대만 간다.

    두 대가 같은 자리로 동시에 주행하면 서로 밀어내고, nav_server 는 회피가
    없어서 상대를 장애물로도 못 본다. 그래서 자리를 자물쇠로 쓴다.

    ★ 순서 규칙: **벨트까지 가까운 쪽 먼저.** pick 을 끝낸 시점에 각자
      현재 위치에서 접안점까지의 거리를 재서 등록하고, 자리가 비면 등록된
      대기자 중 거리가 가장 짧은 쪽이 가져간다. 총 이동 시간이 줄어든다.
      거리가 같으면 이름 순으로 갈라 교착을 막는다.

    ★ 기다리는 쪽은 자기 자리에서 팔을 접은 채 서 있는다 — 통로로 나오지
      않으므로 앞 로봇의 경로를 막지 않는다.
    """

    def __init__(self, x, y):
        self.x, self.y = float(x), float(y)
        self._owner = None
        self._waiting = {}      # 이름 -> pick 완료 시점의 벨트까지 거리

    def distance_from(self, ctx):
        pos, _ = get_world_pose(ctx.base_link)
        return float(math.hypot(self.x - pos[0], self.y - pos[1]))

    def request(self, name, dist):
        self._waiting[name] = dist

    def try_acquire(self, name):
        if self._owner is not None:
            return self._owner == name
        if not self._waiting:
            return False
        # 거리 우선, 동률이면 이름 순 — 둘 다 못 가져가는 상태를 만들지 않는다
        best = min(self._waiting.items(), key=lambda kv: (kv[1], kv[0]))[0]
        if best != name:
            return False
        self._owner = name
        self._waiting.pop(name, None)
        return True

    def release(self, name):
        if self._owner == name:
            self._owner = None

    @property
    def owner(self):
        return self._owner


class BaseTeleporter:
    """robot(SingleManipulator) 를 그대로 재사용한다 — 같은 prim 에 물리 뷰를
    두 개 만들면 서로 충돌해 IK 가 널뛰고 apply_action 이 죽는다(12_place_test.py)."""

    def __init__(self, robot):
        self._robot = robot

    def go(self, pos_xyz, yaw_rad, settle=BASE_MOVE_SETTLE_STEPS):
        quat = quat_from_axis([0, 0, 1], math.degrees(yaw_rad))
        self._robot.set_world_pose(position=np.array(pos_xyz, dtype=float), orientation=quat)
        try:
            self._robot.set_linear_velocity(np.zeros(3))
            self._robot.set_angular_velocity(np.zeros(3))
        except Exception:
            pass
        for _ in range(settle):
            yield


# ══════════════════════════════════════════════════════════════
#  PICK / PLACE FSM — 12_place_test.py 와 같은 구조.
#  다른 점은 목표 xy/z 를 **QR 결과**로 받는다는 것뿐이다.
# ══════════════════════════════════════════════════════════════
class PickFSM:
    NAMES = ["APPROACH", "DESCEND", "GRIP", "HOLD", "LIFT", "DONE"]
    DONE_STATE = 5

    def __init__(self, robot, gripper, grasp_world, carrier_path, tag=""):
        # tag: 두 대를 동시에 돌리면 로그가 뒤섞인다 — 어느 로봇인지 앞에 붙인다.
        self.tag = f"[{tag}] " if tag else ""
        self._robot = robot
        self._gripper = gripper
        self._carrier = carrier_path
        self.grasp_world = np.asarray(grasp_world, dtype=float)
        self.reset()

    def reset(self):
        self.attempt = 0
        self.z_at_grip = None
        self.grip_ok = False
        self.done = False
        self.fail_code = None
        self.lift_rise_m = 0.0
        self.tilt_at_lift_deg = 0.0
        self.state = 0
        self.step = 0
        self.start = None
        self.gripper = "open"
        self.ik_fail_streak = 0
        self._rebuild()

    def _rebuild(self):
        cx, cy, top = self.grasp_world
        self.waypoints = [
            np.array([cx, cy, top + APPROACH_HEIGHT_OFFSET]),
            np.array([cx, cy, top + GRIP_GAPS[self.attempt]]),
            np.array([cx, cy, top + GRIP_GAPS[self.attempt]]),
            np.array([cx, cy, top + GRIP_GAPS[self.attempt]]),
            np.array([cx, cy, top + LIFT_HEIGHT_OFFSET]),
        ]

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            # ★ 첫 프레임에 목표를 그대로 주면 현재 TCP 와의 거리만큼 힘이 튄다.
            return get_tcp_pose(self._robot)
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    def note_ik(self, solved):
        self.ik_fail_streak = 0 if solved else self.ik_fail_streak + 1
        if self.ik_fail_streak >= MAX_IK_FAIL_STEPS:
            self.done, self.fail_code = True, FAIL_UNREACHABLE

    def advance(self):
        if self.done:
            return
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            if self.state == 2:
                self.z_at_grip = measure_prim(self._carrier)[1]
                self._gripper.close()
                self.gripper = "close"
                self.n_steps, dist = GRIP_WAIT, 0.0
            elif self.state == 3:
                self.n_steps, dist = HOLD_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)
            print(f"   {self.tag}[{self.state}] {self.NAMES[self.state]:9s} goal {vec(self.goal)} "
                  f"{dist:.4f} m  {self.n_steps} steps  gripper {self.gripper}")

        self.step += 1
        self._watch_drop()
        if self.step < self.n_steps:
            return
        if self.state == 2 and not self._check_grip():
            return
        if self.state == 4:
            self._judge()
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            self.fail_code = self.fail_code if self.fail_code is not None else FAIL_OK
            print(f"   {self.tag}[{self.DONE_STATE}] PICK DONE  fail_code={self.fail_code}")

    def _watch_drop(self):
        if self.state not in (3, 4) or not self.grip_ok or self.done:
            return
        if self.step % 10 or holding(self._gripper.gripped()):
            return
        print(f"   {self.tag}!! 놓쳤다  {self.NAMES[self.state]}  {self.step}/{self.n_steps}")
        if self.state != 3:
            # LIFT 중에 놓쳤으면 캐리어가 떨어져 자리가 바뀐다. 옛 파지점으로
            # 다시 내려가 봐야 헛손질이라 여기서 끝낸다.
            self.done, self.fail_code = True, FAIL_SLIP
            return
        # ★ HOLD 중 놓침은 실패로 끝내지 않고 **간격을 한 칸 더 내려 재시도**한다.
        #   두 로봇 모두 "시도 1/4 +5 mm 안 붙음 -> 시도 2/4 +2 mm 붙음" 으로
        #   간신히 붙은 뒤 HOLD 초반(10/120)에 풀렸다 — 파지가 불가능한 게
        #   아니라 여유가 빠듯한 것이다. HOLD 시점엔 아직 들어올리기 전이라
        #   캐리어가 원래 자리에 그대로 있으므로 같은 파지점으로 다시 가면 된다.
        self.grip_ok = False
        self._retry_or_fail()

    def _check_grip(self):
        ok = holding(self._gripper.gripped())
        print(f"   {self.tag}── 시도 {self.attempt+1}/{len(GRIP_GAPS)}  간격 "
              f"{GRIP_GAPS[self.attempt]*1000:+.0f} mm  -> {'붙었다' if ok else '안 붙었다'}")
        if ok:
            self.grip_ok = True
            return True
        return self._retry_or_fail()

    def _retry_or_fail(self):
        """간격 사다리(GRIP_GAPS)를 한 칸 내려 DESCEND 부터 다시. 소진되면 실패."""
        self.attempt += 1
        if self.attempt >= len(GRIP_GAPS):
            self.done, self.fail_code = True, FAIL_SLIP
            print(f"   {self.tag}흡착 실패 — GRIP_GAPS 소진")
            return False
        self._gripper.open()
        self.gripper = "open"
        self._rebuild()
        self.state, self.step, self.start = 1, 0, None
        return False

    def _judge(self):
        now = measure_prim(self._carrier)[1]
        self.lift_rise_m = now - self.z_at_grip
        _, q = get_world_pose(self._carrier)
        self.tilt_at_lift_deg = tilt_deg_from_quat(q)
        ok = self.lift_rise_m >= LIFT_OK_MIN_M and self.tilt_at_lift_deg <= TILT_MAX_DEG
        print(f"   {self.tag}판정  rise {self.lift_rise_m*1000:+.1f} mm  tilt "
              f"{self.tilt_at_lift_deg:.2f} deg  -> {'성공' if ok else '실패'}")
        if not ok:
            self.fail_code = FAIL_SLIP


class PlaceFSM:
    """벨트 **윗면** 에 내려놓는다. 12_place_test.py 는 바닥(z=0)이었다."""

    NAMES = ["MOVE", "LOWER", "RELEASE", "RETREAT", "DONE"]
    DONE_STATE = 4

    def __init__(self, robot, gripper, target_xy, surface_z, carrier_height, tag=""):
        self.tag = f"[{tag}] " if tag else ""
        self._robot = robot
        self._gripper = gripper
        gx, gy = target_xy
        # 흡착면이 가야 할 z = 놓을 면 + 캐리어 높이 (원점이 바닥면이므로)
        ref_z = float(surface_z) + float(carrier_height)
        self.waypoints = [
            np.array([gx, gy, ref_z + APPROACH_HEIGHT_OFFSET]),
            np.array([gx, gy, ref_z + PLACE_DROP]),
            np.array([gx, gy, ref_z + PLACE_DROP]),
            np.array([gx, gy, ref_z + APPROACH_HEIGHT_OFFSET]),
        ]
        self.done = False
        self.fail_code = None
        self.state = 0
        self.step = 0
        self.start = None
        self.gripper = "close"
        self.ik_fail_streak = 0

    def current_target(self):
        if self.done:
            return self.waypoints[-1]
        if self.start is None:
            return get_tcp_pose(self._robot)
        return self.start + ease(self.step / float(self.n_steps)) * (self.goal - self.start)

    note_ik = PickFSM.note_ik

    def advance(self):
        if self.done:
            return
        if self.start is None:
            self.start = get_tcp_pose(self._robot)
            self.goal = self.waypoints[self.state]
            if self.state == 2:
                self._gripper.open()
                self.gripper = "open"
                self.n_steps, dist = RELEASE_WAIT, 0.0
            else:
                self.n_steps, dist = steps_for(self.start, self.goal)
            print(f"   {self.tag}[{self.state}] {self.NAMES[self.state]:9s} goal {vec(self.goal)} "
                  f"{dist:.4f} m  {self.n_steps} steps  gripper {self.gripper}")
        self.step += 1
        if self.state in (0, 1) and self.step % 10 == 0 and not holding(self._gripper.gripped()):
            self.done, self.fail_code = True, FAIL_SLIP
            print(f"   {self.tag}!! 이송 중 놓쳤다  {self.NAMES[self.state]}")
            return
        if self.step < self.n_steps:
            return
        self.state += 1
        self.step = 0
        self.start = None
        if self.state >= self.DONE_STATE:
            self.done = True
            self.fail_code = self.fail_code if self.fail_code is not None else FAIL_OK
            print(f"   {self.tag}[{self.DONE_STATE}] PLACE DONE  fail_code={self.fail_code}")


# ══════════════════════════════════════════════════════════════
#  동작 블록
# ══════════════════════════════════════════════════════════════
def run_fsm(robot, solver, fsm, target_quat, label):
    step = 0
    while not fsm.done:
        yield
        flange = tcp_to_flange(fsm.current_target(), target_quat)
        action, solved = solver.compute_inverse_kinematics(
            target_position=flange, target_orientation=target_quat)
        # ★ solved 가 참인데 action 이 None 으로 오는 경우가 실제로 있다
        #   (실측: 두 대를 동시에 돌릴 때 robot2 의 PICK 중
        #    AttributeError: 'NoneType' object has no attribute 'joint_positions'
        #    — apply_action 안에서 죽었다). solved 만 믿고 넘기면 미션이
        #   통째로 예외로 끝나므로, 그 스텝은 IK 실패로 취급하고 넘어간다.
        if solved and action is None:
            solved = False
        if solved:
            robot.apply_action(action)
        fsm.note_ik(solved)
        fsm.advance()
        if step % LOG_INTERVAL == 0:
            print(f"   {label} {fsm.NAMES[min(fsm.state, fsm.DONE_STATE)]:9s} solved={solved}")
        step += 1
    return fsm.fail_code


def hold_ik(robot, solver, tcp, target_quat, n_steps, report=False):
    """목표를 **매 스텝 다시** 건다. 한 번만 걸면 팔이 다른 자세로 흘러내린다.

    report=True 면 끝에 '실제로 도달했는지' 를 찍는다 — 재흡착이 실패할 때
    IK 가 안 풀린 것인지, 풀렸는데 팔이 못 따라간 것인지 구분해야 한다.
    """
    flange = tcp_to_flange(tcp, target_quat)
    n_solved = 0
    for _ in range(n_steps):
        action, solved = solver.compute_inverse_kinematics(
            target_position=flange, target_orientation=target_quat)
        if solved and action is None:      # run_fsm 과 같은 이유
            solved = False
        n_solved += bool(solved)
        if solved:
            robot.apply_action(action)
        yield
    if report:
        now = get_tcp_pose(robot)
        print(f"   hold_ik  목표 {vec(tcp, 4)}  도달 {vec(now, 4)}  "
              f"오차 {np.linalg.norm(now - tcp)*1000:.1f} mm  IK 성공 {n_solved}/{n_steps}")


def scan_with_taught_pose(ctx, spec, joints_deg, label):
    """관측 자세로 팔을 세우고 QR 을 읽는다.

    ★ 관절값은 taught_poses.yaml 의 것을 그대로 쓴다. 옛 배치에서 잡은 자세지만,
      **로봇↔캐리어 상대 위치가 같으면 관절도 같다.** shelves.yaml 의 standoff
      0.55 m 가 그 전제이고, 그 값은 이 티칭 자세에서 역산한 것이다.
    """
    section(f"SCAN — {label}")
    set_arm_stiffness(DRIVE_STIFFNESS_SCAN, ctx.arm)  # ★ 1e8 이면 잔진동으로 디코딩이 깨진다
    set_joints_deg(ctx.robot, joints_deg)
    for _ in range(QR_SETTLE_STEPS):
        yield

    R_opt, p_opt = ctx.vision.optical_pose()
    fwd = R_opt @ np.array([0.0, 0.0, 1.0])     # 광학 +Z = 시선 방향
    print(f"   카메라 위치 {vec(p_opt, 4)}  시선 {vec(fwd, 3)}")

    agg = yield from ctx.vision.scan(spec, dump_tag=label.replace(" ", "_"))
    print(f"   집계  ok={agg.ok}  decoded={agg.decoded!r}  "
          f"{agg.n_used}/{agg.n_total} 프레임  reason={agg.reason}")
    if agg.ok:
        print(f"   QR 중심(월드) {vec(agg.center_world, 4)}  yaw {math.degrees(agg.yaw_rad):+.2f}°  "
              f"pos_std {vec(agg.pos_std_mm, 2)} mm  yaw_std {agg.yaw_std_deg:.2f}°")
    set_arm_stiffness(DRIVE_STIFFNESS_PICK, ctx.arm)
    return agg


def wait_for_stack(timeout_s=STACK_SPAWN_TIMEOUT_S):
    """packaging_flow.py 가 스폰한 스택이 멈출 때까지 기다린 뒤 (경로, 위치) 를 준다.

    ★ 좌표를 박지 않는다. packaging_flow.py 의 SHELF_X(3.25)/SHELF_Z(0.64)는
      실제 ShelfDeck(x 3.694~4.044, 윗면 0.54)과 어긋나 있어 스택이 허공에서
      떨어질 수 있다. 그래서 '어디에 놓이는지' 를 가정하지 않고 찾아간다.
    """
    section("WAIT — 스택 스폰")
    root = stage().GetPrimAtPath(SPAWNED_STACKS_PATH)
    if not root.IsValid():
        raise RuntimeError(f"{SPAWNED_STACKS_PATH} 가 없다 — 새 레이아웃이 맞는지 확인할 것")

    t0 = time.time()
    found = None
    while time.time() - t0 < timeout_s:
        yield
        kids = [c for c in root.GetChildren() if c.IsActive()]
        if kids:
            found = str(kids[0].GetPath())
            break
    if found is None:
        raise RuntimeError(
            f"{timeout_s:.0f}s 안에 스택이 스폰되지 않았다. 확인할 것:\n"
            "      · packaging_flow.py 가 돌고 있는가 (Script Editor 에서 한 번 실행해야 한다)\n"
            "      · 매거진이 포장 트리거(7.45, 4.60, 0.80 ±0.40/0.50/0.60)에 들어갔는가\n"
            "      · 벨트의 surfaceVelocity 가 실제로 물체를 옮기는가")

    print(f"   스폰됨  {found}")
    payload = stage().GetPrimAtPath(found).GetAttribute("flow:stackPayload")
    payload_name = Path(str(payload.Get())).stem if payload and payload.Get() else "?"
    print(f"   payload {payload_name}")

    # 움직임이 멎을 때까지 기다린다 (벨트 이송 + 낙하)
    last = None
    stable = 0
    while stable < STACK_SETTLE_FRAMES and time.time() - t0 < timeout_s * 2:
        yield
        pos, _ = get_world_pose(found)
        stable = stable + 1 if (last is not None and np.linalg.norm(pos - last) < STACK_SETTLE_TOL_M) else 0
        last = pos
    print(f"   멈춘 자리 {vec(last, 4)}")
    return found, last, payload_name


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def magazine_mission(world, ctx, station, shelf, scan_joints, target_quat):
    """한 대의 매거진 미션: 선반 → QR → pick → 팔 접기 → 벨트 자리 대기 →
    이송 → place → 자리 반납.

    ★ 벨트 접안 자리는 하나뿐이라, pick 을 끝낸 뒤 **자리를 얻을 때까지 제자리
      에서 기다린다**. 기다리는 동안 팔은 접힌 상태고 베이스는 선반 앞에 서
      있으므로, 먼저 출발한 로봇의 통로를 막지 않는다.
    """
    tag = ctx.name

    # pick 자세는 매거진 실제 위치에서 낸다 — shelves.yaml 의 theta 로 어느
    # 쪽에 설지를 정한다(theta=0 이면 매거진보다 y 가 작은 쪽, π 면 큰 쪽).
    mag_xy, mag_top, mag_height = measure_prim(ctx.magazine)
    theta = float(shelf["waypoint_start"]["theta"])
    px = float(mag_xy[0])
    py = float(mag_xy[1]) - SHELF_STANDOFF_M * math.cos(theta)
    print(f"   [{tag}] 매거진 {vec([mag_xy[0], mag_xy[1], mag_top], 4)}  "
          f"→ 서는 자리 ({px:+.3f}, {py:+.3f}, {math.degrees(theta):+.1f}°)")

    yield from go_to_pick(ctx, px, py, theta, ctx.shelf_id)
    sync_ik_base_pose(ctx)
    yield from ctx.vision.warm()

    mag_spec = MAGAZINE_SPECS["1"]     # magazine_1_orange → QR 이 "1" 로 디코드된다
    agg = yield from scan_with_taught_pose(ctx, mag_spec, scan_joints, f"매거진 {tag}")
    if not agg.ok:
        print(f"   [{tag}] QR 인식 실패 — 이 로봇의 미션을 중단한다")
        return "QR_FAIL"

    grasp_world, _ = grasp_from_qr(agg, mag_spec)
    err_mm = np.linalg.norm(grasp_world - np.array([mag_xy[0], mag_xy[1], mag_top])) * 1000.0
    print(f"   [{tag}] QR 파지점 {vec(grasp_world, 4)}   "
          f"GT {vec([mag_xy[0], mag_xy[1], mag_top], 4)}   차이 {err_mm:.1f} mm")

    section(f"PICK — 매거진 {tag}")
    sync_ik_base_pose(ctx)
    fsm = PickFSM(ctx.robot, ctx.gripper, grasp_world, ctx.magazine, tag=tag)
    if (yield from run_fsm(ctx.robot, ctx.solver, fsm, target_quat, f"PICK {tag}")) != FAIL_OK:
        print(f"   [{tag}] 매거진 파지 실패 — 중단")
        return "PICK_FAIL"

    # ★ 사용자 요청: pick 하고 팔을 READY(0,0,90,0,90,0)로 되돌린다.
    #   실측 근거는 set_carry_pose 독스트링 — 뻗은 팔로 출발하면 선반에 걸린다.
    if not (yield from set_carry_pose(ctx.robot, ctx.gripper)):
        return "DROP_ON_FOLD"

    # ── ① 벨트 근처 대기 지점까지는 자리가 없어도 미리 간다 ─────────
    #   선반 앞에서 기다리면, 자리가 비고 나서야 8 m 를 달린다. 대기 지점은
    #   벨트에서 1.7 m 라 자리가 비는 즉시 짧은 구간만 남는다.
    #   로봇마다 다른 자리를 준다 — 같은 곳에서 기다리면 둘이 부딪힌다.
    if ctx.staging is not None:
        sx, sy, syaw = ctx.staging
        section(f"STAGING — 벨트 앞 대기 지점 {tag}")
        if not (yield from go_to_place(ctx, sx, sy, syaw, "대기 지점",
                                       via=ctx.staging_via, loose=True)):
            return "STAGING_FAIL"

    # ── ② 벨트 자리 중재 ───────────────────────────────────────
    dist = station.distance_from(ctx)
    station.request(tag, dist)
    print(f"   [{tag}] 벨트 자리 요청  현재 거리 {dist:.2f} m")
    waited = 0
    _, _, carry_action = carry_pose_action(ctx.robot)
    while not station.try_acquire(tag):
        # ★ 기다리는 동안에도 반송 자세를 계속 건다. 안 걸면 팔이 흘러내려
        #   서 있기만 하는데도 매거진을 놓친다(실측).
        ctx.robot.apply_action(carry_action)
        if waited % 30 == 0 and not holding(ctx.gripper.gripped()):
            # 빈손으로 주행해 봐야 벨트에 도착해서야 실패한다 — 여기서 끝낸다.
            print(f"   [{tag}] !! 대기 중 캐리어를 놓쳤다 — 미션 중단")
            station.request(tag, float("inf"))   # 대기열에서 사실상 빼 둔다
            return "DROP_WHILE_WAITING"
        if waited % 120 == 0:
            print(f"   [{tag}] ...벨트 자리 대기 (점유: {station.owner})  "
                  f"gripped={holding(ctx.gripper.gripped())}")
        waited += 1
        yield
    print(f"   [{tag}] 벨트 자리 확보 (대기 {waited} 스텝)")

    try:
        section(f"TRANSPORT — 포장 벨트 {tag}")
        # ① Nav2 로 공용 접근점 P 까지 (회피가 필요한 구간)
        px, py = BELT_APPROACH_P
        if not (yield from go_to_place(ctx, px, py, 0.0, "벨트 접근점", loose=True)):
            if ctx.nav is not None and ctx.nav.dropped_while_driving:
                return "DROP_WHILE_DRIVING"
            return "DRIVE_FAIL"
        # ② 접안은 **정밀 주행기**(nav_server)로. Nav2 의 도착 허용오차는
        #    xy 0.25 m / yaw 14도라 팔이 벨트에 못 닿는다 — 실측: 0.30 m,
        #    25.5도 벗어나 섰더니 배치점까지 0.70 m 가 되어 IK 가 600 번
        #    전부 실패했다. 여기는 여유 0.90 m 의 열린 직선이고 접근 방향이
        #    최종 yaw 와 같아서, 직선으로 미는 nav_server 가 잘 맞는다.
        if not (yield from go_to_place(ctx, BELT_BASE_X, BELT_Y_PACKAGING, 0.0,
                                       "포장 벨트(정밀)", precise=True)):
            if ctx.nav is not None and ctx.nav.dropped_while_driving:
                return "DROP_WHILE_DRIVING"
            return "DRIVE_FAIL"
        sync_ik_base_pose(ctx)
        print(f"   [{tag}] 주행 후 파지 상태  gripped={holding(ctx.gripper.gripped())}")

        approach = np.array([BELT_PLACE_X, BELT_Y_PACKAGING,
                             BELT_TOP_Z + mag_height + APPROACH_HEIGHT_OFFSET])
        if not (yield from reattach_if_needed(ctx.robot, ctx.solver, ctx.gripper, ctx.nav,
                                              approach, target_quat, ctx.magazine, mag_height)):
            return "REATTACH_FAIL"

        section(f"PLACE — 포장 벨트 {tag}")
        sync_ik_base_pose(ctx)
        place = PlaceFSM(ctx.robot, ctx.gripper, (BELT_PLACE_X, BELT_Y_PACKAGING),
                         BELT_TOP_Z, mag_height, tag=tag)
        code = yield from run_fsm(ctx.robot, ctx.solver, place, target_quat, f"PLACE {tag}")
        if code != FAIL_OK:
            return f"PLACE_FAIL({code})"

        # ── ③ 벨트에서 **물러난 뒤에** 자리를 반납한다 ───────────────
        #   ★ 놓기만 하고 그 자리에 서 있으면, 다음 로봇이 바로 그 좌표로
        #     주행해 들이받는다 — nav_server 는 회피를 안 하고 상대 로봇을
        #     장애물로도 못 본다. 반납은 실제로 비켜 준 뒤에 해야 맞다.
        if ctx.staging is not None:
            sx, sy, syaw = ctx.staging
            yield from set_carry_pose(ctx.robot)      # 빈손이지만 팔은 접고 간다
            # 벨트에서 P 는 등 뒤라 정밀 주행기가 회전 없이 후진해서 뺀다.
            qx, qy = BELT_APPROACH_P
            if not (yield from go_to_place(ctx, qx, qy, 0.0, "접근점(후퇴)",
                                           loose=True, precise=True)):
                print(f"   [{tag}] !! 접근점까지 후퇴 실패 — 벨트 앞에 남는다")
            elif not (yield from go_to_place(ctx, sx, sy, syaw, "대기 지점(후퇴)",
                                             loose=True)):
                print(f"   [{tag}] !! 대기 지점까지 후퇴 실패")
    finally:
        # ★ 실패해도 반드시 반납한다 — 안 그러면 상대가 영원히 기다린다.
        station.release(tag)
        print(f"   [{tag}] 벨트 자리 반납")

    set_ready_pose(ctx.robot)
    return f"OK(QR {err_mm:.1f} mm)"


def stack_mission(world, ctx, scan_joints, target_quat):
    """한 대만 돌릴 때의 후속 구간: 포장 라인이 내보낸 스택을 검사 벨트로.

    두 대로 돌릴 때는 쓰지 않는다 — 스택이 하나뿐이라 두 대가 같은 것을
    집으려 든다. 어느 쪽이 가져갈지는 아직 정하지 않았다.
    """
    tag = ctx.name
    set_ready_pose(ctx.robot)
    stack_path, stack_pos, payload_name = yield from wait_for_stack()

    key = "F3-STKB" if "STKB" in payload_name else "F3-STKO"
    stack_spec = STACK_SPECS[key]
    print(f"   스택 사양   {stack_spec.name}  라벨 {stack_spec.qr_side_m*1000:.0f} mm  "
          f"data_side {stack_spec.data_side_m*1000:.2f} mm")

    # 스택의 qr_label_ny 는 -Y 면이다. 매거진 때와 같은 상대 위치(0.55 m 앞)에 선다.
    yield from go_to_pick(ctx, stack_pos[0], stack_pos[1] - SHELF_STANDOFF_M, 0.0, "스택")
    sync_ik_base_pose(ctx)

    agg2 = yield from scan_with_taught_pose(ctx, stack_spec, scan_joints, "스택")
    if not agg2.ok:
        print("\n   스택 QR 인식 실패 — 중단.")
        print(f"   참고: 스택이 멈춘 높이 z={stack_pos[2]:.3f} m. 관측 자세는 선반 높이"
              " (z≈0.6)를 보도록 티칭된 것이라, 높이가 많이 다르면 화각 밖이다.")
        return "STACK_QR_FAIL"

    grasp2, _ = grasp_from_qr(agg2, stack_spec)
    gt2_xy, gt2_top, stack_height = measure_prim(stack_path)
    err2 = np.linalg.norm(grasp2 - np.array([gt2_xy[0], gt2_xy[1], gt2_top])) * 1000.0
    print(f"   QR 파지점 {vec(grasp2, 4)}   GT {vec([gt2_xy[0], gt2_xy[1], gt2_top], 4)}"
          f"   차이 {err2:.1f} mm")

    section("PICK — 스택")
    filter_collision(ctx.gripper_prim, stack_path)
    sync_ik_base_pose(ctx)
    fsm2 = PickFSM(ctx.robot, ctx.gripper, grasp2, stack_path)
    if (yield from run_fsm(ctx.robot, ctx.solver, fsm2, target_quat, "PICK")) != FAIL_OK:
        print("\n   스택 파지 실패 — 중단")
        return "STACK_PICK_FAIL"

    if not (yield from set_carry_pose(ctx.robot, ctx.gripper)):
        return "STACK_DROP_ON_FOLD"

    section("TRANSPORT — 검사 벨트")
    if not (yield from go_to_place(ctx, BELT_BASE_X, BELT_Y_TESTING, 0.0, "검사 벨트")):
        return "STACK_DRIVE_FAIL"
    sync_ik_base_pose(ctx)
    print(f"   주행 후 파지 상태  gripped={holding(ctx.gripper.gripped())}")

    approach2 = np.array([BELT_PLACE_X, BELT_Y_TESTING,
                          BELT_TOP_Z + stack_height + APPROACH_HEIGHT_OFFSET])
    if not (yield from reattach_if_needed(ctx.robot, ctx.solver, ctx.gripper, ctx.nav,
                                          approach2, target_quat, stack_path, stack_height)):
        return "STACK_REATTACH_FAIL"

    section("PLACE — 검사 벨트")
    sync_ik_base_pose(ctx)
    place2 = PlaceFSM(ctx.robot, ctx.gripper, (BELT_PLACE_X, BELT_Y_TESTING),
                      BELT_TOP_Z, stack_height)
    code = yield from run_fsm(ctx.robot, ctx.solver, place2, target_quat, "PLACE")
    return f"STACK fail_code={code}  QR {err2:.1f} mm"


def build_ctxs():
    """돌릴 로봇 목록. ROBOTS=1 이면 기존 한 대짜리 시나리오 그대로다."""
    n = int(os.environ.get("ROBOTS", "2"))
    ctxs = [RobotCtx(name="robot1", carter="/World/Robots/nova_carter1",
                     shelf_id="SHELF-A", magazine=MAGAZINE_SHELF_1,
                     via=SHELF_EXIT_ROBOT1,
                     staging=STAGING_ROBOT1, staging_via=SHELF_EXIT_ROBOT1)]
    if n >= 2:
        # ★ SHELF-B → 포장 벨트는 직선으로 가도 된다. 점유맵에서 재 보면
        #   그 구간의 최소여유가 0.40 m 로 로봇 반경(0.35 m)을 넘긴다 —
        #   SHELF-A 쪽(0.00 m, 선반 관통)과 달리 경유점이 필요 없다.
        ctxs.append(RobotCtx(name="robot2", carter="/World/Robots/nova_carter2",
                             shelf_id="SHELF-B", magazine=MAGAZINE_SHELF_2,
                             via=SHELF_EXIT_ROBOT2,
                             staging=STAGING_ROBOT2, staging_via=SHELF_EXIT_ROBOT2))
    return ctxs


def main():
    world = World(stage_units_in_meters=1.0)
    ctxs = build_ctxs()

    section("SCENE")
    print(f"   로봇 {len(ctxs)}대: " + ", ".join(f"{c.name}({c.shelf_id})" for c in ctxs))
    task = Task2(name="qr_pick_place_2", ctxs=ctxs)
    world.add_task(task)
    world.reset()

    for ctx in ctxs:
        ctx.robot = task.robot(ctx.name)
        ctx.robot.initialize()
        set_ready_pose(ctx.robot)
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    with open(SHELVES_YAML, "r", encoding="utf-8") as fh:
        shelves = {s["shelf_id"]: s for s in yaml.safe_load(fh)["shelves"]}
    with open(TAUGHT_POSES, "r", encoding="utf-8") as fh:
        taught = yaml.safe_load(fh)
    scan_joints = taught["shelf_1_top_close_centered"]["joints_deg"]
    print(f"   scan joints  {scan_joints}  (taught_poses: shelf_1_top_close_centered)")

    section("SOLVER")
    for ctx in ctxs:
        base_pos0, base_quat0 = get_world_pose(ctx.base_link)
        ctx.lula = LulaKinematicsSolver(
            robot_description_path=DESCRIPTION_PATH, urdf_path=URDF_PATH)
        ctx.lula.set_robot_base_pose(robot_position=base_pos0, robot_orientation=base_quat0)
        ctx.solver = ArticulationKinematicsSolver(
            robot_articulation=ctx.robot, kinematics_solver=ctx.lula,
            end_effector_frame_name=EE_LINK_NAME)
        ctx.gripper = SurfaceGripperCtl(task.gripper_node_path(ctx.name))
        ctx.teleporter = BaseTeleporter(ctx.robot)
        ctx.vision = QrVision(world, ctx)
        print(f"   [{ctx.name}] solver/gripper/vision 준비  base {vec(base_pos0, 3)}")

    # ── Nav2 준비 ────────────────────────────────────────────────
    # cobot3_interfaces 는 워크스페이스 로컬 패키지라 install/setup.bash 를
    # source 해야 ros2 가 NavigateTo 타입을 안다. 안 하면 send_goal 이
    # "The passed action type is invalid" 로 죽는데 원인이 안 보인다.
    if USE_NAV:
        if shutil.which("ros2") is None:
            print("   !! ros2 를 못 찾는다 — ROS2 환경을 source 한 셸에서 실행할 것")
            simulation_app.close()
            return
        rc, out = _ros2_run_blocking(
            ["ros2", "interface", "show", NAV_ACTION_TYPE], _clean_ros2_env(), timeout_s=15.0)
        if rc != 0:
            print(f"   !! ros2 가 {NAV_ACTION_TYPE} 를 못 찾는다.")
            print("      source /opt/ros/jazzy/setup.bash && source install/setup.bash 후 실행할 것")
            print(f"      ({out.strip()[-200:]})")
            simulation_app.close()
            return
        for ctx in ctxs:
            ctx.nav = NavDriver(ctx.name)
            print(f"   nav          이송 {ctx.nav.nav2_action}  /  "
                  f"접안 {ctx.nav.precise_action}")
    else:
        print("   nav          꺼짐 (NAV=0) — 텔레포트로 이동한다")

    target_quat = make_target_quat(APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG)
    station = BeltStation(BELT_BASE_X, BELT_Y_PACKAGING)
    t0 = time.time()

    section("MISSION — 매거진 → 포장 벨트 (동시)")
    results = run_concurrent(world, [
        Mission(ctx.name,
                magazine_mission(world, ctx, station, shelves[ctx.shelf_id],
                                 scan_joints, target_quat))
        for ctx in ctxs
    ])

    # 한 대만 돌릴 때는 예전처럼 스택 구간까지 이어서 한다.
    if len(ctxs) == 1 and str(results.get("robot1", "")).startswith("OK"):
        section("MISSION — 스택 → 검사 벨트")
        results.update(run_concurrent(world, [
            Mission("robot1-stack", stack_mission(world, ctxs[0], scan_joints, target_quat))
        ]))

    section("결과")
    for name, res in results.items():
        print(f"   {name:14s} {res}")
    print(f"   총 소요        {time.time() - t0:.1f} s")

    section("HOLD")
    print("   결과 상태로 정지. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=not HEADLESS)
    simulation_app.close()


if __name__ == "__main__":
    main()

"""
PC A 실행 파일:
    isaac_python lula_ik/7_pick_place_color.py

ROS_DOMAIN_ID:
    102

전체 동작:
    1. 기존 6_pick_place.py의 로봇 / 그리퍼 / Lula IK 기능을 불러온다.
    2. BLUE 또는 GREEN 큐브 중 하나를 랜덤하게 선택한다.
    3. 선택된 큐브를 로봇 앞쪽의 랜덤 위치에 배치한다.
    4. 로봇 손목 카메라를 큐브 위쪽으로 이동시킨다.
    5. 손목 카메라의 RGB 영상을 /rgb 토픽으로 Publish한다.
    6. 외부 Color Detector가 색상을 판별한다.
    7. /color_id 토픽으로 색상 ID를 전달받는다.
    8. BLUE / GREEN에 해당하는 위치로 큐브를 Pick & Place한다.

색상 ID:
    1 = BLUE
    2 = GREEN

※ 이 코드는 기존 6_pick_place.py의 기능을 재사용한다.
"""


# ============================================================
# 1. Python 기본 라이브러리
# ============================================================

# 다른 Python 파일(6_pick_place.py)을 모듈처럼 불러오기 위해 사용
import importlib.util

# ROS_DOMAIN_ID 환경변수 설정 등에 사용
import os

# 현재 파일을 기준으로 다른 파일의 경로를 만들기 위해 사용
from pathlib import Path

# 시간 측정
# 예: 10초마다 경고 출력, DDS 충돌 해제 대기 등
import time

# 오류가 발생했을 때 전체 traceback을 출력하기 위해 사용
import traceback


# ============================================================
# 2. ROS_DOMAIN_ID 설정
# ============================================================
#
# 이미 ROS_DOMAIN_ID가 설정되어 있다면 기존 값을 그대로 사용하고,
# 설정되어 있지 않은 경우에만 102로 설정한다.
#
# PC A와 PC B가 ROS 2로 통신하려면
# ROS_DOMAIN_ID가 서로 동일해야 한다.
# ============================================================

os.environ.setdefault("ROS_DOMAIN_ID", "102")


# ============================================================
# 3. 기존 6_pick_place.py 불러오기
# ============================================================
#
# 이 코드에서 로봇 제어 기능을 처음부터 다시 만들지 않고,
# 이전 단계에서 만든 6_pick_place.py의 기능을 재사용한다.
#
# 아래에서 불러온 모듈은 이후 코드에서
#
#     base.World
#     base.M0609Task
#     base.init_gripper()
#     base.set_ready_pose()
#     base.create_ik_solver()
#     base.PickPlaceFSM
#
# 같은 형태로 사용된다.
# ============================================================

spec = importlib.util.spec_from_file_location(
    "pick_place_base",
    Path(__file__).with_name("6_pick_place.py")
)

base = importlib.util.module_from_spec(spec)

spec.loader.exec_module(base)


# ============================================================
# 4. Isaac Sim ROS 2 Bridge 활성화
# ============================================================
#
# Isaac Sim과 ROS 2 사이에서
# Topic / Message 통신을 하기 위해 필요한 Extension이다.
# ============================================================

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")

# Extension 활성화를 Isaac Sim에 반영
base.simulation_app.update()


# ============================================================
# 5. ROS 2 관련 라이브러리
# ============================================================

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# 카메라 영상 메시지
from sensor_msgs.msg import Image

# 색상 ID 메시지
from std_msgs.msg import Int32


# ============================================================
# 6. Isaac Sim 관련 라이브러리
# ============================================================

# 실제 물리 영향을 받는 큐브
# DynamicCuboid:
#     로봇이 집을 수 있는 물리 큐브
#
# VisualCuboid:
#     충돌/물리 목적이 아니라 시각적 표시용 큐브
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid

# Isaac Sim Camera
from isaacsim.sensors.camera import Camera

# 회전 행렬 → Quaternion 변환
from isaacsim.core.utils.rotations import rot_matrix_to_quat


# ============================================================
# 7. 기타 라이브러리
# ============================================================

import numpy as np

# USD Stage에서 /World Prim 등을 생성하기 위해 사용
from pxr import UsdGeom


# ============================================================
# 8. 색상 설정
# ============================================================
#
# Dictionary 구조:
#
#     color_id : RGB 색상값
#
# 1 = BLUE
# 2 = GREEN
#
# Isaac Sim의 색상 값은 일반적으로
# 0.0 ~ 1.0 범위의 RGB 값을 사용한다.
# ============================================================

COLORS = {
    1: np.array([0.0, 0.15, 1.0]),
    2: np.array([0.0, 0.8, 0.05]),
}


# ============================================================
# 9. 색상별 Place 위치
# ============================================================
#
# 색상 검출 결과에 따라 큐브를 놓을 XY 위치이다.
#
# color_id = 1 (BLUE)
#     → [0.45, -0.18]
#
# color_id = 2 (GREEN)
#     → [0.45,  0.18]
#
# 즉 BLUE와 GREEN 큐브를 서로 다른 위치로 분류한다.
# ============================================================

PLACES = {
    1: np.array([0.45, -0.18]),
    2: np.array([0.45, 0.18]),
}


# ============================================================
# 10. 카메라 관찰 높이
# ============================================================
#
# 로봇 TCP를 큐브 위쪽으로 이동시킬 때 사용하는 Z 높이
# ============================================================

OBSERVE_Z = 0.35


# ============================================================
# 11. 손목 카메라 위치 Offset
# ============================================================
#
# 카메라를 로봇 손가락 바로 중앙에 놓으면
# Gripper 자체가 카메라 화면을 가릴 수 있다.
#
# 그래서 카메라 렌즈를 손가락 끝부분에서 약간 옆/위쪽으로
# 이동시킨다.
#
# [X, Y, Z]
# ============================================================

CAMERA_OFFSET = np.array([0.18, 0.0, 0.20])


# ============================================================
# 12. 손목 카메라를 큐브 방향으로 조준
# ============================================================

def aim_wrist_camera(camera, pick):
    """
    로봇이 관찰 위치에 도착한 후,
    실제 카메라 위치를 기준으로 큐브를 바라보도록 방향을 계산한다.

    Parameters
    ----------
    camera
        Isaac Sim Camera 객체

    pick
        현재 선택된 큐브의 XY 위치

    ----------------------------------------------------------
    USD Camera 좌표축
    ----------------------------------------------------------

        +X = 오른쪽
        +Y = 위쪽
        -Z = 카메라가 바라보는 앞쪽

    카메라는 link_6 아래에 parent되어 있지만,
    여기서는 World 기준 위치를 이용해 최종 방향을 계산한다.
    """

    # --------------------------------------------------------
    # 현재 카메라의 World 위치 가져오기
    #
    # 반환값:
    #     position    = 카메라 위치
    #     orientation = 카메라 방향
    #
    # 여기서는 위치만 필요하므로 orientation은 "_"로 받는다.
    # --------------------------------------------------------
    position, _ = camera.get_world_pose(
        camera_axes="usd"
    )

    # --------------------------------------------------------
    # 카메라 → 큐브 방향 벡터 계산
    #
    # np.r_[pick, base.PICK_Z]
    #
    # pick이 [X, Y]라면
    #
    #     [X, Y, PICK_Z]
    #
    # 형태의 3차원 좌표가 된다.
    # --------------------------------------------------------
    forward = np.r_[pick, base.PICK_Z] - position

    # 방향 벡터 길이를 1로 만들어 단위 벡터로 변환
    forward /= np.linalg.norm(forward)

    # --------------------------------------------------------
    # 카메라의 오른쪽 방향 벡터 계산
    #
    # 외적(Cross Product)을 이용한다.
    # --------------------------------------------------------
    right = np.cross(
        forward,
        np.array([0., 1., 0.])
    )

    # 단위 벡터로 변환
    right /= np.linalg.norm(right)

    # --------------------------------------------------------
    # 카메라의 위쪽 방향 벡터 계산
    # --------------------------------------------------------
    up = np.cross(
        right,
        forward
    )

    # --------------------------------------------------------
    # 카메라 회전 행렬 생성
    #
    # USD Camera는 -Z 방향이 앞쪽이므로
    # 마지막 축에는 -forward를 사용한다.
    # --------------------------------------------------------
    rotation = np.column_stack(
        (right, up, -forward)
    )

    # --------------------------------------------------------
    # 회전 행렬 → Quaternion 변환 후
    # 카메라 방향 적용
    # --------------------------------------------------------
    camera.set_world_pose(
        orientation=rot_matrix_to_quat(rotation),
        camera_axes="usd"
    )

    # 카메라 위치와 목표 위치 출력
    print(
        f"Wrist camera at {position}, "
        f"aimed at {np.r_[pick, base.PICK_Z]}"
    )


# ============================================================
# 13. ColorTask
# ============================================================
#
# 기존 6_pick_place.py의 M0609Task를 상속한다.
#
# 즉,
#
#     기존 로봇 Scene 구성
#         +
#     색상 큐브
#         +
#     색상별 Place Marker
#
# 를 추가한 Task이다.
# ============================================================

class ColorTask(base.M0609Task):

    # ========================================================
    # USD Scene 불러오기
    # ========================================================

    def _load_usd(self):

        # 현재 Isaac Sim USD Stage 가져오기
        stage = base.omni.usd.get_context().get_stage()

        # /World Prim 찾기
        world_prim = stage.GetPrimAtPath("/World")

        # /World가 없다면 새로 생성
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(
                stage,
                "/World"
            ).GetPrim()

        # 기존 로봇 USD 파일을 /World에 Reference로 연결
        world_prim.GetReferences().AddReference(
            base.USD_PATH
        )

        # ----------------------------------------------------
        # 중요:
        #
        # lesson 6의 loader를 그대로 호출하지 않는다.
        #
        # 기존 loader는 simulation_app.update()를 여러 번 호출하면서
        # USD 안에 이미 작성된 Camera / ROS Graph가 먼저 실행될 수 있다.
        #
        # 그렇게 되면 이 코드가 만든 /rgb Publisher 외에
        # 다른 /rgb Publisher까지 생길 수 있다.
        #
        # 따라서 먼저 USD 안의 OmniGraph와 기존 Cube를 비활성화한 후
        # simulation_app.update()를 실행한다.
        # ----------------------------------------------------

        # ----------------------------------------------------
        # Parent Prim을 비활성화하면 그 아래 Prim Handle도
        # 무효화될 수 있다.
        #
        # 따라서 바로 Prim을 수정하지 않고
        # 먼저 "경로(Path)"만 모두 수집한다.
        # ----------------------------------------------------
        disable_paths = [
            prim.GetPath()
            for prim in stage.Traverse()
            if prim.GetTypeName() == "OmniGraph"
            or prim.GetName() == "Cube"
        ]

        # ----------------------------------------------------
        # 저장해 둔 Path를 이용하여 Prim을 다시 찾은 뒤 비활성화
        # ----------------------------------------------------
        for path in disable_paths:

            prim = stage.GetPrimAtPath(path)

            # Prim이 유효하고 현재 활성화되어 있을 때만 비활성화
            if prim.IsValid() and prim.IsActive():
                prim.SetActive(False)

                print(
                    f"Disabled before first update: {path}"
                )

        # Isaac Sim Scene에 변경 내용을 충분히 반영
        for _ in range(15):
            base.simulation_app.update()

        print(
            "   USD          loaded "
            "(authored camera graphs disabled)"
        )

    # ========================================================
    # Scene 설정
    # ========================================================

    def set_up_scene(self, scene):

        # ----------------------------------------------------
        # 부모 클래스(M0609Task)의 Scene 설정을 먼저 실행
        #
        # 여기에서 기본 M0609 로봇 등이 설정된다.
        # ----------------------------------------------------
        super().set_up_scene(scene)

        # 생성한 색상 큐브들을 저장할 Dictionary
        self.cubes = {}

        # ----------------------------------------------------
        # COLORS에 등록된 색상별로
        #
        # 1. 실제 Pick 대상 Dynamic Cube 생성
        # 2. Place 위치 표시용 Visual Cube 생성
        #
        # 을 수행한다.
        # ----------------------------------------------------
        for color_id, color in COLORS.items():

            # ------------------------------------------------
            # 실제 로봇이 Pick할 Dynamic Cube
            # ------------------------------------------------
            self.cubes[color_id] = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/SortCube{color_id}",
                    name=f"sort_cube_{color_id}",

                    # 처음에는 작업 영역 바깥쪽에 배치
                    position=np.array([
                        -0.4,
                        color_id * 0.15,
                        0.025
                    ]),

                    # 큐브 한 변 크기
                    size=0.05,

                    # 큐브 질량
                    mass=0.05,

                    # BLUE / GREEN 색상
                    color=color
                )
            )

            # ------------------------------------------------
            # 색상별 Place 위치를 보여주는 Marker
            #
            # 로봇이 집는 물체가 아니라
            # "여기가 BLUE 자리 / GREEN 자리"를 보여주는
            # 시각적 표시이다.
            # ------------------------------------------------
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/ColorMarker{color_id}",
                    name=f"marker_{color_id}",

                    # PLACES의 XY + 아주 낮은 Z
                    position=np.r_[
                        PLACES[color_id],
                        0.001
                    ],

                    # 바닥에 납작하게 표시
                    scale=np.array([
                        0.10,
                        0.10,
                        0.002
                    ]),

                    color=color
                )
            )

    # ========================================================
    # 매 Play 시작 시 큐브 랜덤 배치
    # ========================================================

    def randomize(self):

        # NumPy Random Generator 생성
        rng = np.random.default_rng()

        # ----------------------------------------------------
        # BLUE(1), GREEN(2) 중 하나를 랜덤 선택
        # ----------------------------------------------------
        selected = int(
            rng.choice([1, 2])
        )

        # ----------------------------------------------------
        # 선택된 큐브의 Pick 위치도 랜덤하게 결정
        #
        # X:
        #     0.25 ~ 0.34
        #
        # Y:
        #    -0.06 ~ 0.06
        # ----------------------------------------------------
        pick = rng.uniform(
            [0.25, -0.06],
            [0.34, 0.06]
        )

        # ----------------------------------------------------
        # 모든 색상 큐브 확인
        # ----------------------------------------------------
        for color_id, cube in self.cubes.items():

            # 선택된 큐브라면 랜덤 Pick 위치로 이동
            #
            # 선택되지 않은 큐브라면 작업영역 바깥쪽으로 이동
            xy = (
                pick
                if color_id == selected
                else np.array([
                    -0.4,
                    color_id * 0.15
                ])
            )

            # ------------------------------------------------
            # 큐브 위치와 방향 초기화
            # ------------------------------------------------
            cube.set_world_pose(
                position=np.r_[
                    xy,
                    0.025
                ],
                orientation=np.array([
                    1.,
                    0.,
                    0.,
                    0.
                ])
            )

            # 이전 Play에서 남아 있을 수 있는
            # 선형 속도를 0으로 초기화
            cube.set_linear_velocity(
                np.zeros(3)
            )

            # 이전 Play에서 남아 있을 수 있는
            # 회전 속도를 0으로 초기화
            cube.set_angular_velocity(
                np.zeros(3)
            )

        # 최종 선택된 Pick XY 위치 반환
        return pick


# ============================================================
# 14. ROS 2 ColorBridge Node
# ============================================================
#
# 이 노드는 Isaac Sim과 외부 Color Detector 사이에서
# 다리(Bridge) 역할을 한다.
#
# Publish:
#
#     /rgb
#         손목 카메라 RGB 영상
#
# Subscribe:
#
#     /color_id
#         외부 Color Detector가 판별한 색상
#
#         1 = BLUE
#         2 = GREEN
# ============================================================

class ColorBridge(Node):

    def __init__(self):

        # ROS 2 Node 이름
        super().__init__(
            "m0609_color_sort_sim"
        )

        # ----------------------------------------------------
        # /rgb Publisher
        #
        # Isaac Sim 손목 카메라 영상을 외부 Detector로 전송
        # ----------------------------------------------------
        self.publisher = self.create_publisher(
            Image,
            "/rgb",
            qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # /color_id Subscriber
        #
        # 외부 Detector에서 판별한 색상 ID 수신
        # ----------------------------------------------------
        self.subscription = self.create_subscription(
            Int32,
            "/color_id",
            self.receive,
            qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # 현재 /color_id를 받아도 되는 상태인지 여부
        #
        # False:
        #     검출 결과를 무시
        #
        # True:
        #     검출 결과를 사용
        # ----------------------------------------------------
        self.accepting = False

        # 최종 확정된 색상 ID
        self.color_id = None

        # 직전에 수신된 색상 ID
        self.last_id = None

        # 동일한 색상 ID가 연속으로 들어온 횟수
        self.count = 0

        # /rgb Publisher 충돌 여부
        self.conflict = False

        # 마지막 충돌 로그 출력 시간
        self.last_conflict_log = -float("inf")

        # 충돌이 사라지기 시작한 시간
        self.clear_since = None

    # ========================================================
    # /color_id Callback
    # ========================================================

    def receive(self, msg):

        # 현재 검출 결과를 받는 단계가 아니라면 무시
        if not self.accepting:
            return

        # ----------------------------------------------------
        # 같은 ID가 연속으로 들어왔으면 count + 1
        #
        # 다른 ID가 들어왔다면 count를 다시 1로 시작
        # ----------------------------------------------------
        self.count = (
            self.count + 1
            if msg.data == self.last_id
            else 1
        )

        # 이번에 받은 ID 저장
        self.last_id = msg.data

        # ----------------------------------------------------
        # PLACES에 등록된 유효한 색상 ID이며
        # 같은 결과가 3번 이상 연속으로 들어온 경우에만
        # 최종 색상으로 확정한다.
        #
        # 즉 순간적인 오검출을 바로 사용하지 않는다.
        # ----------------------------------------------------
        if msg.data in PLACES and self.count >= 3:
            self.color_id = msg.data

    # ========================================================
    # Isaac Sim Camera → /rgb Publish
    # ========================================================

    def publish_rgb(self, camera):

        # Isaac Sim Camera에서 RGBA 이미지 가져오기
        rgba = camera.get_rgba()

        # 영상이 아직 준비되지 않았으면 종료
        if rgba is None or rgba.size == 0:
            return

        # ----------------------------------------------------
        # RGBA → RGB
        #
        # Alpha 채널은 필요 없으므로 앞의 3채널만 사용한다.
        #
        # np.ascontiguousarray():
        # ROS 메시지로 전달하기 좋은 연속 메모리 형태로 만든다.
        # ----------------------------------------------------
        rgb = np.ascontiguousarray(
            rgba[:, :, :3],
            dtype=np.uint8
        )

        # ROS Image 메시지 생성
        msg = Image()

        # 현재 ROS 시간 기록
        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # Camera Frame 이름
        msg.header.frame_id = (
            "wrist_camera_optical_frame"
        )

        # 이미지 높이 / 너비
        msg.height, msg.width = rgb.shape[:2]

        # 이미지 Encoding
        msg.encoding = "rgb8"

        # 한 행의 Byte 수
        #
        # RGB이므로 픽셀 하나당 3Byte
        msg.step = msg.width * 3

        # NumPy 이미지 → Byte 데이터
        msg.data = rgb.tobytes()

        # /rgb 토픽으로 Publish
        self.publisher.publish(msg)

    # ========================================================
    # /rgb Publisher 충돌 확인
    # ========================================================

    def check_rgb_source(self):
        """
        색상 검출 중에만 호출된다.

        /rgb를 Publish하는 Node가 이 코드 하나인지 확인한다.

        이유:
            /rgb Publisher가 여러 개 존재하면
            Color Detector가 어떤 카메라 영상을 받아야 하는지
            불분명해질 수 있다.

        DDS Endpoint가 안정화된 후 자동으로 검출을 재시작한다.
        """

        # 현재 /rgb Topic의 Publisher 정보 가져오기
        endpoints = self.get_publishers_info_by_topic(
            "/rgb"
        )

        # 현재 monotonic 시간
        now = time.monotonic()

        # ----------------------------------------------------
        # 정상 상태에서는 /rgb Publisher가 정확히 1개여야 한다.
        # ----------------------------------------------------
        if len(endpoints) != 1:

            # 충돌 상태 표시
            self.conflict = True

            # 충돌 해제 시간 초기화
            self.clear_since = None

            # 색상 결과 수신 중지
            self.accepting = False

            # 기존 검출 결과 제거
            self.color_id = self.last_id = None

            # 연속 검출 횟수 초기화
            self.count = 0

            # ------------------------------------------------
            # 충돌 상태가 계속되는 동안 로그가 너무 많이
            # 출력되지 않도록 5초 간격으로 출력한다.
            # ------------------------------------------------
            if now - self.last_conflict_log >= 5.0:

                # 현재 /rgb를 Publish 중인 Node 이름 목록
                names = [
                    f"{e.node_namespace.rstrip('/')}/{e.node_name}"
                    for e in endpoints
                ]

                self.get_logger().error(
                    f"/rgb publishers ({len(endpoints)}): "
                    f"{names}. "
                    "Waiting for only "
                    "/m0609_color_sort_sim."
                )

                self.last_conflict_log = now

            return

        # ----------------------------------------------------
        # 이전에는 충돌 상태였는데
        # 이제 Publisher가 하나로 돌아온 경우
        # ----------------------------------------------------
        if self.conflict:

            # Publisher 하나 상태가 처음 확인된 시점 기록
            if self.clear_since is None:
                self.clear_since = now

            # ------------------------------------------------
            # 바로 정상 처리하지 않고 1초 동안 기다린다.
            #
            # DDS Endpoint 상태가 실제로 안정화되는 시간을 준다.
            # ------------------------------------------------
            if now - self.clear_since < 1.0:
                return

            # 충돌 해제
            self.conflict = False

            # 이전 색상 판별 결과 초기화
            self.color_id = self.last_id = None

            # 연속 판별 횟수 초기화
            self.count = 0

            self.get_logger().info(
                "/rgb conflict cleared; "
                "restarting color detection"
            )

        # ----------------------------------------------------
        # /rgb Publisher 상태가 정상이라면
        # /color_id 결과를 다시 받을 수 있도록 한다.
        # ----------------------------------------------------
        self.accepting = True


# ============================================================
# 15. Main
# ============================================================

def main():

    # ========================================================
    # ROS 2 초기화
    # ========================================================

    rclpy.init()

    # Isaac Sim ↔ Color Detector 연결용 ROS Node 생성
    bridge = ColorBridge()

    try:

        # ====================================================
        # 1. Isaac Sim World 생성
        # ====================================================
        #
        # stage_units_in_meters=1.0
        #
        # Isaac Sim Stage에서
        # 1 unit = 1 meter
        # ====================================================

        world = base.World(
            stage_units_in_meters=1.0
        )

        # ====================================================
        # 2. ColorTask 생성
        # ====================================================

        task = ColorTask(
            name="color_sort"
        )

        # World에 Task 등록
        world.add_task(task)

        # Scene 초기화
        world.reset()


        # ====================================================
        # 3. Robot 가져오기 및 초기화
        # ====================================================

        robot = task.robot

        robot.initialize()


        # ====================================================
        # 4. Gripper 초기화
        # ====================================================

        base.init_gripper(
            robot,
            world
        )


        # ====================================================
        # 5. 로봇 Ready Pose 이동
        # ====================================================

        base.set_ready_pose(robot)


        # ====================================================
        # 6. End-Effector Prim Path 찾기
        # ====================================================
        #
        # 손목 카메라를 End-Effector 아래에 붙이기 위해
        # EE Link의 Prim 경로를 찾는다.
        # ====================================================

        ee_path = base.find_prim_path(
            base.ROBOT_PRIM_PATH,
            base.EE_LINK_NAME
        )


        # ====================================================
        # 7. 손목 Camera 생성
        # ====================================================
        #
        # Resolution:
        #     640 x 480
        # ====================================================

        camera = Camera(
            prim_path=ee_path + "/ColorWristCamera",
            resolution=(640, 480)
        )


        # ====================================================
        # 8. Camera 위치 설정
        # ====================================================
        #
        # 손가락 자체가 카메라 화면을 가리지 않도록
        # CAMERA_OFFSET만큼 이동시킨다.
        #
        # 초기 orientation은
        # [1, 0, 0, 0] Quaternion
        #
        # 이후 aim_wrist_camera()에서 실제 큐브 방향으로
        # 다시 조정된다.
        # ====================================================

        camera.set_local_pose(
            translation=CAMERA_OFFSET,
            orientation=np.array([
                1.,
                0.,
                0.,
                0.
            ]),
            camera_axes="ros"
        )


        # ====================================================
        # 9. Camera Clipping Range 설정
        # ====================================================
        #
        # USD Camera의 기본 Near Clip 값은
        # Scene 단위에 따라 너무 멀 수 있다.
        #
        # 이 Scene에서는 큐브가 손목 카메라에서
        # 약 0.35m 정도 거리에 있기 때문에
        # near_distance를 0.01m로 설정한다.
        # ====================================================

        camera.set_clipping_range(
            near_distance=0.01,
            far_distance=10.0
        )

        # Camera 초기화
        camera.initialize()

        # Camera 정보 출력
        print(
            f"Wrist camera clipping range (m): "
            f"{camera.get_clipping_range()}"
        )

        print(
            f"/rgb source: "
            f"{ee_path}/ColorWristCamera "
            f"(640x480 rgb8)"
        )


        # ====================================================
        # 10. Lula IK Solver 생성
        # ====================================================
        #
        # 목표 TCP 위치를 Joint 값으로 변환하기 위해 사용한다.
        # ====================================================

        solver = base.create_ik_solver(robot)


        # ====================================================
        # 11. End-Effector 목표 방향 생성
        # ====================================================
        #
        # Roll / Pitch / Yaw 형태의 목표 방향을
        # Quaternion으로 만든다.
        # ====================================================

        quat = base.make_target_quat(
            180.,
            0.,
            0.
        )


        # ====================================================
        # 12. 상태 변수 초기화
        # ====================================================

        # 이전 Frame에서 Play 상태였는지
        was_playing = False

        # 현재 동작 단계
        phase = "idle"

        # Simulation Frame Count
        tick = 0


        # ====================================================
        # 13. Isaac Sim Main Loop
        # ====================================================

        while base.simulation_app.is_running():

            # ------------------------------------------------
            # Simulation 한 Step 진행 + Render
            # ------------------------------------------------
            world.step(render=True)

            # ------------------------------------------------
            # ROS 2 Callback 처리
            #
            # timeout_sec=0.0이므로
            # 기다리지 않고 현재 들어온 메시지만 처리한다.
            # ------------------------------------------------
            rclpy.spin_once(
                bridge,
                timeout_sec=0.0
            )

            # 현재 Isaac Sim이 Play 상태인지 확인
            playing = world.is_playing()


            # =================================================
            # STOP 상태
            # =================================================

            if not playing:

                # STOP 중에는 Color Detection 결과를 받지 않는다.
                bridge.accepting = False

                # 다음 Play를 새로운 실행으로 판단하기 위해 초기화
                was_playing = False

                continue


            # =================================================
            # 새로운 Play가 시작된 순간
            # =================================================

            if not was_playing:

                # Scene Reset
                world.reset()

                # Robot 재초기화
                robot.initialize()

                # Gripper 재초기화
                base.init_gripper(
                    robot,
                    world
                )

                # Ready Pose로 이동
                base.set_ready_pose(robot)

                # ------------------------------------------------
                # BLUE / GREEN 중 하나의 큐브를 랜덤 선택하고
                # Pick 위치도 랜덤으로 결정
                # ------------------------------------------------
                pick = task.randomize()

                # 아직 색상 검출 단계가 아니므로 결과 수신 중지
                bridge.accepting = False

                # 이전 검출 결과 초기화
                bridge.color_id = bridge.last_id = None

                # 연속 색상 판정 횟수 초기화
                bridge.count = 0

                # 현재 TCP 위치 저장
                start = base.get_tcp_pose(robot)

                # ------------------------------------------------
                # 카메라 관찰 위치 생성
                #
                # XY:
                #     큐브의 Pick XY
                #
                # Z:
                #     OBSERVE_Z = 0.35
                # ------------------------------------------------
                observation = np.r_[
                    pick,
                    OBSERVE_Z
                ]

                # Tick 초기화
                tick = 0

                # 첫 번째 Phase:
                # 로봇을 관찰 위치로 이동
                phase = "observe_move"

                print(
                    f"Pick XY: {pick}; "
                    "waiting for camera detection"
                )


            # 이제 Play 상태임을 저장
            was_playing = True


            # =================================================
            # 현재 Pick & Place 수행 중인지 확인
            # =================================================

            sorting = phase == "sort"


            # =================================================
            # PHASE 1
            # observe_move
            #
            # 로봇을 큐브 위 관찰 위치까지 이동
            # =================================================

            if phase == "observe_move":

                # ------------------------------------------------
                # 시작 위치 → Observation 위치를
                # LERP(선형 보간) 방식으로 부드럽게 이동
                #
                # tick / 180 값이 1보다 커지면
                # 최댓값을 1로 제한한다.
                # ------------------------------------------------
                target = base.lerp(
                    start,
                    observation,
                    min(tick / 180., 1.)
                )

                # 이동 중 Gripper는 OPEN
                gripper = "open"

                # ------------------------------------------------
                # 다음 단계로 넘어가는 조건
                #
                # 1. 최소 240 Tick 이상 지남
                #
                # 2. 현재 TCP와 Observation 위치 차이가
                #    0.02m 미만
                # ------------------------------------------------
                if (
                    tick >= 240
                    and np.linalg.norm(
                        base.get_tcp_pose(robot)
                        - observation
                    ) < 0.02
                ):

                    # 손목 카메라를 실제 큐브 방향으로 조준
                    aim_wrist_camera(
                        camera,
                        pick
                    )

                    # 카메라 방향 변경 후
                    # 추가로 30 Tick 기다리기 위한 값
                    camera_ready_tick = tick + 30

                    # 다음 Phase
                    phase = "camera_settle"

                    # 기존 Color Detection 결과 초기화
                    bridge.color_id = bridge.last_id = None

                    bridge.count = 0

                    bridge.accepting = False

                    # ------------------------------------------------
                    # 검출 대기 시간 측정 시작
                    #
                    # 이후 10초 동안 결과가 없으면
                    # 안내 메시지를 출력한다.
                    # ------------------------------------------------
                    wait_start = time.monotonic()


            # =================================================
            # PHASE 2
            # camera_settle
            #
            # 카메라 방향을 바꾼 뒤 영상이 안정될 때까지 대기
            # =================================================

            elif phase == "camera_settle":

                # 로봇 위치 유지
                target = observation

                # Gripper OPEN 유지
                gripper = "open"

                # 30 Tick 대기 완료
                if tick >= camera_ready_tick:

                    # 색상 검출 Phase로 이동
                    phase = "detect"

                    # /color_id 결과 수신 허용
                    bridge.accepting = True


            # =================================================
            # PHASE 3
            # detect
            #
            # Camera RGB 영상을 보내고
            # 외부 Color Detector의 결과를 기다린다.
            # =================================================

            elif phase == "detect":

                # 관찰 위치 유지
                target = observation

                # Gripper OPEN 유지
                gripper = "open"

                # ------------------------------------------------
                # /rgb Publisher가 하나만 존재하는지 확인
                # ------------------------------------------------
                bridge.check_rgb_source()

                # ------------------------------------------------
                # 매 Simulation Frame마다 영상을 보내지 않고
                # 6 Tick마다 한 번씩 /rgb Publish
                # ------------------------------------------------
                if tick % 6 == 0:
                    bridge.publish_rgb(camera)

                # ------------------------------------------------
                # 색상 판별 성공
                # ------------------------------------------------
                if bridge.color_id is not None:

                    # 더 이상 Color Detection 결과를 받을 필요 없음
                    bridge.accepting = False

                    # ------------------------------------------------
                    # Pick 위치 설정
                    #
                    # 현재 랜덤 배치된 큐브의 위치
                    # ------------------------------------------------
                    base.PICK_XY = pick

                    # ------------------------------------------------
                    # Place 위치 설정
                    #
                    # color_id에 따라
                    #
                    # BLUE  → BLUE 위치
                    # GREEN → GREEN 위치
                    # ------------------------------------------------
                    base.PLACE_XY = PLACES[
                        bridge.color_id
                    ]

                    # ------------------------------------------------
                    # 기존 6_pick_place.py의
                    # Pick & Place FSM 생성
                    # ------------------------------------------------
                    fsm = base.PickPlaceFSM(robot)

                    # 다음 Phase
                    phase = "sort"

                    print(
                        f"Detected color_id={bridge.color_id}; "
                        f"place XY={base.PLACE_XY}"
                    )

                # ------------------------------------------------
                # 10초 이상 색상 결과가 오지 않은 경우
                # ------------------------------------------------
                elif time.monotonic() - wait_start > 10:

                    print(
                        "Waiting for /color_id: "
                        "check detector, camera image "
                        "and ROS_DOMAIN_ID=102"
                    )

                    # 다시 10초 측정 시작
                    wait_start = time.monotonic()


            # =================================================
            # PHASE 4
            # sort
            #
            # 실제 Pick & Place 실행
            # =================================================

            elif phase == "sort":

                # ------------------------------------------------
                # FSM에서 새로운 이동 Segment가 시작되는 순간
                # 필요한 값들을 초기화한다.
                # ------------------------------------------------
                if (
                    fsm.start is None
                    and fsm.state < fsm.DONE_STATE
                ):

                    # 현재 TCP 위치 = Segment 시작 위치
                    fsm.start = base.get_tcp_pose(robot)

                    # 현재 State의 목표 Waypoint
                    fsm.goal = fsm.waypoints[
                        fsm.state
                    ]

                    # ------------------------------------------------
                    # 현재 State에 지정된 Gripper 동작이 있다면
                    # 해당 상태로 변경
                    # ------------------------------------------------
                    fsm.gripper = fsm.GRIPPER_STATES.get(
                        fsm.state,
                        fsm.gripper
                    )

                    # ------------------------------------------------
                    # 현재 Segment에 필요한 Step 수 계산
                    #
                    # Gripper 동작 State라면:
                    #     GRIPPER_WAIT 사용
                    #
                    # 이동 State라면:
                    #     시작점 → 목표점 거리에 따라 Step 계산
                    # ------------------------------------------------
                    fsm.n_steps = (
                        base.GRIPPER_WAIT
                        if fsm.state in fsm.GRIPPER_STATES
                        else base.steps_for(
                            fsm.start,
                            fsm.goal
                        )[0]
                    )

                # 현재 FSM Step에서 이동해야 할 목표 위치
                target = fsm.current_target()

                # 현재 Gripper 상태
                gripper = fsm.gripper


            # =================================================
            # 14. Lula IK 계산
            # =================================================
            #
            # 위의 Phase에서 정해진 target을
            # 실제 로봇 Joint Action으로 변환한다.
            # =================================================

            action, solved = solver.compute_inverse_kinematics(

                # TCP 목표를 Flange 기준 목표로 변환
                target_position=base.tcp_to_flange(
                    target,
                    quat
                ),

                # End-Effector 방향
                target_orientation=quat
            )


            # =================================================
            # 15. IK 계산 성공 시 Robot Joint Action 적용
            # =================================================

            if solved:

                robot.apply_action(action)

                # ------------------------------------------------
                # 현재 Pick & Place 정렬 단계인 경우
                # FSM Step도 함께 진행
                # ------------------------------------------------
                if sorting:

                    # ------------------------------------------------
                    # 현재 Segment가 끝났다면
                    # 다음 FSM State로 이동
                    #
                    # Endpoint에서 한 번 멈춘 뒤 다음 Segment로
                    # 넘어가도록 하는 구조이다.
                    # ------------------------------------------------
                    if (
                        fsm.step >= fsm.n_steps
                        and fsm.state < fsm.DONE_STATE
                    ):
                        fsm._next()

                    # ------------------------------------------------
                    # 아직 현재 Segment가 끝나지 않았다면
                    # Step 증가
                    #
                    # IK가 실패한 Frame에서는 이 부분이 실행되지
                    # 않기 때문에 FSM이 진행되지 않고 멈춰 있게 된다.
                    # ------------------------------------------------
                    elif fsm.state < fsm.DONE_STATE:
                        fsm.step += 1


            # =================================================
            # 16. Gripper Action 적용
            # =================================================
            #
            # OPEN / CLOSE 명령을 Gripper에 전달한다.
            # =================================================

            robot.apply_action(
                robot.gripper.forward(
                    action=gripper
                )
            )


            # =================================================
            # 17. Simulation Tick 증가
            # =================================================

            tick += 1


    # ========================================================
    # 오류 발생
    # ========================================================

    except Exception:

        # ----------------------------------------------------
        # SimulationApp.close()가 실행되기 전에
        # 오류 내용을 확실히 출력하기 위해 traceback 출력
        # ----------------------------------------------------
        traceback.print_exc()

        # 오류를 다시 발생시켜 상위에서도 알 수 있도록 함
        raise


    # ========================================================
    # 프로그램 종료 시 항상 실행
    # ========================================================

    finally:

        # ROS Node 제거
        bridge.destroy_node()

        # ROS 2 종료
        rclpy.shutdown()

        # Isaac Sim 종료
        base.simulation_app.close()


# ============================================================
# 16. Python 파일 직접 실행 시 main() 호출
# ============================================================

if __name__ == "__main__":
    main()
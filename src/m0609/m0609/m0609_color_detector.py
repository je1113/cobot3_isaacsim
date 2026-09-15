"""
손목(Wrist) RGB 카메라 영상에서 색상을 검출하는 ROS 2 노드

검출 결과:
    0 = 색상 판별 불확실 / 검출 실패
    1 = BLUE
    2 = GREEN

입력 토픽:
    /rgb

출력 토픽:
    /color_id
    /color_debug
"""

# ============================================================
# 기본 라이브러리
# ============================================================
import os
import time

# ============================================================
# 영상 처리 라이브러리
# ============================================================
import cv2
import numpy as np

# ============================================================
# ROS 2 관련 라이브러리
# ============================================================
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# ============================================================
# ROS 2 메시지 타입
# ============================================================
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


# ============================================================
# 색상 검출 함수
# ============================================================
def detect_color(
    bgr,
    roi_fraction=0.18,
    min_area=150,
    return_details=False
):
    """
    손목 카메라가 바라보는 중앙 영역에서 BLUE / GREEN 색상을 검출한다.

    Parameters
    ----------
    bgr : numpy.ndarray
        OpenCV의 BGR 이미지

    roi_fraction : float
        전체 이미지 중 색상 판별에 사용할 중앙 영역의 비율
        기본값: 0.18

    min_area : int
        색상 영역으로 인정할 최소 픽셀 면적
        기본값: 150

    return_details : bool
        False:
            color_id만 반환

        True:
            color_id, 각 색상의 면적, ROI 영역을 함께 반환

    Returns
    -------
    color_id : int
        0 = UNKNOWN
        1 = BLUE
        2 = GREEN
    """

    # --------------------------------------------------------
    # 1. 입력 이미지 크기 확인
    # --------------------------------------------------------
    h, w = bgr.shape[:2]

    # --------------------------------------------------------
    # 2. 이미지 중앙 ROI(Region Of Interest) 계산
    #
    # 전체 화면이 아니라 카메라 중앙의 작은 영역만 검사한다.
    # 이렇게 하면 화면 옆쪽의 색상 물체가 검출 결과에 영향을
    # 주는 것을 줄일 수 있다.
    # --------------------------------------------------------
    dx = int(w * (1 - roi_fraction) / 2)
    dy = int(h * (1 - roi_fraction) / 2)

    # 중앙 ROI만 잘라낸 후 BGR → HSV 변환
    hsv = cv2.cvtColor(
        bgr[dy:h - dy, dx:w - dx],
        cv2.COLOR_BGR2HSV
    )

    # BLUE / GREEN 각각의 가장 큰 영역 크기를 저장한다.
    areas = []

    # --------------------------------------------------------
    # 3. HSV 범위를 이용하여 BLUE / GREEN 검출
    #
    # 첫 번째 범위  -> BLUE
    # 두 번째 범위  -> GREEN
    # --------------------------------------------------------
    for low, high in [
        ((95, 40, 50), (135, 255, 255)),   # BLUE
        ((35, 40, 50), (85, 255, 255)),    # GREEN
    ]:

        # HSV 범위에 포함되는 픽셀만 흰색(255)으로 만든 마스크 생성
        mask = cv2.inRange(
            hsv,
            np.array(low),
            np.array(high)
        )

        # ----------------------------------------------------
        # 작은 노이즈 제거
        #
        # MORPH_OPEN:
        #   작은 점이나 잡음 형태의 영역을 제거하는 형태학 연산
        # ----------------------------------------------------
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), np.uint8)
        )

        # ----------------------------------------------------
        # 연결된 색상 영역을 각각 분리
        #
        # count  : 연결 영역 개수
        # labels : 각 픽셀이 어느 영역에 속하는지
        # stats  : 각 영역의 위치 / 크기 / 면적 정보
        # ----------------------------------------------------
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)

        # label 0은 배경이므로 제외한다.
        #
        # ROI 자체가 이미 카메라 중앙을 제한하고 있기 때문에
        # 정확한 화면 중심과 겹치는지 추가 검사하지 않는다.
        #
        # 작은 큐브 또는 약간 옆으로 이동한 큐브까지 검출하기 위함이다.
        candidates = range(1, count)

        # 해당 색상에서 가장 큰 연결 영역의 면적을 찾는다.
        areas.append(
            max(
                (
                    int(stats[label, cv2.CC_STAT_AREA])
                    for label in candidates
                ),
                default=0
            )
        )

    # --------------------------------------------------------
    # 4. BLUE / GREEN 중 가장 큰 색상 영역 확인
    #
    # areas[0] = BLUE 면적
    # areas[1] = GREEN 면적
    # --------------------------------------------------------
    largest = max(areas)

    # --------------------------------------------------------
    # 5. 최종 색상 ID 결정
    #
    # 조건 1:
    #   가장 큰 영역이 min_area보다 작으면 UNKNOWN
    #
    # 조건 2:
    #   가장 큰 영역이 다른 색상보다 1.5배 이상 크지 않으면
    #   판별이 애매하다고 판단하여 UNKNOWN
    #
    # 그 외:
    #   BLUE가 크면 1
    #   GREEN이 크면 2
    # --------------------------------------------------------
    color_id = (
        0
        if largest < min_area or largest < 1.5 * min(areas)
        else int(np.argmax(areas)) + 1
    )

    # 디버깅용 상세 정보까지 요청한 경우
    if return_details:
        return color_id, areas, (dx, dy, w - dx, h - dy)

    # 일반적인 경우 색상 ID만 반환
    return color_id


# ============================================================
# ROS 2 Color Detector Node
# ============================================================
class ColorDetector(Node):

    def __init__(self):
        # ----------------------------------------------------
        # ROS 2 노드 이름
        # ----------------------------------------------------
        super().__init__('m0609_color_detector')

        # ----------------------------------------------------
        # ROS 2 Parameter 선언
        #
        # roi_fraction:
        #   영상 중앙에서 검사할 ROI 크기
        #
        # min_area:
        #   색상으로 인정할 최소 픽셀 영역
        # ----------------------------------------------------
        self.declare_parameter('roi_fraction', 0.18)
        self.declare_parameter('min_area', 150)

        # Parameter 값 가져오기
        self.roi_fraction = float(
            self.get_parameter('roi_fraction').value
        )

        self.min_area = int(
            self.get_parameter('min_area').value
        )

        # ----------------------------------------------------
        # Parameter 값 유효성 검사
        # ----------------------------------------------------
        if not 0 < self.roi_fraction <= 1 or self.min_area < 1:
            raise ValueError(
                'roi_fraction must be in (0, 1]; '
                'min_area must be positive'
            )

        # ----------------------------------------------------
        # ROS Image ↔ OpenCV 이미지 변환용 CvBridge
        # ----------------------------------------------------
        self.bridge = CvBridge()

        # ----------------------------------------------------
        # 색상 판별 결과 Publisher
        #
        # /color_id
        #
        # 0 = UNKNOWN
        # 1 = BLUE
        # 2 = GREEN
        # ----------------------------------------------------
        self.publisher = self.create_publisher(
            Int32,
            '/color_id',
            qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # RGB 카메라 Subscriber
        #
        # /rgb 토픽에서 Image 메시지를 받으면
        # on_image() 함수 실행
        # ----------------------------------------------------
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.on_image,
            qos_profile_sensor_data
        )

        # ----------------------------------------------------
        # 디버깅 영상 Publisher
        #
        # ROI 사각형과 색상 검출 정보를 영상 위에 표시한다.
        # ----------------------------------------------------
        self.debug_publisher = self.create_publisher(
            Image,
            '/color_debug',
            qos_profile_sensor_data
        )

        # 마지막 상세 로그 출력 시간
        self.last_report = -float('inf')

        # 이전에 검출된 color_id
        # 색상이 변경되었을 때만 로그를 출력하기 위해 사용
        self.last_id = None

        # 시작 로그
        self.get_logger().info(
            f'DOMAIN={os.environ.get("ROS_DOMAIN_ID")}; '
            f'/rgb -> /color_id; '
            f'debug=/color_debug; '
            f'blue=1, green=2, unknown=0'
        )

    # ========================================================
    # RGB 이미지 Callback
    # ========================================================
    def on_image(self, msg):
        """
        /rgb 토픽에서 이미지가 들어올 때마다 실행되는 Callback.

        처리 순서:
            1. ROS Image → OpenCV BGR 이미지 변환
            2. BLUE / GREEN 색상 판별
            3. 디버깅 영상 생성
            4. /color_debug Publish
            5. /color_id Publish
        """

        try:
            # ------------------------------------------------
            # 1. ROS Image → OpenCV BGR 이미지
            # ------------------------------------------------
            bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # ------------------------------------------------
            # 2. 색상 판별
            #
            # return_details=True이므로
            #
            # color_id : 최종 색상 ID
            # areas    : [BLUE 면적, GREEN 면적]
            # box      : ROI 좌표
            # ------------------------------------------------
            color_id, areas, box = detect_color(
                bgr,
                self.roi_fraction,
                self.min_area,
                True
            )

            # ------------------------------------------------
            # 3. 원본 영상을 복사하여 디버깅 영상 생성
            # ------------------------------------------------
            debug = bgr.copy()

            # ROI 영역 좌표
            x0, y0, x1, y1 = box

            # ------------------------------------------------
            # ROI 영역을 빨간색 사각형으로 표시
            # ------------------------------------------------
            cv2.rectangle(
                debug,
                (x0, y0),
                (x1 - 1, y1 - 1),
                (0, 0, 255),
                2
            )

            # ------------------------------------------------
            # 영상 위에 표시할 검출 정보
            #
            # 예:
            # id=1 blue=350 green=0 min=150
            # ------------------------------------------------
            label = (
                f'id={color_id} '
                f'blue={areas[0]} '
                f'green={areas[1]} '
                f'min={self.min_area}'
            )

            # 검출 결과 텍스트 표시
            cv2.putText(
                debug,
                label,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                1
            )

            # ------------------------------------------------
            # 4. OpenCV 이미지 → ROS Image 메시지
            # ------------------------------------------------
            debug_msg = self.bridge.cv2_to_imgmsg(
                debug,
                encoding='bgr8'
            )

            # 원본 이미지의 시간 / frame 정보 유지
            debug_msg.header = msg.header

            # 디버깅 영상 Publish
            self.debug_publisher.publish(debug_msg)

            # ------------------------------------------------
            # 5. 상세 검출 로그
            #
            # 너무 많은 로그가 출력되지 않도록
            # 3초에 한 번만 출력한다.
            # ------------------------------------------------
            if time.monotonic() - self.last_report >= 3:
                self.get_logger().info(
                    f'{label}; '
                    f'encoding={msg.encoding}; '
                    f'frame={msg.header.frame_id}'
                )

                self.last_report = time.monotonic()

        # ----------------------------------------------------
        # 이미지 변환 / OpenCV 처리 중 오류가 발생하면
        # UNKNOWN(0)으로 처리
        # ----------------------------------------------------
        except (CvBridgeError, ValueError, cv2.error) as exc:
            self.get_logger().warning(
                f'Invalid camera image: {exc}'
            )

            color_id = 0

        # ----------------------------------------------------
        # 최종 색상 ID Publish
        #
        # /color_id
        #
        # 0 = UNKNOWN
        # 1 = BLUE
        # 2 = GREEN
        # ----------------------------------------------------
        self.publisher.publish(
            Int32(data=color_id)
        )

        # ----------------------------------------------------
        # 이전 검출값과 다른 경우에만 로그 출력
        #
        # 예:
        # color_id=1
        # ----------------------------------------------------
        if color_id != self.last_id:
            self.get_logger().info(
                f'color_id={color_id}'
            )

            self.last_id = color_id


# ============================================================
# Main
# ============================================================
def main(args=None):

    # --------------------------------------------------------
    # ROS_DOMAIN_ID가 외부에서 설정되어 있지 않은 경우에만
    # 기본값 102 사용
    # --------------------------------------------------------
    os.environ.setdefault(
        'ROS_DOMAIN_ID',
        '102'
    )

    # ROS 2 초기화
    rclpy.init(args=args)

    # ColorDetector 노드 생성
    node = ColorDetector()

    try:
        # 노드 실행
        # /rgb 이미지가 들어올 때마다 on_image() 실행
        rclpy.spin(node)

    # Ctrl + C로 종료하는 경우
    except KeyboardInterrupt:
        pass

    finally:
        # 노드 제거
        node.destroy_node()

        # ROS 2 종료
        rclpy.shutdown()


# ============================================================
# 이 파일을 직접 실행했을 때 main() 실행
# ============================================================
if __name__ == '__main__':
    main()

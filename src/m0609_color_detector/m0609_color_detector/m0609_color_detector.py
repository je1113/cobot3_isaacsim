"""PC B: /rgb (sensor_msgs/Image) 를 받아 HSV 로 파랑/초록을 감지하고 /color_id (std_msgs/Int32) 발행.

  0 = 미감지, 1 = 파랑, 2 = 초록

실행:  ros2 run m0609_color_detector m0609_color_detector
디버그 창:  ros2 run m0609_color_detector m0609_color_detector --ros-args -p show_window:=true
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

COLOR_NONE, COLOR_BLUE, COLOR_GREEN = 0, 1, 2
NAMES = {COLOR_NONE: "미감지", COLOR_BLUE: "파랑 →Place 1", COLOR_GREEN: "초록 →Place 2"}

# OpenCV HSV 범위: H 0~179, S 0~255, V 0~255
# 순색 파랑(0,0,255) H=120, 순색 초록(0,255,0) H=60. 실제 /rgb 보면서 조정.
BLUE_LOWER, BLUE_UPPER = (100, 100, 50), (130, 255, 255)
GREEN_LOWER, GREEN_UPPER = (40, 100, 50), (80, 255, 255)


def image_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → OpenCV BGR 배열 (cv_bridge 불필요)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return buf.reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgba8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2BGR)
    if msg.encoding == "bgra8":
        return buf.reshape(msg.height, msg.width, 4)[:, :, :3]
    raise ValueError(f"지원하지 않는 encoding: {msg.encoding}")


def detect_color(bgr: np.ndarray, min_area: int):
    """BGR 이미지 → (color_id, blue_area, green_area, blue_mask, green_mask)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(hsv, np.array(BLUE_LOWER), np.array(BLUE_UPPER))
    green_mask = cv2.inRange(hsv, np.array(GREEN_LOWER), np.array(GREEN_UPPER))

    # 작은 노이즈 점 제거
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    blue_area = int(cv2.countNonZero(blue_mask))
    green_area = int(cv2.countNonZero(green_mask))

    color_id = COLOR_NONE
    if max(blue_area, green_area) >= min_area:
        color_id = COLOR_BLUE if blue_area > green_area else COLOR_GREEN
    return color_id, blue_area, green_area, blue_mask, green_mask


class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector_node")
        self.declare_parameter("min_area", 300)        # 픽셀 수. 640x640 기준
        self.declare_parameter("show_window", False)
        self._min_area = self.get_parameter("min_area").value
        self._show = self.get_parameter("show_window").value

        self.pub = self.create_publisher(Int32, "/color_id", 10)
        self.sub = self.create_subscription(Image, "/rgb", self.image_callback, 10)
        self.get_logger().info("color_detector 시작: /rgb → /color_id")

    def image_callback(self, msg: Image):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        color_id, blue_area, green_area, blue_mask, green_mask = detect_color(bgr, self._min_area)

        self.pub.publish(Int32(data=color_id))
        self.get_logger().info(
            f"color_id = {color_id} ({NAMES[color_id]})  blue_area={blue_area}  green_area={green_area}"
        )

        if self._show:
            self._draw(bgr, blue_mask, green_mask, color_id)

    @staticmethod
    def _draw(bgr, blue_mask, green_mask, color_id):
        vis = bgr.copy()
        for mask, col in ((blue_mask, (255, 0, 0)), (green_mask, (0, 255, 0))):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, contours, -1, col, 2)
        cv2.putText(vis, f"color_id = {color_id}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("color_detector", vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

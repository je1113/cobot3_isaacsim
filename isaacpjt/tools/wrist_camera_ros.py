"""
손목 카메라(rsd455)를 ROS2 로 내보낸다 — image + camera_info.

씬의 ROS2 카메라 그래프는 Carter 의 hawk 카메라에만 있고, 그마저도
simple_factory_layout.usda 가 over 로 연결을 끊어 놨다. 손목 카메라는 아예
그래프가 없어서 /camera_info 를 받을 수가 없다. 이 스크립트가 그 그래프를
런타임에 만든다.

단독 실행:
    ros_set                       # /opt/ros/jazzy 소싱
    isaac_ros                     # Isaac ROS2 bridge 라이브러리 경로 등록
    isaac_python isaacpjt/tools/wrist_camera_ros.py

    # 다른 터미널에서
    ros2 topic echo /wrist_camera/color/camera_info --once
    ros2 topic hz   /wrist_camera/color/image_raw

다른 스크립트에서 쓰려면:
    from wrist_camera_ros import attach_wrist_camera
    attach_wrist_camera(camera_prim_path=..., namespace="wrist_camera")

토픽 이름과 프레임 이름은 src/cobot3_bringup/config/frames.yaml 의
wrist_camera 항목과 맞춰 놨다.
"""

import os

DEFAULT_WIDTH  = int(os.environ.get("WRIST_CAM_W", "1280"))
DEFAULT_HEIGHT = int(os.environ.get("WRIST_CAM_H", "720"))

# frames.yaml 의 wrist_camera 와 같은 값
CAMERA_PRIM = ("/World/Robots/nova_carter1/m0609/short_gripper/rsd455/RSD455/"
               "Camera_OmniVision_OV9782_Color")
NAMESPACE   = "wrist_camera"
FRAME_ID    = "camera_color_optical_frame"
GRAPH_PATH  = "/World/Graphs/WristCamera"


def expected_intrinsics(stage, camera_prim_path, width, height):
    """USD 카메라 파라미터에서 K 를 계산한다.

    camera_info 로 실제로 받는 값과 대조하라고 만든 것이다. 파이프라인에서는
    이 계산값이 아니라 camera_info 토픽 값을 써야 한다 (checklist).
    """
    from pxr import UsdGeom

    cam = UsdGeom.Camera(stage.GetPrimAtPath(camera_prim_path))
    if not cam:
        return None
    f  = cam.GetFocalLengthAttr().Get()
    ha = cam.GetHorizontalApertureAttr().Get()
    va = cam.GetVerticalApertureAttr().Get()
    fx = f * width / ha
    fy = f * height / va
    return {
        "focal_length_mm": f,
        "horizontal_aperture_mm": ha,
        "vertical_aperture_mm": va,
        "width": width, "height": height,
        "fx": fx, "fy": fy, "cx": width / 2.0, "cy": height / 2.0,
        # 정사각 픽셀이 되려면 va 가 이 값이어야 한다
        "vertical_aperture_for_square_px": ha * height / width,
        "square_pixels": abs(fx - fy) < 1e-6,
    }


def attach_wrist_camera(camera_prim_path=CAMERA_PRIM,
                        namespace=NAMESPACE,
                        frame_id=FRAME_ID,
                        graph_path=GRAPH_PATH,
                        width=DEFAULT_WIDTH,
                        height=DEFAULT_HEIGHT,
                        publish_depth=True):
    """손목 카메라용 ROS2 퍼블리시 그래프를 만든다. 그래프 핸들을 돌려준다.

    뷰포트를 쓰지 않고 IsaacCreateRenderProduct 로 오프스크린 렌더 프로덕트를
    만든다. 헤드리스에서도 그대로 돈다.
    """
    import omni.graph.core as og
    import omni.usd
    import usdrt.Sdf
    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(camera_prim_path).IsValid():
        raise RuntimeError(f"카메라 프림이 없다: {camera_prim_path}")

    keys = og.Controller.Keys
    nodes = [
        ("OnTick",        "omni.graph.action.OnPlaybackTick"),
        ("Context",       "isaacsim.ros2.bridge.ROS2Context"),
        ("RunOneFrame",   "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("PubRgb",        "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("PubInfo",       "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ]
    connect = [
        ("OnTick.outputs:tick",                     "RunOneFrame.inputs:execIn"),
        ("RunOneFrame.outputs:step",                "RenderProduct.inputs:execIn"),
        ("RenderProduct.outputs:execOut",           "PubRgb.inputs:execIn"),
        ("RenderProduct.outputs:execOut",           "PubInfo.inputs:execIn"),
        ("RenderProduct.outputs:renderProductPath", "PubRgb.inputs:renderProductPath"),
        ("RenderProduct.outputs:renderProductPath", "PubInfo.inputs:renderProductPath"),
        ("Context.outputs:context",                 "PubRgb.inputs:context"),
        ("Context.outputs:context",                 "PubInfo.inputs:context"),
    ]
    values = [
        ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
        ("RenderProduct.inputs:width",  width),
        ("RenderProduct.inputs:height", height),
        ("PubRgb.inputs:type",           "rgb"),
        ("PubRgb.inputs:topicName",      "color/image_raw"),
        ("PubRgb.inputs:frameId",        frame_id),
        ("PubRgb.inputs:nodeNamespace",  namespace),
        ("PubInfo.inputs:topicName",     "color/camera_info"),
        ("PubInfo.inputs:frameId",       frame_id),
        ("PubInfo.inputs:nodeNamespace", namespace),
    ]

    if publish_depth:
        nodes.append(("PubDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connect += [
            ("RenderProduct.outputs:execOut",           "PubDepth.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "PubDepth.inputs:renderProductPath"),
            ("Context.outputs:context",                 "PubDepth.inputs:context"),
        ]
        values += [
            ("PubDepth.inputs:type",          "depth"),
            ("PubDepth.inputs:topicName",     "depth/image_rect_raw"),
            ("PubDepth.inputs:frameId",       frame_id),
            ("PubDepth.inputs:nodeNamespace", namespace),
        ]

    graph, _, _, _ = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connect, keys.SET_VALUES: values},
    )
    return graph


# ══════════════════════════════════════════════════════════════
#  단독 실행
# ══════════════════════════════════════════════════════════════
def _main():
    import sys
    from pathlib import Path

    from isaacsim import SimulationApp

    headless = os.environ.get("WRIST_CAM_HEADLESS", "1") == "1"
    app = SimulationApp({"headless": headless})

    import omni.usd
    from isaacsim.core.api import SimulationContext

    sys.stdout.reconfigure(line_buffering=True)

    world_usd = str(Path(__file__).resolve().parent.parent /
                    "worlds/simple_factory_layout.usda")

    ctx = omni.usd.get_context()
    ctx.open_stage(world_usd)
    for _ in range(60):
        app.update()
    stage = ctx.get_stage()
    stage.Load()
    for _ in range(60):
        app.update()

    attach_wrist_camera()
    app.update()

    k = expected_intrinsics(stage, CAMERA_PRIM, DEFAULT_WIDTH, DEFAULT_HEIGHT)
    print("=" * 72)
    print("  손목 카메라 ROS2 발행 시작")
    print("=" * 72)
    print(f"   카메라 프림  {CAMERA_PRIM}")
    print(f"   해상도       {DEFAULT_WIDTH} x {DEFAULT_HEIGHT}")
    print(f"   frame_id     {FRAME_ID}")
    print(f"   토픽         /{NAMESPACE}/color/image_raw")
    print(f"                /{NAMESPACE}/color/camera_info")
    print(f"                /{NAMESPACE}/depth/image_rect_raw")
    if k:
        print()
        print("   USD 파라미터로 계산한 K (camera_info 와 대조용):")
        print(f"       fx {k['fx']:.3f}   fy {k['fy']:.3f}   "
              f"cx {k['cx']:.1f}   cy {k['cy']:.1f}")
        if not k["square_pixels"]:
            print(f"       주의 — fx != fy 다. 에셋 aperture 비"
                  f"({k['horizontal_aperture_mm']}:{k['vertical_aperture_mm']})와")
            print(f"       해상도 비({DEFAULT_WIDTH}:{DEFAULT_HEIGHT})가 안 맞는다.")
            print(f"       정사각 픽셀을 원하면 vertical aperture 를 "
                  f"{k['vertical_aperture_for_square_px']:.4f} mm 로 바꿔라.")
    print()
    print("   다른 터미널에서:")
    print(f"       ros2 topic echo /{NAMESPACE}/color/camera_info --once")
    print()
    print("   Ctrl-C 로 종료.")

    sim = SimulationContext(stage_units_in_meters=1.0)
    sim.play()
    try:
        while app.is_running():
            sim.step(render=True)
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    _main()

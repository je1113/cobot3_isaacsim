"""
서피스(흡착) 그리퍼를 M0609 에 붙이고 열고 닫기

    isaac_python 8_surface_gripper.py

RG2 는 URDF 를 임포트해 관절을 구동하지만, 서피스 그리퍼는 관절이 없다.
닿은 물체를 조인트로 붙여 버리는 이상화된 구속이라 Open/Close 명령만 있다.

붙이는 방식은 RG2 와 같다. 그리퍼를 /World/m0609 아래 형제로 두고
link_6 과 FixedJoint 로 묶는다 (Robot Assembler 가 하는 것과 동일).

  link_6 ──FixedJoint(-90 deg about Y)── surface_gripper(/Root, RigidBody 1.0 kg)
                                            └─ SurfaceGripper (IsaacSurfaceGripper)
                                            └─ suction_cup/Suction_Joint (attachment point)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.storage.native import get_assets_root_path


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")

# NVIDIA 기본 에셋. 로컬에 받아 뒀으면 그 경로를 넣어도 된다.
GRIPPER_USD = (get_assets_root_path()
               + "/Isaac/Robots/UniversalRobots/ur10/grippers/short_gripper.usd")

ROBOT_PRIM = "/World/m0609"
EE_LINK = f"{ROBOT_PRIM}/link_6"
GRIPPER_PRIM = f"{ROBOT_PRIM}/surface_gripper"
GRIPPER_NODE = f"{GRIPPER_PRIM}/SurfaceGripper"


# ══════════════════════════════════════════════════════════════
#  그리퍼 장착 파라미터
# ══════════════════════════════════════════════════════════════
# short_gripper 는 로컬 +X 로 뻗는다. 로봇 툴축은 link_6 로컬 +Z 이므로
# Y축 -90 도 돌려 +X 를 +Z 에 맞춘다.
MOUNT_QUAT = Gf.Quatf(0.70710678, Gf.Vec3f(0.0, -0.70710678, 0.0))

# 장착 오프셋. 마운트 판이 x=-2~8mm 라 0 이면 2mm 가 link_6 안으로 들어간다.
# 딱 맞추고 싶으면 0.002 를 준다.
MOUNT_OFFSET = Gf.Vec3f(0.0, 0.0, 0.0)

# 흡착면까지의 거리 (link_6 로컬 +Z). gripper_tip 바깥면이 로컬 x=161mm.
TCP_OFFSET = np.array([0.0, 0.0, 0.161])

# 파지 한계. 에셋 기본값은 0 = 무제한이라 아무거나 다 들어 올린다.
#   지름 30mm 컵을 -0.6bar 로 당길 때 이론 42N, 실용 절반 기준
COAXIAL_FORCE_LIMIT = 20.0   # N, 흡착면 수직 방향
SHEAR_FORCE_LIMIT = 10.0     # N, 흡착면 평행 방향
MAX_GRIP_DISTANCE = 0.02     # m, 이 거리 안에 물체가 있으면 붙는다

CYCLE_STEPS = 180


# ══════════════════════════════════════════════════════════════
#  씬 구성
# ══════════════════════════════════════════════════════════════
def load_robot(stage):
    """조립된 M0609 USD 를 /World 아래에 참조로 올린다"""
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(USD_PATH)
    for _ in range(15):
        simulation_app.update()


def attach_surface_gripper(stage):
    """서피스 그리퍼를 참조로 올리고 link_6 에 FixedJoint 로 묶는다"""
    # 1) 그리퍼를 로봇 아래 형제 prim 으로 참조
    #    short_gripper.usd 의 defaultPrim 이 /Root 라, 이 prim 이 곧 RigidBody 가 된다
    grip = UsdGeom.Xform.Define(stage, GRIPPER_PRIM)
    grip.GetPrim().GetReferences().AddReference(GRIPPER_USD)
    simulation_app.update()

    # 2) 물리가 스냅하기 전에 시각 위치를 미리 맞춰 둔다
    #    gripper_world = link6_world * (offset, MOUNT_QUAT)
    cache = UsdGeom.XformCache()
    link6_world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(EE_LINK))
    local = Gf.Matrix4d().SetRotate(Gf.Quatd(MOUNT_QUAT))
    local.SetTranslateOnly(Gf.Vec3d(MOUNT_OFFSET))
    world = local * link6_world

    xf = UsdGeom.Xformable(grip.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(world)

    # 3) FixedJoint. localRot0 에 마운트 회전을 넣으면
    #    link6_frame * MOUNT_QUAT == gripper_frame 이 강제된다
    joint = UsdPhysics.FixedJoint.Define(stage, f"{EE_LINK}/surface_gripper_joint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(EE_LINK)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(GRIPPER_PRIM)])
    joint.CreateLocalPos0Attr().Set(MOUNT_OFFSET)
    joint.CreateLocalRot0Attr().Set(MOUNT_QUAT)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))

    # 4) 파지 한계. 이걸 안 넣으면 무게 제한 없이 다 붙는다
    node = stage.GetPrimAtPath(GRIPPER_NODE)
    node.GetAttribute("isaac:coaxialForceLimit").Set(COAXIAL_FORCE_LIMIT)
    node.GetAttribute("isaac:shearForceLimit").Set(SHEAR_FORCE_LIMIT)
    node.GetAttribute("isaac:maxGripDistance").Set(MAX_GRIP_DISTANCE)

    print(f"[attach] {GRIPPER_PRIM} -> {EE_LINK}")
    print(f"[attach] coaxial={COAXIAL_FORCE_LIMIT}N shear={SHEAR_FORCE_LIMIT}N "
          f"maxDist={MAX_GRIP_DISTANCE*1000:.0f}mm")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
def main():
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    load_robot(stage)
    attach_surface_gripper(stage)

    world.reset()

    # 서피스 그리퍼 제어. 관절이 없으므로 Open/Close 두 가지뿐이다
    from isaacsim.robot.surface_gripper import GripperView

    gripper = GripperView(paths=GRIPPER_NODE)

    closed = False
    step = 0
    while simulation_app.is_running():
        world.step(render=True)
        step += 1

        if step % CYCLE_STEPS == 0:
            closed = not closed
            # 양수 = 닫기(흡착), 음수 = 열기(해제)
            gripper.apply_gripper_action(np.array([1.0 if closed else -1.0]))
            print(f"[{step:5d}] {'CLOSE (흡착)' if closed else 'OPEN (해제)'}")

        if step % 60 == 0:
            status = gripper.get_surface_gripper_status()
            gripped = gripper.get_gripped_objects()
            print(f"        status={status} gripped={gripped}")

    simulation_app.close()


if __name__ == "__main__":
    main()

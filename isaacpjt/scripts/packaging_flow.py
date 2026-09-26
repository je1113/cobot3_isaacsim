"""Runtime packaging flow for simple_factory_layout.usda.

Run this file once from Isaac Sim's Script Editor after opening the stage.
The USD contains the conveyors, trigger volumes, and shelf.  This controller
implements the conditional spawn and shelf transfer behavior.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


MAGAZINE_ROOT = Sdf.Path("/World/Magazines")
STACK_ROOT = Sdf.Path("/World/Environment/PackagingUnloaderZone/SpawnedStacks")

INPUT_CENTER = Gf.Vec3d(7.45, 4.60, 0.80)
INPUT_HALF_EXTENT = Gf.Vec3d(0.40, 0.50, 0.60)

STACK_SPAWN = Gf.Vec3d(7.45, 2.80, 0.64)
UNLOADER_SPEED_MPS = 0.20
OUTPUT_TRIGGER_X = 4.20

# ★ Measured 2026-09-24 (Isaac Sim, 15_stack_scan_pick_test.py) — the values
#   below (3.25 / 0.64 / 2.25..) placed stacks nowhere near the real shelf
#   collider (/World/Environment/PackagingUnloaderZone/OutputShelf/ShelfDeck,
#   world x [3.876,4.051], y [0.271,1.671], top z 0.54): they landed ~0.6 m
#   off in x and fell through the floor once physics actually loaded (no
#   collider under the old spot). Corrected to ShelfDeck's own local
#   translate (siblings under the same parent offset, so ShelfDeck's local
#   x/y = the right local x/y here too), then nudged +0.08 m further east
#   (toward BeltTop/ConveyorFrame, world x [4.05,8.75]) per user direction so
#   a freshly-arrived stack sits nearer the conveyor-facing edge of the deck.
#   Only slot 0 (SHELF_SLOT_Y[0]=2.80, world y 0.971) was actually verified
#   end-to-end (SCAN QR decode 5/5, PICK 3/5) in Isaac Sim; slots 1-3 got the
#   same constant y-delta (+0.55) to keep the round-robin spacing intact, but
#   slot 3 (local y 3.91) falls right at/past ShelfDeck's local y upper bound
#   (~3.50) — re-verify before relying on it.
SHELF_X = 3.963325659785515 + 0.08
SHELF_Z = 0.54
SHELF_SLOT_Y = (2.80, 3.17, 3.54, 3.91)

ORANGE_STACK_PAYLOAD = "../assets/F3_STKB_1.usda"
BLUE_STACK_PAYLOAD = "../assets/F3_STKO_1.usda"


@dataclass
class MovingStack:
    path: Sdf.Path
    x: float


class PackagingFlowController:
    def __init__(self) -> None:
        self._stage = None
        self._subscription = None
        self._stop_subscription = None
        self._last_sim_time = None
        self._processed_magazines: set[str] = set()
        self._moving_stacks: dict[str, MovingStack] = {}
        self._stack_ids = itertools.count(1)
        self._shelf_index = 0

    def start(self) -> None:
        self.stop()
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            raise RuntimeError("Open simple_factory_layout.usda before starting packaging_flow.py")

        stream = omni.kit.app.get_app().get_update_event_stream()
        self._subscription = stream.create_subscription_to_pop(
            self._on_update,
            name="PackagingFlowController",
        )
        # Stop only rewinds what physics moved. Stacks are authored straight
        # into USD (DefinePrim), so Stop leaves them behind and the next Play
        # starts with last run's stacks still on the belt/shelf. Clear them
        # ourselves, the same way magazine_spawner.on_stop clears magazines.
        self._stop_subscription = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP),
                self._on_timeline_stop,
                name="PackagingFlowController.stop",
            )
        )
        print("[PackagingFlow] Started")

    def stop(self) -> None:
        self._subscription = None
        self._stop_subscription = None
        self._last_sim_time = None

    def _on_timeline_stop(self, _event) -> None:
        removed = self._clear_stacks()
        self._processed_magazines.clear()
        self._moving_stacks.clear()
        self._stack_ids = itertools.count(1)
        self._shelf_index = 0
        self._last_sim_time = None
        print(f"[PackagingFlow] Stop - removed {removed} stack(s), flow state reset")

    def _clear_stacks(self) -> int:
        """Remove every stack this flow spawned under STACK_ROOT.

        STACK_ROOT itself is defined in the world USD and stays. Children are
        matched by the "stack_" prefix rather than tracked in memory so stacks
        from an earlier run of this script (re-run in Script Editor) go too.
        """
        stage = self._stage or omni.usd.get_context().get_stage()
        if stage is None:
            return 0
        root = stage.GetPrimAtPath(STACK_ROOT)
        if not root:
            return 0

        paths = [
            child.GetPath()
            for child in root.GetAllChildren()
            if child.GetName().startswith("stack_")
        ]
        for path in paths:
            # The spec may live in whichever layer was the edit target at spawn
            # time, so remove it from every editable layer that holds it.
            for layer in stage.GetLayerStack():
                if layer.permissionToEdit and layer.GetPrimAtPath(path):
                    edits = Sdf.BatchNamespaceEdit()
                    edits.Add(path, Sdf.Path.emptyPath)
                    if not layer.Apply(edits):
                        print(f"[PackagingFlow] Could not remove {path} from {layer.identifier}")
        return len(paths)

    def _on_update(self, _event) -> None:
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            self._last_sim_time = None
            return

        sim_time = float(timeline.get_current_time())
        if self._last_sim_time is None:
            self._last_sim_time = sim_time
            return

        dt = sim_time - self._last_sim_time
        self._last_sim_time = sim_time
        if dt <= 0.0:
            return

        self._detect_packaged_magazines()
        self._advance_stacks(min(dt, 0.1))

    @staticmethod
    def _inside_box(position: Gf.Vec3d, center: Gf.Vec3d, half_extent: Gf.Vec3d) -> bool:
        return all(
            abs(float(position[i] - center[i])) <= float(half_extent[i])
            for i in range(3)
        )

    def _world_position(self, prim: Usd.Prim) -> Gf.Vec3d:
        matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
        return Gf.Vec3d(matrix.ExtractTranslation())

    def _detect_packaged_magazines(self) -> None:
        root = self._stage.GetPrimAtPath(MAGAZINE_ROOT)
        if not root:
            return

        for prim in Usd.PrimRange(root):
            name_lower = prim.GetName().lower()
            path_text = str(prim.GetPath())
            if not prim.IsActive() or "magazine" not in name_lower:
                continue
            if path_text in self._processed_magazines:
                continue

            position = self._world_position(prim)
            if not self._inside_box(position, INPUT_CENTER, INPUT_HALF_EXTENT):
                continue

            if "orange" in name_lower:
                payload = ORANGE_STACK_PAYLOAD
            elif "blue" in name_lower:
                payload = BLUE_STACK_PAYLOAD
            else:
                print(f"[PackagingFlow] Ignored magazine without orange/blue in name: {path_text}")
                self._processed_magazines.add(path_text)
                continue

            self._spawn_stack(prim, payload)
            self._processed_magazines.add(path_text)
            prim.SetActive(False)

    def _spawn_stack(self, magazine_prim: Usd.Prim, payload: str) -> None:
        stack_number = next(self._stack_ids)
        source_name = magazine_prim.GetName().replace("-", "_")
        stack_path = STACK_ROOT.AppendChild(f"stack_{stack_number:03d}_{source_name}")
        stack = self._stage.DefinePrim(stack_path, "Xform")
        stack.GetPayloads().AddPayload(payload)
        stack.CreateAttribute("flow:sourceMagazine", Sdf.ValueTypeNames.String).Set(
            str(magazine_prim.GetPath())
        )
        stack.CreateAttribute("flow:stackPayload", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(payload))
        UsdGeom.XformCommonAPI(stack).SetTranslate(STACK_SPAWN)
        xform = UsdGeom.XformCommonAPI(stack)
        xform.SetTranslate(STACK_SPAWN)
        xform.SetRotate(Gf.Vec3f(0.0, 0.0, 90.0))
        # ★ 2026-09-26: 2.0 -> 1.0 (원본 에셋 크기 그대로) — 15_stack_scan_
        #   pick_test.py 에서 절반 크기로 SCAN(크립 여유거리)·PICK(흡착+LIFT)
        #   전부 성공 확인 후 사용자 지시로 production 에도 적용한다. 질량도
        #   부피비(0.5^3)만큼 같이 낮춘다 — physics:mass=1(kg) 이 F3_STKB_1/
        #   F3_STKO_1.usda 양쪽 다 스케일과 무관한 명시적 오버라이드라, 안
        #   낮추면 밀도가 8배가 되어(그 테스트에서 실제로 LIFT 도중 놓치는
        #   원인이었다) 흡착이 훨씬 불리해진다.
        #   ★★ 주의 — shelves.yaml PKG-OUT 의 waypoint_start/arm_teach_pose
        #   는 지금까지의 2.0 배 스택 크기에 맞춰 사람이 직접 티칭한 값이다
        #   (QR 라벨 위치·카메라 시야가 스택 크기에 따라 달라진다). 스택이
        #   작아졌으니 그 자세들도 다시 확인/재티칭해야 실제 회수 흐름이
        #   맞물린다 — 이 커밋만으로는 그 재티칭까지 포함하지 않는다.
        xform.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
        UsdPhysics.MassAPI(stack).CreateMassAttr().Set(1.0 * (0.5 ** 3))
        self._moving_stacks[str(stack_path)] = MovingStack(stack_path, float(STACK_SPAWN[0]))
        print(f"[PackagingFlow] {magazine_prim.GetName()} -> {payload}")

    def _advance_stacks(self, dt: float) -> None:
        completed = []
        for path_text, moving in self._moving_stacks.items():
            prim = self._stage.GetPrimAtPath(moving.path)
            if not prim or not prim.IsActive():
                completed.append(path_text)
                continue

            moving.x -= UNLOADER_SPEED_MPS * dt
            if moving.x <= OUTPUT_TRIGGER_X:
                self._move_to_shelf(prim)
                completed.append(path_text)
            else:
                UsdGeom.XformCommonAPI(prim).SetTranslate(
                    Gf.Vec3d(moving.x, STACK_SPAWN[1], STACK_SPAWN[2])
                )

        for path_text in completed:
            self._moving_stacks.pop(path_text, None)

    def _move_to_shelf(self, stack_prim: Usd.Prim) -> None:
        slot_y = SHELF_SLOT_Y[self._shelf_index % len(SHELF_SLOT_Y)]
        layer = self._shelf_index // len(SHELF_SLOT_Y)
        shelf_position = Gf.Vec3d(SHELF_X, slot_y, SHELF_Z + 0.30 * layer)
        UsdGeom.XformCommonAPI(stack_prim).SetTranslate(shelf_position)
        stack_prim.CreateAttribute("flow:onShelf", Sdf.ValueTypeNames.Bool).Set(True)
        self._shelf_index += 1
        print(f"[PackagingFlow] Moved {stack_prim.GetName()} to shelf slot {self._shelf_index}")


try:
    PACKAGING_FLOW.stop()
except (NameError, AttributeError):
    pass

PACKAGING_FLOW = PackagingFlowController()
PACKAGING_FLOW.start()


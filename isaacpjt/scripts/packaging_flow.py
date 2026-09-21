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
from pxr import Gf, Sdf, Usd, UsdGeom


MAGAZINE_ROOT = Sdf.Path("/World/Magazines")
STACK_ROOT = Sdf.Path("/World/Environment/PackagingUnloaderZone/SpawnedStacks")

INPUT_CENTER = Gf.Vec3d(7.45, 4.60, 0.80)
INPUT_HALF_EXTENT = Gf.Vec3d(0.40, 0.50, 0.60)

STACK_SPAWN = Gf.Vec3d(7.45, 2.80, 0.64)
UNLOADER_SPEED_MPS = 0.20
OUTPUT_TRIGGER_X = 4.20

SHELF_X = 3.25
SHELF_Z = 0.64
SHELF_SLOT_Y = (2.25, 2.62, 2.99, 3.36)

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
        print("[PackagingFlow] Started")

    def stop(self) -> None:
        self._subscription = None
        self._last_sim_time = None

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


"""
매거진 스폰
"""
import itertools
import random

import carb
from omni.physx import get_physx_interface
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

try:  # Isaac Sim 5.1
    from omni.kit.scripting import BehaviorScript
except ImportError:  # Newer Kit behavior extension
    from omni.behavior.scripting.core import BehaviorScript


def transformed_range(bounds, matrix):
    result = Gf.Range3d()
    for i in range(8):
        result.UnionWith(matrix.Transform(bounds.GetCorner(i)))
    return result


def classify_bounds(bounds):
    """Any complete exit from the region counts, including falling below it."""
    lo, hi = bounds.GetMin(), bounds.GetMax()
    if any(hi[i] < -0.51 or lo[i] > 0.51 for i in range(3)):
        return "outside"
    return "inside"


class PickupGate:
    """One release per magazine, after a continuous confirmed absence."""
    def __init__(self, confirm_seconds=0.3, warmup_seconds=0.5):
        self.confirm_seconds = confirm_seconds
        self.warmup_seconds = warmup_seconds
        self.age = 0.0
        self.absent_time = 0.0
        self.seen_inside = False
        self.released = False

    def update(self, observation, dt):
        if self.released:
            return False
        self.age += dt
        if observation == "inside":
            self.seen_inside = True
        if (self.age < self.warmup_seconds or not self.seen_inside
                or observation not in ("outside", "removed")):
            self.absent_time = 0.0
            return False
        self.absent_time += dt
        if self.absent_time + 1e-9 >= self.confirm_seconds:
            self.released = True
            return True
        return False


class MagazineSpawner(BehaviorScript):
    def on_init(self):
        self._runtime = None
        self._owned_paths = set()
        self._owned_parents = set()
        self._shelves = []
        self._needs_reset = True
        self._playing = False
        self._ids = itertools.count(1)
        self._rng = random.Random()
        self._physx = get_physx_interface()
        try:
            self._configure()
            self._runtime = self.stage.GetRootLayer()
            # Recover only this controller's objects if a running scene was saved.
            for shelf in self._shelves:
                parent = self.stage.GetPrimAtPath(shelf["root"])
                if parent:
                    for child in parent.GetAllChildren():
                        if child.GetAttribute("spawn:owner").Get() == str(self.prim_path):
                            self._owned_paths.add(child.GetPath())
            self._reset()  # Preview: one per shelf as soon as the scene script loads.
        except Exception as exc:
            carb.log_error(f"[MagazineSpawner] Initialization failed: {exc}")

    def _configure(self):
        controller = self.stage.GetPrimAtPath(self.prim_path)
        self._confirm = float(controller.GetAttribute("spawn:confirmSeconds").Get())
        self._warmup = float(controller.GetAttribute("spawn:warmupSeconds").Get())
        self._assets = {}
        for color in ("orange", "blue"):
            attr = controller.GetAttribute(f"spawn:{color}Asset")
            asset = attr.Get()
            if not asset or not asset.path:
                raise RuntimeError(f"Missing spawn:{color}Asset")
            # Resolve against the layer that authored the asset, never process cwd.
            spec = next(p for p in attr.GetPropertyStack() if p.HasDefaultValue())
            resolved = asset.resolvedPath or Sdf.ComputeAssetPathRelativeToLayer(spec.layer, asset.path)
            layer = Sdf.Layer.FindOrOpen(resolved)
            if not layer or not layer.defaultPrim:
                raise RuntimeError(f"Cannot load {color} asset/defaultPrim: {resolved}")
            self._assets[color] = resolved

        all_filters = []
        for prim in self.stage.Traverse():
            rel = prim.GetRelationship("physics:filteredPairs")
            if rel:
                all_filters.append((prim.GetPath(), rel.GetTargets()))

        for group_path in controller.GetRelationship("spawn:shelves").GetTargets():
            group = self.stage.GetPrimAtPath(group_path)
            slot_paths = group.GetRelationship("spawn:slots").GetTargets()
            region_paths = group.GetRelationship("spawn:pickupRegion").GetTargets()
            if len(slot_paths) != 4 or len(region_paths) != 1:
                raise RuntimeError(f"Expected four slots and one region: {group_path}")
            templates = []
            for slot in slot_paths:
                prim = self.stage.GetPrimAtPath(slot)
                if not prim or prim.IsActive():
                    raise RuntimeError(f"Source slot must exist and be inactive: {slot}")
                source = prim.GetPrimStack()[0]
                templates.append((source.layer, source.path,
                                  [p for p, targets in all_filters if slot in targets]))
            region = self.stage.GetPrimAtPath(region_paths[0])
            if not region or not region.IsA(UsdGeom.Cube):
                raise RuntimeError(f"Missing pickup region: {region_paths[0]}")
            self._shelves.append({
                "path": group_path, "templates": templates,
                "region": region_paths[0],
                "root": group_path.AppendPath("top_magazines/Spawned"),
                "current": None, "blocked": False,
            })
        if len(self._shelves) != 2:
            raise RuntimeError("Expected two shelf groups")

    def _reset(self):
        if self._runtime is None:
            return
        self._clear_spawned()
        for shelf in self._shelves:
            shelf.update(current=None, blocked=False)
            self._spawn(shelf)
        self._needs_reset = False

    def on_play(self):
        self._playing = True
        if self._needs_reset and self._runtime is not None:
            self._reset()

    def on_pause(self):
        self._playing = False  # Preserve current objects and debounce state.

    def on_stop(self):
        self._playing = False
        self._needs_reset = True
        if self._runtime is not None:
            self._clear_spawned()
        for shelf in self._shelves:
            shelf["current"] = None

    def _clear_spawned(self):
        # Never Clear() the root layer. Remove only paths created by this script.
        for path in list(self._owned_paths):
            for layer in self.stage.GetLayerStack():
                spec = layer.GetPrimAtPath(path)
                if spec and layer.permissionToEdit:
                    edits = Sdf.BatchNamespaceEdit()
                    edits.Add(path, Sdf.Path.emptyPath)
                    if not layer.Apply(edits):
                        raise RuntimeError(f"Cannot clean spawned path: {path}")
        self._owned_paths.clear()
        with Usd.EditContext(self.stage, self._runtime):
            for path in list(self._owned_parents):
                prim = self.stage.GetPrimAtPath(path)
                if prim and not prim.GetAllChildren():
                    self.stage.RemovePrim(path)
        self._owned_parents.clear()
        with Usd.EditContext(self.stage, self._runtime):
            for shelf in self._shelves:
                group = self.stage.GetPrimAtPath(shelf["path"])
                if group:
                    group.RemoveProperty("spawn:currentMagazine")
                    group.RemoveProperty("spawn:status")

    def on_destroy(self):
        self._playing = False
        if self._runtime is not None:
            self._clear_spawned()
            self._runtime = None

    def _status(self, shelf, status, current=None):
        with Usd.EditContext(self.stage, self._runtime):
            group = self.stage.GetPrimAtPath(shelf["path"])
            group.CreateAttribute("spawn:status", Sdf.ValueTypeNames.String).Set(status)
            group.CreateRelationship("spawn:currentMagazine").SetTargets([current] if current else [])

    def _spawn(self, shelf):
        """The only creation point; called once at reset or after one confirmed pick."""
        if shelf["current"] is not None or shelf["blocked"]:
            return
        slot_index = self._rng.randrange(4)
        color = self._rng.choice(("orange", "blue"))
        source_layer, source_path, incoming_filters = shelf["templates"][slot_index]
        number = next(self._ids)
        kind = 1 if color == "orange" else 2
        path = shelf["root"].AppendChild(f"magazine_{kind}_{color}_{number:06d}")
        try:
            with Usd.EditContext(self.stage, self._runtime):
                if not self.stage.GetPrimAtPath(shelf["root"]):
                    self._owned_parents.add(shelf["root"])
                self.stage.DefinePrim(shelf["root"], "Xform")
                self._owned_paths.add(path)
                # Publish the copy and resolved payload in one Sdf change block.
                # Otherwise composition can briefly try the unanchored relative path.
                with Sdf.ChangeBlock():
                    if not Sdf.CopySpec(source_layer, source_path, self._runtime, path):
                        raise RuntimeError(f"Cannot copy slot: {source_path}")
                    spec = self._runtime.GetPrimAtPath(path)
                    spec.payloadList.ClearEdits()
                    spec.payloadList.explicitItems = [Sdf.Payload(self._assets[color])]
                    spec.active = True
                prim = self.stage.GetPrimAtPath(path)
                prim.Load()
                UsdGeom.Imageable(prim).CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
                prim.CreateAttribute("spawn:owner", Sdf.ValueTypeNames.String).Set(str(self.prim_path))
                prim.CreateAttribute("spawn:color", Sdf.ValueTypeNames.String).Set(color)
                prim.CreateAttribute("spawn:slotIndex", Sdf.ValueTypeNames.Int).Set(slot_index + 1)
                prim.CreateAttribute("spawn:waiting", Sdf.ValueTypeNames.Bool).Set(True)
                # Preserve the source slot's incoming collision exclusions,
                # including the original gripper exclusion, for the new path.
                if incoming_filters:
                    api = UsdPhysics.FilteredPairsAPI.Apply(prim)
                    rel = api.CreateFilteredPairsRel()
                    existing = rel.GetTargets()
                    rel.SetTargets(existing + [p for p in incoming_filters if p not in existing])
                parts = self._track_parts(prim)
            gate = PickupGate(self._confirm, self._warmup)
            item = {"path": path, "parts": parts, "gate": gate, "last_problem": None}
            bounds = self._bounds(item, shelf, live=False)
            if bounds is None or classify_bounds(bounds) != "inside":
                raise RuntimeError(f"Spawn geometry is outside pickup region: {path}")
            gate.seen_inside = True
            shelf["current"] = item
            self._status(shelf, "waiting", path)
            carb.log_info(f"[MagazineSpawner] {shelf['path'].name}: slot {slot_index + 1}, {color}: {path}")
        except Exception as exc:
            shelf["blocked"] = True
            with Usd.EditContext(self.stage, self._runtime):
                self.stage.RemovePrim(path)
            self._status(shelf, "error")
            carb.log_error(f"[MagazineSpawner] Spawn stopped for {shelf['path']}: {exc}")

    def _track_parts(self, prim):
        bodies = [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
                  if p.HasAPI(UsdPhysics.RigidBodyAPI)
                  and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get()]
        # A transform-controlled asset without a rigid body can still be tracked in USD.
        tracked = bodies or [prim]
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        xforms = UsdGeom.XformCache()
        parts = []
        for body in tracked:
            bounds = bbox.ComputeUntransformedBound(body).ComputeAlignedRange()
            if bounds.IsEmpty():
                raise RuntimeError(f"No loaded geometry for {body.GetPath()}")
            world = xforms.GetLocalToWorldTransform(body)
            parts.append({"path": body.GetPath(), "bounds": bounds,
                          "scale": Gf.Transform(world).GetScale(), "physical": bool(bodies)})
        return parts

    def _bounds(self, item, shelf, live=True):
        cache = UsdGeom.XformCache()
        region = self.stage.GetPrimAtPath(shelf["region"])
        if not region:
            return None
        region_inverse = cache.GetLocalToWorldTransform(region).GetInverse()
        result = Gf.Range3d()
        for part in item["parts"]:
            body = self.stage.GetPrimAtPath(part["path"])
            if not body or not body.IsActive():
                return None
            if live and part["physical"]:
                pose = self._physx.get_rigidbody_transformation(str(part["path"]))
                # Never use a stale USD pose when a physical pose is unavailable.
                if not pose or not pose.get("ret_val"):
                    return None
                p, q = pose["position"], pose["rotation"]
                rigid = Gf.Matrix4d(1.0)
                rigid.SetRotate(Gf.Quatd(float(q[3]), Gf.Vec3d(*[float(q[i]) for i in range(3)])))
                rigid.SetTranslateOnly(Gf.Vec3d(*[float(p[i]) for i in range(3)]))
                world = Gf.Matrix4d().SetScale(part["scale"]) * rigid
            else:
                world = cache.GetLocalToWorldTransform(body)
            result.UnionWith(transformed_range(part["bounds"], world * region_inverse))
        return result

    def on_update(self, current_time, delta_time):
        if not self._playing or self._runtime is None:
            return
        dt = min(max(float(delta_time), 0.0), 0.1)
        if dt <= 0.0:
            return
        for shelf in self._shelves:
            item = shelf["current"]
            if item is None or shelf["blocked"]:
                continue
            try:
                prim = self.stage.GetPrimAtPath(item["path"])
                if not prim or not prim.IsActive():
                    observation = "removed"
                else:
                    bounds = self._bounds(item, shelf)
                    observation = "unknown" if bounds is None else classify_bounds(bounds)
                if observation == "unknown":
                    if item["gate"].age > 2.0 and item["last_problem"] != observation:
                        carb.log_warn(f"[MagazineSpawner] {item['path']}: {observation}; refill held")
                        item["last_problem"] = observation
                else:
                    item["last_problem"] = None
                if item["gate"].update(observation, dt):
                    if prim and prim.IsActive():
                        with Usd.EditContext(self.stage, self._runtime):
                            prim.GetAttribute("spawn:waiting").Set(False)
                    # Keep the picked object alive for transport/packaging.
                    shelf["current"] = None
                    self._spawn(shelf)
            except Exception as exc:
                shelf["blocked"] = True
                self._status(shelf, "error", item["path"])
                carb.log_error(f"[MagazineSpawner] {shelf['path']} stopped: {exc}")


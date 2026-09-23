"""Stop simulation, select one horizontal floor Mesh/Cube, then run in Script Editor.
Install this folder at: <project>/assets/factory_floor_gray_grid/
"""
from pathlib import Path
import os
import omni.usd
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf

TILE_SIZE_METERS = 0.5
ctx = omni.usd.get_context()
stage = ctx.get_stage()
selected = ctx.get_selection().get_selected_prim_paths()
if len(selected) != 1:
    raise RuntimeError('Select exactly one floor Mesh or Cube, not the whole Environment.')
floor = stage.GetPrimAtPath(selected[0])
if not (floor.IsA(UsdGeom.Mesh) or floor.IsA(UsdGeom.Cube)):
    raise RuntimeError('Select the actual floor Mesh or Cube in the Stage tree.')
map_file = Path(stage.GetRootLayer().realPath)
if not stage.GetRootLayer().realPath:
    raise RuntimeError('Save the map first.')
material_file = (map_file.parent / '../assets/environment/factory_floor_gray_grid/material.usda').resolve()
if not material_file.is_file():
    raise FileNotFoundError(str(material_file))
if floor.IsInstanceProxy() or floor.IsInstance():
    raise RuntimeError('Instanced floors are not supported by this helper.')
meters = UsdGeom.GetStageMetersPerUnit(stage)
world = UsdGeom.XformCache().GetLocalToWorldTransform(floor)
if abs(world.TransformDir(Gf.Vec3d(0,0,1)).GetNormalized()[2]) < .999:
    raise RuntimeError('This helper requires a horizontal floor with local Z facing up.')
mat_path = '/World/Environment/Floor'
with Usd.EditContext(stage, stage.GetRootLayer()):
    material = UsdShade.Material.Define(stage, mat_path)
    material.GetPrim().GetReferences().SetReferences([
        Sdf.Reference(os.path.relpath(material_file, map_file.parent).replace(os.sep, '/'))
    ])
    if floor.IsA(UsdGeom.Cube):
        # A visual skin avoids reliance on renderer-specific Cube UVs.
        # No collision, rigid-body or physics-material properties are edited.
        half = UsdGeom.Cube(floor).GetSizeAttr().Get() / 2
        dz = 0.001 / (meters * world.TransformDir(Gf.Vec3d(0,0,1)).GetLength())
        z = half + dz
        surface = UsdGeom.Mesh.Define(stage, floor.GetPath().AppendChild('GrayGridVisual'))
        points = [(-half,-half,z),(half,-half,z),(half,half,z),(-half,half,z)]
        surface.CreatePointsAttr(points)
        surface.CreateFaceVertexCountsAttr([4])
        surface.CreateFaceVertexIndicesAttr([0,1,2,3])
        surface.CreateSubdivisionSchemeAttr('none')
        surface.CreateDoubleSidedAttr(True)
    else:
        surface = UsdGeom.Mesh(floor)
        points = surface.GetPointsAttr().Get()
        if not points:
            raise RuntimeError('The floor mesh has no points.')
    uv = []
    for point in points:
        w = world.Transform(Gf.Vec3d(*point))
        uv.append(Gf.Vec2f(w[0]*meters/TILE_SIZE_METERS, w[1]*meters/TILE_SIZE_METERS))
    st = UsdGeom.PrimvarsAPI(surface).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.BlockIndices()
    st.Set(uv)
    UsdShade.MaterialBindingAPI.Apply(surface.GetPrim()).Bind(material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
print('Applied gray grid. Grid spacing:', TILE_SIZE_METERS, 'm. Save the map with Ctrl+S.')

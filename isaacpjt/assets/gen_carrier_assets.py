"""
후공정 캐리어 에셋 생성기 — JEDEC 트레이 스택 + 리드프레임 매거진

    python3 gen_carrier_assets.py            # 같은 폴더에 .usda 두 개 생성

Isaac Sim 라이브러리에는 반도체 트레이/매거진이 없어서 규격 치수로 직접 만든다.
모든 형상은 UsdGeom.Cube 라 PhysX 가 박스 콜라이더로 바로 쓴다 (메시 불필요).
단위는 m, Z-up.

생성 파일
  tray_stack_6.usda   JEDEC 2인치 트레이 322.6 x 135.9 x 7.62 mm 6장 스택.
                      아래 5장은 칩 적재 (300 g), 맨 위 1장은 빈 뚜껑 트레이 (150 g).
                      트레이마다 별도 RigidBody 라 위 트레이가 아래 트레이의 칩을 덮는다.
                      클립형 스택 캐리어(덮개판+스트랩+바닥 갈고리+로보틱 플랜지)가 별도 강체로
                      스택을 감싸며, 플랜지를 들면 스택 전체가 딸려 올라온다.
  magazine_small.usda 리드프레임 매거진 250 x 140 x 110 mm, 1.5 kg, 슬롯 20개(피치 5 mm).
                      슬롯마다 리드프레임 1장(시각 전용) + 상부 브리지 + 로보틱 플랜지(판 80x80x6, 기둥 20x20x18).
"""

from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

HERE = Path(__file__).resolve().parent

# ──────────────────────────────────────────────────────────────
#  JEDEC 트레이 규격 (m)
# ──────────────────────────────────────────────────────────────
TRAY_L = 0.3226          # 긴 변
TRAY_W = 0.1359          # 짧은 변
TRAY_T = 0.00762         # 스태킹 피치 = 공칭 두께
TRAY_FLOOR = 0.002       # 바닥판 두께
TRAY_RIM_W = 0.006       # 외곽 림 폭
TRAY_RIB_H = 0.002       # 림 위 스태킹 리브 높이
TRAY_RIB_W = 0.003       # 스태킹 리브 폭
TRAY_DIV_T = 0.002       # 포켓 칸막이 두께
TRAY_NOTCH_W = 0.020     # 짧은 변 중앙 핸드홀 노치 폭 (그리퍼 돌기용)
POCKET_ROWS, POCKET_COLS = 3, 8
CHIP_SIZE = 0.030        # 칩 한 변
CHIP_H = 0.0025          # 칩 높이
MASS_TRAY_EMPTY = 0.15
MASS_TRAY_LOADED = 0.30
STACK_COUNT = 6          # 5장 적재 + 1장 뚜껑

# 스택 캐리어(클립): 덮개판 + 양 끝 스트랩 + 바닥 갈고리 + 로보틱 플랜지. 별도 강체.
#   플랜지를 들면 갈고리가 맨 아래 트레이를 받쳐 스택 전체가 올라온다 (밴딩 대용).
CARRIER_PLATE_T = 0.004  # 덮개판 두께
CARRIER_SLACK = 0.001    # 덮개판과 뚜껑 트레이 사이 여유
CARRIER_STRAP_T = 0.003  # 스트랩 두께 (짧은 변 바깥면에 붙음)
CARRIER_STRAP_W = 0.020  # 스트랩 폭 = 노치 폭
CARRIER_HOOK_D = 0.015   # 갈고리가 바닥 트레이 밑으로 들어가는 깊이
CARRIER_HOOK_T = 0.003   # 갈고리 두께 (= 스택이 바닥에서 뜨는 높이)
MASS_CARRIER = 0.25

COLOR_TRAY = Gf.Vec3f(0.22, 0.22, 0.24)
COLOR_CARRIER = Gf.Vec3f(0.55, 0.58, 0.62)
COLOR_CHIP = Gf.Vec3f(0.35, 0.60, 0.40)

# ──────────────────────────────────────────────────────────────
#  매거진 규격 (m)
# ──────────────────────────────────────────────────────────────
MAG_L = 0.250
MAG_W = 0.140
MAG_H = 0.110
MAG_BASE_T = 0.004
MAG_WALL_T = 0.004
MAG_SLOTS = 20
MAG_SLOT_PITCH = 0.005
MAG_RAIL_T = 0.002       # 슬롯 레일 두께
MAG_RAIL_D = 0.006       # 레일이 안쪽으로 튀어나온 깊이
MAG_RAIL_Z0 = 0.006      # 첫 레일 높이
MAG_BRIDGE_W = 0.040     # 상부 브리지 폭 (L 방향)
MAG_BRIDGE_T = 0.008
FLANGE_STEM = 0.020      # 기둥 한 변
FLANGE_STEM_H = 0.018    # 기둥 높이 = 핑거 두께 8 + 여유 10
FLANGE_PLATE = 0.080     # 판 한 변
FLANGE_PLATE_T = 0.006
MASS_MAGAZINE = 1.5      # 알루미늄 본체 약 1.1 kg + 리드프레임 20장 약 0.4 kg
LF_T = 0.001             # 리드프레임 두께 (실제 0.15~0.25 mm, 시각용으로 1 mm)
LF_INSET = 0.005         # 리드프레임이 앞뒤 끝에서 들어간 양

COLOR_ALU = Gf.Vec3f(0.75, 0.76, 0.78)
COLOR_FLANGE = Gf.Vec3f(0.95, 0.55, 0.15)
COLOR_LEADFRAME = Gf.Vec3f(0.72, 0.45, 0.20)   # 구리 리드프레임 색


# ──────────────────────────────────────────────────────────────
#  헬퍼
# ──────────────────────────────────────────────────────────────
def new_stage(path: Path, default_prim: str) -> Usd.Stage:
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, default_prim)
    stage.SetDefaultPrim(root.GetPrim())
    return stage


def add_box(stage, path, center, size, color, collide=True):
    """size 만큼 스케일된 Cube. 박스 콜라이더 + 얇은 형상용 contact offset."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = cube.AddTranslateOp(); xf.Set(Gf.Vec3d(*center))
    sc = cube.AddScaleOp();    sc.Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([color])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        p = cube.GetPrim()
        p.AddAppliedSchema("PhysxCollisionAPI")
        p.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.002)
        p.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    return cube


def make_rigid(stage, path, mass, translate=(0, 0, 0)):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(mass)
    return xform


# ──────────────────────────────────────────────────────────────
#  트레이 한 장 (원점 = 바닥면 중심)
# ──────────────────────────────────────────────────────────────
def build_tray(stage, path, loaded: bool):
    make_rigid(stage, path, MASS_TRAY_LOADED if loaded else MASS_TRAY_EMPTY)

    # 바닥판
    add_box(stage, f"{path}/floor",
            (0, 0, TRAY_FLOOR / 2), (TRAY_L, TRAY_W, TRAY_FLOOR), COLOR_TRAY)

    rim_h = TRAY_T - TRAY_RIB_H - TRAY_FLOOR          # 림 높이 (바닥판 위)
    rim_zc = TRAY_FLOOR + rim_h / 2
    rib_zc = TRAY_FLOOR + rim_h + TRAY_RIB_H / 2

    # 긴 변 림 2개 (통짜)
    for s in (-1, 1):
        y = s * (TRAY_W - TRAY_RIM_W) / 2
        add_box(stage, f"{path}/rim_long_{'p' if s > 0 else 'n'}",
                (0, y, rim_zc), (TRAY_L, TRAY_RIM_W, rim_h), COLOR_TRAY)
        yr = s * (TRAY_W - TRAY_RIB_W) / 2
        add_box(stage, f"{path}/rib_long_{'p' if s > 0 else 'n'}",
                (0, yr, rib_zc), (TRAY_L, TRAY_RIB_W, TRAY_RIB_H), COLOR_TRAY)

    # 짧은 변 림: 중앙에 노치(핸드홀) 남기고 두 토막
    seg_len = (TRAY_W - TRAY_NOTCH_W) / 2
    for s in (-1, 1):
        x = s * (TRAY_L - TRAY_RIM_W) / 2
        xr = s * (TRAY_L - TRAY_RIB_W) / 2
        for t in (-1, 1):
            y = t * (TRAY_NOTCH_W / 2 + seg_len / 2)
            tag = f"{'p' if s > 0 else 'n'}{'p' if t > 0 else 'n'}"
            add_box(stage, f"{path}/rim_short_{tag}",
                    (x, y, rim_zc), (TRAY_RIM_W, seg_len, rim_h), COLOR_TRAY)
            add_box(stage, f"{path}/rib_short_{tag}",
                    (xr, y, rib_zc), (TRAY_RIB_W, seg_len, TRAY_RIB_H), COLOR_TRAY)

    # 포켓 칸막이
    inner_l = TRAY_L - 2 * TRAY_RIM_W
    inner_w = TRAY_W - 2 * TRAY_RIM_W
    pitch_x = inner_l / POCKET_COLS
    pitch_y = inner_w / POCKET_ROWS
    div_h = rim_h
    div_zc = TRAY_FLOOR + div_h / 2
    for c in range(1, POCKET_COLS):
        x = -inner_l / 2 + c * pitch_x
        add_box(stage, f"{path}/div_x_{c:02d}",
                (x, 0, div_zc), (TRAY_DIV_T, inner_w, div_h), COLOR_TRAY)
    for r in range(1, POCKET_ROWS):
        y = -inner_w / 2 + r * pitch_y
        add_box(stage, f"{path}/div_y_{r:02d}",
                (0, y, div_zc), (inner_l, TRAY_DIV_T, div_h), COLOR_TRAY)

    # 칩 (트레이 강체에 고정, 질량은 트레이 mass 에 포함)
    if loaded:
        for r in range(POCKET_ROWS):
            for c in range(POCKET_COLS):
                x = -inner_l / 2 + (c + 0.5) * pitch_x
                y = -inner_w / 2 + (r + 0.5) * pitch_y
                add_box(stage, f"{path}/chip_{r}{c}",
                        (x, y, TRAY_FLOOR + CHIP_H / 2),
                        (CHIP_SIZE, CHIP_SIZE, CHIP_H), COLOR_CHIP)


def build_stack_carrier(stage, path):
    make_rigid(stage, path, MASS_CARRIER)
    z_top = STACK_COUNT * TRAY_T                       # 뚜껑 트레이 윗면
    z_plate = z_top + CARRIER_SLACK + CARRIER_PLATE_T / 2
    add_box(stage, f"{path}/plate",
            (0, 0, z_plate), (TRAY_L + 2 * CARRIER_STRAP_T, TRAY_W, CARRIER_PLATE_T),
            COLOR_CARRIER)
    z_hook_c = -CARRIER_HOOK_T / 2
    strap_top = z_top + CARRIER_SLACK + CARRIER_PLATE_T
    strap_h = strap_top + CARRIER_HOOK_T
    for s in (-1, 1):
        tag = 'p' if s > 0 else 'n'
        x_strap = s * (TRAY_L / 2 + CARRIER_STRAP_T / 2)
        add_box(stage, f"{path}/strap_{tag}",
                (x_strap, 0, strap_top - strap_h / 2),
                (CARRIER_STRAP_T, CARRIER_STRAP_W, strap_h), COLOR_CARRIER)
        x_hook = s * (TRAY_L / 2 - CARRIER_HOOK_D / 2)
        add_box(stage, f"{path}/hook_{tag}",
                (x_hook, 0, z_hook_c),
                (CARRIER_HOOK_D, CARRIER_STRAP_W, CARRIER_HOOK_T), COLOR_CARRIER)
    z_stem = strap_top + FLANGE_STEM_H / 2
    add_box(stage, f"{path}/flange_stem",
            (0, 0, z_stem), (FLANGE_STEM, FLANGE_STEM, FLANGE_STEM_H), COLOR_FLANGE)
    z_fl = strap_top + FLANGE_STEM_H + FLANGE_PLATE_T / 2
    add_box(stage, f"{path}/flange_plate",
            (0, 0, z_fl), (FLANGE_PLATE, FLANGE_PLATE, FLANGE_PLATE_T), COLOR_FLANGE)


def build_tray_stack(path: Path):
    stage = new_stage(path, "/TrayStack")
    # 트레이는 캐리어 갈고리 두께만큼 띄워서 시작 (스택이 갈고리 위에 얹힌 상태)
    for i in range(STACK_COUNT):
        loaded = i < STACK_COUNT - 1        # 맨 위 한 장만 빈 뚜껑
        name = f"/TrayStack/Tray_{i:02d}" + ("_cover" if not loaded else "")
        build_tray(stage, name, loaded)
        stage.GetPrimAtPath(name).GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(0, 0, CARRIER_HOOK_T + i * TRAY_T))
    build_stack_carrier(stage, "/TrayStack/Carrier")
    stage.GetPrimAtPath("/TrayStack/Carrier").GetAttribute("xformOp:translate").Set(
        Gf.Vec3d(0, 0, CARRIER_HOOK_T))
    stage.GetRootLayer().documentation = (
        "JEDEC 2-inch tray stack: 5 loaded + 1 empty cover, pitch 7.62 mm, "
        "each tray a separate rigid body, plus a clip-type stack carrier (top plate, end straps, "
        "bottom hooks, robotic flange) as its own rigid body. Generated by gen_carrier_assets.py")
    stage.Save()


# ──────────────────────────────────────────────────────────────
#  매거진 (원점 = 바닥면 중심)
# ──────────────────────────────────────────────────────────────
def build_magazine(path: Path):
    stage = new_stage(path, "/Magazine")
    root = "/Magazine/body"
    make_rigid(stage, root, MASS_MAGAZINE)

    # 바닥판
    add_box(stage, f"{root}/base",
            (0, 0, MAG_BASE_T / 2), (MAG_L, MAG_W, MAG_BASE_T), COLOR_ALU)

    # 측벽 2개 (긴 변 방향, 앞뒤 개방)
    for s in (-1, 1):
        y = s * (MAG_W - MAG_WALL_T) / 2
        add_box(stage, f"{root}/wall_{'p' if s > 0 else 'n'}",
                (0, y, MAG_H / 2), (MAG_L, MAG_WALL_T, MAG_H), COLOR_ALU)
        # 슬롯 레일: 안쪽 면에 수평 레일 (슬롯 수 + 1)
        y_rail = s * (MAG_W / 2 - MAG_WALL_T - MAG_RAIL_D / 2)
        for k in range(MAG_SLOTS + 1):
            z = MAG_RAIL_Z0 + k * MAG_SLOT_PITCH
            add_box(stage, f"{root}/rail_{'p' if s > 0 else 'n'}_{k:02d}",
                    (0, y_rail, z), (MAG_L, MAG_RAIL_D, MAG_RAIL_T), COLOR_ALU)

    # 적재물: 슬롯마다 리드프레임 한 장씩 (매거진 강체에 고정, 시각 전용)
    #   슬롯 k 는 레일 k 와 레일 k+1 사이. 리드프레임은 레일 k 위에 얹힌다.
    lf_w = MAG_W - 2 * MAG_WALL_T - MAG_RAIL_D        # 레일 홈에 양쪽 3 mm 씩 물림
    for k in range(MAG_SLOTS):
        z = MAG_RAIL_Z0 + k * MAG_SLOT_PITCH + MAG_RAIL_T / 2 + LF_T / 2
        add_box(stage, f"{root}/leadframe_{k:02d}",
                (0, 0, z), (MAG_L - 2 * LF_INSET, lf_w, LF_T),
                COLOR_LEADFRAME, collide=False)

    # 상부 브리지 (측벽 위를 가로지름)
    z_bridge = MAG_H + MAG_BRIDGE_T / 2
    add_box(stage, f"{root}/bridge",
            (0, 0, z_bridge), (MAG_BRIDGE_W, MAG_W, MAG_BRIDGE_T), COLOR_ALU)

    # 로보틱 플랜지: 기둥 + 판
    z_stem = MAG_H + MAG_BRIDGE_T + FLANGE_STEM_H / 2
    add_box(stage, f"{root}/flange_stem",
            (0, 0, z_stem), (FLANGE_STEM, FLANGE_STEM, FLANGE_STEM_H), COLOR_FLANGE)
    z_plate = MAG_H + MAG_BRIDGE_T + FLANGE_STEM_H + FLANGE_PLATE_T / 2
    add_box(stage, f"{root}/flange_plate",
            (0, 0, z_plate), (FLANGE_PLATE, FLANGE_PLATE, FLANGE_PLATE_T), COLOR_FLANGE)

    stage.GetRootLayer().documentation = (
        "Lead-frame magazine 250x140x110 mm, 1.5 kg, 20 slots @5 mm each holding one lead frame, open front/back, "
        "top bridge with robotic handling flange. Generated by gen_carrier_assets.py")
    stage.Save()


if __name__ == "__main__":
    build_tray_stack(HERE / "tray_stack_6.usda")
    build_magazine(HERE / "magazine_small.usda")
    print("wrote", HERE / "tray_stack_6.usda")
    print("wrote", HERE / "magazine_small.usda")

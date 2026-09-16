"""
후공정 캐리어 에셋 생성기 — JEDEC 트레이 스택 + 리드프레임 매거진

    python3 gen_carrier_assets.py            # 같은 폴더에 .usda 두 개 생성

Isaac Sim 라이브러리에는 반도체 트레이/매거진이 없어서 규격 치수로 직접 만든다.
모든 형상은 UsdGeom.Cube 라 PhysX 가 박스 콜라이더로 바로 쓴다 (메시 불필요).
단위는 m, Z-up.

생성 파일
  tray_stack_6.usda   JEDEC 2인치 트레이 322.6 x 135.9 x 7.62 mm 6장 스택 + 클립 캐리어
                      (덮개판+스트랩+바닥 갈고리+로보틱 플랜지). 겉모습만 만들고 안에 묻히는
                      칩·칸막이·바닥판은 생략. 전체가 RigidBody 하나, 질량 1.0 kg, 밀도 균일.
  magazine_small.usda 리드프레임 매거진 250 x 140 x 110 mm, 슬롯 20개(피치 5 mm), 슬롯마다
                      리드프레임 1장, 상부 브리지 + 로보틱 플랜지(판 80x80x6, 기둥 20x20x18).
                      전체가 RigidBody 하나, 질량 1.0 kg, 밀도 균일. 충돌체는 바닥·측벽·브리지·
                      플랜지 6개뿐이고 레일·리드프레임은 시각 전용.
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
STACK_COUNT = 6          # 5장 적재 + 1장 뚜껑
MASS_STACK = 1.0         # 스택 + 캐리어 전체 (강체 하나, 밀도 균일)

# 스택 캐리어(클립): 덮개판 + 양 끝 스트랩 + 바닥 갈고리 + 로보틱 플랜지. 스택과 같은 강체.
#   현업에서 밴드로 묶은 스택을 용기에 넣는 것을 강체 하나로 단순화한 것.
CARRIER_PLATE_T = 0.004  # 덮개판 두께
CARRIER_SLACK = 0.001    # 덮개판과 뚜껑 트레이 사이 여유
CARRIER_STRAP_T = 0.003  # 스트랩 두께 (짧은 변 바깥면에 붙음)
CARRIER_STRAP_W = 0.020  # 스트랩 폭 = 노치 폭
CARRIER_HOOK_D = 0.015   # 갈고리가 바닥 트레이 밑으로 들어가는 깊이
CARRIER_HOOK_T = 0.003   # 갈고리 두께 (= 스택이 바닥에서 뜨는 높이)

COLOR_TRAY = Gf.Vec3f(0.22, 0.22, 0.24)
COLOR_CARRIER = Gf.Vec3f(0.55, 0.58, 0.62)

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
MASS_MAGAZINE = 1.0      # 강체 하나, 밀도 균일 (실물은 1~2 kg, M0609 하중 고려해 1 kg)
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
    """
    size 만큼 스케일된 Cube. 박스 콜라이더가 붙는다.

    contactOffset 은 일부러 건드리지 않고 PhysX 기본값(0.02 m)을 쓴다.
    예전에 트레이를 6장 따로 쌓았을 때 떨림을 줄이려고 0.002 로 낮췄는데,
    그러면 흡착 그리퍼가 몇 mm 떨어진 상태에서 접촉을 못 만들어
    붙을 대상을 찾지 못한다. 지금은 둘 다 강체 하나라 낮출 이유가 없다.
    """
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = cube.AddTranslateOp(); xf.Set(Gf.Vec3d(*center))
    sc = cube.AddScaleOp();    sc.Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([color])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
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
#  트레이 스택 = 강체 하나 (원점 = 캐리어 갈고리 바닥면 중심)
#    겉모습은 트레이 6장 + 클립 캐리어와 같지만, 안에 묻히는 칩·칸막이·바닥판은
#    만들지 않는다. 옆면은 원래도 림과 리브 바깥면이 나란해서 평면이므로
#    아래 5장은 통짜 블록 하나로 대체하고, 양 끝 노치 기둥만 층별로 남긴다.
# ──────────────────────────────────────────────────────────────
def add_tray_top(stage, root, z0):
    """맨 위 트레이의 보이는 면: 바닥판 윗면(포켓 바닥) + 림 + 리브 + 칸막이."""
    rim_h = TRAY_T - TRAY_RIB_H - TRAY_FLOOR
    rim_zc = z0 + TRAY_FLOOR + rim_h / 2
    rib_zc = z0 + TRAY_FLOOR + rim_h + TRAY_RIB_H / 2
    for s in (-1, 1):
        t = 'p' if s > 0 else 'n'
        add_box(stage, f"{root}/top_rim_long_{t}",
                (0, s * (TRAY_W - TRAY_RIM_W) / 2, rim_zc), (TRAY_L, TRAY_RIM_W, rim_h), COLOR_TRAY)
        add_box(stage, f"{root}/top_rib_long_{t}",
                (0, s * (TRAY_W - TRAY_RIB_W) / 2, rib_zc), (TRAY_L, TRAY_RIB_W, TRAY_RIB_H), COLOR_TRAY)
    seg_len = (TRAY_W - TRAY_NOTCH_W) / 2
    for s in (-1, 1):
        for t in (-1, 1):
            y = t * (TRAY_NOTCH_W / 2 + seg_len / 2)
            tag = f"{'p' if s > 0 else 'n'}{'p' if t > 0 else 'n'}"
            add_box(stage, f"{root}/top_rim_short_{tag}",
                    (s * (TRAY_L - TRAY_RIM_W) / 2, y, rim_zc), (TRAY_RIM_W, seg_len, rim_h), COLOR_TRAY)
            add_box(stage, f"{root}/top_rib_short_{tag}",
                    (s * (TRAY_L - TRAY_RIB_W) / 2, y, rib_zc), (TRAY_RIB_W, seg_len, TRAY_RIB_H), COLOR_TRAY)
    inner_l = TRAY_L - 2 * TRAY_RIM_W
    inner_w = TRAY_W - 2 * TRAY_RIM_W
    div_zc = z0 + TRAY_FLOOR + rim_h / 2
    for c in range(1, POCKET_COLS):
        add_box(stage, f"{root}/top_div_x_{c:02d}",
                (-inner_l / 2 + c * inner_l / POCKET_COLS, 0, div_zc), (TRAY_DIV_T, inner_w, rim_h), COLOR_TRAY)
    for r in range(1, POCKET_ROWS):
        add_box(stage, f"{root}/top_div_y_{r:02d}",
                (0, -inner_w / 2 + r * inner_w / POCKET_ROWS, div_zc), (inner_l, TRAY_DIV_T, rim_h), COLOR_TRAY)


def add_stack_body(stage, root, z0):
    """아래 5장 + 맨 위 트레이 바닥판을 통짜로. 양 끝 노치 기둥은 층별 바닥판 조각만 남긴다."""
    n_low = STACK_COUNT - 1
    h_low = n_low * TRAY_T
    core_l = TRAY_L - 2 * TRAY_RIM_W
    # 중앙 통짜 블록: 아래 5장 전체 + 맨 위 트레이 바닥판까지
    add_box(stage, f"{root}/core",
            (0, 0, z0 + (h_low + TRAY_FLOOR) / 2), (core_l, TRAY_W, h_low + TRAY_FLOOR), COLOR_TRAY)
    seg_len = (TRAY_W - TRAY_NOTCH_W) / 2
    for s in (-1, 1):
        xe = s * (TRAY_L - TRAY_RIM_W) / 2
        t = 'p' if s > 0 else 'n'
        # 노치 양옆 기둥: 아래 5장 높이로 통짜
        for u in (-1, 1):
            y = u * (TRAY_NOTCH_W / 2 + seg_len / 2)
            add_box(stage, f"{root}/end_col_{t}{'p' if u > 0 else 'n'}",
                    (xe, y, z0 + h_low / 2), (TRAY_RIM_W, seg_len, h_low), COLOR_TRAY)
        # 노치 기둥 안: 층마다 바닥판 조각만 (림·리브가 끊긴 자리)
        for i in range(n_low + 1):
            add_box(stage, f"{root}/end_floor_{t}_{i:02d}",
                    (xe, 0, z0 + i * TRAY_T + TRAY_FLOOR / 2), (TRAY_RIM_W, TRAY_NOTCH_W, TRAY_FLOOR), COLOR_TRAY)


def add_stack_carrier(stage, root):
    """클립 캐리어: 덮개판 + 양 끝 스트랩 + 바닥 갈고리 + 로보틱 플랜지 (같은 강체)."""
    z_top = CARRIER_HOOK_T + STACK_COUNT * TRAY_T
    strap_top = z_top + CARRIER_SLACK + CARRIER_PLATE_T
    add_box(stage, f"{root}/carrier_plate",
            (0, 0, z_top + CARRIER_SLACK + CARRIER_PLATE_T / 2),
            (TRAY_L + 2 * CARRIER_STRAP_T, TRAY_W, CARRIER_PLATE_T), COLOR_CARRIER)
    for s in (-1, 1):
        t = 'p' if s > 0 else 'n'
        add_box(stage, f"{root}/carrier_strap_{t}",
                (s * (TRAY_L / 2 + CARRIER_STRAP_T / 2), 0, strap_top / 2),
                (CARRIER_STRAP_T, CARRIER_STRAP_W, strap_top), COLOR_CARRIER)
        add_box(stage, f"{root}/carrier_hook_{t}",
                (s * (TRAY_L / 2 - CARRIER_HOOK_D / 2), 0, CARRIER_HOOK_T / 2),
                (CARRIER_HOOK_D, CARRIER_STRAP_W, CARRIER_HOOK_T), COLOR_CARRIER)
    add_box(stage, f"{root}/flange_stem",
            (0, 0, strap_top + FLANGE_STEM_H / 2), (FLANGE_STEM, FLANGE_STEM, FLANGE_STEM_H), COLOR_FLANGE)
    add_box(stage, f"{root}/flange_plate",
            (0, 0, strap_top + FLANGE_STEM_H + FLANGE_PLATE_T / 2),
            (FLANGE_PLATE, FLANGE_PLATE, FLANGE_PLATE_T), COLOR_FLANGE)


def build_tray_stack(path: Path):
    stage = new_stage(path, "/TrayStack")
    root = "/TrayStack"
    prim = stage.GetPrimAtPath(root)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASS_STACK)   # 밀도는 미지정 → 부피 비례 균일
    z0 = CARRIER_HOOK_T
    add_stack_body(stage, root, z0)
    add_tray_top(stage, root, z0 + (STACK_COUNT - 1) * TRAY_T)
    add_stack_carrier(stage, root)
    stage.GetRootLayer().documentation = (
        "JEDEC 2-inch tray stack (5 loaded + 1 cover) with clip carrier and robotic flange, "
        "authored as ONE rigid body with only exterior geometry; mass 1.0 kg, uniform density. "
        "Generated by gen_carrier_assets.py")
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
                    (0, y_rail, z), (MAG_L, MAG_RAIL_D, MAG_RAIL_T), COLOR_ALU,
                    collide=False)                     # 시각 전용 (그리퍼가 닿지 않음)

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
        "Lead-frame magazine 250x140x110 mm, ONE rigid body, 1.0 kg uniform density, 20 slots @5 mm each holding one lead frame (visual only), open front/back, "
        "top bridge with robotic handling flange. Generated by gen_carrier_assets.py")
    stage.Save()


if __name__ == "__main__":
    build_tray_stack(HERE / "tray_stack_6.usda")
    build_magazine(HERE / "magazine_small.usda")
    print("wrote", HERE / "tray_stack_6.usda")
    print("wrote", HERE / "magazine_small.usda")

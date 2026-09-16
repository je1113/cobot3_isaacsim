"""
후공정 캐리어 에셋 생성기 — JEDEC 트레이 스택 2종 + 리드프레임 매거진 2종

    python3 gen_carrier_assets.py            # 같은 폴더에 .usda 네 개 생성

Isaac Sim 라이브러리에는 반도체 트레이/매거진이 없어서 규격 치수로 직접 만든다.
모든 형상은 UsdGeom.Cube 라 PhysX 가 박스 콜라이더로 바로 쓴다 (메시 불필요).
단위는 m, Z-up, 원점은 바닥면 중심. 네 개 모두 루트 prim 이 곧 RigidBody 이고
질량 1.0 kg, 밀도 균일(질량만 적고 밀도는 PhysX 가 부피 비례로 나눔).

═══════════════════════════════════════════════════════════════════════
 규격 요약 (mm)                                       몸체 / 손잡이 분리
═══════════════════════════════════════════════════════════════════════

 1) tray_1_orange.usda   주황 손잡이 트레이 스택 1     전체 328.6 x 135.9 x 77.7
    몸체   JEDEC 2인치 트레이 322.6 x 135.9 x 7.62, 6장(칩 5 + 뚜껑 1), 피치 7.62
           높이 45.7, z 3.0 ~ 48.7 (갈고리 위에 3 mm 떠 있음)
           아래 5장은 통짜 블록, 뚜껑만 림·리브·포켓(3x8) 표현
           짧은 변 양끝 중앙에 폭 20 노치, 색 짙은 회색
    손잡이 클립 캐리어 — 갈고리 15x20x3 x2 (z 0~3), 스트랩 3x20x53.7 x2,
           덮개판 328.6x135.9x4 (z 49.7~53.7), 색 회색
           로보틱 플랜지 — 기둥 20x20x18 (z 53.7~71.7), 판 80x80x6 (z 71.7~77.7)
           색 주황 (0.95, 0.55, 0.15)

 2) tray_2_blue.usda     파란 손잡이 트레이 스택 2     전체 328.6 x 135.9 x 97.0
    몸체   같은 트레이 8장(칩 7 + 뚜껑 1), 높이 61.0, z 3.0 ~ 64.0
    손잡이 클립 캐리어 — 갈고리 15x20x3 x2, 스트랩 3x20x69.0 x2,
           덮개판 328.6x135.9x4 (z 65.0~69.0), 색 회색
           로보틱 플랜지 — 기둥 20x20x18 (z 69.0~87.0), 판 100x60x10 (z 87.0~97.0)
           색 파랑 (0.15, 0.40, 0.90)

 3) magazine_1_orange.usda  주황 손잡이 매거진 1      전체 250 x 140 x 142
    몸체   바닥판 250x140x4, 측벽 250x4x110 x2 (앞뒤 개방), 브리지 40x140x8 (z 110~118)
           슬롯 20개 피치 5 (레일 21개 x 양쪽, 시각 전용), 리드프레임 20장 240x126x1
           색 알루미늄 (0.75, 0.76, 0.78), 리드프레임 구리색
    손잡이 로보틱 플랜지 — 기둥 20x20x18 (z 118~136), 판 80x80x6 (z 136~142)
           색 주황

 4) magazine_2_blue.usda    파란 손잡이 매거진 2      전체 300 x 200 x 106
    몸체   바닥판 300x200x4, 측벽 300x4x70 x2, 브리지 40x200x8 (z 70~78)
           슬롯 12개 피치 5 (레일 13개 x 양쪽), 리드프레임 12장 290x186x1
           1번보다 넓고 낮다. 색 동일
    손잡이 로보틱 플랜지 — 기둥 20x20x18 (z 78~96), 판 100x60x10 (z 96~106)
           색 파랑

 플랜지 기둥 높이 18 = 그리퍼 핑거 두께 8 + 여유 10. 흡착 팁이 50x50 이라
 판은 그보다 크게 잡는다. 파란 판은 흡착면을 넓히려고 길고 두껍게 했다.
"""

from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics

HERE = Path(__file__).resolve().parent

# ──────────────────────────────────────────────────────────────
#  공통 색
# ──────────────────────────────────────────────────────────────
COLOR_TRAY      = Gf.Vec3f(0.22, 0.22, 0.24)
COLOR_CARRIER   = Gf.Vec3f(0.55, 0.58, 0.62)
COLOR_ALU       = Gf.Vec3f(0.75, 0.76, 0.78)
COLOR_LEADFRAME = Gf.Vec3f(0.72, 0.45, 0.20)
COLOR_ORANGE    = Gf.Vec3f(0.95, 0.55, 0.15)
COLOR_BLUE      = Gf.Vec3f(0.15, 0.40, 0.90)

# ──────────────────────────────────────────────────────────────
#  로보틱 플랜지 두 종류 (기둥은 같고 판만 다르다)
# ──────────────────────────────────────────────────────────────
FLANGE_STEM   = 0.020    # 기둥 한 변
FLANGE_STEM_H = 0.018    # 기둥 높이 = 핑거 두께 8 + 여유 10

FLANGE_ORANGE = {"plate": (0.080, 0.080, 0.006), "color": COLOR_ORANGE}
FLANGE_BLUE   = {"plate": (0.100, 0.060, 0.010), "color": COLOR_BLUE}

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

# 스택 캐리어(클립): 덮개판 + 양 끝 스트랩 + 바닥 갈고리. 스택과 같은 강체.
#   현업에서 밴드로 묶은 스택을 용기에 넣는 것을 강체 하나로 단순화한 것.
CARRIER_PLATE_T = 0.004  # 덮개판 두께
CARRIER_SLACK = 0.001    # 덮개판과 뚜껑 트레이 사이 여유
CARRIER_STRAP_T = 0.003  # 스트랩 두께 (짧은 변 바깥면에 붙음)
CARRIER_STRAP_W = 0.020  # 스트랩 폭 = 노치 폭
CARRIER_HOOK_D = 0.015   # 갈고리가 바닥 트레이 밑으로 들어가는 깊이
CARRIER_HOOK_T = 0.003   # 갈고리 두께 (= 스택이 바닥에서 뜨는 높이)

# ──────────────────────────────────────────────────────────────
#  매거진 공통 규격 (m). 외형 L/W/H 와 슬롯 수는 종류별로 준다
# ──────────────────────────────────────────────────────────────
MAG_BASE_T = 0.004
MAG_WALL_T = 0.004
MAG_SLOT_PITCH = 0.005
MAG_RAIL_T = 0.002       # 슬롯 레일 두께
MAG_RAIL_D = 0.006       # 레일이 안쪽으로 튀어나온 깊이
MAG_RAIL_Z0 = 0.006      # 첫 레일 높이
MAG_BRIDGE_W = 0.040     # 상부 브리지 폭 (L 방향)
MAG_BRIDGE_T = 0.008
LF_T = 0.001             # 리드프레임 두께 (실제 0.15~0.25 mm, 시각용으로 1 mm)
LF_INSET = 0.005         # 리드프레임이 앞뒤 끝에서 들어간 양

MASS = 1.0               # 네 개 모두. M0609 하중 고려해 1 kg 로 통일

# ──────────────────────────────────────────────────────────────
#  만들 파일 네 개
# ──────────────────────────────────────────────────────────────
TRAYS = [
    {"file": "tray_1_orange.usda", "count": 6, "flange": FLANGE_ORANGE,
     "doc": "JEDEC 2-inch tray stack, 6 trays (5 loaded + 1 cover) with clip carrier, orange flange 80x80x6"},
    {"file": "tray_2_blue.usda",   "count": 8, "flange": FLANGE_BLUE,
     "doc": "JEDEC 2-inch tray stack, 8 trays (7 loaded + 1 cover) with clip carrier, blue flange 100x60x10"},
]
MAGAZINES = [
    {"file": "magazine_1_orange.usda", "L": 0.250, "W": 0.140, "H": 0.110, "slots": 20, "flange": FLANGE_ORANGE,
     "doc": "Lead-frame magazine 250x140x110, 20 slots @5 mm, orange flange 80x80x6"},
    {"file": "magazine_2_blue.usda",   "L": 0.300, "W": 0.200, "H": 0.070, "slots": 12, "flange": FLANGE_BLUE,
     "doc": "Lead-frame magazine 300x200x70 (wider, lower), 12 slots @5 mm, blue flange 100x60x10"},
]


# ──────────────────────────────────────────────────────────────
#  헬퍼
# ──────────────────────────────────────────────────────────────
def new_stage(path: Path, default_prim: str) -> Usd.Stage:
    """루트 prim 을 만들고 그 자체를 RigidBody 로 둔다"""
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, default_prim)
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(MASS)
    return stage


def add_box(stage, path, center, size, color, collide=True):
    """
    size 만큼 스케일된 Cube. 박스 콜라이더가 붙는다.

    contactOffset 은 건드리지 않고 PhysX 기본값(0.02 m)을 쓴다.
    낮추면 흡착 그리퍼가 몇 mm 떨어진 상태에서 접촉을 못 만들어 붙지 않는다.
    """
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([color])
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def add_flange(stage, root, z_base, flange):
    """로보틱 플랜지: 기둥 + 판. z_base 는 기둥이 시작하는 높이. 판 윗면 높이를 돌려준다"""
    px, py, pt = flange["plate"]
    color = flange["color"]
    add_box(stage, f"{root}/flange_stem",
            (0, 0, z_base + FLANGE_STEM_H / 2), (FLANGE_STEM, FLANGE_STEM, FLANGE_STEM_H), color)
    add_box(stage, f"{root}/flange_plate",
            (0, 0, z_base + FLANGE_STEM_H + pt / 2), (px, py, pt), color)
    return z_base + FLANGE_STEM_H + pt


# ──────────────────────────────────────────────────────────────
#  트레이 스택 = 강체 하나 (원점 = 캐리어 갈고리 바닥면 중심)
#    겉모습은 트레이 N장 + 클립 캐리어와 같지만, 안에 묻히는 칩·칸막이·바닥판은
#    만들지 않는다. 옆면은 원래도 림과 리브 바깥면이 나란해서 평면이므로
#    아래 N-1장은 통짜 블록 하나로 대체하고, 양 끝 노치 기둥만 층별로 남긴다.
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


def add_stack_body(stage, root, z0, count):
    """아래 (count-1)장 + 맨 위 트레이 바닥판을 통짜로. 양 끝 노치 기둥은 층별 바닥판 조각만 남긴다."""
    n_low = count - 1
    h_low = n_low * TRAY_T
    core_l = TRAY_L - 2 * TRAY_RIM_W
    add_box(stage, f"{root}/core",
            (0, 0, z0 + (h_low + TRAY_FLOOR) / 2), (core_l, TRAY_W, h_low + TRAY_FLOOR), COLOR_TRAY)
    seg_len = (TRAY_W - TRAY_NOTCH_W) / 2
    for s in (-1, 1):
        xe = s * (TRAY_L - TRAY_RIM_W) / 2
        t = 'p' if s > 0 else 'n'
        for u in (-1, 1):
            y = u * (TRAY_NOTCH_W / 2 + seg_len / 2)
            add_box(stage, f"{root}/end_col_{t}{'p' if u > 0 else 'n'}",
                    (xe, y, z0 + h_low / 2), (TRAY_RIM_W, seg_len, h_low), COLOR_TRAY)
        for i in range(n_low + 1):
            add_box(stage, f"{root}/end_floor_{t}_{i:02d}",
                    (xe, 0, z0 + i * TRAY_T + TRAY_FLOOR / 2), (TRAY_RIM_W, TRAY_NOTCH_W, TRAY_FLOOR), COLOR_TRAY)


def add_stack_carrier(stage, root, count, flange):
    """클립 캐리어: 덮개판 + 양 끝 스트랩 + 바닥 갈고리 + 로보틱 플랜지 (같은 강체)."""
    z_top = CARRIER_HOOK_T + count * TRAY_T
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
    return add_flange(stage, root, strap_top, flange)


def build_tray_stack(spec):
    path = HERE / spec["file"]
    stage = new_stage(path, "/TrayStack")
    root = "/TrayStack"
    z0 = CARRIER_HOOK_T
    add_stack_body(stage, root, z0, spec["count"])
    add_tray_top(stage, root, z0 + (spec["count"] - 1) * TRAY_T)
    top = add_stack_carrier(stage, root, spec["count"], spec["flange"])
    stage.GetRootLayer().documentation = (
        f"{spec['doc']}. Total height {top*1000:.1f} mm. ONE rigid body, exterior only, "
        f"mass {MASS} kg uniform density. Generated by gen_carrier_assets.py")
    stage.Save()
    return path, top


# ──────────────────────────────────────────────────────────────
#  매거진 = 강체 하나 (원점 = 바닥면 중심)
# ──────────────────────────────────────────────────────────────
def build_magazine(spec):
    path = HERE / spec["file"]
    L, W, H, slots = spec["L"], spec["W"], spec["H"], spec["slots"]

    # 레일이 측벽 안에 들어가는지 확인 (마지막 레일 윗면 < 벽 높이)
    last_rail_top = MAG_RAIL_Z0 + slots * MAG_SLOT_PITCH + MAG_RAIL_T / 2
    assert last_rail_top < H, f"{spec['file']}: 슬롯 {slots}개가 벽 높이 {H*1000:.0f} mm 를 넘는다"

    stage = new_stage(path, "/Magazine")
    root = "/Magazine"

    add_box(stage, f"{root}/base", (0, 0, MAG_BASE_T / 2), (L, W, MAG_BASE_T), COLOR_ALU)

    for s in (-1, 1):
        t = 'p' if s > 0 else 'n'
        add_box(stage, f"{root}/wall_{t}",
                (0, s * (W - MAG_WALL_T) / 2, H / 2), (L, MAG_WALL_T, H), COLOR_ALU)
        y_rail = s * (W / 2 - MAG_WALL_T - MAG_RAIL_D / 2)
        for k in range(slots + 1):
            add_box(stage, f"{root}/rail_{t}_{k:02d}",
                    (0, y_rail, MAG_RAIL_Z0 + k * MAG_SLOT_PITCH),
                    (L, MAG_RAIL_D, MAG_RAIL_T), COLOR_ALU, collide=False)

    # 슬롯마다 리드프레임 한 장 (시각 전용). 레일 홈에 양쪽 3 mm 씩 물린다
    lf_w = W - 2 * MAG_WALL_T - MAG_RAIL_D
    for k in range(slots):
        z = MAG_RAIL_Z0 + k * MAG_SLOT_PITCH + MAG_RAIL_T / 2 + LF_T / 2
        add_box(stage, f"{root}/leadframe_{k:02d}",
                (0, 0, z), (L - 2 * LF_INSET, lf_w, LF_T), COLOR_LEADFRAME, collide=False)

    add_box(stage, f"{root}/bridge",
            (0, 0, H + MAG_BRIDGE_T / 2), (MAG_BRIDGE_W, W, MAG_BRIDGE_T), COLOR_ALU)
    top = add_flange(stage, root, H + MAG_BRIDGE_T, spec["flange"])

    stage.GetRootLayer().documentation = (
        f"{spec['doc']}. Total height {top*1000:.1f} mm. ONE rigid body, mass {MASS} kg "
        f"uniform density, 6 colliders (rails/lead frames visual only). Generated by gen_carrier_assets.py")
    stage.Save()
    return path, top


if __name__ == "__main__":
    for spec in TRAYS:
        p, top = build_tray_stack(spec)
        print(f"wrote {p.name:24s} {spec['count']}장  전체 높이 {top*1000:.1f} mm")
    for spec in MAGAZINES:
        p, top = build_magazine(spec)
        print(f"wrote {p.name:24s} {spec['L']*1000:.0f}x{spec['W']*1000:.0f}x{spec['H']*1000:.0f}  "
              f"슬롯 {spec['slots']}  전체 높이 {top*1000:.1f} mm")

"""
후공정 캐리어 에셋 생성기 — JEDEC 트레이 스택 2종 + 리드프레임 매거진 2종

    python3 gen_carrier_assets.py            # .usda 네 개 + QR PNG 두 개 생성
    python3 carrier_code.py                  # QR 코드 규칙 확인 / 자가 테스트

    필요 패키지: pip install segno   (QR 생성용. 없으면 기존 PNG 를 그대로 쓴다)

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
    QR     내용 "F1-260921-MGZO-1", 50x50, 양쪽 측벽 바깥면 왼쪽 위 (z 50~100)
             y=-70 면 x -115~-65 / y=+70 면 x +65~+115  (둘 다 바깥에서 보면 왼쪽 위)
    손잡이 로보틱 플랜지 — 기둥 20x20x18 (z 118~136), 판 80x80x6 (z 136~142)
           색 주황

 4) magazine_2_blue.usda    파란 손잡이 매거진 2      전체 300 x 200 x 106
    몸체   바닥판 300x200x4, 측벽 300x4x70 x2, 브리지 40x200x8 (z 70~78)
           슬롯 12개 피치 5 (레일 13개 x 양쪽), 리드프레임 12장 290x186x1
           1번보다 넓고 낮다. 색 동일
    QR     내용 "F1-260921-MGZB-1", 50x50, 양쪽 측벽 바깥면 왼쪽 위 (z 10~60)
             y=-100 면 x -140~-90 / y=+100 면 x +90~+140  (둘 다 바깥에서 보면 왼쪽 위)
    손잡이 로보틱 플랜지 — 기둥 20x20x18 (z 78~96), 판 100x60x10 (z 96~106)
           색 파랑

 플랜지 기둥 높이 18 = 그리퍼 핑거 두께 8 + 여유 10. 흡착 팁이 50x50 이라
 판은 그보다 크게 잡는다. 파란 판은 흡착면을 넓히려고 길고 두껍게 했다.

 QR 내용은 carrier_code.py 가 정하는 캐리어 코드다.

     F1-260921-MGZO-1
     │  │      │    └ 일련번호
     │  │      └ 품목 코드 — '-' 로 자른 세 번째 토큰. 이것만 보면 된다
     │  └ 로트 날짜 YYMMDD (고정값)
     └ 라인 코드

   MGZO 주황 매거진 / MGZB 파란 매거진 / TRYO 주황 트레이 / TRYB 파란 트레이
   앞 3자가 종류(MGZ, TRY), 끝 1자가 손잡이 색(O, B)이다.
   읽는 쪽은 carrier_code.parse_code(text) 하나로 색·종류·에셋 파일을 얻는다.
   트레이 두 종은 코드만 잡아 뒀고 데칼은 아직 안 붙였다.

 QR 은 오류정정 H(30% 복원), 528x528 px PNG 를 UsdPreviewSurface 텍스처로 붙인다.
 UsdGeom.Cube 는 UV 가 없어 텍스처가 안 먹으므로 QR 만 Mesh 사각형으로 만든다.
 콜라이더는 없다. 벽에 인쇄된 라벨이지 별도 물체가 아니다.
 벽면보다 0.5 mm 띄워 z-fighting 을 피한다 (그래서 y 방향 bbox 가 0.5 mm 커진다).
 QR 은 양쪽 측벽에 하나씩, 같은 높이에 붙는다. 어느 방향에서 접근해도 읽히게 하려는 것.
 두 벽은 보는 방향이 반대라 "왼쪽"이 서로 다른 축이다.
   -y 벽을 바깥(-y)에서 보면 +x 가 오른쪽 -> 왼쪽 위 = x 최소 / z 최대
   +y 벽을 바깥(+y)에서 보면 -x 가 오른쪽 -> 왼쪽 위 = x 최대 / z 최대
 그래서 x 위치와 UV 를 면마다 뒤집는다. 안 뒤집으면 한쪽이 거울상이라 스캔이 안 된다.
 머티리얼은 같은 PNG 를 쓰는 두 면이 하나(Looks/qr_mat)를 공유한다.
"""

import sys
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:          # 같은 폴더의 carrier_code 를 쓰려고
    sys.path.insert(0, str(HERE))
from carrier_code import (               # noqa: E402
    CODES, BASE_ASSET, parse_code, usd_name, png_name)

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
#  QR 데칼 (매거진 식별용)
# ──────────────────────────────────────────────────────────────
#  매거진 양쪽 측벽(긴 벽, y = ±W/2 바깥면) 왼쪽 위 구석에 하나씩 붙인다.
#  "왼쪽"은 각 벽을 바깥에서 봤을 때 기준이라 두 면의 x 방향이 서로 반대다.
QR_SIZE   = 0.050        # 50 x 50 mm
QR_MARGIN = 0.010        # 벽 위 모서리 / (바깥에서 본) 왼쪽 모서리에서 띄우는 거리
QR_PROUD  = 0.0005       # 벽면보다 0.5 mm 띄워 z-fighting 방지
QR_SCALE  = 16           # PNG 모듈당 픽셀 (29모듈 x 16 = 464 px)
QR_BORDER = 4            # 흰 여백 모듈 수 (QR 규격 최소 4)

#  트레이 스택은 측면이 낮아 (6장 45.7 mm) 50 mm 가 안 들어간다.
#  평평하게 이어진 면 안에 들어가는 최대치로 40 mm 를 쓴다.
STK_QR_SIZE   = 0.040    # 40 x 40 mm
STK_QR_MARGIN = 0.004    # 스택 면은 여유가 적어 여백도 줄인다

#  QR 내용은 carrier_code.py 가 정하는 캐리어 코드다 — F1-MGZO-1
#  '-' 로 자른 두 번째 토큰(MGZO/MGZB/STKO/STKB)만 보면 어떤 캐리어인지 안다.
#  9자 영숫자 + 오류정정 H 라 버전 1(21모듈), 여백 4 포함 29모듈이다.
#    매거진 50 mm / 29 = 모듈 1.72 mm
#    스택   40 mm / 29 = 모듈 1.38 mm
#  시뮬 카메라가 모듈당 2 px 이상 담게 해상도·거리를 잡아야 읽힌다.

# ──────────────────────────────────────────────────────────────
#  만들 파일 네 개
# ──────────────────────────────────────────────────────────────
TRAYS = [
    {"file": "tray_1_orange.usda", "kind": "STKO", "count": 6, "flange": FLANGE_ORANGE,
     "doc": "JEDEC 2-inch tray stack, 6 trays (5 loaded + 1 cover) with clip carrier, orange flange 80x80x6"},
    {"file": "tray_2_blue.usda",   "kind": "STKB", "count": 8, "flange": FLANGE_BLUE,
     "doc": "JEDEC 2-inch tray stack, 8 trays (7 loaded + 1 cover) with clip carrier, blue flange 100x60x10"},
]
MAGAZINES = [
    {"file": "magazine_1_orange.usda", "kind": "MGZO", "L": 0.250, "W": 0.140, "H": 0.110, "slots": 20, "flange": FLANGE_ORANGE,
     "doc": "Lead-frame magazine 250x140x110, 20 slots @5 mm, orange flange 80x80x6"},
    {"file": "magazine_2_blue.usda",   "kind": "MGZB", "L": 0.300, "W": 0.200, "H": 0.070, "slots": 12, "flange": FLANGE_BLUE,
     "doc": "Lead-frame magazine 300x200x70 (wider, lower), 12 slots @5 mm, blue flange 100x60x10"},
]

#  품목 코드 -> 바탕 스펙. QR 붙은 개체 16 종은 여기서 형상을 가져다 쓴다
SPEC = {s["kind"]: s for s in TRAYS + MAGAZINES}


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


def make_qr_png(content, filename):
    """
    QR PNG 를 만든다. segno 가 없으면 이미 있는 파일을 그대로 쓴다.

    오류정정 레벨 H(30% 복원)로 만들어 시뮬 카메라에서 일부 가려도 읽히게 한다.
    """
    path = HERE / filename
    try:
        import segno
    except ImportError:
        if path.exists():
            print(f"  [skip] segno 없음 — 기존 {filename} 사용")
            return filename
        raise RuntimeError(f"segno 가 없고 {filename} 도 없다. pip install segno")
    segno.make(content, error="h", micro=False).save(
        str(path), scale=QR_SCALE, border=QR_BORDER, dark="black", light="white")
    return filename


def add_qr_decal(stage, root, name, png, cz, side, L, W,
                 size=QR_SIZE, margin=QR_MARGIN):
    """
    측벽 바깥면 왼쪽 위에 QR 텍스처 사각형을 붙인다.

    side = -1 이면 y = -W/2 벽, +1 이면 y = +W/2 벽.

    두 벽은 보는 방향이 반대라 "왼쪽"이 서로 다른 축이다.
      -y 벽을 바깥(-y)에서 보면 +x 가 오른쪽 -> 왼쪽은 x 가 작은 쪽
      +y 벽을 바깥(+y)에서 보면 -x 가 오른쪽 -> 왼쪽은 x 가 큰 쪽
    그래서 x 위치와 UV 를 side 에 따라 뒤집는다. 안 뒤집으면 한쪽 QR 이
    거울상으로 붙어 스캐너가 못 읽는다.

    UsdGeom.Cube 는 UV 가 없어 텍스처를 못 입히므로 Mesh 로 만든다.
    콜라이더는 붙이지 않는다. 벽에 인쇄된 라벨이지 별도 물체가 아니다.
    """
    h = size / 2.0
    z0, z1 = cz - h, cz + h
    y_face = side * (W / 2.0 + QR_PROUD)

    # 바깥에서 봤을 때 왼쪽 끝 x, 오른쪽 끝 x
    if side < 0:
        x_left = -L / 2.0 + margin                    # -y 벽: 왼쪽 = x 작은 쪽
        x_right = x_left + size
    else:
        x_right = L / 2.0 - margin - size             # +y 벽: 왼쪽 = x 큰 쪽
        x_left = x_right + size
    # 화면 좌하 -> 우하 -> 우상 -> 좌상 순서로 st (0,0) (1,0) (1,1) (0,1)
    pts = [Gf.Vec3f(x_left, y_face, z0), Gf.Vec3f(x_right, y_face, z0),
           Gf.Vec3f(x_right, y_face, z1), Gf.Vec3f(x_left, y_face, z1)]
    lo = Gf.Vec3f(min(x_left, x_right), y_face, z0)
    hi = Gf.Vec3f(max(x_left, x_right), y_face, z1)

    mesh = UsdGeom.Mesh.Define(stage, f"{root}/{name}")
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([lo, hi])
    mesh.CreateDoubleSidedAttr(True)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying
    ).Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])

    # 같은 PNG 를 쓰는 면끼리 머티리얼 하나를 공유한다
    mat_path = f"{root}/Looks/qr_mat"
    if stage.GetPrimAtPath(mat_path).IsValid():
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            UsdShade.Material.Get(stage, mat_path))
        return (lo[0], hi[0], z0, z1)
    mat = UsdShade.Material.Define(stage, mat_path)

    reader = UsdShade.Shader.Define(stage, f"{mat_path}/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(stage, f"{mat_path}/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./{png}")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    surf = UsdShade.Shader.Define(stage, f"{mat_path}/surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    surf.CreateOutput("surface", Sdf.ValueTypeNames.Token)

    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return (lo[0], hi[0], z0, z1)


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


def build_tray_stack(spec, code=None, filename=None):
    """
    code 를 주면 그 코드 QR 을 양쪽 긴 면에 붙이고 <CODE>.usda 로 저장한다.
    안 주면 QR 없는 기본 에셋이다.
    """
    path = HERE / (filename or spec["file"])
    stage = new_stage(path, "/TrayStack")
    root = "/TrayStack"
    z0 = CARRIER_HOOK_T
    add_stack_body(stage, root, z0, spec["count"])
    add_tray_top(stage, root, z0 + (spec["count"] - 1) * TRAY_T)
    top = add_stack_carrier(stage, root, spec["count"], spec["flange"])

    # QR — 트레이가 쌓여 평평하게 이어진 y = ±TRAY_W/2 면의 왼쪽 위
    #   그 면은 바닥 갈고리 위(z0)부터 맨 위 트레이 윗면까지 끊기지 않는다
    qr_box = None
    if code:
        face_top = z0 + spec["count"] * TRAY_T
        z_bot = face_top - STK_QR_MARGIN - STK_QR_SIZE
        assert z_bot >= z0, (
            f"{code}: QR {STK_QR_SIZE*1000:.0f} mm 아래끝이 z {z_bot*1000:.1f} 로 "
            f"평평한 면 바닥 {z0*1000:.1f} 아래로 내려간다")
        png = make_qr_png(code, png_name(code))
        cz = face_top - STK_QR_MARGIN - STK_QR_SIZE / 2
        qr_box = [add_qr_decal(stage, root, "qr_label_ny", png, cz, -1, TRAY_L, TRAY_W,
                               size=STK_QR_SIZE, margin=STK_QR_MARGIN),
                  add_qr_decal(stage, root, "qr_label_py", png, cz, +1, TRAY_L, TRAY_W,
                               size=STK_QR_SIZE, margin=STK_QR_MARGIN)]

    stage.GetRootLayer().documentation = (
        f"{spec['doc']}. Total height {top*1000:.1f} mm. ONE rigid body, exterior only, "
        f"mass {MASS} kg uniform density."
        + (f" QR '{code}' {STK_QR_SIZE*1000:.0f}x{STK_QR_SIZE*1000:.0f} on left-top of"
           f" both long side faces (visual only)." if code else "")
        + " Generated by gen_carrier_assets.py")
    stage.Save()
    return path, top, qr_box


# ──────────────────────────────────────────────────────────────
#  매거진 = 강체 하나 (원점 = 바닥면 중심)
# ──────────────────────────────────────────────────────────────
def build_magazine(spec, code=None, filename=None):
    """
    code 를 주면 그 코드 QR 을 양쪽 측벽에 붙이고 <CODE>.usda 로 저장한다.
    안 주면 QR 없는 기본 에셋이다.
    """
    path = HERE / (filename or spec["file"])
    L, W, H, slots = spec["L"], spec["W"], spec["H"], spec["slots"]

    # 레일이 측벽 안에 들어가는지 확인 (마지막 레일 윗면 < 벽 높이)
    last_rail_top = MAG_RAIL_Z0 + slots * MAG_SLOT_PITCH + MAG_RAIL_T / 2
    assert last_rail_top < H, f"{path.name}: 슬롯 {slots}개가 벽 높이 {H*1000:.0f} mm 를 넘는다"

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

    # QR 라벨 — 측면(y = -W/2 바깥면) 왼쪽 위 구석
    #   그 벽을 바깥에서 보면 +x 가 오른쪽이므로 왼쪽은 x 가 작은 쪽이다
    qr_box = None
    if code:
        assert QR_SIZE + 2 * QR_MARGIN <= H, \
            f"{path.name}: QR {QR_SIZE*1000:.0f} + 여백이 벽 높이 {H*1000:.0f} mm 를 넘는다"
        png = make_qr_png(code, png_name(code))
        cz = H - QR_MARGIN - QR_SIZE / 2
        # 양쪽 측벽에 하나씩. 어느 방향에서 접근해도 읽히게 한다
        qr_box = [add_qr_decal(stage, root, "qr_label_ny", png, cz, -1, L, W),
                  add_qr_decal(stage, root, "qr_label_py", png, cz, +1, L, W)]

    stage.GetRootLayer().documentation = (
        f"{spec['doc']}. Total height {top*1000:.1f} mm. ONE rigid body, mass {MASS} kg "
        f"uniform density, 6 colliders (rails/lead frames visual only)."
        + (f" QR '{code}' {QR_SIZE*1000:.0f}x{QR_SIZE*1000:.0f} on left-top of"
           f" both side walls, visual only." if code else "")
        + " Generated by gen_carrier_assets.py")
    stage.Save()
    return path, top, qr_box


def build(spec, code=None, filename=None):
    """트레이 스택이든 매거진이든 스펙에 맞는 빌더로 보낸다"""
    fn = build_tray_stack if spec["kind"].startswith("STK") else build_magazine
    return fn(spec, code=code, filename=filename)


if __name__ == "__main__":
    # 1) QR 없는 기본 에셋 네 개
    for spec in TRAYS + MAGAZINES:
        p, top, _ = build(spec)
        print(f"wrote {p.name:24s} 기본(QR 없음)  전체 높이 {top*1000:.1f} mm")

    # 2) 코드 QR 을 양면에 붙인 개체 16 종
    print()
    for code in CODES:
        info = parse_code(code)
        spec = SPEC[info.kind]
        p, top, qr_box = build(spec, code=code, filename=usd_name(code))
        (x0, x1, z0, z1) = qr_box[0]
        print(f"wrote {p.name:16s} {info.family:8s} {info.color:6s} "
              f"바탕 {BASE_ASSET[info.kind]:22s} QR '{code}' 양면 "
              f"{(x1-x0)*1000:.0f}x{(z1-z0)*1000:.0f} mm  z {z0*1000:.1f}~{z1*1000:.1f}")

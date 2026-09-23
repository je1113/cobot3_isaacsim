#!/usr/bin/env python3
"""씬 USD 하나를 읽어 각 선반의 순찰선(정차 y·theta·x 범위)을 계산한다.

왜 있나 — shelves.yaml 의 waypoint 는 "매거진이 선반의 어느 긴 변에 붙어
있는가" 로 정해진다. 그 면이 바뀌면 로봇이 서는 변도 반대로 가야 하는데,
씬 파일을 눈으로 보고 옮겨 적다 틀리면 로봇이 선반을 사이에 두고 반대편을
순찰한다(실제로 그랬다). 그래서 **씬에서 직접 뽑는다**.

    python3 isaacpjt/tools/derive_patrol_from_scene.py                    # 기본 씬
    python3 isaacpjt/tools/derive_patrol_from_scene.py <다른.usda>        # 임의 씬
    python3 isaacpjt/tools/derive_patrol_from_scene.py --yaml             # yaml 조각
    python3 isaacpjt/tools/derive_patrol_from_scene.py --check            # shelves.yaml 대조

★ pxr(USD) 없이 도는 순수 파이썬이다. Isaac 안이 아니라 아무 셸에서나 쓰라고
  만들었다 — 좌표가 의심스러울 때 제일 먼저 돌리는 것이 이 파일이다.
"""
import math
import re
import sys
from pathlib import Path

NUM = r'-?\d+\.?\d*(?:[eE][-+]?\d+)?'
WS_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENE = WS_ROOT / "isaacpjt/worlds/simple_factory_layout.usda"

# 매거진 라벨 중심에서 차체가 떨어져 서는 거리. shelves.yaml 의 SHELF-A 가
# 실측으로 고정한 값이고(0.40), 두 선반이 같은 기하라 그대로 쓴다.
STANDOFF_M = 0.40
# 순찰 구간의 x 범위. 선반 기둥 x[-2.79,-0.29] 밖(동쪽)에서 시작해 서쪽 슬롯
# (-2.44)을 지나칠 때까지 훑는다 — 근거는 shelves.yaml SHELF-A 주석.
X_START, X_END = 0.60, -2.60


def _vec(s, n):
    return [float(x) for x in re.findall(NUM, s)][:n]


def _quat_mat(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _ident():
    return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def parse(path):
    """USDA -> {프림경로: {world(4x4), type, size, active}}"""
    src = Path(path).read_text(encoding="utf-8")
    body = src[src.index('def Xform "World"'):]
    prims, stack, pend, cur = {}, [], None, None
    for ln in body.split("\n"):
        st = ln.strip()
        if not st:
            continue
        m = re.match(r'(?:def|over)\s+(\w+)?\s*"([^"]+)"', st)
        if m:
            pend = {"type": m.group(1) or "", "name": m.group(2), "t": [0.0] * 3,
                    "r": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "s": [1.0] * 3,
                    "active": True, "size": None}
        if "{" in st and "dictionary" not in st:
            stack.append(pend or {"type": "", "name": "<a>", "t": [0.0] * 3,
                                  "r": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                  "s": [1.0] * 3, "active": True, "size": None})
            cur, pend = stack[-1], None
        if cur is not None and "=" in st and "xformOpOrder" not in st:
            rhs = st.split("=", 1)[1]
            if "xformOp:translate" in st:
                cur["t"] = _vec(rhs, 3)
            elif "xformOp:scale" in st:
                cur["s"] = _vec(rhs, 3)
            elif "xformOp:orient" in st:
                q = _vec(rhs, 4)
                if len(q) == 4:
                    cur["r"] = _quat_mat(*q)
            elif re.match(r"active\s*=\s*false", st):
                cur["active"] = False
            elif re.match(r"(double|float)\s+size\s*=", st):
                cur["size"] = _vec(rhs, 1)[0]
        if st.startswith("}") and stack:
            p = stack.pop()
            p["path"] = "/".join(a["name"] for a in stack) + "/" + p["name"]
            M = _ident()
            for a in stack + [p]:
                L = _ident()
                for i in range(3):
                    for j in range(3):
                        L[i][j] = a["r"][i][j] * a["s"][j]
                    L[i][3] = a["t"][i]
                M = _mul(M, L)
            p["world"] = M
            p["dead"] = (not p["active"]) or any(not a["active"] for a in stack)
            prims[p["path"]] = p
            cur = stack[-1] if stack else None
    return prims


def pos(p):
    return [p["world"][i][3] for i in range(3)]


def scale(p):
    return [math.sqrt(sum(p["world"][i][j] ** 2 for i in range(3)))
            for j in range(3)]


def deck_y_range(prims, shelf):
    """선반판(Level_2)의 월드 y 구간."""
    key = f"World/Environment/PickupZone/{shelf}/Level_2"
    d = prims.get(key)
    if d is None:
        return None
    sz = d["size"] if d["size"] is not None else 2.0
    c, s = pos(d), scale(d)
    return (c[1] - sz * s[1] / 2, c[1] + sz * s[1] / 2)


def magazines_of(prims, group):
    """그 선반의 매거진 프림. (이름, 월드좌표, 스폰슬롯여부) 목록.

    ★ 기본 씬(simple_factory_layout.usda)의 매거진은 active=false 인 **스폰
      슬롯**이다 — magazine_spawner.py 가 런타임에 켠다. 정적으로는 "없는"
      프림이지만 런타임 매거진이 정확히 그 자리에 생기므로, 순찰선을 정하는
      근거로는 살아있는 프림과 똑같이 써야 한다. 이걸 빼먹으면 기본 씬에서
      "매거진을 못 찾았다" 가 되어 아무 답도 못 낸다.
    """
    live, slots = [], []
    pre = f"World/Magazines/{group}/top_magazines/"
    for k, v in prims.items():
        if k.startswith(pre) and v["type"] == "":
            (live if not v["dead"] else slots).append(
                (k.rsplit("/", 1)[1], pos(v), v["dead"]))
    return sorted(live) if live else sorted(slots)


SHELVES = [("SHELF-A", "robot1", "Shelf_01", "shelf_1_magaines"),
           ("SHELF-B", "robot2", "Shelf_02", "shelf_2_magaines")]


def check_against_shelves(snippets):
    """shelves.yaml 의 값이 씬에서 뽑은 값과 같은가. 다르면 1 을 돌려준다.

    ★ "고쳤는데 로봇이 옛 자리로 간다" 를 가르는 첫 번째 물음이 이것이다 —
      파일이 틀린 것인지, 파일은 맞는데 안 읽힌 것인지.
    """
    import yaml as _yaml
    path = WS_ROOT / "src/cobot3_bringup/config/shelves.yaml"
    doc = _yaml.safe_load(path.read_text(encoding="utf-8"))
    by_id = {s.get("shelf_id"): s for s in (doc.get("shelves") or [])}
    print(f"\n# --- shelves.yaml 대조 ({path}) ---")
    bad = 0
    for shelf_id, want_y, want_th in snippets:
        sh = by_id.get(shelf_id)
        if sh is None:
            print(f"  ★NG {shelf_id}: shelves.yaml 에 없다")
            bad += 1
            continue
        for key in ("waypoint_start", "waypoint_end"):
            wp = sh.get(key) or {}
            gy, gth = wp.get("y"), wp.get("theta")
            ok = (gy is not None and abs(float(gy) - want_y) <= 0.03
                  and gth is not None
                  and abs((float(gth) - want_th + math.pi) % (2 * math.pi) - math.pi) <= 0.01)
            bad += 0 if ok else 1
            print(f"  {'OK ' if ok else '★NG'} {shelf_id} {key:15s} "
                  f"파일 y={gy} theta={gth}   씬 y={want_y:.4f} theta={want_th:.6f}")
    if bad:
        print("\n  => 파일이 씬과 다르다. shelves.yaml 을 고쳐라 (--yaml 로 값을 뽑을 수 있다).")
    else:
        print("\n  => 파일은 씬과 일치한다. 그래도 로봇이 엉뚱한 데로 가면 파일이")
        print("     아니라 전달 경로다. 순서대로 확인해라:")
        print("       1) ros2 param get /robot2/task_manager shelves_yaml")
        print("          -> 노드가 읽는 파일 경로. 클론이 여럿이면 여기서 갈린다.")
        print("       2) 기동 로그의 '순찰 경로 [...]' 줄. 출처가 거기 찍힌다.")
        print("          [shelves.yaml SHELF-B] 가 아니면 조회가 실패한 것이고,")
        print("          실패 이유는 바로 위 경고에 있다.")
        print("       3) task_manager 는 __init__ 에서 한 번만 읽는다 — 노드 재시작.")
        print("     ※ 'param get patrol_route' 는 쓸모없다. 그건 선언된 기본값일")
        print("        뿐이고 실제로 쓰는 값(self.patrol_route)이 아니다.")
    return 1 if bad else 0


def main(argv):
    as_yaml = "--yaml" in argv
    do_check = "--check" in argv
    args = [a for a in argv if not a.startswith("--")]
    scene = Path(args[0]) if args else DEFAULT_SCENE
    if not scene.is_absolute():
        scene = (Path.cwd() / scene).resolve()
    prims = parse(scene)
    print(f"# 씬: {scene}")
    snippets = []
    for shelf_id, robot, shelf, group in SHELVES:
        dy = deck_y_range(prims, shelf)
        mags = magazines_of(prims, group)
        if dy is None or not mags:
            print(f"\n{shelf_id}: 선반판 또는 매거진을 못 찾았다 "
                  f"(deck={dy}, magazines={[m[0] for m in mags]})")
            continue
        my = sum(m[1][1] for m in mags) / len(mags)
        south, north = dy
        # 매거진이 붙어 있는 긴 변 = 가까운 쪽 면. 로봇은 그 면 **바깥**에 선다.
        on_south = abs(my - south) <= abs(my - north)
        stop_y = my - STANDOFF_M if on_south else my + STANDOFF_M
        # 선반이 로봇 왼쪽에 오도록 — 남쪽에 서면 +x(0도), 북쪽에 서면 -x(180도)
        theta = 0.0 if on_south else math.pi
        print(f"\n{shelf_id}  ({robot})")
        print(f"  선반판 y[{south:8.4f}, {north:8.4f}]   (긴 변 2개: 남 {south:.4f} · 북 {north:.4f})")
        if mags and mags[0][2]:
            print(f"  (매거진 {len(mags)}개가 전부 active=false 스폰 슬롯이다 — "
                  f"magazine_spawner.py 가 런타임에 켠다. 자리는 이 값이 맞다)")
        for name, p, _slot in mags:
            print(f"  매거진 {name:22s} ({p[0]:8.4f}, {p[1]:8.4f}, {p[2]:6.3f})")
        print(f"  -> 매거진이 붙은 긴 변 : {'남쪽' if on_south else '북쪽'}"
              f" (남 면까지 {abs(my-south):.3f} m, 북 면까지 {abs(my-north):.3f} m)")
        print(f"  -> 로봇이 설 변        : {'남쪽(y 작은 쪽)' if on_south else '북쪽(y 큰 쪽)'}")
        print(f"  -> 정차선 y            : {stop_y:.4f}   (매거진에서 {STANDOFF_M} m)")
        print(f"  -> theta               : {theta:.6f} rad ({math.degrees(theta):.0f}도)")
        snippets.append((shelf_id, stop_y, theta))
    if as_yaml:
        print("\n# --- shelves.yaml 에 넣을 값 ---")
        for shelf_id, y, th in snippets:
            print(f"# {shelf_id}")
            print(f"    waypoint_start: {{x: {X_START}, y: {y:.4f}, theta: {th:.6f}}}")
            print(f"    waypoint_end: {{x: {X_END}, y: {y:.4f}, theta: {th:.6f}}}")
    if do_check:
        return check_against_shelves(snippets)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

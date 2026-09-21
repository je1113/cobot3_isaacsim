"""
로더 대기 자리 좌표 검증 — Isaac · ROS 불필요.

    python3 isaacpjt/tools/check_staging_poses.py

두 로봇이 같은 로더로 갈 때 한 대는 차선 밖 대기 자리에서 기다린다
(task_manager.py 의 배송 Selector). 그 좌표가 지금 지도에서 여전히 쓸 수 있는
값인지 확인한다.

★ 이 스크립트가 있는 이유
  좌표를 점유격자에서 골랐는데, 지도는 레이아웃을 손볼 때마다 바뀐다. 실제로
  한 번 바뀌면서(원점이 x -10.075 에서 -6.575 로 이동) 그전에 고른 대기 자리
  하나가 장애물 안으로 들어갔다. 지도를 새로 만들면 이걸 돌려라.

읽는 곳 (전부 레포 안의 정본이다 — 값을 복사해 두지 않는다)
  cobot3_navigation/maps/simple_factory_layout.yaml + .png   점유격자
  cobot3_bringup/launch/mission_nodes.launch.py              대기 자리 좌표
  cobot3_bringup/config/stations.yaml                        로더 주차 자세
  cobot3_navigation/params/robot1_nav2_params.yaml           footprint

확인하는 것
  1. 대기 자리가 자유공간인가 (차체가 들어갈 여유 반경까지)
  2. 로더 도착각이 충분히 작은가
     ─ 로더 주차점 주변에 제자리회전 여유가 없으면, 도착 yaw 오차만큼
       nav_server 의 _fine_align 이 그 자리에서 몸을 돌리다 장애물을 친다.
  3. 대기 자리가 이탈 경로에서 비켜 있는가
     ─ 배치를 마친 로봇은 후진으로 같은 쪽으로 빠져나온다.
  4. 두 대기 자리가 서로 겹치지 않는가
"""

import io
import math
import re
import struct
import sys
import zlib
from pathlib import Path

WS = Path(__file__).resolve().parent.parent.parent
MAP_YAML = WS / "src/cobot3_navigation/maps/simple_factory_layout.yaml"
LAUNCH = WS / "src/cobot3_bringup/launch/mission_nodes.launch.py"
STATIONS = WS / "src/cobot3_bringup/config/stations.yaml"
NAV_PARAMS = WS / "src/cobot3_navigation/params/robot1_nav2_params.yaml"

# 이탈 궤적 근사. nav_server 는 후진하면서 ALIGN_MAX_W(0.15 rad/s) 상한으로
# 천천히 돌아 목표 방향에 맞춘 뒤 그 기울기로 직진한다. 그래서 로더 바로
# 뒤에서는 접근선을 그대로 따라 나오고, 조금 멀어진 뒤부터 옆으로 흐른다.
EGRESS_STRAIGHT_M = 1.35        # 이만큼은 접근선을 따라 그대로 나온다
EGRESS_DRIFT_DEG = 14.0         # 그 뒤 기울기


class Grid:
    """ROS 점유격자(map_server 규약). 순수 파이썬 PNG 디코더를 쓴다."""

    def __init__(self, yaml_path):
        text = io.open(yaml_path, encoding="utf-8").read()
        self.res = float(re.search(r"resolution:\s*([-\d.]+)", text).group(1))
        ox, oy = [float(v) for v in
                  re.search(r"origin:\s*\[([^\]]+)\]", text).group(1).split(",")[:2]]
        self.ox, self.oy = ox, oy
        self.occ_t = float(re.search(r"occupied_thresh:\s*([-\d.]+)", text).group(1))
        self.free_t = float(re.search(r"free_thresh:\s*([-\d.]+)", text).group(1))
        png = yaml_path.parent / re.search(r"image:\s*(\S+)", text).group(1)
        self.w, self.h, self.bpp, self.rows = self._decode(png)

    @staticmethod
    def _decode(path):
        data = path.read_bytes()
        off, idat, w = 8, b"", None
        while off < len(data):
            ln, typ = struct.unpack(">I4s", data[off:off + 8])
            if typ == b"IHDR":
                w, h, bit, color = struct.unpack(">IIBB", data[off + 8:off + 18])
                assert bit == 8, f"8비트 PNG 만 읽는다 (bitdepth={bit})"
            elif typ == b"IDAT":
                idat += data[off + 8:off + 8 + ln]
            off += 12 + ln
        raw = zlib.decompress(idat)
        bpp = {0: 1, 2: 3, 4: 2, 6: 4}[color]
        stride, rows, prev, pos = w * bpp, [], bytearray(w * bpp), 0
        for _ in range(h):
            f = raw[pos]; pos += 1
            line = bytearray(raw[pos:pos + stride]); pos += stride
            if f == 1:
                for i in range(bpp, stride):
                    line[i] = (line[i] + line[i - bpp]) & 0xFF
            elif f == 2:
                for i in range(stride):
                    line[i] = (line[i] + prev[i]) & 0xFF
            elif f == 3:
                for i in range(stride):
                    a = line[i - bpp] if i >= bpp else 0
                    line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
            elif f == 4:
                for i in range(stride):
                    a = line[i - bpp] if i >= bpp else 0
                    b, c = prev[i], (prev[i - bpp] if i >= bpp else 0)
                    pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                    pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                    line[i] = (line[i] + pr) & 0xFF
            rows.append(line); prev = line
        return w, h, bpp, rows

    def cell(self, col, row):
        if not (0 <= col < self.w and 0 <= row < self.h):
            return "out"
        i, px = col * self.bpp, self.rows[row]
        if self.bpp == 4:
            if px[i + 3] == 0:
                return "unknown"
            v = (px[i] + px[i + 1] + px[i + 2]) / 3.0
        elif self.bpp == 3:
            v = (px[i] + px[i + 1] + px[i + 2]) / 3.0
        elif self.bpp == 2:
            if px[i + 1] == 0:
                return "unknown"
            v = px[i]
        else:
            v = px[i]
        occ = (255 - v) / 255.0
        return "occ" if occ > self.occ_t else ("free" if occ < self.free_t else "unknown")

    def _rc(self, x, y):
        return int((x - self.ox) / self.res), self.h - 1 - int((y - self.oy) / self.res)

    def at(self, x, y):
        return self.cell(*self._rc(x, y))

    def disc(self, x, y, radius):
        """반경 안의 (점유 셀 수, 미지/지도밖 셀 수)."""
        c0, r0 = self._rc(x, y)
        n = int(math.ceil(radius / self.res))
        occ = unk = 0
        for dc in range(-n, n + 1):
            for dr in range(-n, n + 1):
                if (dc * self.res) ** 2 + (dr * self.res) ** 2 > radius * radius:
                    continue
                s = self.cell(c0 + dc, r0 + dr)
                if s == "occ":
                    occ += 1
                elif s in ("unknown", "out"):
                    unk += 1
        return occ, unk

    def nearest_obstacle(self, x, y, rmax=3.0):
        c0, r0 = self._rc(x, y)
        n = int(rmax / self.res)
        best = None
        for dc in range(-n, n + 1):
            for dr in range(-n, n + 1):
                d = math.hypot(dc * self.res, dr * self.res)
                if d > rmax or (best is not None and d >= best):
                    continue
                if self.cell(c0 + dc, r0 + dr) == "occ":
                    best = d
        return best


def _literals(path):
    """파이썬 소스의 최상위 단순 대입을 순서대로 eval 한다."""
    import ast
    ns = {"__builtins__": __builtins__}
    for node in ast.parse(io.open(path, encoding="utf-8").read()).body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                ns[node.targets[0].id] = eval(
                    compile(ast.Expression(node.value), "<c>", "eval"), ns)
            except Exception:
                pass
    return ns


def _footprint_radius():
    """제자리회전 스윕 반경 = base_link 에서 가장 먼 footprint 꼭짓점."""
    text = io.open(NAV_PARAMS, encoding="utf-8").read()
    m = re.search(r"footprint:\s*\"(\[.*?\])\"", text, re.S)
    pts = eval(m.group(1))
    return max(math.hypot(a, b) for a, b in pts), pts


def _loader_pose():
    """stations.yaml 의 PACKAGING 스테이션 place_pose."""
    text = io.open(STATIONS, encoding="utf-8").read()
    block = re.search(r"station_type:\s*PACKAGING.*?place_pose:\s*\{([^}]*)\}",
                      text, re.S)
    vals = dict(re.findall(r"(\w+):\s*([-\d.]+|null)", block.group(1)))
    return float(vals["x"]), float(vals["y"])


def _egress_y(x, lx, ly, ret):
    """로더에서 후진해 나오는 궤적의 y 근사."""
    if x > lx - EGRESS_STRAIGHT_M:
        return ly
    drift = math.copysign(1.0, ret[1] - ly)
    return ly + drift * (lx - EGRESS_STRAIGHT_M - x) * math.tan(
        math.radians(EGRESS_DRIFT_DEG))


def _max_rotation_deg(grid, x, y, fp, sweep, limit=45.0, pad=0.02):
    """주차점에서 제자리회전할 수 있는 최대 yaw 오차(도).

    footprint 사각형을 1도씩 실제로 돌리면서, 스윕 반경 안의 점유 셀이 사각형
    안으로 들어오는 각을 찾는다. 양방향 중 작은 쪽을 돌려준다 — 도착 yaw 오차는
    부호가 어느 쪽이든 나올 수 있다.
    """
    xs = [a for a, _ in fp]
    ys = [b for _, b in fp]
    xmin, xmax = min(xs) - pad, max(xs) + pad
    ymin, ymax = min(ys) - pad, max(ys) + pad

    c0, r0 = grid._rc(x, y)
    n = int(math.ceil(sweep / grid.res)) + 1
    obstacles = []
    for dc in range(-n, n + 1):
        for dr in range(-n, n + 1):
            d = math.hypot(dc * grid.res, dr * grid.res)
            if d > sweep + grid.res:
                continue
            if grid.cell(c0 + dc, r0 + dr) == "occ":
                obstacles.append((dc * grid.res, -dr * grid.res))   # 지도 y 는 위가 +
    if not obstacles:
        return limit

    def blocked(deg):
        t = math.radians(deg)
        ct, st = math.cos(t), math.sin(t)
        for ox_, oy_ in obstacles:
            # 장애물을 로봇 프레임으로 (yaw=deg 인 로봇 기준)
            bx = ox_ * ct + oy_ * st
            by = -ox_ * st + oy_ * ct
            if xmin <= bx <= xmax and ymin <= by <= ymax:
                return True
        return False

    best = limit
    for sign in (1, -1):
        for deg in range(0, int(limit) + 1):
            if blocked(sign * deg):
                best = min(best, float(max(deg - 1, 0)))
                break
    return best


def main():
    grid = Grid(MAP_YAML)
    sweep, fp = _footprint_radius()
    half_w = max(abs(b) for _, b in fp)
    lx, ly = _loader_pose()
    launch = _literals(LAUNCH)
    staging = launch["STAGING_BY_ROBOT"]
    # 이탈 목적지 = RETURN 이 향하는 순찰 시작점.
    tm = _literals(WS / "src/cobot3_orchestrator/cobot3_orchestrator/task_manager.py")
    ret = tm["PATROL_ROUTE"][0][:2]

    print(f"지도 {grid.w}x{grid.h} res={grid.res} origin=({grid.ox:.3f},{grid.oy:.3f})")
    print(f"로더 주차점 ({lx}, {ly})   제자리회전 스윕 반경 {sweep:.3f} m")

    # 이탈 궤적을 순찰 시작점으로 잡는데, 그 좌표가 지도 밖이면 근사가 무의미하다.
    # 실패로 세지는 않는다 — 이 도구가 볼 일이 아니라 순찰 경로 쪽 문제다.
    for i, wp in enumerate(tm["PATROL_ROUTE"]):
        inside = (grid.ox <= wp[0] <= grid.ox + grid.w * grid.res
                  and grid.oy <= wp[1] <= grid.oy + grid.h * grid.res)
        if not inside:
            print(f"  ⚠ PATROL_ROUTE[{i}] ({wp[0]}, {wp[1]}) 가 지도 밖이다 — "
                  f"AMCL 이 그 자리를 못 잡고, 아래 이탈 근사도 못 믿는다")

    fails = []

    def check(ok, label):
        print(f"  {'PASS' if ok else 'FAIL'}  {label}")
        if not ok:
            fails.append(label)

    print("\n── 로더 주차점: 그 자리에서 몸을 돌릴 수 있나 ──")
    near = grid.nearest_obstacle(lx, ly)
    print(f"  최근접 장애물 {near if near is None else round(near, 3)} m, "
          f"스윕 반경 {sweep:.3f} m")
    # ★ 앞모서리만 보면 틀린다. base_link 가 차체 앞쪽에 있어서 뒷모서리가
    #   훨씬 크게 돈다(앞 0.286 m vs 뒤 0.656 m). 그래서 "몇 도까지 돌 수
    #   있나" 는 footprint 사각형 전체를 실제로 돌려 보며 재야 한다.
    max_angle = _max_rotation_deg(grid, lx, ly, fp, sweep)
    if max_angle >= 45.0:
        print("  45도까지 돌려도 안 닿는다 — 도착각 제약이 느슨하다")
    else:
        print(f"  도착 yaw 오차 상한 {max_angle:.0f}도 — 그 이상이면 "
              f"_fine_align 이 돌다가 장애물을 친다")

    print("\n── 대기 자리 ──")
    for name, pose in sorted(staging.items()):
        x, y = pose[0], pose[1]
        occ, unk = grid.disc(x, y, sweep + 0.15)
        angle = math.degrees(math.atan2(ly - y, lx - x))
        gap = abs(_egress_y(x, lx, ly, ret) - y) - 2 * half_w
        print(f"  {name} ({x:.2f}, {y:.2f})  {grid.at(x, y)}  "
              f"도착각 {angle:+.1f}도  이탈여유 {gap:+.2f} m")
        check(occ == 0 and unk == 0,
              f"{name} 자유공간 (반경 {sweep + 0.15:.2f} m 안 점유 {occ}, 미지 {unk})")
        check(abs(angle) <= max_angle,
              f"{name} 도착각 {abs(angle):.1f}도 <= 상한 {max_angle:.0f}도")
        check(gap >= 0.20, f"{name} 이탈 경로 여유 {gap:+.2f} m >= 0.20 m")

    names = sorted(staging)
    if len(names) >= 2:
        print("\n── 대기 자리끼리 ──")
        a, b = staging[names[0]], staging[names[1]]
        sep = math.hypot(a[0] - b[0], a[1] - b[1])
        check(sep >= 2 * sweep,
              f"두 자리 간격 {sep:.2f} m >= {2 * sweep:.2f} m (차체 둘이 안 겹친다)")

    print()
    if fails:
        print(f"실패 {len(fails)}건 — 대기 자리를 다시 골라야 한다")
        print("  후보를 찾으려면: 로더에서 1.8~4.5 m, 도착각 상한 안, 이탈선 반대편,")
        print("  그리고 위 반경이 전부 자유공간인 점을 훑으면 된다.")
        return 1
    print("전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())

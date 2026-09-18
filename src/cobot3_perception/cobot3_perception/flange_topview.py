"""
플랜지 상방 관측 — 손목 카메라로 flange_plate 를 위에서 찍어 중심 xy · 윗면 z · yaw 를 잰다.

ROS 의존성 없이 numpy + cv2 만 쓴다. Isaac 스크립트(12_pick_test.py 등)와
나중의 pose_resolver 노드가 같은 코드를 쓰게 하려는 것이다.

왜 QR 이 아니라 플랜지인가 (isaacpjt/tools/diag_qr_reproj.py · eval_qr_pose.py):
  QR 라벨은 옆벽에 서 있어서 매거진 yaw 가 카메라에서 보면 '화면 밖으로 기우는'
  회전이 된다. 36 mm 코드를 273 mm 에서 보면 1도에 좌우 변 길이 차가 0.2 px 인데
  cv2 꼭짓점 잔차가 RMS 2.1 px (BR 은 3.5 px) 라 yaw 가 1σ 2.3도로 흔들린다.
  플랜지를 위에서 보면 yaw 는 '화면 안의 회전'이고, 80 mm 변 전체에 직선을
  맞추므로 수백 점이 평균된다. 게다가 컵을 놓을 바로 그 면을 재므로
  QR -> 플랜지 레버암(114/153 mm)을 타고 오차가 커지지 않는다.

알고리즘:
  1. 예상 중심(QR 에서 온 대략값)을 투영해 탐색 창을 정한다.
  2. 마스크 -> 예상 위치에 가장 가까운 연결 성분.
     깊이가 있으면  : 픽셀마다 world z 를 풀어 |z - 윗면 prior| < depth_band 인 곳.
                      윗면 z 는 그 안의 중앙값으로 잰다.
     깊이가 없으면  : HSV 색 마스크 + 크기로 높이 보정 (아래 5).
     색만으로는 약하다. 렌더에서 주황 플랜지(S 87, H 25)와 그늘진 리드프레임
     (S 79, H 21)이 거의 붙어 있고, 금속 그리퍼 표면에 비친 주황도 같은 색이다.
     플랜지 윗면은 매거진에서 가장 높은 면이고 그 다음 면(bridge)이 24 mm
     아래라 깊이로는 깨끗하게 갈린다.
  3. 외곽선 픽셀을 광선으로 풀어 '윗면 z' 평면과 교차 -> 평면 위 metric 점.
  4. minAreaRect 로 초기 사각형 -> 점을 네 변에 배정(모서리 근처 제외) ->
     변마다 직선 맞춤 -> 네 교점이 꼭짓점, 그 평균이 중심.
  5. (깊이 없을 때만) 크기(80 mm 기준)로 높이를 한 번 보정하고 3~4 를 다시 한다.
     (평면이 가정보다 dz 높으면 metric 크기가 H/(H-dz) 배로 커진다)
  6. yaw 는 네 변 방향의 평균. 정사각형이라 90도 모호성이 있어서
     yaw_prior (QR 대략값, 오차 ±45도 안) 에 가장 가까운 것을 고른다.

좌표 규약:
  R_wo, p_wo = world <- camera_color_optical_frame (x 오른쪽, y 아래, z 시선)
  yaw 는 world z 축 둘레, 매거진 로컬 +x 축 기준 (라디안)
"""

from dataclasses import dataclass, field
import math

import cv2
import numpy as np


# 렌더된 색에서 잰 HSV 범위 (cv2: H 0~179). 저작 displayColor 는
# 주황 (0.95, 0.55, 0.15) -> H 15, 파랑 (0.15, 0.40, 0.90) -> H 110 이지만
# 톤매핑 뒤 값이 조금 옮겨지므로 넓게 잡는다.
HSV_RANGES = {
    "orange": ((5, 110, 60), (35, 255, 255)),
    "blue":   ((95, 110, 40), (130, 255, 255)),
}


@dataclass
class FlangeObservation:
    ok: bool
    reason: str = ""
    center_world: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    yaw_rad: float = float("nan")
    size_m: tuple = (float("nan"), float("nan"))
    top_z_used: float = float("nan")
    depth_correction_m: float = 0.0
    fill_ratio: float = float("nan")
    edge_rms_mm: float = float("nan")
    center_px: tuple = (float("nan"), float("nan"))
    corners_world: np.ndarray = field(default_factory=lambda: np.full((4, 3), np.nan))

    def as_dict(self):
        return dict(
            ok=self.ok, reason=self.reason,
            center_world=[round(float(v), 5) for v in self.center_world],
            yaw_deg=round(math.degrees(self.yaw_rad), 4) if self.ok else None,
            size_mm=[round(float(v) * 1000, 2) for v in self.size_m],
            top_z_used=round(float(self.top_z_used), 5),
            depth_correction_mm=round(self.depth_correction_m * 1000, 2),
            fill_ratio=round(float(self.fill_ratio), 4),
            edge_rms_mm=round(float(self.edge_rms_mm), 3),
            center_px=[round(float(v), 1) for v in self.center_px],
        )


# ══════════════════════════════════════════════════════════════
#  기하
# ══════════════════════════════════════════════════════════════
def project_points(K, dist, R_wo, p_wo, pts_world):
    pts_world = np.atleast_2d(np.asarray(pts_world, dtype=float))
    pc = (pts_world - p_wo) @ R_wo                  # optical 좌표
    rvec = np.zeros(3); tvec = np.zeros(3)
    uv, _ = cv2.projectPoints(pc.reshape(-1, 1, 3), rvec, tvec, K, dist)
    return uv.reshape(-1, 2), pc[:, 2]


def pixel_rays(K, dist, R_wo, p_wo, px):
    """픽셀 -> world 광선 방향 (N x 3, 정규화 안 됨. z 성분이 대략 광학축 성분)"""
    px = np.asarray(px, dtype=np.float64).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(px, K, dist).reshape(-1, 2)
    return np.column_stack([norm, np.ones(len(norm))]) @ R_wo.T


def ray_plane_intersect(K, dist, R_wo, p_wo, px, plane_point, plane_normal):
    """픽셀 -> world 광선 -> 임의 평면(점 + 법선)과의 교점 (N x 3).

    QR 이 붙은 벽처럼 수직 평면(법선이 수평)에도 쓸 수 있게 pixels_to_plane 을
    일반화한 것. 카메라가 평면과 거의 나란히 보고 있으면(광선이 평면과 스칠 듯
    거의 평행하면) t 분모가 0 에 가까워져 수치가 불안정해진다 — 근거리 정면
    관측(QR/플랜지 상방)에서는 문제되지 않는다.
    """
    rays = pixel_rays(K, dist, R_wo, p_wo, px)
    plane_point = np.asarray(plane_point, dtype=float)
    plane_normal = np.asarray(plane_normal, dtype=float)
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    denom = rays @ plane_normal
    t = ((plane_point - p_wo) @ plane_normal) / denom
    return p_wo + rays * t[:, None]


def pixels_to_plane(K, dist, R_wo, p_wo, px, plane_z):
    """픽셀 -> world 광선 -> z = plane_z 수평 평면과의 교점 (N x 3)"""
    return ray_plane_intersect(K, dist, R_wo, p_wo, px,
                               plane_point=[0.0, 0.0, plane_z], plane_normal=[0.0, 0.0, 1.0])


def backproject_depth(K, dist, R_wo, p_wo, px, depth_vals):
    """픽셀 + '광학 z 거리'(distance_to_image_plane, 카메라 프레임 z 좌표와 같다) -> world 점 (N x 3)"""
    rays = pixel_rays(K, dist, R_wo, p_wo, px)
    return p_wo + rays * np.asarray(depth_vals, dtype=float).reshape(-1, 1)


def _wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _fit_square(pts2, nominal):
    """평면 점(외곽선)에 네 변 직선을 맞춘다.

    반환: (center2, yaw_mod90, (w, h), corners2 4x2, edge_rms) 또는 None
    """
    rect = cv2.minAreaRect(pts2.astype(np.float32))
    (cx, cy), (w0, h0), ang = rect
    if w0 < 1e-6 or h0 < 1e-6:
        return None
    th = math.radians(ang)
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    local = (pts2 - [cx, cy]) @ R                     # 사각형 프레임
    hw, hh = w0 / 2.0, h0 / 2.0

    # 각 점을 가장 가까운 변에 배정. 모서리 근처(변 길이의 15%)는 버린다 —
    # 모서리는 안티에일리어싱으로 둥글어져 직선 맞춤을 끌어당긴다.
    d = np.column_stack([np.abs(local[:, 0] - hw), np.abs(local[:, 0] + hw),
                         np.abs(local[:, 1] - hh), np.abs(local[:, 1] + hh)])
    side = np.argmin(d, axis=1)
    keep_x = np.abs(local[:, 1]) < 0.70 * hh          # 좌우 변 (x = ±hw)
    keep_y = np.abs(local[:, 0]) < 0.70 * hw          # 상하 변 (y = ±hh)

    lines, rms = [], []
    for k in range(4):
        m = (side == k) & (keep_x if k < 2 else keep_y)
        if m.sum() < 8:
            return None
        p = local[m]
        if k < 2:     # x = a*y + b
            A = np.column_stack([p[:, 1], np.ones(len(p))])
            coef, *_ = np.linalg.lstsq(A, p[:, 0], rcond=None)
            res = p[:, 0] - A @ coef
        else:         # y = a*x + b
            A = np.column_stack([p[:, 0], np.ones(len(p))])
            coef, *_ = np.linalg.lstsq(A, p[:, 1], rcond=None)
            res = p[:, 1] - A @ coef
        lines.append(coef)
        rms.append(float(np.sqrt(np.mean(res ** 2))))

    def intersect(lx, ly):
        # x = ax*y + bx,  y = ay*x + by
        ax, bx = lx
        ay, by = ly
        x = (ax * by + bx) / (1.0 - ax * ay)
        return np.array([x, ay * x + by])

    xr, xl, yb, yt = lines          # +hw, -hw, +hh, -hh
    corners_local = np.array([intersect(xl, yt), intersect(xr, yt),
                              intersect(xr, yb), intersect(xl, yb)])
    corners = corners_local @ R.T + [cx, cy]
    center = corners.mean(axis=0)

    # 사각형 프레임이 φ 만큼 더 돌았다면 좌우 변(x = a*y + b)은 a = -tan φ,
    # 상하 변(y = a*x + b)은 a = tan φ 가 된다. 네 변을 평균한다.
    local_yaw = float(np.mean([-math.atan(xr[0]), -math.atan(xl[0]),
                               math.atan(yb[0]), math.atan(yt[0])]))
    yaw = th + local_yaw

    w = float(np.mean([np.linalg.norm(corners[1] - corners[0]),
                       np.linalg.norm(corners[2] - corners[3])]))
    h = float(np.mean([np.linalg.norm(corners[3] - corners[0]),
                       np.linalg.norm(corners[2] - corners[1])]))
    return center, yaw, (w, h), corners, float(np.sqrt(np.mean(np.square(rms))))


# ══════════════════════════════════════════════════════════════
#  본체
# ══════════════════════════════════════════════════════════════
def detect_flange(bgr, K, dist, R_wo, p_wo, *,
                  expected_center_world, top_z_prior, yaw_prior_rad,
                  color=None, depth=None, depth_band_m=0.012,
                  flange_size_m=(0.08, 0.08),
                  search_radius_m=0.05, size_tol=0.10, min_fill=0.90,
                  max_depth_correction_m=0.015, debug=None):
    """flange_plate 를 찾아 FlangeObservation 을 돌려준다.

    expected_center_world : 플랜지 윗면 중심의 대략값 (QR 에서 온 것)
    top_z_prior           : 윗면 z 대략값
    yaw_prior_rad         : 90도 모호성 해소용. 오차 ±45도 안이면 된다.
    depth                 : 광학 z 거리 영상 (m, HxW, 컬러와 정렬). 있으면 깊이로 자른다.
    color                 : depth 가 없을 때 쓰는 HSV_RANGES 키
    debug                 : dict 를 넘기면 중간 산출물(mask, contour)을 채운다.
    """
    K = np.asarray(K, dtype=float)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=float)
    R_wo = np.asarray(R_wo, dtype=float)
    p_wo = np.asarray(p_wo, dtype=float)
    exp_c = np.asarray(expected_center_world, dtype=float).copy()
    exp_c[2] = top_z_prior
    nominal = float(np.mean(flange_size_m))
    H, W = bgr.shape[:2]

    # 1. 탐색 창
    uv_c, depth_c = project_points(K, dist, R_wo, p_wo, exp_c)
    if depth_c[0] <= 0:
        return FlangeObservation(False, "예상 위치가 카메라 뒤에 있다")
    u0, v0 = uv_c[0]
    r_px = K[0, 0] * (search_radius_m + nominal * 0.75) / depth_c[0]
    if not (-r_px < u0 < W + r_px and -r_px < v0 < H + r_px):
        return FlangeObservation(False, f"예상 위치가 화면 밖이다 uv=({u0:.0f},{v0:.0f})")

    # 2. 마스크 + 연결 성분
    roi = np.zeros((H, W), np.uint8)
    cv2.circle(roi, (int(round(u0)), int(round(v0))), int(math.ceil(r_px)), 255, -1)
    zmap = None
    if depth is not None:
        d = np.asarray(depth, dtype=np.float64).reshape(H, W)
        vv, uu = np.nonzero(roi)
        dd = d[vv, uu]
        good = np.isfinite(dd) & (dd > 0)
        vv, uu, dd = vv[good], uu[good], dd[good]
        norm = cv2.undistortPoints(np.column_stack([uu, vv]).astype(np.float64)
                                   .reshape(-1, 1, 2), K, dist).reshape(-1, 2)
        dirs = np.column_stack([norm, np.ones(len(norm))]) @ R_wo.T
        zw = p_wo[2] + dirs[:, 2] * dd          # 광학 z 거리 * (광선의 world z 성분)
        zmap = np.full((H, W), np.nan)
        zmap[vv, uu] = zw
        mask = np.zeros((H, W), np.uint8)
        sel = np.abs(zw - top_z_prior) < depth_band_m
        mask[vv[sel], uu[sel]] = 255
    else:
        if color is None:
            return FlangeObservation(False, "depth 도 color 도 없다")
        lo, hi = HSV_RANGES[color]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.bitwise_and(mask, roi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    if debug is not None:
        debug["mask"] = mask
        debug["expected_px"] = (float(u0), float(v0))
        debug["search_radius_px"] = float(r_px)
    if n <= 1:
        what = f"윗면 z±{depth_band_m*1000:.0f} mm" if depth is not None else f"{color} 색"
        return FlangeObservation(False, f"{what} 영역이 탐색 창 안에 없다")

    expect_area = (K[0, 0] * nominal / depth_c[0]) ** 2
    best, best_d = None, None
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 0.25 * expect_area:
            continue
        d = math.hypot(cents[i][0] - u0, cents[i][1] - v0)
        if best is None or d < best_d:
            best, best_d = i, d
    if best is None:
        big = int(stats[1:, cv2.CC_STAT_AREA].max())
        return FlangeObservation(
            False, f"충분히 큰 성분이 없다 (최대 {big} px, 기대 {expect_area:.0f} px)")

    comp = (labels == best).astype(np.uint8) * 255
    contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    if debug is not None:
        debug["contour"] = contour

    # 3~5. 평면 교차 -> 사각형 맞춤 -> (깊이 없으면) 크기로 높이 보정 -> 한 번 더
    top_z = float(top_z_prior)
    if zmap is not None:
        top_z = float(np.nanmedian(zmap[labels == best]))
    fit = None
    for it in range(1 if zmap is not None else 2):
        pts = pixels_to_plane(K, dist, R_wo, p_wo, contour, top_z)
        fit = _fit_square(pts[:, :2], nominal)
        if fit is None:
            return FlangeObservation(False, "변 직선 맞춤 실패 (변마다 점이 부족)")
        _, _, (w, h), _, _ = fit
        if it == 1 or zmap is not None:
            break
        scale = ((w + h) / 2.0) / nominal
        H_assumed = p_wo[2] - top_z
        dz = H_assumed - H_assumed / scale         # +면 실제 평면이 더 높다
        if abs(dz) > max_depth_correction_m:
            return FlangeObservation(
                False, f"크기로 본 높이 보정이 {dz*1000:+.1f} mm 로 너무 크다 "
                       f"(측정 {w*1000:.1f} x {h*1000:.1f} mm) — 다른 물체를 잡았을 수 있다")
        top_z += dz

    center2, yaw_m, (w, h), corners2, edge_rms = fit
    if abs(w - flange_size_m[0]) > size_tol * flange_size_m[0] and \
       abs(w - flange_size_m[1]) > size_tol * flange_size_m[1]:
        return FlangeObservation(False, f"크기 불일치 {w*1000:.1f} x {h*1000:.1f} mm")
    if abs(h - flange_size_m[1]) > size_tol * flange_size_m[1] and \
       abs(h - flange_size_m[0]) > size_tol * flange_size_m[0]:
        return FlangeObservation(False, f"크기 불일치 {w*1000:.1f} x {h*1000:.1f} mm")

    # 채움률: 성분 면적 / 꼭짓점 사각형을 다시 투영한 면적
    corners_w = np.column_stack([corners2, np.full(4, top_z)])
    quad_px, _ = project_points(K, dist, R_wo, p_wo, corners_w)
    quad_area = abs(cv2.contourArea(quad_px.astype(np.float32)))
    fill = float(stats[best, cv2.CC_STAT_AREA]) / max(quad_area, 1.0)
    if fill < min_fill:
        return FlangeObservation(False, f"채움률 {fill:.2f} < {min_fill} — 가려졌거나 다른 형상")

    # 6. 90도 모호성 해소
    k = round((yaw_prior_rad - yaw_m) / (math.pi / 2))
    yaw = _wrap_pi(yaw_m + k * math.pi / 2)

    center_w = np.array([center2[0], center2[1], top_z])
    c_px, _ = project_points(K, dist, R_wo, p_wo, center_w)
    return FlangeObservation(
        ok=True, center_world=center_w, yaw_rad=yaw, size_m=(w, h),
        top_z_used=top_z, depth_correction_m=top_z - float(top_z_prior),
        fill_ratio=fill, edge_rms_mm=edge_rms * 1000.0,
        center_px=(float(c_px[0][0]), float(c_px[0][1])), corners_world=corners_w)


def draw_observation(bgr, obs, K, dist, R_wo, p_wo, debug=None):
    """확인용 그림. 원본을 건드리지 않고 사본에 그린다."""
    vis = bgr.copy()
    if debug and "expected_px" in debug:
        u, v = debug["expected_px"]
        cv2.circle(vis, (int(u), int(v)), int(debug["search_radius_px"]), (255, 255, 0), 1)
        cv2.drawMarker(vis, (int(u), int(v)), (255, 255, 0), cv2.MARKER_CROSS, 16, 1)
    if debug and "contour" in debug:
        cv2.polylines(vis, [debug["contour"].astype(np.int32)], True, (0, 255, 255), 1)
    if obs.ok:
        quad, _ = project_points(K, dist, R_wo, p_wo, obs.corners_world)
        cv2.polylines(vis, [quad.astype(np.int32)], True, (0, 255, 0), 2)
        c = obs.center_world
        ax = c + 0.04 * np.array([math.cos(obs.yaw_rad), math.sin(obs.yaw_rad), 0.0])
        pc, _ = project_points(K, dist, R_wo, p_wo, np.vstack([c, ax]))
        cv2.arrowedLine(vis, tuple(pc[0].astype(int)), tuple(pc[1].astype(int)),
                        (0, 0, 255), 2, tipLength=0.2)
    # cv2 폰트는 한글을 못 그린다. 사유는 로그로 보고 여기엔 ASCII 만 쓴다.
    if obs.ok:
        text = (f"OK  yaw {math.degrees(obs.yaw_rad):+.2f} deg  "
                f"{obs.size_m[0]*1000:.1f}x{obs.size_m[1]*1000:.1f} mm  fill {obs.fill_ratio:.2f}")
    else:
        text = "FAIL (see log)"
    cv2.putText(vis, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return vis

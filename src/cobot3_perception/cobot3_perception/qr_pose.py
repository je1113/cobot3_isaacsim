"""
근거리 QR 자세추정 — 깊이로 벽 평면을 맞춰 위치와 yaw 를 잰다.

체크리스트 3-2 항목:
    QR 꼭짓점 검출 + PnP로 T_cam_QR 계산
    depth로 선반 평면 피팅, roll/pitch는 평면에서 취하고 PnP에서는 위치+yaw만 사용
    다중 프레임 평균 및 이상치 제거
    QR 디코딩 결과(ID)를 자세와 함께 발행

ROS 의존성 없이 numpy + cv2 만 쓴다 (flange_topview.py 와 같은 이유 — 노드와
Isaac 테스트가 같은 코드를 쓴다).

왜 이미지(PnP)만으로는 yaw 가 안 되는가 (isaacpjt/tools/diag_qr_reproj.py,
eval_qr_pose.py, docs/08_Grasp_Pose_Error_Reduction.md):
  QR 라벨은 옆벽에 서 있다. 매거진 yaw(world z 축 회전)는 카메라에서 보면
  라벨이 '화면 밖으로 기우는' 회전 — 즉 원근(foreshortening) 신호다.
  코드영역 36 mm 를 273 mm 에서 보면 1도에 좌우 변 길이 차가 0.2 px 인데
  cv2 꼭짓점 잔차는 RMS 2.1 px 다. 신호가 잡음보다 10 배 작아 필터로도 안 된다.

깊이가 이 문제를 피하는 이유:
  회전축(world Z, 카메라 앞쪽에서 거의 수직)을 이미지 평면 안에 두고 보면
  yaw 는 위 foreshortening 신호로만 보인다 — 사영 기하의 근본적인 조건불량이다.
  반면 깊이는 픽셀마다 실제 3D 좌표를 준다. 벽면(wall_n, 250x110 mm, QR 라벨
  보다 훨씬 넓다)에 평면을 맞추면 그 법선은 사영 왜곡과 무관한 진짜 3D 방향이다.
  매거진이 항상 똑바로 서 있다면(pitch/roll = 0, shelf 에 얹혀 있으니 타당한
  가정) 벽면 법선의 수평 방향이 곧 매거진의 yaw 다 — 12_pick_test.py 의
  flange_topview 검증(0.03 도)과 원리가 같다.

  레일들이 벽보다 4 mm 튀어나와 있어(gen_carrier_assets.py) 평면에서 벗어난
  점으로 잡히므로, 넉넉한 탐색창 + 반복 평면 피팅(RANSAC 비슷하게 벗어난 점을
  버리고 다시 피팅)으로 자연히 걸러진다.

알고리즘:
  1. cv2.QRCodeDetector 로 꼭짓점 + ID 검출 (decode 실패해도 detect 는 될 수
     있다 — presence 만 필요한 3-1 에서 유용. 여기 3-2 는 decode 까지 요구한다)
  2. 꼭짓점 bbox 를 넉넉히 확장한 탐색창 안의 깊이를 3D 로 역투영
  3. 반복 평면 피팅(SVD, 이상치 제거)으로 벽 평면(점, 법선) 확정
  4. 법선이 수평인지 확인(허용 tilt 안이면 OK) → QR 프레임 z 축 = -법선,
     y 축 = world -Z(라벨은 항상 수직이라 이미지 아래 = 세계 아래), x = y×z
  5. 검출된 QR 중심 픽셀을 이 평면에 광선-평면 교차 → 위치
     (회전이 이미 평면에서 나왔으므로 PnP 로 위치를 다시 풀 필요가 없다 —
      6DOF PnP 대신 '평면 고정 후 위치만' 을 광선 교차로 직접 계산한 것)
  6. cv2.solvePnP(IPPE_SQUARE) 결과도 같이 남겨 교차검증(둘이 크게 다르면
     저신뢰로 표시)
  7. 다중 프레임: 중앙값 기준 MAD 로 이상치를 버리고 평균한다

좌표 규약: docs/08, grasp.yaml 의 QR 프레임과 동일
  x_qr = 라벨 오른쪽, y_qr = 라벨 아래, z_qr = 라벨 안쪽(= -바깥법선)
"""

from dataclasses import dataclass, field
import math
import os

import cv2
import numpy as np

from .flange_topview import project_points, pixel_rays, ray_plane_intersect, backproject_depth


# QR 라벨의 여백(quiet zone) 포함 크기와 실제 코드 영역 크기 (frames.yaml qr 절)
LABEL_SIDE_M = 0.050
DATA_SIDE_M = 0.0362

# OpenCV 마커 프레임(x 오른쪽, y 위) -> 우리 QR 프레임(x 오른쪽, y 아래, z 안쪽)
_R_CV_TO_QR = np.diag([1.0, -1.0, -1.0])


@dataclass
class QRPoseObservation:
    ok: bool
    reason: str = ""
    decoded: str = ""
    center_world: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    yaw_rad: float = float("nan")
    wall_tilt_deg: float = float("nan")     # 법선이 수평에서 벗어난 정도 (QA)
    plane_rms_mm: float = float("nan")
    n_plane_points: int = 0
    pnp_center_world: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    pnp_yaw_rad: float = float("nan")
    cross_check_mm: float = float("nan")    # depth 위치와 PnP 위치 차이
    corners_px: np.ndarray = field(default_factory=lambda: np.full((4, 2), np.nan))

    def as_dict(self):
        return dict(
            ok=self.ok, reason=self.reason, decoded=self.decoded,
            center_world=[round(float(v), 5) for v in self.center_world],
            yaw_deg=round(math.degrees(self.yaw_rad), 4) if self.ok else None,
            wall_tilt_deg=round(float(self.wall_tilt_deg), 3),
            plane_rms_mm=round(float(self.plane_rms_mm), 3),
            n_plane_points=int(self.n_plane_points),
            pnp_yaw_deg=(round(math.degrees(self.pnp_yaw_rad), 3)
                        if np.isfinite(self.pnp_yaw_rad) else None),
            cross_check_mm=round(float(self.cross_check_mm), 3),
        )


@dataclass
class AggregatedQRPose:
    ok: bool
    reason: str = ""
    decoded: str = ""
    center_world: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    yaw_rad: float = float("nan")
    n_used: int = 0
    n_total: int = 0
    pos_std_mm: np.ndarray = field(default_factory=lambda: np.full(3, np.nan))
    yaw_std_deg: float = float("nan")

    def as_dict(self):
        return dict(
            ok=self.ok, reason=self.reason, decoded=self.decoded,
            center_world=[round(float(v), 5) for v in self.center_world],
            yaw_deg=round(math.degrees(self.yaw_rad), 4) if self.ok else None,
            n_used=self.n_used, n_total=self.n_total,
            pos_std_mm=[round(float(v), 4) for v in self.pos_std_mm],
            yaw_std_deg=round(float(self.yaw_std_deg), 4),
        )


# ══════════════════════════════════════════════════════════════
#  검출 (presence 만, decode 안 되도 됨 — 3-1 에서 재사용)
# ══════════════════════════════════════════════════════════════
_WECHAT = None
_WECHAT_TRIED = False


def _wechat_detector():
    """WeChat QR 검출기(있으면). CNN 검출 + zxing 디코더라 기울기에 훨씬 강하다.

    ★ 왜 필요한가 — Isaac 렌더 프레임에서 cv2.QRCodeDetector 가 **기울어진 라벨을
      못 읽는다**. 실측: 손목캠이 라벨을 13도 비스듬히 8.3 px/모듈 로 잡은
      프레임에서 detectAndDecode / detectAndDecodeMulti / detectAndDecodeCurved 가
      전부 빈 문자열을 냈는데, 같은 프레임을 WeChat 검출기는 한 번에 읽었다.
      (같은 PNG 를 정면에서 4 px/모듈 로 줄여 주면 cv2 도 읽는다 — 해상도가
       아니라 원근/기울기가 원인이다)

    ★ 없는 빌드도 있다(모델 파일이 빠진 opencv). 그래서 실패하면 조용히
      None 을 돌려주고 기존 cv2 경로로 간다 — 있으면 좋고 없어도 돌아간다.
    """
    global _WECHAT, _WECHAT_TRIED
    if not _WECHAT_TRIED:
        _WECHAT_TRIED = True
        try:
            _WECHAT = cv2.wechat_qrcode_WeChatQRCode()
        except Exception:
            _WECHAT = None
    return _WECHAT


_EXTERNAL_DECODER = None


def set_external_decoder(fn):
    """이 인터프리터의 cv2 에 WeChat 이 없을 때 쓸 디코더를 주입한다.

    fn(bgr_or_gray) -> 디코딩된 문자열(없으면 "").

    ★ 왜 주입인가 — Isaac 이 번들한 cv2 는 contrib 가 아니라서
      (4.11.0, /isaacsim/exts/omni.pip.compute/pip_prebundle/cv2)
      wechat_qrcode_WeChatQRCode 가 아예 없다. 실측: 같은 프레임 9장을
      시스템 파이썬의 cv2 는 9/9 읽는데 Isaac 안에서는 0/9 다. 그렇다고
      이 모듈이 Isaac 을 알게 만들 수는 없다 — 여긴 ROS 노드도 쓰는
      공용 모듈이고, 거기선 WeChat 이 그냥 있다. 그래서 "밖에서 어떻게
      읽을지" 는 Isaac 스크립트가 정해서 넣어 준다.
    """
    global _EXTERNAL_DECODER
    _EXTERNAL_DECODER = fn


def _external_decode(image):
    if _EXTERNAL_DECODER is None or _wechat_detector() is not None:
        return ""       # 로컬에 WeChat 이 있으면 밖으로 나갈 이유가 없다
    try:
        return _EXTERNAL_DECODER(image) or ""
    except Exception:
        return ""


def _bright_quad_candidates(gray, min_px=40, max_frac=0.5):
    """흰 QR 라벨처럼 보이는 밝은 사각형 후보의 bbox 를 큰 것부터 돌려준다.

    ★ 왜 필요한가 — cv2.QRCodeDetector 는 **화면 전체**를 주면 앞쪽의 어두운
      로봇 팔 같은 방해물 때문에 검출 자체를 실패한다. 그런데 라벨 주변만
      잘라 주면 같은 프레임에서 정사각형도 0.92 의 정확한 꼭짓점을 낸다(실측).
      그래서 '어디를 잘라 줄지' 만 먼저 찾아 준다. 0.01 초면 끝난다.
    """
    h, w = gray.shape[:2]
    _, bw = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    raw = [cv2.boundingRect(c) for c in cnts]

    # ★ 조각을 먼저 합친다. 라벨은 흰 바탕에 검은 모듈이 박혀 있어서 임계화하면
    #   **한 라벨이 여러 조각으로 쪼개진다.** 실측: 라벨 자리에서 153x103 과
    #   83x66 두 조각이 따로 잡혔고, 그 조각만 잘라 주면 detect() 가 QR 을
    #   못 찾아 꼭짓점이 하나도 안 나왔다(밝은후보 4개, out=0). 9x9 닫기로는
    #   파인더 패턴(모듈 7칸 ~ 70px)만 한 검은 영역을 못 메운다. 상자를
    #   합치는 쪽이 커널을 키우는 것보다 안전하다 — 커널을 키우면 라벨이
    #   선반 배경까지 빨아들인다.
    boxes = _merge_nearby_boxes(raw)

    out = []
    for (x, y, cw, ch) in boxes:
        if not (min_px <= cw <= w * max_frac and min_px <= ch <= h * max_frac):
            continue
        if not (0.5 <= cw / float(ch) <= 2.0):     # 라벨은 정사각형에 가깝다
            continue
        out.append((x, y, cw, ch))
    out.sort(key=lambda b: -b[2] * b[3])
    return out


def _texture_quad_candidates(gray, min_px=60, max_frac=0.5):
    """QR 라벨처럼 **흑백 전환이 조밀한** 영역의 bbox 를 큰 것부터 돌려준다.

    ★ 왜 밝기(_bright_quad_candidates)만으로는 부족한가 — 라벨이 배경보다
      어두울 수 있다. 실측: shelf_2 매거진은 180도 돌아 있어 조명을 반대로
      받는다. 그 라벨은 뒤쪽 흰 벽보다 **어둡게** 찍혀서, 밝기 임계(200)로는
      벽이 잡히고 라벨은 안 잡힌다 — 후보가 0~1개 나오는데 전부 엉뚱한 자리라
      꼭짓점을 하나도 못 얻었다(0/3 프레임). 같은 프레임에서 이 대비 기반
      탐색은 179x178 짜리 라벨을 정확히 집어내고 cv2.detect 도 성공한다.

      QR 은 밝기와 무관하게 "검은 모듈과 흰 모듈이 촘촘히 번갈아 나오는 사각형"
      이다. 그 성질을 직접 본다 — Laplacian 으로 경계 밀도를 재고, 닫기로
      뭉쳐서 덩어리를 만든다.
    """
    h, w = gray.shape[:2]
    g = cv2.GaussianBlur(gray, (5, 5), 0)
    edge = cv2.convertScaleAbs(cv2.Laplacian(g, cv2.CV_32F, ksize=3))
    _, bw = cv2.threshold(edge, 40, 255, cv2.THRESH_BINARY)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((31, 31), np.uint8))
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        x, y, cw, ch = cv2.boundingRect(c)
        if not (min_px <= cw <= w * max_frac and min_px <= ch <= h * max_frac):
            continue
        if not (0.5 <= cw / float(ch) <= 2.0):     # 라벨은 정사각형에 가깝다
            continue
        out.append((x, y, cw, ch))
    out.sort(key=lambda b: -b[2] * b[3])
    return out


def _label_candidates(gray):
    """라벨 후보 — 대비 기반을 먼저, 밝기 기반을 그다음에.

    대비 기반이 밝기와 무관해서 더 일반적이다(실측 6/6). 밝기 기반은 남겨
    둔다 — 두 방식이 서로 다른 프레임에서 성공한 적이 있어 버릴 이유가 없다.
    """
    seen, out = set(), []
    for b in list(_texture_quad_candidates(gray)) + list(_bright_quad_candidates(gray)):
        if b in seen:
            continue
        seen.add(b)
        out.append(b)
    return out


def _merge_nearby_boxes(boxes, pad_frac=0.35, max_iters=4):
    """맞닿거나 가까운 bbox 를 하나로 합친다 (합쳐진 것 + 원본 둘 다 돌려준다).

    원본도 남기는 이유: 합친 상자가 배경까지 삼켜 버린 경우에도 원래 조각으로
    한 번 더 시도할 수 있게 하려는 것이다. 호출부가 큰 것부터 몇 개만 쓴다.
    """
    cur = [tuple(b) for b in boxes]
    merged = list(cur)
    for _ in range(max_iters):
        used = [False] * len(cur)
        nxt = []
        changed = False
        for i, (x1, y1, w1, h1) in enumerate(cur):
            if used[i]:
                continue
            px, py = pad_frac * w1, pad_frac * h1
            ax1, ay1, ax2, ay2 = x1 - px, y1 - py, x1 + w1 + px, y1 + h1 + py
            for j in range(i + 1, len(cur)):
                if used[j]:
                    continue
                x2, y2, w2, h2 = cur[j]
                qx, qy = pad_frac * w2, pad_frac * h2
                bx1, by1, bx2, by2 = x2 - qx, y2 - qy, x2 + w2 + qx, y2 + h2 + qy
                if ax1 < bx2 and bx1 < ax2 and ay1 < by2 and by1 < ay2:
                    nx1, ny1 = min(x1, x2), min(y1, y2)
                    nx2, ny2 = max(x1 + w1, x2 + w2), max(y1 + h1, y2 + h2)
                    x1, y1, w1, h1 = nx1, ny1, nx2 - nx1, ny2 - ny1
                    px, py = pad_frac * w1, pad_frac * h1
                    ax1, ay1, ax2, ay2 = x1 - px, y1 - py, x1 + w1 + px, y1 + h1 + py
                    used[j] = True
                    changed = True
            used[i] = True
            nxt.append((x1, y1, w1, h1))
        cur = nxt
        merged.extend(cur)
        if not changed:
            break
    # 중복 제거(순서 유지)
    seen, out = set(), []
    for b in merged:
        if b not in seen:
            seen.add(b)
            out.append(b)
    return out


def _decode_text(image):
    """텍스트만 얻는다. cv2 가 실패하면 WeChat 으로 한 번 더."""
    text, _, _ = cv2.QRCodeDetector().detectAndDecode(image)
    if text:
        return text
    wechat = _wechat_detector()
    if wechat is None:
        return ""
    try:
        texts, _ = wechat.detectAndDecode(image)
    except Exception:
        return ""
    return texts[0] if texts else ""


def detect_qr_quads(gray_or_bgr, allow_external=True):
    """QR 후보 사각형을 전부 돌려준다. decode 는 시도하되 실패해도 포함한다.

    반환: [(corners_4x2, decoded_text_or_''), ...]

    allow_external : False 면 _external_decode() (상주 WeChat 프로세스에
        이미지를 PNG 로 떨궈 IPC 로 물어보는 경로, 2026-09-26 추가)를 아예
        안 부른다 — patrol 중 고빈도 폴링(5Hz)처럼 매 프레임 디스크 I/O +
        IPC 왕복 지연을 감당 못 하는 자리에서 쓴다(사용자 지시: "이동하면서
        스캔하는거라 지연이 생기면 안 된다"). 그 자리에서는 예전처럼 로컬
        cv2.QRCodeDetector 만으로 판정한다 — 정확도는 떨어질 수 있지만
        patrol 은 원래도 "못 봤으면 다음 tick 에 다시 본다" 는 폴링이라
        속도가 정확도보다 중요하다. 멈춰서 확정하는 자리(carrier_scan
        서비스, PKG-OUT SCAN)는 기본값(True)을 그대로 써서 WeChat 을 쓴다.

    detectAndDecodeMulti() 를 우선 쓰지 않는다 — 이 OpenCV 빌드(4.6)에서 QR
    이 하나만 있을 때도 detectAndDecodeMulti 가 종종 decode 에 실패하는데
    (검출은 된다) 단일용 detectAndDecode() 는 같은 이미지에서 성공한다
    (합성 테스트로 확인). 그래서 단일 경로를 먼저 시도하고, 거기서 안 되면
    다중 QR 이 실제로 있을 수 있는 경우를 대비해 Multi 로 보강한다.

    detect() 는 파인더 패턴 3 개만 있으면 되므로 decode() 보다 더 먼 거리 /
    더 낮은 해상도에서도 성공한다 (qr_decode_range.py 실측 근거) — presence
    용도(3-1)에서는 decode 가 안 돼도 이 결과를 쓴다.
    """
    # ── WeChat 이 있으면 먼저 쓴다 (기울기에 강하다) ──────────────────
    wechat = _wechat_detector()
    if wechat is not None:
        try:
            texts, pts_list = wechat.detectAndDecode(gray_or_bgr)
        except Exception:
            texts, pts_list = (), ()
        # ★ 이 빌드의 WeChat 은 꼭짓점으로 **입력 이미지 경계**를 그대로 돌려준다
        #   (실측: 2560x1440 프레임에 [[0,0],[2559,0],[2559,1439],[0,1439]]).
        #   그대로 쓰면 평면 적합 ROI 가 화면 전체가 되어 자세가 엉뚱해진다.
        #   그래서 '이미지 전체에 가까운 사각형' 은 버린다 — 디코딩 텍스트만
        #   쓸모가 있고, 꼭짓점은 아래 cv2 경로에서 얻어야 한다.
        h, w = gray_or_bgr.shape[:2]
        hits = []
        for t, pt in zip(texts, pts_list):
            if not t:
                continue
            q = np.asarray(pt, dtype=np.float64).reshape(4, 2)
            area = cv2.contourArea(q.astype(np.float32))
            if area > 0.5 * w * h:      # 화면의 절반을 넘으면 경계 반환이다
                continue
            hits.append((q, t))
        if hits:
            return hits

    # ★ 로컬 cv2 에 WeChat 이 없으면(예: Isaac 번들) 프레임당 **한 번**만
    #   외부 디코더로 텍스트를 받아 둔다. 꼭짓점은 아래 cv2 경로가 낸다 —
    #   실패하던 건 언제나 decode 쪽이지 검출 쪽이 아니었다
    #   ("검출만 된 사각형은 있었을 수 있다"가 그 로그다).
    ext_text = _external_decode(gray_or_bgr) if allow_external else ""
    dbg = os.environ.get("QR_DEBUG") == "1"

    detector = cv2.QRCodeDetector()
    out = []

    text, pts, _ = detector.detectAndDecode(gray_or_bgr)
    if pts is not None:
        # ★ text 가 비면 ext_text 로 메운다. 예전엔 여기서 빈 문자열인 채로
        #   out 에 넣고 아래 'if out: return out' 로 **조기 반환**해 버려서,
        #   외부 디코더 결과도 crop 폴백도 못 써 보고 끝났다. 실측: 전체
        #   프레임에서 detect 가 되는 쪽(robot2)만 그 경로를 타서 혼자
        #   0/3 으로 실패했고, detect 가 안 되는 쪽(robot1)은 폴백까지 내려가
        #   2/3 으로 성공했다 — 같은 장면인데 결과가 갈렸다.
        out.append((np.asarray(pts, dtype=np.float64).reshape(4, 2), text or ext_text))
        if out[-1][1]:
            return out       # 흔한 경우(QR 한 개, decode 성공) — 바로 끝낸다

    ok, decoded_infos, points, _ = detector.detectAndDecodeMulti(gray_or_bgr)
    if ok and points is not None:
        for text2, pts2 in zip(decoded_infos, np.asarray(points)):
            if text2:
                out.append((np.asarray(pts2, dtype=np.float64).reshape(4, 2), text2))
    if any(t for _, t in out):
        return [o for o in out if o[1]] or out

    # 그래도 텍스트가 없으면 detect() 단독으로 재시도한다.
    # (out 이 이미 차 있으면 같은 사각형을 또 넣지 않는다)
    if not out:
        found, pts = detector.detect(gray_or_bgr)
        if found and pts is not None:
            out.append((np.asarray(pts, dtype=np.float64).reshape(4, 2), ext_text))
            if out[-1][1]:
                return out

    # ── 마지막 수단: 밝은 라벨 후보를 잘라 그 안에서 다시 찾는다 ──────
    #   화면 전체로는 못 찾아도 라벨 주변만 주면 찾는다(위 주석 참고).
    gray = gray_or_bgr if gray_or_bgr.ndim == 2 else cv2.cvtColor(gray_or_bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    cands = _label_candidates(gray)
    if dbg:
        print(f"      [qr] ext={ext_text!r}  out={len(out)}  밝은후보={len(cands)}  "
              f"상위={[tuple(c) for c in cands[:3]]}")
    for (x, y, cw, ch) in cands[:6]:
        margin = int(0.25 * max(cw, ch))
        ox, oy = max(0, x - margin), max(0, y - margin)
        sub = gray[oy:min(h, y + ch + margin), ox:min(w, x + cw + margin)]
        ok, pts = detector.detect(sub)
        if not (ok and pts is not None):
            continue
        quad = np.asarray(pts, dtype=np.float64).reshape(4, 2) + np.array([ox, oy], dtype=np.float64)
        sides = [float(np.linalg.norm(quad[i] - quad[(i + 1) % 4])) for i in range(4)]
        if min(sides) / max(sides) < 0.7:      # 정사각형에서 너무 멀면 QR 이 아니다
            continue
        # ★ 텍스트는 **이 crop 에 대해** 얻는다. 프레임 전체의 ext_text 를 그냥
        #   갖다 붙이면, 라벨이 아닌 밝은 사각형(선반 모서리 등)에도 '1' 이
        #   붙어서 그대로 반환된다 — 실측: 그렇게 잡힌 엉뚱한 꼭짓점으로 평면을
        #   맞춰 "벽면 법선이 77.3도 기울어 있다" 로 끝났다. crop 단위로 물어야
        #   아닌 것은 빈 문자열이 되어 다음 후보로 넘어간다.
        txt = _decode_text(sub) or (_external_decode(sub) if allow_external else "")
        out.append((quad, txt))
        if txt:
            return out
    return out


# ══════════════════════════════════════════════════════════════
#  평면 피팅
# ══════════════════════════════════════════════════════════════
def _fit_plane_robust(points, max_iters=4, thresh_m=0.003, min_points=30):
    """3D 점에 평면을 맞추고, 평면에서 thresh_m 넘게 벗어난 점을 반복해서 버린다.

    레일(벽보다 4 mm 튀어나옴), 배경(선반 안쪽), 매거진 본체 윗면 등 다른
    깊이의 물체가 탐색창에 섞여 들어와도 반복하면서 벽면만 남는다.

    한계: 오염된 점이 절반 가까이 되면(예: 넓은 한 덩어리 장애물) 첫 라운드의
    평균이 이미 두 무리 사이로 쏠려서 못 갈라질 수 있다 — 호출부(estimate_qr_pose)
    에서 QR 자체(항상 벽에 붙어 있다)의 좁은 패치로 먼저 대략의 평면을 잡고
    그걸로 넓은 탐색창을 걸러 낸 뒤 여기 넘기는 2 단계 방식을 쓴다.
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < min_points:
        return None
    centroid = normal = None
    for it in range(max_iters):
        centroid = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
        normal = vt[-1]
        resid = (pts - centroid) @ normal
        inlier = np.abs(resid) < thresh_m
        if inlier.sum() < min_points:
            break
        if inlier.all():
            pts = pts[inlier]
            break
        pts = pts[inlier]
        thresh_m = max(thresh_m * 0.6, 0.0008)     # 다음 라운드는 더 엄격하게
    if normal is None or len(pts) < min_points:
        return None
    centroid = pts.mean(axis=0)
    _, s, vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = vt[-1]
    resid = (pts - centroid) @ normal
    rms = float(np.sqrt(np.mean(resid ** 2)))
    return centroid, normal, rms, len(pts)


def _wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ══════════════════════════════════════════════════════════════
#  본체 — 한 프레임
# ══════════════════════════════════════════════════════════════
def estimate_qr_pose(bgr, depth, K, dist, R_wo, p_wo, *,
                     expected_id=None, roi_margin=2.5,
                     plane_thresh_m=0.003, min_plane_points=40,
                     max_wall_tilt_deg=8.0, cross_check_warn_mm=8.0,
                     data_side_m=DATA_SIDE_M, allow_external=True):
    """QR 한 장을 검출해 깊이-평면 기반 pose 를 잰다.

    expected_id   : 있으면 그 ID 로 디코딩된 것만 받는다 (여러 QR 이 보일 때)
    allow_external: detect_qr_quads() 참고 — patrol 고빈도 폴링처럼 지연을
                    못 견디는 자리는 False 로 호출한다.
    roi_margin    : 꼭짓점 bbox 대비 평면 탐색창 확장 배수. wall_n 이 QR 라벨
                    보다 훨씬 넓어서(250x110 vs 50x50) 키울수록 평면 피팅이
                    안정된다. 너무 크면 다른 매거진/선반이 섞일 수 있다.
    """
    K = np.asarray(K, dtype=float)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=float)
    R_wo = np.asarray(R_wo, dtype=float)
    p_wo = np.asarray(p_wo, dtype=float)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    H, W = gray.shape[:2]

    candidates = detect_qr_quads(gray, allow_external=allow_external)
    quad = decoded = None
    for c, text in candidates:
        if text == "":
            continue
        if expected_id is None or text == expected_id:
            quad, decoded = c, text
            break
    if quad is None:
        return QRPoseObservation(False, "decode 실패 (검출만 된 사각형은 있었을 수 있다)"
                                 if candidates else "QR 이 안 보인다")

    # ── PnP (교차검증용) ──
    h = data_side_m / 2.0
    obj_pts = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    okp, rvec, tvec = cv2.solvePnP(obj_pts, quad, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    pnp_center_world = np.full(3, np.nan)
    pnp_yaw = float("nan")
    if okp:
        R_pnp, _ = cv2.Rodrigues(rvec)
        pnp_center_world = R_wo @ tvec.reshape(3) + p_wo
        R_pnp_world = R_wo @ R_pnp @ _R_CV_TO_QR
        pnp_yaw = math.atan2(R_pnp_world[1, 0], R_pnp_world[0, 0])

    # ── 평면 탐색창 ──
    c_min, c_max = quad.min(axis=0), quad.max(axis=0)
    size = c_max - c_min
    pad = size * (roi_margin - 1.0) / 2.0
    u0, v0 = np.clip(c_min - pad, 0, [W - 1, H - 1])
    u1, v1 = np.clip(c_max + pad, 0, [W - 1, H - 1])
    u0, v0, u1, v1 = int(u0), int(v0), int(u1) + 1, int(v1) + 1
    if depth is None:
        return QRPoseObservation(
            False, "깊이 영상이 없다 (roll/pitch 를 평면에서 못 얻는다)",
            decoded=decoded, pnp_center_world=pnp_center_world, pnp_yaw_rad=pnp_yaw)
    d = np.asarray(depth, dtype=np.float64).reshape(H, W)[v0:v1, u0:u1]
    vv, uu = np.nonzero(np.isfinite(d) & (d > 0))
    if len(vv) < min_plane_points:
        return QRPoseObservation(False, f"탐색창 안 유효 깊이 점이 {len(vv)}개뿐이다",
                                 decoded=decoded, pnp_center_world=pnp_center_world,
                                 pnp_yaw_rad=pnp_yaw)
    px = np.column_stack([uu + u0, vv + v0]).astype(np.float64)
    pts3d = backproject_depth(K, dist, R_wo, p_wo, px, d[vv, uu])

    # 시드: QR 자체(꼭짓점 안쪽, 축소해 안티에일리어싱된 가장자리를 피한다)의
    # 깊이 중앙값. 라벨은 항상 벽에 붙어 있으므로 레일 등 다른 깊이가 섞일
    # 걱정 없이 '진짜 벽' 위의 한 점을 준다.
    # 2 단계 평면 피팅. QR 자체(꼭짓점 안쪽, 축소해 안티에일리어싱된 가장자리를
    # 피한다)는 항상 벽에 딱 붙어 있으므로(레일 위에 인쇄되지 않는다) 먼저 그
    # 좁은 패치로 대략의 평면을 잡고, 그 평면과의 수직거리로 넓은 탐색창을
    # 걸러낸 뒤에야 최종 반복 피팅을 돌린다. 이렇게 안 하면 벽 면적의 상당
    # 부분을 차지하는 장애물(주기적인 레일 등)이 섞였을 때 전체 평균이 이미
    # 두 무리 사이로 쏠려 반복 피팅으로도 못 갈라지는 경우가 있다.
    inner_mask = np.zeros((v1 - v0, u1 - u0), np.uint8)
    quad_c = quad.mean(axis=0)
    inner = (quad_c + 0.7 * (quad - quad_c)) - [u0, v0]      # ROI 로컬 좌표
    cv2.fillConvexPoly(inner_mask, inner.astype(np.int32), 255)
    inside = inner_mask[vv, uu] > 0
    pts_for_fit = pts3d
    if inside.sum() >= 8:
        seed_fit = _fit_plane_robust(pts3d[inside], thresh_m=0.0015, min_points=8)
        if seed_fit is not None:
            s_centroid, s_normal, _, _ = seed_fit
            resid = (pts3d - s_centroid) @ s_normal
            filtered = pts3d[np.abs(resid) < plane_thresh_m]
            if len(filtered) >= min_plane_points:
                pts_for_fit = filtered

    fit = _fit_plane_robust(pts_for_fit, thresh_m=plane_thresh_m, min_points=min_plane_points)
    if fit is None:
        return QRPoseObservation(False, "평면 피팅 실패 (이상치 제거 후 점이 부족)",
                                 decoded=decoded, pnp_center_world=pnp_center_world,
                                 pnp_yaw_rad=pnp_yaw)
    centroid, normal, rms, n_used = fit

    # 법선을 카메라 쪽(바깥)으로 향하게 정렬
    if np.dot(normal, p_wo - centroid) < 0:
        normal = -normal

    tilt_deg = math.degrees(math.asin(np.clip(abs(normal[2]), -1.0, 1.0)))
    if tilt_deg > max_wall_tilt_deg:
        return QRPoseObservation(
            False, f"벽면 법선이 {tilt_deg:.1f}도 기울어 있다 (매거진이 안 서 있거나 "
                   f"평면을 잘못 잡았다)", decoded=decoded, plane_rms_mm=rms * 1000,
            n_plane_points=n_used, pnp_center_world=pnp_center_world, pnp_yaw_rad=pnp_yaw)

    # QR 프레임: z = -법선(안쪽), y = 세계 -Z(라벨은 항상 수직), x = y × z
    z_qr = -normal / np.linalg.norm(normal)
    y_raw = np.array([0.0, 0.0, -1.0])
    x_qr = np.cross(y_raw, z_qr)
    x_qr /= np.linalg.norm(x_qr)
    y_qr = np.cross(z_qr, x_qr)
    yaw = math.atan2(x_qr[1], x_qr[0])

    # 위치: 검출된 QR 중심 픽셀을 이 평면에 광선-평면 교차
    center_px = quad.mean(axis=0, keepdims=True)
    center_world = ray_plane_intersect(K, dist, R_wo, p_wo, center_px,
                                       plane_point=centroid, plane_normal=normal)[0]

    cross_check_mm = (float(np.linalg.norm(center_world - pnp_center_world) * 1000)
                      if okp else float("nan"))
    reason = ""
    if okp and cross_check_mm > cross_check_warn_mm:
        reason = (f"PnP 와 위치가 {cross_check_mm:.1f} mm 차이난다 "
                  f"(참고용 경고 — depth 결과를 신뢰하되 원인 확인 권장)")

    return QRPoseObservation(
        ok=True, reason=reason, decoded=decoded, center_world=center_world, yaw_rad=yaw,
        wall_tilt_deg=tilt_deg, plane_rms_mm=rms * 1000.0, n_plane_points=n_used,
        pnp_center_world=pnp_center_world, pnp_yaw_rad=pnp_yaw,
        cross_check_mm=cross_check_mm, corners_px=quad)


# ══════════════════════════════════════════════════════════════
#  다중 프레임 평균 + 이상치 제거
# ══════════════════════════════════════════════════════════════
def _circular_mean(angles):
    v = np.mean([[math.cos(a), math.sin(a)] for a in angles], axis=0)
    return math.atan2(v[1], v[0])


def aggregate_qr_poses(observations, pos_mad_k=3.5, yaw_mad_k_deg=3.5, min_mad_deg=0.05):
    """여러 프레임의 QRPoseObservation 을 중앙값 기준 MAD 로 이상치 제거 후 평균한다.

    한 개만 있어도 동작한다(그대로 돌려준다). 전부 이상치로 떨어지면(=편차가
    다 커서 아무것도 안 남으면) 방어적으로 전부 쓴다 — 아무것도 안 돌려주는
    것보다는 낫다.
    """
    ok = [o for o in observations if o.ok]
    if not ok:
        reasons = [o.reason for o in observations if o.reason]
        return AggregatedQRPose(False, reason="; ".join(reasons) or "유효 관측 없음",
                                n_total=len(observations))
    decoded_set = {o.decoded for o in ok if o.decoded}
    if len(decoded_set) > 1:
        return AggregatedQRPose(False, f"서로 다른 ID 가 섞였다: {decoded_set}",
                                n_total=len(observations))
    decoded = next(iter(decoded_set), "")

    pos = np.array([o.center_world for o in ok])
    yaw = np.array([o.yaw_rad for o in ok])

    med_pos = np.median(pos, axis=0)
    mad_pos = np.median(np.abs(pos - med_pos), axis=0) * 1.4826 + 1e-6
    keep_pos = np.all(np.abs(pos - med_pos) < pos_mad_k * mad_pos, axis=1)

    med_yaw = _circular_mean(yaw.tolist())        # 초기 기준점 (평균으로 근사)
    dyaw = np.array([_wrap_pi(y - med_yaw) for y in yaw])
    mad_yaw = max(np.median(np.abs(dyaw)) * 1.4826, math.radians(min_mad_deg))
    keep_yaw = np.abs(dyaw) < math.radians(yaw_mad_k_deg) if mad_yaw == 0 else \
        np.abs(dyaw) < yaw_mad_k_deg * mad_yaw

    keep = keep_pos & keep_yaw
    if not keep.any():
        keep = np.ones_like(keep, dtype=bool)

    final_pos = pos[keep].mean(axis=0)
    final_yaw = _circular_mean(yaw[keep].tolist())
    pos_std_mm = pos[keep].std(axis=0) * 1000.0 if keep.sum() > 1 else np.zeros(3)
    yaw_std = (float(np.std([_wrap_pi(y - final_yaw) for y in yaw[keep]]))
              if keep.sum() > 1 else 0.0)

    return AggregatedQRPose(
        ok=True, decoded=decoded, center_world=final_pos, yaw_rad=final_yaw,
        n_used=int(keep.sum()), n_total=len(observations),
        pos_std_mm=pos_std_mm, yaw_std_deg=math.degrees(yaw_std))


def draw_observation(bgr, obs, K, dist, R_wo, p_wo):
    """확인용 그림."""
    vis = bgr.copy()
    if np.all(np.isfinite(obs.corners_px)):
        cv2.polylines(vis, [obs.corners_px.astype(np.int32)], True, (0, 255, 255), 1)
    if obs.ok:
        c = obs.center_world
        ax = c + 0.04 * np.array([math.cos(obs.yaw_rad), math.sin(obs.yaw_rad), 0.0])
        pc, _ = project_points(K, dist, R_wo, p_wo, np.vstack([c, ax]))
        cv2.arrowedLine(vis, tuple(pc[0].astype(int)), tuple(pc[1].astype(int)),
                        (0, 0, 255), 2, tipLength=0.2)
        text = (f"OK '{obs.decoded}' yaw {math.degrees(obs.yaw_rad):+.2f} deg  "
                f"tilt {obs.wall_tilt_deg:.2f} deg  n={obs.n_plane_points}")
    else:
        text = f"FAIL {obs.reason[:70]}"
    cv2.putText(vis, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return vis

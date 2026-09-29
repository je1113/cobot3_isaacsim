#!/usr/bin/env python3
"""
시연 영상 빌드 스크립트 — docs/demo_video_prompt.md 를 그대로 코드로 옮긴 것.

    python3 build_demo.py --clips <클립 폴더> --font <NotoSansKR.ttf> --out demo.mp4

클립 폴더에는 파일명 첫 글자가 챕터 번호(1~9)인 webm 이 있어야 한다.
출력: 1280×720 · 30 fps · H.264 · 무음. 색·자막·컷 포인트는 프롬프트 문서 §1-1, §2, §4 기준.
의존: opencv-python-headless · pillow · imageio-ffmpeg (ffmpeg 바이너리).
"""
import argparse
import glob
import os
import subprocess
import sys
from collections import deque

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1280, 720, 30
XF = 15  # 크로스 디졸브 프레임 수 (0.5 s)

# ── 팔레트 (§1-1, 실제 클립 프레임에서 뽑은 색) ─────────────────────────
C = dict(
    bg=(0xE0, 0xE3, 0xE1), bg2=(0xD4, 0xD6, 0xD5), bg3=(0xBF, 0xC4, 0xC4),
    line=(0xAF, 0xB5, 0xB4), sub=(0x60, 0x65, 0x62), panel=(0x2C, 0x47, 0x4C),
    text=(0xE0, 0xE3, 0xE1), acc1=(0x68, 0xA7, 0xB1), acc2=(0x44, 0x68, 0x90),
)

PROCESS = ["시작", "순찰", "Pick", "순찰선 복귀", "이송", "Place", "스택 Pick", "스택 Place"]
DOCK_MINI = ["접근", "진입", "충전", "이탈", "복귀"]

TITLE = "AMR Carrier Transfer System"
SLOGAN = "순찰에서 이송, 회수, 충전까지 — 사람 개입 없이, 모든 단계가 기록으로 남는다"

# ── 챕터 정의 (§2 자막, §4 컷 포인트) ───────────────────────────────────
# segs: (원본 시작 s, 원본 끝 s, 배속[, 부제 덮어쓰기, 미니 바 인덱스])
# hl:   (원본 시작, 원본 끝, 제목, 부제) — 배속 없이 1× 로 확대 리플레이
CHAPTERS = [
    dict(n=1, clip=1, code="START", title="Start",
         card="두 로봇이 같은 주차 패드에서 출발한다. 순찰 시작점으로 이동한 뒤 팔을 관측 자세로 잡고 순찰에 들어간다.",
         sub="주차 패드 → 순찰 시작점 · 팔 관측 자세 → 순찰 시작",
         state="state=start → pose → patrol",
         segs=[(0, 95.3, 3)],
         extra=[(60, 78, "두 로봇의 출발 지점이 같아 출발이 서로 막히지 않도록 robot2 는 60 s 늦게 출발하도록 설계 (start_delay_s)")],
         hl=None,
         badge=("✓ 01 Start 완료", "robot1 순찰 시작 · robot2 60 s 뒤 출발")),
    dict(n=2, clip=2, code="PATROL", title="Patrol",
         card="선반 정차점 사이를 왕복하며 손목 카메라로 QR을 폴링한다. 보이면 그 자리에 선다.",
         sub="선반 정차점 왕복 · QR 감지 → 정지 · 판독",
         state="state=patrol → hold → scan",
         segs=[(0, 20, 6), (20, 65, 3), (65, 84, 3)],
         hl=(68, 74, "QR 감지 → 정지", "팔이 선반을 향해 멈추는 순간"),
         badge=("✓ 02 Patrol 완료", "QR 감지 → HOLD → SCAN")),
    dict(n=3, clip=3, code="PICK", title="Pick",
         card="멈춘 그 자리에서 판독한 위치로 팔을 내려 흡착 컵으로 매거진을 집는다.",
         sub="흡착 컵 Ø50 · 매거진 파지",
         state="state=pick detected=True",
         segs=[(0, 33.9, 3)],
         hl=(7, 13, "흡착 파지", "흡착 컵이 상자에 닿아 들어 올리는 순간"),
         badge=("✓ 03 Pick 완료", "매거진 파지 → 등판 적재")),
    dict(n=4, clip=4, code="RETURN TO ROUTE", title="Return to Route",
         card="선반 앞 좁은 통로에서 바로 장거리 플래닝을 시키지 않는다. 순찰이 검증한 경로로 시작점까지 먼저 빠져나온다.",
         sub="좁은 통로 → 순찰 시작점 · Nav2 전 안전 지점 확보",
         state="state=return_to_start",
         segs=[(0, 14, 6), (14, 32.5, 3)],
         hl=None,
         badge=("✓ 04 순찰선 복귀", "patrol_to(cmd_vel) → patrol_route[0]")),
    dict(n=5, clip=5, code="NAV", title="Nav",
         card="여유 있는 지점에서 Nav2에 로더까지 주행을 맡긴다. 상대 로봇이 로더 차선을 쓰고 있으면 대기 후 진입한다.",
         sub="Nav2 로 로더까지 주행",
         state="state=nav",
         segs=[(0, 42.7, 3)],
         hl=None,
         badge=("✓ 05 이송 완료", "로더 정차점 도착")),
    dict(n=6, clip=6, code="PLACE", title="Place",
         card="정차점에서 벨트 쪽으로 천천히 들어가 매거진을 내려놓고 같은 선으로 되나온다.",
         sub="크립 진입 → 로더에 배치 → 후진",
         state="state=creep_in → place → creep_out",
         segs=[(0, 36.1, 3)],
         hl=(27, 33, "로더 배치", "벨트에 내려놓는 순간"),
         badge=("✓ 06 Place 완료", "매거진 → 로더 · TraceEvent 기록")),
    dict(n=7, clip=7, code="STACK PICK", title="Stack Pick",
         card="웹이 배차한 RECOVER 작업. 패키지 언로더에 나온 스택을 판독하고 집은 뒤 대기점으로 물러난다.",
         sub="언로더 대기 → 스택 판독 · 파지 → 대기점 후퇴",
         state="state=stack_scan → stack_pick → stack_retreat",
         segs=[(0, 18, 7), (18, 80.5, 3)],
         hl=(28, 34, "스택 파지", "패키지 언로더에서 스택을 집는 순간"),
         badge=("✓ 07 Stack Pick 완료", "웹 배정(pending_pickup) → 스택 파지")),
    dict(n=8, clip=8, code="STACK PLACE", title="Stack Place",
         card="대기점에서 Nav2로 검사 스테이션까지 가서 투입구에 스택을 놓는다.",
         sub="검사 스테이션 주행 · 투입구 배치",
         state="state=stack_deliver → stack_place",
         segs=[(0, 33, 6), (33, 58.9, 3)],
         hl=(50, 56, "스택 배치", "검사 스테이션 벨트에 놓는 순간"),
         badge=("✓ 08 Stack Place 완료", "사이클 종료 · 8 / 8 COMPLETE")),
    dict(n=9, clip=9, code="DOCKING SYSTEM", title="Docking", new=True,
         card="작업을 마친 로봇의 가동시간이 15분을 넘으면 복귀 대신 도크로 향한다. 충전이 끝나면 순찰 시작점으로 돌아가 다시 일한다.",
         sub="가동시간 초과 → 도크 복귀 · 충전 · 재출발",
         state="state=dock_nav → dock_in → dock → dock_out → returning",
         segs=[(0, 136, 6, "배터리(가동시간) 15분 초과 · 다음 작업 대신 도크로", 0),
               (136, 144, 3, "도크 앞 대기점에서 직진 진입", 1),
               (150, 152.5, 1, "충전 중 · 이 동안 웹 작업은 거절되고 상대 로봇에게 넘어간다", 2),
               (288, 300, 3, "후진 이탈 · 도크 구역에서는 회전하지 않는다", 3),
               (300, 331, 3, "순찰 시작점으로 복귀 · 가동시간 타이머 0", 4)],
         hl=None,
         badge=("✓ Docking 완료", "충전 후 순찰 시작점 복귀 · 가동시간 타이머 0")),
]


# ── 폰트 ────────────────────────────────────────────────────────────────
class Fonts:
    def __init__(self, path):
        self.path = path
        self._cache = {}

    def get(self, size, weight=400):
        k = (size, weight)
        if k not in self._cache:
            f = ImageFont.truetype(self.path, size)
            try:
                f.set_variation_by_axes([weight])
            except Exception:
                pass
            self._cache[k] = f
        return self._cache[k]


FONTS = None


# ── 오버레이 (PIL 로 그려 bbox 만 잘라 둔다) ─────────────────────────────
class Ov:
    def __init__(self, rgba):
        a = np.array(rgba)
        ys, xs = np.where(a[:, :, 3] > 0)
        if len(ys) == 0:
            self.empty = True
            return
        self.empty = False
        y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
        sub = a[y0:y1, x0:x1]
        self.x, self.y = x0, y0
        self.bgr = sub[:, :, :3][:, :, ::-1].astype(np.float32)
        self.a = (sub[:, :, 3:4].astype(np.float32) / 255.0)

    def apply(self, frame, mult=1.0):
        if self.empty or mult <= 0:
            return frame
        h, w = self.a.shape[:2]
        reg = frame[self.y:self.y + h, self.x:self.x + w].astype(np.float32)
        a = self.a * mult
        frame[self.y:self.y + h, self.x:self.x + w] = (reg * (1 - a) + self.bgr * a).astype(np.uint8)
        return frame


def canvas():
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    return im, ImageDraw.Draw(im)


def rgba(c, a=255):
    return (c[0], c[1], c[2], a)


def text_w(d, s, f):
    b = d.textbbox((0, 0), s, font=f)
    return b[2] - b[0]


def wrap(d, s, f, maxw):
    words = s.split(" ")
    lines, cur = [], ""
    for w_ in words:
        t = (cur + " " + w_).strip()
        if text_w(d, t, f) <= maxw:
            cur = t
        else:
            if cur:
                lines.append(cur)
            cur = w_
    if cur:
        lines.append(cur)
    return lines


def rrect(d, box, r, fill):
    d.rounded_rectangle(box, radius=r, fill=fill)


def check(d, cx, cy, r, col):
    d.line([(cx - r * 0.5, cy), (cx - r * 0.1, cy + r * 0.45), (cx + r * 0.55, cy - r * 0.45)], fill=col, width=3)


# ── 각 화면 요소 ──────────────────────────────────────────────────────────
def ov_step_panel(n, title, sub):
    im, d = canvas()
    f_lab, f_t, f_s = FONTS.get(18, 500), FONTS.get(40, 700), FONTS.get(21, 400)
    pad = 22
    lines = wrap(d, sub, f_s, 520)
    h = pad + 22 + 8 + 48 + 6 + 28 * len(lines) + pad - 6
    w = max(text_w(d, title, f_t), max(text_w(d, l, f_s) for l in lines), 200) + pad * 2
    x0, y0 = 40, H - 40 - h
    rrect(d, (x0, y0, x0 + w, y0 + h), 10, rgba(C["panel"], 217))
    d.text((x0 + pad, y0 + pad - 2), f"STEP {n:02d} / 08" if n <= 8 else "DOCKING", font=f_lab, fill=rgba(C["acc1"]))
    d.text((x0 + pad, y0 + pad + 24), title, font=f_t, fill=rgba(C["text"]))
    for i, l in enumerate(lines):
        d.text((x0 + pad, y0 + pad + 24 + 52 + i * 28), l, font=f_s, fill=rgba(C["text"], 230))
    return Ov(im)


def ov_progress(labels, current, done, label="PROCESS", counter=None):
    im, d = canvas()
    f_lab, f_i = FONTS.get(16, 500), FONTS.get(13, 400)
    n = len(labels)
    w = 60 + n * 66
    h = 96
    x0, y0 = W - 40 - w, H - 40 - h
    rrect(d, (x0, y0, x0 + w, y0 + h), 10, rgba(C["panel"], 217))
    d.text((x0 + 18, y0 + 12), label, font=f_lab, fill=rgba(C["acc1"]))
    if counter:
        d.text((x0 + w - 18 - text_w(d, counter, f_lab), y0 + 12), counter, font=f_lab, fill=rgba(C["text"], 200))
    cy = y0 + 54
    xs = [x0 + 30 + 33 + i * 66 for i in range(n)]
    d.line([(xs[0], cy), (xs[-1], cy)], fill=rgba(C["line"], 140), width=2)
    for i, (x, lab) in enumerate(zip(xs, labels)):
        if i in done:
            d.ellipse((x - 9, cy - 9, x + 9, cy + 9), fill=rgba(C["acc2"]))
            check(d, x, cy, 9, rgba(C["text"]))
        elif i == current:
            d.ellipse((x - 10, cy - 10, x + 10, cy + 10), outline=rgba(C["acc1"]), width=3)
            d.ellipse((x - 4, cy - 4, x + 4, cy + 4), fill=rgba(C["acc1"]))
        else:
            d.ellipse((x - 7, cy - 7, x + 7, cy + 7), outline=rgba(C["line"], 180), width=2)
        col = rgba(C["text"]) if i in done or i == current else rgba(C["text"], 140)
        d.text((x - text_w(d, lab, f_i) / 2, cy + 16), lab, font=f_i, fill=col)
    return Ov(im)


def ov_chip(text, y=24, align="right", size=16, weight=500, fg=None, bg=None, x=None):
    im, d = canvas()
    f = FONTS.get(size, weight)
    tw = text_w(d, text, f)
    w, h = tw + 28, size + 18
    if x is None:
        x = W - 40 - w if align == "right" else (W - w) // 2 if align == "center" else 40
    rrect(d, (x, y, x + w, y + h), 8, rgba(bg or C["panel"], 200))
    d.text((x + 14, y + 7), text, font=f, fill=rgba(fg or C["text"]))
    return Ov(im)


def ov_card(n, code, title, desc, new=False):
    im, d = canvas()
    f_lab, f_t, f_d = FONTS.get(22, 500), FONTS.get(64, 700), FONTS.get(24, 400)
    x0, y0 = 90, 210
    lab = f"{n:02d} · {code}"
    d.text((x0, y0), lab, font=f_lab, fill=rgba(C["acc2"]))
    if new:
        lx = x0 + text_w(d, lab, f_lab) + 16
        rrect(d, (lx, y0 + 1, lx + 64, y0 + 27), 6, rgba(C["acc1"]))
        d.text((lx + 12, y0 + 3), "NEW", font=FONTS.get(17, 700), fill=rgba(C["panel"]))
    d.text((x0 - 2, y0 + 36), title, font=f_t, fill=rgba(C["panel"]))
    d.line([(x0, y0 + 122), (x0 + 56, y0 + 122)], fill=rgba(C["acc1"]), width=4)
    for i, l in enumerate(wrap(d, desc, f_d, 640)):
        d.text((x0, y0 + 142 + i * 34), l, font=f_d, fill=rgba(C["sub"]))
    return Ov(im)


def ov_highlight(title, sub):
    im, d = canvas()
    f_lab, f_t, f_s = FONTS.get(18, 500), FONTS.get(40, 700), FONTS.get(21, 400)
    pad = 22
    w = max(text_w(d, title, f_t), text_w(d, sub, f_s)) + pad * 2
    h = 128
    x0, y0 = 40, H - 40 - h
    rrect(d, (x0, y0, x0 + w, y0 + h), 10, rgba(C["panel"], 217))
    d.text((x0 + pad, y0 + pad - 2), "HIGHLIGHT", font=f_lab, fill=rgba(C["acc1"]))
    d.text((x0 + pad, y0 + pad + 22), title, font=f_t, fill=rgba(C["text"]))
    d.text((x0 + pad, y0 + pad + 74), sub, font=f_s, fill=rgba(C["text"], 230))
    return Ov(im)


def ov_badge(head, summary):
    im, d = canvas()
    f_h, f_s = FONTS.get(38, 700), FONTS.get(22, 400)
    pad = 28
    w = max(text_w(d, head, f_h), text_w(d, summary, f_s)) + pad * 2
    h = 118
    x0, y0 = (W - w) // 2, H - 150 - h
    rrect(d, (x0, y0, x0 + w, y0 + h), 12, rgba(C["panel"], 225))
    d.text((x0 + pad, y0 + 18), head, font=f_h, fill=rgba(C["text"]))
    d.text((x0 + pad, y0 + 72), summary, font=f_s, fill=rgba(C["acc1"]))
    return Ov(im)


def ov_centered_text(text, y, size, weight, color, alpha=255):
    im, d = canvas()
    f = FONTS.get(size, weight)
    d.text(((W - text_w(d, text, f)) / 2, y), text, font=f, fill=rgba(color, alpha))
    return Ov(im)


def ov_underline(y, w=72):
    im, d = canvas()
    d.line([((W - w) / 2, y), ((W + w) / 2, y)], fill=rgba(C["acc1"]), width=4)
    return Ov(im)


def ov_chapter_chips(y):
    im, d = canvas()
    names = ["Start", "Patrol", "Pick", "Return", "Nav", "Place", "Stack Pick", "Stack Place", "Docking"]
    f_n, f_t = FONTS.get(13, 500), FONTS.get(18, 600)
    cw, gap = 118, 10
    total = len(names) * cw + (len(names) - 1) * gap
    x = (W - total) // 2
    for i, nm in enumerate(names):
        box = (x, y, x + cw, y + 64)
        rrect(d, box, 8, rgba(C["bg2"], 235))
        d.rectangle((x, y + 60, x + cw, y + 64), fill=rgba(C["acc1"] if i == 8 else C["line"]))
        d.text((x + 12, y + 8), f"{i + 1:02d}", font=f_n, fill=rgba(C["acc2"]))
        if i == 8:
            d.text((x + cw - 40, y + 8), "NEW", font=FONTS.get(12, 700), fill=rgba(C["acc1"]))
        d.text((x + 12, y + 28), nm, font=f_t, fill=rgba(C["panel"]))
        x += cw + gap
    return Ov(im)


def ov_data_left():
    im, d = canvas()
    f_lab, f_t, f_i, f_s = FONTS.get(22, 500), FONTS.get(52, 700), FONTS.get(24, 600), FONTS.get(19, 400)
    x0, y0 = 90, 150
    d.text((x0, y0), "DATA COLLECTION", font=f_lab, fill=rgba(C["acc2"]))
    d.text((x0 - 2, y0 + 36), "작업 로그 및 트레이스 DB", font=f_t, fill=rgba(C["panel"]))
    d.line([(x0, y0 + 108), (x0 + 56, y0 + 108)], fill=rgba(C["acc1"]), width=4)
    return Ov(im)


def ov_data_item(i, head, sub):
    im, d = canvas()
    f_i, f_s, f_n = FONTS.get(24, 600), FONTS.get(19, 400), FONTS.get(16, 700)
    x0, y = 90, 300 + i * 84
    d.ellipse((x0, y + 4, x0 + 30, y + 34), outline=rgba(C["acc2"]), width=2)
    d.text((x0 + 10, y + 7), str(i + 1), font=f_n, fill=rgba(C["acc2"]))
    d.text((x0 + 46, y), head, font=f_i, fill=rgba(C["panel"]))
    d.text((x0 + 46, y + 34), sub, font=f_s, fill=rgba(C["sub"]))
    return Ov(im)


def ov_pipeline(y):
    im, d = canvas()
    f = FONTS.get(17, 500)
    items = ["작업 수행", "/trace/event", "event_logger", "PostgreSQL", "웹 관제"]
    x = 90
    for i, it in enumerate(items):
        w = text_w(d, it, f) + 26
        rrect(d, (x, y, x + w, y + 36), 7, rgba(C["panel"], 225))
        d.text((x + 13, y + 7), it, font=f, fill=rgba(C["text"]))
        x += w + 8
        if i < len(items) - 1:
            d.text((x, y + 6), "→", font=f, fill=rgba(C["sub"]))
            x += 26
    return Ov(im)


# ── 프레임 소스 ──────────────────────────────────────────────────────────
def fit(fr, zoom=1.0):
    h, w = fr.shape[:2]
    s = max(W / w, H / h) * zoom
    fr = cv2.resize(fr, (int(round(w * s)), int(round(h * s))), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC)
    h, w = fr.shape[:2]
    y0, x0 = (h - H) // 2, (w - W) // 2
    return fr[y0:y0 + H, x0:x0 + W].copy()


class Src:
    """webm(GStreamer matroskamux)은 타임스탬프가 불규칙하고 시간 탐색이 안 된다.
    그래서 순차 디코딩만 쓴다. 뒤로 가야 하면 파일을 다시 연다."""

    def __init__(self, path):
        self.path = path
        self._open()
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        cnt = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
        self.dur = cnt / fps  # fps=1000 이면 cnt 가 ms 라 결과는 같다

    def _open(self):
        self.cap = cv2.VideoCapture(self.path)
        self.pos = -1.0
        self.last = None

    def _read(self):
        ok, fr = self.cap.read()
        if not ok:
            return False
        self.last = fr
        self.pos = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        return True

    def frame_at(self, t):
        if t < self.pos:
            self.cap.release()
            self._open()
        while self.pos < t:
            if not self._read():
                break
        return self.last

    def frames(self, a, b, speed, zoom=1.0):
        b = min(b, self.dur - 0.05)
        n = max(1, int(round((b - a) * FPS / speed)))
        for k in range(n):
            fr = self.frame_at(a + k * speed / FPS)
            yield fit(fr, zoom)


# ── 출력 (ffmpeg 파이프 + 크로스 디졸브) ───────────────────────────────
class Out:
    def __init__(self, ffmpeg, path):
        self.p = subprocess.Popen(
            [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{W}x{H}",
             "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "21",
             "-pix_fmt", "yuv420p", "-movflags", "+faststart", path],
            stdin=subprocess.PIPE)
        self.hold = deque()
        self.xf_left = 0
        self.n = 0

    def begin(self, xfade=True):
        if xfade:
            self.xf_left = len(self.hold)
        else:
            self.flush()

    def push(self, fr):
        if self.xf_left > 0 and self.hold:
            prev = self.hold.popleft()
            a = 1.0 - self.xf_left / (XF + 1)
            fr = cv2.addWeighted(prev, 1 - a, fr, a, 0)
            self.xf_left -= 1
        self.hold.append(fr)
        if len(self.hold) > XF:
            self._write(self.hold.popleft())

    def _write(self, fr):
        self.p.stdin.write(fr.tobytes())
        self.n += 1

    def flush(self):
        while self.hold:
            self._write(self.hold.popleft())
        self.xf_left = 0

    def close(self):
        self.flush()
        self.p.stdin.close()
        self.p.wait()


def fade(t, t0, dur=0.4):
    return max(0.0, min(1.0, (t - t0) / dur))


def whiten(fr, amount=0.6, blur=0):
    bg = np.empty_like(fr)
    bg[:] = np.array(C["bg"][::-1], dtype=np.uint8)
    out = cv2.addWeighted(fr, 1 - amount, bg, amount, 0)
    if blur:
        out = cv2.GaussianBlur(out, (0, 0), blur)
    return out


# ── 블록 렌더링 ──────────────────────────────────────────────────────────
def render_intro(out, bgfr):
    bg = whiten(bgfr, 0.7, 6)
    els = [
        (0.0, ov_centered_text("ISAAC SIM · MOBILE MANIPULATOR", 208, 20, 500, C["acc2"])),
        (0.5, ov_centered_text(TITLE, 244, 76, 700, C["panel"])),
        (0.5, ov_underline(352)),
        (1.5, ov_centered_text("순찰 · QR 판독 · Pick & Place · 스택 회수 · 도킹", 372, 26, 400, C["sub"])),
        (3.0, ov_chapter_chips(450)),
        (5.0, ov_chip("작업 지시 방식: 자동 순찰 · 웹 배정 (RECOVER)", y=560, align="center", size=18)),
    ]
    out.begin(xfade=False)
    for k in range(8 * FPS):
        t = k / FPS
        fr = bg.copy()
        for t0, ov in els:
            ov.apply(fr, fade(t, t0))
        out.push(fr)


def render_chapter(out, ch, src, thumbs):
    n = ch["n"]
    is_dock = n == 9
    first = fit(src.frame_at(ch["segs"][0][0]))
    # (a) 챕터 카드 3 s
    card_bg = whiten(first, 0.62, 4)
    card = ov_card(n, ch["code"], ch["title"], ch["card"], new=ch.get("new", False))
    out.begin()
    for k in range(3 * FPS):
        fr = card_bg.copy()
        card.apply(fr, fade(k / FPS, 0.2, 0.5))
        out.push(fr)
    # (b) 본 클립
    state_chip = ov_chip(ch["state"], y=24, align="right", size=15, weight=400)
    done = set(range(n - 1)) if not is_dock else set()
    prog_cache = {}
    thumb = None
    out.begin()
    for seg in ch["segs"]:
        a, b, speed = seg[0], seg[1], seg[2]
        sub = seg[3] if len(seg) > 3 else ch["sub"]
        mini = seg[4] if len(seg) > 4 else None
        panel = ov_step_panel(n, ch["title"], sub)
        if is_dock:
            prog = ov_progress(DOCK_MINI, mini, set(range(mini)), label="DOCKING", counter=f"{mini + 1} / 5")
        else:
            prog = ov_progress(PROCESS, n - 1, done, counter=f"STEP {n:02d} / 08")
        extras = [(ea, eb, ov_chip(txt, y=24, align="center", size=17)) for ea, eb, txt in ch.get("extra", [])]
        k = 0
        for fr in src.frames(a, b, speed):
            t_src = a + k * speed / FPS
            last = fr.copy()
            panel.apply(fr)
            prog.apply(fr)
            state_chip.apply(fr, 0.9)
            for ea, eb, ov in extras:
                if ea <= t_src <= eb:
                    ov.apply(fr, fade(t_src, ea, 1.0) * fade(eb, t_src, 1.0))
            if thumb is None and t_src >= (a + b) / 2:
                thumb = fr.copy()
            out.push(fr)
            k += 1
    # (c) 하이라이트 — 원본 속도 1×, 1.4× 확대
    if ch.get("hl"):
        ha, hb, ht, hs = ch["hl"]
        hov = ov_highlight(ht, hs)
        tag = ov_chip("↗ REPLAY · 확대 화면", y=24, align="right", size=15)
        out.begin()
        i = 0
        for fr in src.frames(ha, hb, 1.0, zoom=1.4):
            last = fr.copy()
            hov.apply(fr)
            tag.apply(fr)
            if i == int((hb - ha) * FPS / 2):
                thumb = fr.copy()
            out.push(fr)
            i += 1
    thumbs.append((f"{n:02d} {ch['title']}", thumb if thumb is not None else last))
    # (d) 완료 배지 2 s
    badge = ov_badge(*ch["badge"])
    if is_dock:
        prog = ov_progress(DOCK_MINI, -1, set(range(5)), label="DOCKING", counter="COMPLETE")
    else:
        prog = ov_progress(PROCESS, -1, done | {n - 1}, counter="COMPLETE" if n == 8 else f"STEP {n:02d} / 08")
    bg = whiten(last, 0.25)
    out.begin()
    for k in range(2 * FPS + 10):
        fr = bg.copy()
        prog.apply(fr)
        badge.apply(fr, fade(k / FPS, 0.1, 0.3))
        out.push(fr)


def render_data(out, bgfr):
    bg = whiten(bgfr, 0.75, 6)
    els = [
        (0.0, ov_data_left()),
        (1.0, ov_data_item(0, "TraceEvent 자동 기록", "단계 · 시각 · 물체 상태를 로봇이 단계마다 발행")),
        (2.0, ov_data_item(1, "append-only 로그 축적", "magazine_log · stack_log — 고치지 않고 행을 더한다")),
        (3.0, ov_data_item(2, "관제와 배차에 활용", "LogsPage · pending_pickup 회수 큐")),
        (5.0, ov_pipeline(570)),
    ]
    out.begin()
    for k in range(8 * FPS):
        t = k / FPS
        fr = bg.copy()
        for t0, ov in els:
            ov.apply(fr, fade(t, t0))
        out.push(fr)


def render_outro(out, bgfr, thumbs):
    bg = whiten(bgfr, 0.75, 6)
    title = ov_centered_text(TITLE, 96, 60, 700, C["panel"])
    ul = ov_underline(184)
    slogan = ov_centered_text(SLOGAN, 600, 24, 400, C["sub"])
    # 썸네일 9장을 두 줄로
    tw, th, gap = 236, 133, 14
    cols = 5
    rows = [(thumbs[:5], 232), (thumbs[5:], 232 + th + gap + 30)]
    thumb_ovs = []
    f_l = FONTS.get(15, 500)
    for row, y in rows:
        total = len(row) * tw + (len(row) - 1) * gap
        x = (W - total) // 2
        for i, (lab, im) in enumerate(row):
            small = cv2.resize(im, (tw, th), interpolation=cv2.INTER_AREA)
            pil = Image.fromarray(small[:, :, ::-1]).convert("RGBA")
            c, d = canvas()
            rrect(d, (x - 3, y - 3, x + tw + 3, y + th + 3), 6, rgba(C["line"]))
            c.paste(pil, (x, y))
            d = ImageDraw.Draw(c)
            d.text((x, y + th + 6), lab, font=f_l, fill=rgba(C["panel"]))
            thumb_ovs.append((1.0 + 0.35 * (len(thumb_ovs)), Ov(c)))
            x += tw + gap
    out.begin()
    for k in range(9 * FPS):
        t = k / FPS
        fr = bg.copy()
        title.apply(fr, fade(t, 0.0))
        ul.apply(fr, fade(t, 0.0))
        for t0, ov in thumb_ovs:
            ov.apply(fr, fade(t, t0))
        slogan.apply(fr, fade(t, 5.5))
        if t > 8.2:
            fr = whiten(fr, fade(t, 8.2, 0.8))
        out.push(fr)


def main():
    global FONTS
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", required=True)
    ap.add_argument("--font", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ffmpeg", default=None)
    ap.add_argument("--only", type=int, default=None, help="챕터 하나만 렌더(테스트용)")
    args = ap.parse_args()
    FONTS = Fonts(args.font)
    ffmpeg = args.ffmpeg
    if not ffmpeg:
        import imageio_ffmpeg
        ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()

    clips = {}
    for p in glob.glob(os.path.join(args.clips, "*.webm")):
        base = os.path.basename(p)
        key = base.split("-", 1)[1] if "-" in base and base.split("-", 1)[0].isalnum() and len(base.split("-", 1)[0]) == 8 else base
        if key[0].isdigit():
            clips[int(key[0])] = p
    missing = [c["clip"] for c in CHAPTERS if c["clip"] not in clips]
    if missing:
        sys.exit(f"클립 없음: {missing}")
    srcs = {k: Src(v) for k, v in clips.items()}

    out = Out(ffmpeg, args.out)
    thumbs = []
    chapters = [c for c in CHAPTERS if args.only is None or c["n"] == args.only]
    if args.only is None:
        render_intro(out, srcs[1].frame_at(2.0))
    for ch in chapters:
        print(f"chapter {ch['n']} {ch['title']}", flush=True)
        render_chapter(out, ch, srcs[ch["clip"]], thumbs)
    if args.only is None:
        render_data(out, srcs[6].frame_at(30.0))
        render_outro(out, srcs[1].frame_at(2.0), thumbs)
    out.close()
    print(f"done: {out.n} frames = {out.n / FPS:.1f} s -> {args.out}")


if __name__ == "__main__":
    main()

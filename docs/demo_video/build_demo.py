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
CARD_S = 9  # 챕터 카드 길이 (s)
TEXT_SPEED = 1.0   # 글자 화면(인트로·카드·배지·데이터·아웃트로)에만 곱하는 배속. --text-speed
ROBOT_SPEED = 1.0  # 로봇 영상(본 클립·하이라이트)에만 곱하는 배속. --robot-speed 로 바꾼다

# ── 팔레트 (§1-1, 실제 클립 프레임에서 뽑은 색) ─────────────────────────
C = dict(
    bg=(0xE0, 0xE3, 0xE1), bg2=(0xD4, 0xD6, 0xD5), bg3=(0xBF, 0xC4, 0xC4),
    line=(0xAF, 0xB5, 0xB4), sub=(0x60, 0x65, 0x62), panel=(0x2C, 0x47, 0x4C),
    text=(0xE0, 0xE3, 0xE1), acc1=(0x68, 0xA7, 0xB1), acc2=(0x44, 0x68, 0x90),
)

PROCESS = ["시작", "순찰", "Pick", "통로 이탈", "이송", "Place", "스택 Pick", "스택 Place"]
DOCK_MINI = ["접근", "진입", "충전", "이탈", "복귀"]

TITLE_TOP = "[두산로보틱스] 지능형로보틱스 엔지니어"
TITLE_1 = "반도체 후공정 라인의"
TITLE_2 = "부품 캐리어 이송·피킹 시스템"
TITLE_SUB = "협동3- 디지털 트윈 기반 로봇 자동화 시뮬레이션 시스템 구현"
TEAM = [("A-2", "FABorite"), ("팀원", "전주은 고은빈 박성은 김희성"), ("[멘토]", "손미란 강사님")]
TITLE = TITLE_1 + " " + TITLE_2
SLOGAN = "순찰에서 이송, 회수, 충전까지 — 사람 개입 없이, 모든 단계가 기록으로 남습니다"

# ── 챕터 정의 (§2 자막, §4 컷 포인트 — v2: 2026-09-29 피드백 31개 반영) ──
# segs: (원본 시작 s, 원본 끝 s, 배속[, 부제 덮어쓰기, 미니 바 인덱스])
# hl:   (원본 시작, 원본 끝, 제목, 부제) — 배속 없이 1× 로 확대 리플레이
# extra: (원본 시작, 원본 끝, 자막) — 화면 상단 가운데 칩
CHAPTERS = [
    dict(n=1, clip=1, code="START", title="Start",
         card="두 로봇이 같은 주차 패드에서 출발합니다.\n순찰 시작점으로 이동한 뒤 팔을 관측 자세로 잡고 순찰에 들어갑니다.",
         sub="시작 좌표 (도킹 스테이션) → 순찰 시작점으로 이동\n(Nav2를 통한 장애물 회피)",
         state="state=start → pose → patrol",
         segs=[(0, 95.3, 4.5)],
         extra=[(60, 95.3, "두 로봇의 출발 지점이 같아 출발이 서로 막히지 않도록 robot2 는 60 s 늦게 출발하도록 설계했습니다 (start_delay_s)", "left")],
         hl=None,
         badge=("✓ 01 Start 완료", "robot1 순찰 시작 · robot2 60 s 뒤 출발")),
    dict(n=2, clip=2, code="PATROL", title="Patrol",
         card="패트롤 좌표 사이를 왕복하며 손목 카메라로 QR을 탐색합니다.\n보이면 그 자리에 정차합니다.",
         sub="패트롤 좌표 왕복 · QR 탐색 → 정차 · 판독",
         state="state=patrol → hold → scan",
         # 0–10 팔 인식 자세(현재의 0.5배), 10–66 느린 주행은 뒷부분(66–91) 화면 속도에 맞춰 12×
         segs=[(0, 10, 3.75), (10, 66, 15), (66, 90.7, 3.75)],
         hl=(78, 90.7, "매거진 앞 정차", "QR을 인식하고 그 자리에 정차합니다"), hl_speed=1.25,
         badge=("✓ 02 Patrol 완료", "QR 탐색 → 정차 → 판독")),
    dict(n=3, clip=3, code="PICK", title="Pick",
         card="멈춘 그 자리에서 판독한 위치로 로봇팔을 움직여\n흡착 그리퍼로 매거진을 집습니다.",
         sub="흡착 그리퍼 · 매거진 파지",
         state="state=pick detected=True",
         segs=[(0, 33.9, 2.25)],
         hl=None,
         badge=("✓ 03 Pick 완료", "매거진 파지")),
    dict(n=4, clip=4, code="RETREAT", title="Retreat",
         card="선반 앞 좁은 통로에서 바로 Nav2로 움직이지 않습니다.\nPatrol 시작 좌표까지 먼저 cmd_vel로 빠져나옵니다.",
         sub="좁은 통로 → Patrol 시작 좌표 · cmd_vel 후진",
         state="state=return_to_start",
         segs=[(0, 14, 6), (14, 32.5, 3)],
         hl=None,
         badge=("✓ 04 Retreat 완료", "cmd_vel → patrol 시작 좌표")),
    dict(n=5, clip=5, code="NAV", title="Nav",
         card="여유 있는 지점에서 Nav2로 패키징 로더까지 움직입니다.\n상대 로봇이 로더 차선을 쓰고 있으면 대기 후 진입합니다.",
         sub="Nav2 로 패키징 로더까지 주행",
         state="state=nav",
         segs=[(0, 39.3, 1.5)],  # 39 s 이후 정지 → 버림
         hl=None,
         badge=("✓ 05 이송 완료", "패키징 로더 정차점 도착")),
    dict(n=6, clip=6, code="PLACE", title="Place",
         card="지정 좌표에서 매거진을 내려 놓습니다.",
         sub="진입 → 로더에 배치 → 후진",
         state="state=creep_in → place → creep_out",
         segs=[(0, 36.1, 3)],
         extra=[(0, 36.1, "장애물 근처에서는 Nav2가 아닌 cmd_vel로 움직입니다", "left")],
         hl=None,
         badge=("✓ 06 Place 완료", "매거진 → 패키징 로더 · TraceEvent 기록")),
    dict(n=7, clip=7, code="STACK PICK", title="Stack Pick",
         card="웹을 통해 회수 작업이 배정되었을 경우,\n패키지 언로더에서 나온 스택을 집습니다.",
         sub="패키지 언로더 · 스택 파지 → 대기점 후퇴",
         state="state=stack_scan → stack_pick → stack_retreat",
         # 15 s 홈 자세부터. 팔 자세 변경(19–30, 42–48)은 현재의 0.5배. 집은 뒤 정지(48–65)는 버리고 바로 후진.
         segs=[(15, 19, 3), (19, 30, 1.5), (30, 42, 3), (42, 48, 1.5), (65, 80.5, 3)],
         extra=[(19, 48, "Pick 실패 시 최대 2회 재시도 (총 3번)")],
         hl=None,
         badge=("✓ 07 Stack Pick 완료", "웹 배정(pending_pickup) → 스택 파지")),
    dict(n=8, clip=8, code="STACK PLACE", title="Stack Place",
         card="Nav2로 테스트 로더까지 가서 스택을 놓습니다.",
         sub="테스트 로더 주행 · 스택 배치",
         state="state=stack_deliver → stack_place",
         # 0–12 삭제. 배치 동작(45–53)은 현재의 0.75배.
         segs=[(33, 45, 3), (45, 53, 2.25), (53, 58.9, 3)],  # 0–33 위치 조정 구간은 버림
         hl=None,
         badge=("✓ 08 Stack Place 완료", "사이클 종료 · 8 / 8 COMPLETE")),
    dict(n=9, clip=9, code="DOCKING SYSTEM", title="Docking", tag="ADDITIONAL",
         card="작업을 마친 로봇의 배터리가 일정 수준 이하로 내려갈 경우 패트롤 복귀 대신 도킹 스테이션으로 향합니다.\n충전이 끝나면 패트롤로 복귀하여 업무를 계속 수행합니다.",
         sub="배터리 임계값 이하 → 도킹 스테이션 복귀 · 충전 · 재출발",
         state_center="배터리 부족 시 도킹 스테이션으로 자동으로 복귀한 다음 다시 업무를 지속하러 감을 보여주는 예시 화면입니다.",
         # 0–84 우회 구간 삭제. 전 구간 배속을 절반으로(2배 길게).
         segs=[(84, 136, 3, "배터리 임계값 이하 · 다음 작업 대신 도킹 스테이션으로", 0),
               (136, 160, 2, "도크 앞 대기점에서 직진 진입", 1),
               (160, 165, 1 / 3, "충전 중 · 이 동안 웹 작업은 거절되고\n상대 로봇에게 넘어갑니다", 2),
               (242, 291, 2.5, "후진 이탈 · 도크 구역에서는 회전하지 않습니다", 3),
               (291, 331, 1.5, "패트롤로 복귀 · 업무를 계속 수행합니다", 4)],
         hl=None,
         badge=("✓ Docking 완료", "충전 후 패트롤 복귀")),
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
    if "\n" in s:
        out = []
        for part in s.split("\n"):
            out.extend(wrap(d, part, f, maxw))
        return out
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


def ov_card(n, code, title, desc, tag=None):
    im, d = canvas()
    f_lab, f_t, f_d = FONTS.get(22, 500), FONTS.get(64, 700), FONTS.get(24, 400)
    x0, y0 = 90, 210
    lab = f"{n:02d} · {code}"
    d.text((x0, y0), lab, font=f_lab, fill=rgba(C["acc2"]))
    if tag:
        f_tag = FONTS.get(15, 700)
        lx = x0 + text_w(d, lab, f_lab) + 16
        tw = text_w(d, tag, f_tag) + 22
        rrect(d, (lx, y0 + 1, lx + tw, y0 + 27), 6, rgba(C["acc1"]))
        d.text((lx + 11, y0 + 4), tag, font=f_tag, fill=rgba(C["panel"]))
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
    d.text((x0 + (w - text_w(d, head, f_h)) / 2, y0 + 18), head, font=f_h, fill=rgba(C["text"]))
    d.text((x0 + (w - text_w(d, summary, f_s)) / 2, y0 + 72), summary, font=f_s, fill=rgba(C["acc1"]))
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
            d.text((x + cw - 36, y + 8), "ADD", font=FONTS.get(12, 700), fill=rgba(C["acc1"]))
        d.text((x + 12, y + 28), nm, font=f_t, fill=rgba(C["panel"]))
        x += cw + gap
    return Ov(im)


def ov_data_left():
    im, d = canvas()
    f_lab, f_t, f_i, f_s = FONTS.get(22, 500), FONTS.get(52, 700), FONTS.get(24, 600), FONTS.get(19, 400)
    x0, y0 = 90, 150
    d.text((x0, y0), "DATA COLLECTION", font=f_lab, fill=rgba(C["acc2"]))
    d.text((x0 - 2, y0 + 36), "물류 이력 추적(Traceability) 및 관제 DB", font=f_t, fill=rgba(C["panel"]))
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
def ov_team(y):
    im, d = canvas()
    f_k, f_v = FONTS.get(19, 700), FONTS.get(19, 400)
    rows = [(k, v) for k, v in TEAM]
    kw = max(text_w(d, k, f_k) for k, _ in rows) + 16
    vw = max(text_w(d, v, f_v) for _, v in rows)
    x0 = W - 90 - kw - vw
    for i, (k, v) in enumerate(rows):
        d.text((x0, y + i * 32), k, font=f_k, fill=rgba(C["panel"]))
        d.text((x0 + kw, y + i * 32), v, font=f_v, fill=rgba(C["sub"]))
    return Ov(im)


def render_intro(out, bgfr):
    bg = whiten(bgfr, 0.78, 6)
    els = [
        (0.0, ov_centered_text(TITLE_TOP, 118, 20, 700, C["panel"])),
        (0.5, ov_centered_text(TITLE_1, 168, 66, 800, C["panel"])),
        (0.9, ov_centered_text(TITLE_2, 248, 66, 300, C["panel"])),
        (1.6, ov_centered_text(TITLE_SUB, 350, 24, 400, C["sub"])),
        (3.0, ov_chapter_chips(430)),
        (4.5, ov_team(560)),
    ]
    out.begin(xfade=False)
    for k in range(int((8 * FPS) / TEXT_SPEED)):
        t = k * TEXT_SPEED / FPS
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
    card = ov_card(n, ch["code"], ch["title"], ch["card"], tag=ch.get("tag"))
    out.begin()
    for k in range(int((CARD_S * FPS) / TEXT_SPEED)):
        fr = card_bg.copy()
        card.apply(fr, fade(k * TEXT_SPEED / FPS, 0.2, 0.5))
        out.push(fr)
    # (b) 본 클립
    state_chip = ov_chip(ch["state_center"], y=24, align="center", size=17, weight=500) if ch.get("state_center") else None
    done = set(range(n - 1)) if not is_dock else set()
    prog_cache = {}
    thumb = None
    out.begin()
    for seg in ch["segs"]:
        a, b, speed = seg[0], seg[1], seg[2] * ROBOT_SPEED
        sub = seg[3] if len(seg) > 3 else ch["sub"]
        mini = seg[4] if len(seg) > 4 else None
        panel = ov_step_panel(n, DOCK_MINI[mini] if is_dock and mini is not None else ch["title"], sub)
        if is_dock:
            prog = ov_progress(DOCK_MINI, mini, set(range(mini)), label="DOCKING", counter=f"{mini + 1} / 5")
        else:
            prog = ov_progress(PROCESS, n - 1, done, counter=f"STEP {n:02d} / 08")
        extras = [(e[0], e[1], ov_chip(e[2], y=24, align=(e[3] if len(e) > 3 else "center"), size=17))
                  for e in ch.get("extra", [])]
        k = 0
        for fr in src.frames(a, b, speed):
            t_src = a + k * speed / FPS
            last = fr.copy()
            panel.apply(fr)
            prog.apply(fr)
            if state_chip is not None:
                state_chip.apply(fr, 0.9)
            for ea, eb, ov in extras:
                if ea <= t_src <= eb:
                    ov.apply(fr, fade(t_src, ea, 1.0) * fade(eb, t_src, 1.0))
            if thumb is None and t_src >= (a + b) / 2:
                thumb = last.copy()
            out.push(fr)
            k += 1
    # (c) 하이라이트 — 원본 속도 1×, 1.4× 확대
    if ch.get("hl"):
        ha, hb, ht, hs = ch["hl"]
        hov = ov_highlight(ht, hs)
        tag = ov_chip("↗ REPLAY · 확대 화면", y=24, align="right", size=15)
        out.begin()
        i = 0
        hsp = ch.get("hl_speed", 1.0) * ROBOT_SPEED
        for fr in src.frames(ha, hb, hsp, zoom=1.4):
            last = fr.copy()
            hov.apply(fr)
            tag.apply(fr)
            if i == int((hb - ha) * FPS / hsp / 2):
                thumb = last.copy()
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
    for k in range(int((2 * FPS + 10) / TEXT_SPEED)):
        fr = bg.copy()
        prog.apply(fr)
        badge.apply(fr, fade(k * TEXT_SPEED / FPS, 0.1, 0.3))
        out.push(fr)


def render_data(out, bgfr):
    bg = whiten(bgfr, 0.75, 6)
    els = [
        (0.0, ov_data_left()),
        (4.0, ov_data_item(0, "이동·인식·파지·도킹 등 태스크 스텝별 상태 데이터 자동 수집", "PostgreSQL에 로봇의 상태, 성공 여부, 실패 원인 등을 자동으로 저장")),
        (8.0, ov_data_item(1, "데이터 무결성(Integrity)을 보장하는 로그 적재", "magazine_log, stack_log에 적재")),
        (12.0, ov_data_item(2, "Top-view 기반 전체 공정 및 개별 로봇 상태 실시간 모니터링", "로봇별 순찰(Patrol) 구역 및 타겟 객체에 대한 개별 작업 지시")),
    ]
    out.begin()
    for k in range(int((17 * FPS) / TEXT_SPEED)):
        t = k * TEXT_SPEED / FPS
        fr = bg.copy()
        for t0, ov in els:
            ov.apply(fr, fade(t, t0))
        out.push(fr)


def render_outro(out, bgfr, thumbs):
    bg = whiten(bgfr, 0.75, 6)
    title = ov_centered_text(TITLE, 104, 46, 700, C["panel"])
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
            thumb_ovs.append((1.0 + 1.4 * (len(thumb_ovs)), Ov(c)))
            x += tw + gap
    out.begin()
    for k in range(int((19 * FPS) / TEXT_SPEED)):
        t = k * TEXT_SPEED / FPS
        fr = bg.copy()
        title.apply(fr, fade(t, 0.0))
        ul.apply(fr, fade(t, 0.0))
        for t0, ov in thumb_ovs:
            ov.apply(fr, fade(t, t0))
        slogan.apply(fr, fade(t, 14.0))
        if t > 18.0:
            fr = whiten(fr, fade(t, 18.0, 0.9))
        out.push(fr)


def main():
    global FONTS
    ap = argparse.ArgumentParser()
    ap.add_argument("--clips", required=True)
    ap.add_argument("--font", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ffmpeg", default=None)
    ap.add_argument("--only", type=int, default=None, help="챕터 하나만 렌더(테스트용)")
    ap.add_argument("--text-speed", type=float, default=1.0,
                    help="글자 화면(인트로·카드·배지·데이터·아웃트로)에만 곱할 배속")
    ap.add_argument("--robot-speed", type=float, default=1.0,
                    help="로봇 영상 구간에만 곱할 배속 (카드·배지·인트로·데이터·아웃트로는 그대로)")
    args = ap.parse_args()
    FONTS = Fonts(args.font)
    global ROBOT_SPEED, TEXT_SPEED
    ROBOT_SPEED = args.robot_speed
    TEXT_SPEED = args.text_speed
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
        render_intro(out, fit(srcs[1].frame_at(2.0)))
    for ch in chapters:
        print(f"chapter {ch['n']} {ch['title']}", flush=True)
        render_chapter(out, ch, srcs[ch["clip"]], thumbs)
    if args.only is None:
        render_data(out, fit(srcs[6].frame_at(30.0)))
        render_outro(out, fit(srcs[1].frame_at(2.0)), thumbs)
    out.close()
    print(f"done: {out.n} frames = {out.n / FPS:.1f} s -> {args.out}")


if __name__ == "__main__":
    main()

"""
캐리어 코드 규칙 + 파서 — 의존성 없는 순수 파이썬

QR 에 들어가는 문자열은 이렇게 생겼다.

    F1-MGZO-1
    │  │    └ 일련번호 (같은 공장·품목에서 몇 번째 개체인지)
    │  └ 품목 코드 4자  ← 이것만 보면 어떤 캐리어인지 안다
    └ 공장 코드 (F1 · F2 · F3). 라인은 QR 이 복잡해져 일부러 넣지 않았다

'-' 로 자른 두 번째 토큰(인덱스 1)이 품목 코드다.
품목 코드는 앞 3자가 종류, 끝 1자가 손잡이 색이다.

    코드    앞 3자 = 종류      끝 1자 = 색   바탕 에셋
    ─────────────────────────────────────────────────────────────
    MGZO    MGZ 리드프레임매거진  O 주황      magazine_1_orange.usda
    MGZB    MGZ 리드프레임매거진  B 파랑      magazine_2_blue.usda
    STKO    STK JEDEC트레이스택   O 주황      tray_1_orange.usda
    STKB    STK JEDEC트레이스택   B 파랑      tray_2_blue.usda

파일 이름은 코드의 '-' 를 '_' 로 바꾼 것이다. F1-MGZO-1 -> F1_MGZO_1.usda

쓰는 쪽:

    from carrier_code import parse_code
    info = parse_code(decoded_text)        # 카메라가 읽은 문자열
    if info and info.color == "orange":
        ...

kind_of() 는 인덱스에 기대지 않고 아는 품목 코드와 같은 토큰을 찾는다.
나중에 로트 날짜 같은 필드가 중간에 끼어도(F1-260921-MGZO-1) 그대로 읽힌다.
"""

from collections import namedtuple

SEP = "-"
KIND_INDEX = 1           # 표준 형식에서 품목 코드가 있는 자리 (참고용)

FAMILY = {
    "MGZ": "magazine",   # 리드프레임 매거진
    "STK": "stack",      # JEDEC 트레이 스택
}
COLOR = {
    "O": "orange",
    "B": "blue",
}

#  품목 코드 -> 바탕이 되는 기본 에셋
BASE_ASSET = {
    "MGZO": "magazine_1_orange.usda",
    "MGZB": "magazine_2_blue.usda",
    "STKO": "tray_1_orange.usda",
    "STKB": "tray_2_blue.usda",
}

#  손잡이 색 RGB (gen_carrier_assets.py 의 COLOR_ORANGE / COLOR_BLUE 와 같은 값)
RGB = {
    "orange": (0.95, 0.55, 0.15),
    "blue":   (0.15, 0.40, 0.90),
}

#  실제로 만드는 개체 16 종
CODES = [
    "F1-MGZO-1", "F1-MGZO-2", "F2-MGZO-1", "F2-MGZO-2",   # 주황 매거진
    "F1-MGZB-1", "F1-MGZB-2", "F2-MGZB-1", "F2-MGZB-2",   # 파란 매거진
    "F3-STKO-1", "F3-STKO-2", "F3-STKO-3", "F3-STKO-4",   # 주황 스택
    "F3-STKB-1", "F3-STKB-2", "F3-STKB-3", "F3-STKB-4",   # 파란 스택
]

Carrier = namedtuple("Carrier", "code plant kind family color serial base_asset rgb")


# ──────────────────────────────────────────────────────────────
#  이름 짓기
# ──────────────────────────────────────────────────────────────
def usd_name(code):
    """F1-MGZO-1 -> F1_MGZO_1.usda"""
    return code.replace(SEP, "_") + ".usda"


def png_name(code):
    """F1-MGZO-1 -> qr_F1_MGZO_1.png"""
    return "qr_" + code.replace(SEP, "_") + ".png"


# ──────────────────────────────────────────────────────────────
#  읽기
# ──────────────────────────────────────────────────────────────
def kind_of(text):
    """
    문자열에서 품목 코드만 꺼낸다. 없으면 None.

    카메라가 읽은 값을 그대로 넣어도 되게 공백은 떼고 대문자로 맞춘다.
    자리(인덱스)가 아니라 아는 코드와 같은 토큰을 찾으므로, 중간에 필드가
    더 끼어도 깨지지 않는다.
    """
    if not text:
        return None
    for token in str(text).strip().upper().split(SEP):
        if token in BASE_ASSET:
            return token
    return None


def parse_code(text):
    """
    캐리어 코드를 Carrier 로 푼다. 못 읽으면 None.

    >>> parse_code("F1-MGZO-1").color
    'orange'
    >>> parse_code("F3-STKB-4").family
    'stack'
    """
    kind = kind_of(text)
    if kind is None:
        return None
    parts = str(text).strip().upper().split(SEP)
    i = parts.index(kind)
    color = COLOR[kind[3]]
    return Carrier(code=SEP.join(parts), plant=parts[0], kind=kind,
                   family=FAMILY[kind[:3]], color=color,
                   serial=parts[i + 1] if len(parts) > i + 1 else "",
                   base_asset=BASE_ASSET[kind], rgb=RGB[color])


def is_orange(text):
    """주황 손잡이 캐리어냐"""
    info = parse_code(text)
    return info is not None and info.color == "orange"


def is_magazine(text):
    """매거진이냐 (트레이 스택이 아니라)"""
    info = parse_code(text)
    return info is not None and info.family == "magazine"


if __name__ == "__main__":
    for code in CODES:
        info = parse_code(code)
        print(f"{code:12s} split('-')[{KIND_INDEX}] = {kind_of(code)}  "
              f"{info.family:8s} {info.color:6s} -> {usd_name(code):18s} {png_name(code)}")
    assert len(CODES) == len(set(CODES)) == 16
    for bad in ("", "1", "F1", "F1-XXXX-1", None):
        assert parse_code(bad) is None, bad
    assert is_orange("F1-MGZO-1") and not is_orange("F1-MGZB-1")
    assert is_magazine("F1-MGZO-1") and not is_magazine("F3-STKO-1")
    assert kind_of("F1-260921-MGZO-1") == "MGZO"      # 필드가 끼어도 읽힌다
    print("self-test OK")

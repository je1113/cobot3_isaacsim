"""yaml 읽기/쓰기 — 설정의 주인은 파일이다(docs/DB구성.md §10-1).

두 가지를 지킨다:

1. **주석을 잃지 않는다.** 이 리포의 yaml 은 주석이 본문만큼 중요하다
   (place.yaml 의 "왜 벨트 높이인가", grasp.yaml 의 "+5mm 를 빼지 마라").
   화면에서 한 번 저장했다고 그게 날아가면 안 되므로 ruamel 로 왕복한다.

2. **동시 편집을 감지한다(WBS 3.2).** revision = 파일 내용의 sha256 앞 12자.
   PUT 은 If-Match 로 자기가 읽은 revision 을 보내고, 그 사이에 파일이
   바뀌었으면 412 로 거절한다. 파일에는 revision 을 적지 않는다 —
   적으면 파일이 자기 해시를 담게 되어 순환한다.

쓰기는 같은 디렉터리에 임시파일 → os.replace 라 중간에 죽어도 반쪽 파일이 안 남는다.
"""

from __future__ import annotations

import hashlib
import io
import os
import tempfile
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML

from .errors import NotFound, PreconditionFailed

_yaml = YAML()
_yaml.preserve_quotes = True
_yaml.width = 4096  # 긴 주석/리스트가 접히지 않게
_yaml.indent(mapping=2, sequence=4, offset=2)


def revision_of(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()[:12]


def load(path: Path) -> tuple[Any, str]:
    """(데이터, revision). 파일이 없으면 404."""
    if not path.is_file():
        raise NotFound(f"설정 파일이 없다: {path}", path=str(path))
    raw = path.read_bytes()
    return _yaml.load(io.BytesIO(raw)), revision_of(raw)


def peek_revision(path: Path) -> str:
    if not path.is_file():
        raise NotFound(f"설정 파일이 없다: {path}", path=str(path))
    return revision_of(path.read_bytes())


def save(path: Path, data: Any, if_match: str | None) -> str:
    """원자적 저장. if_match 가 현재 revision 과 다르면 412.

    if_match=None 은 '검사하지 않음'이다 — 서버 내부에서 부르는 경로에만 쓴다.
    화면에서 오는 PUT 은 반드시 If-Match 를 달아야 한다(라우터가 강제).
    """
    current = peek_revision(path)
    if if_match is not None and if_match != current:
        raise PreconditionFailed(
            "다른 사람이 먼저 저장했다. 화면을 새로 고친 뒤 다시 시도할 것.",
            expected=if_match,
            actual=current,
        )

    buf = io.BytesIO()
    _yaml.dump(data, buf)
    new_raw = buf.getvalue()

    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(new_raw)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise

    return revision_of(new_raw)


def merge_into(target: Any, src: Any) -> Any:
    """`target` 을 **같은 객체 그대로** 두고 내용만 `src` 에 맞춘다.

    ★ 왜 `parent[key] = new_dict` 로 갈아끼우면 안 되는가:
      ruamel 은 주석을 **노드에 붙여서** 들고 있다. 어떤 키 뒤에 있는 주석 블록은
      그 키를 가진 CommentedMap 이 소유한다. 그 Map 을 통째로 평범한 dict 로
      바꾸면 주석도 같이 사라진다 — 화면에서 선반 하나를 고쳤을 뿐인데
      그 아래 붙어 있던 설명이 조용히 날아간다.

      실제로 그렇게 잃었다: shelf_01 을 저장했더니 그 뒤의
      "── 아래는 PATROL_ROUTE 의 두 정차점이다" 블록이 통째로 없어졌다.
      헤더 주석만 확인하는 테스트는 이걸 못 잡는다.

    그래서 컨테이너는 유지하고 키 단위로만 넣고/고치고/뺀다.
    살아남은 키에 붙은 주석은 그대로 남는다. 지워진 키의 주석은 함께 사라지는데,
    그건 의도한 것이다 — 없는 것을 설명하는 주석은 거짓말이 된다.
    """
    # 안 바뀐 값은 아예 건드리지 않는다. 화면은 노드 전체를 되돌려 보내므로
    # 실제로 바뀐 칸은 보통 하나뿐이고, 나머지를 그냥 두는 것이 가장 안전하다.
    if plain(target) == plain(src):
        return target

    if isinstance(target, dict) and isinstance(src, dict):
        for key in [k for k in target if k not in src]:
            del target[key]
        for key, value in src.items():
            if key in target:
                target[key] = merge_into(target[key], value)
            else:
                target[key] = value
        return target

    if isinstance(target, list) and isinstance(src, list):
        # ★ 슬라이스 대입(target[:] = src)을 쓰면 안 된다.
        #   CommentedSeq 에 슬라이스로 대입하면 **그 뒤에 붙은 주석 블록이 사라진다.**
        #   (실측: 리스트 뒤의 주석이 슬라이스 대입에서만 없어지고,
        #    같은 객체 재대입이나 다른 키 수정에서는 멀쩡하다.)
        #   그래서 인덱스 단위로만 고친다.
        for i, value in enumerate(src):
            if i < len(target):
                target[i] = merge_into(target[i], value)
            else:
                target.append(value)
        while len(target) > len(src):
            target.pop()
        return target

    return src


def plain(data: Any) -> Any:
    """ruamel 의 CommentedMap/Seq 를 순수 dict/list 로 — JSON 응답용.

    ruamel 타입은 dict/list 를 상속하지만 그대로 내보내면 스칼라 래퍼가
    섞여 나갈 수 있어 한 번 벗긴다.
    """
    if isinstance(data, dict):
        return {str(k): plain(v) for k, v in data.items()}
    if isinstance(data, (list, tuple)):
        return [plain(v) for v in data]
    if isinstance(data, (str, bool, int, float)) or data is None:
        return data
    return str(data)

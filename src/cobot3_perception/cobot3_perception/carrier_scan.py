"""
주행 중 매거진/QR 존재 검출 결과를 map 좌표로 모으고 중복을 없앤다.

체크리스트 3-1 항목:
    스캔 자세에서 매거진/QR 존재 검출
    검출 결과를 map 좌표 기준 대략 위치로 발행
    동일 매거진 중복 검출 방지 (위치 기반 병합, QR ID 활용)

ROS 의존성 없음 (flange_topview.py, qr_pose.py 와 같은 이유).

설계:
  주행 중에는 한 매거진이 여러 프레임에 걸쳐 반복 검출된다
  (isaacpjt/tools/out/sweep_scan.yaml 실측: 한 매거진이 디코드 창 안에서
  보통 2~5 개 샘플 x 에 걸쳐 걸린다). 그래서 이 모듈은 "탐지 하나"가 아니라
  "탐지 스트림"을 받아 같은 물체를 하나로 합친다.

  QR 이 아직 decode 안 됐어도(presence 만, qr_pose.detect_qr_quads 의 detect()
  단독 경로) 위치 병합에 쓴다 — decode 는 qr_decode_range.py 실측상 detect
  보다 짧은 거리에서만 되므로, decode 안 된 표본도 버리면 안 된다.
  ID 가 나중에 decode 되면 그 클러스터에 소급 적용한다.

병합 규칙:
  - 새 탐지의 map 위치가 기존 클러스터 중 merge_radius_m 이내에 있으면 합친다
    (위치 평균을 갱신). ID 가 있는 클러스터끼리는 ID 가 다르면 절대 안 합친다
    (위치가 가까워도 서로 다른 매거진일 수 있으므로 ID 가 최종 중재자다).
  - ID 없는 클러스터에 ID 있는 탐지가 들어오면 그 클러스터에 ID 를 확정한다.
  - 한 클러스터가 min_hits 번 미만 걸리면(스치듯 한 프레임만 잡힌 경우) 노이즈로
    보고 최종 출력에서 뺀다.

한계 (합성 테스트로 확인, isaacpjt/tools/test_carrier_scan.py):
  merge_radius_m 이 매거진 간 실제 간격보다 크면, ID 가 늦게 decode 되는 동안
  이웃 매거진의 위치 추정치끼리 겹쳐 오병합할 수 있다. 실측 매거진 피치는
  0.55~0.62 m (layout_measured.yaml) 이라 기본값 0.15 m 로는 여유가 크지만,
  선반 배치가 바뀌면 이 여유를 다시 확인해야 한다.
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class CarrierDetection:
    """검출 한 건 — carrier_detector 가 프레임마다 내는 원시 결과."""
    map_xy: np.ndarray            # 대략 위치 (map 프레임, 2D 로 충분 — 선반은 바닥에 고정)
    decoded_id: str = ""          # QR 이 decode 됐으면 ID, 아니면 빈 문자열
    stamp: float = 0.0


@dataclass
class CarrierCluster:
    id: int
    decoded_id: str = ""
    map_xy: np.ndarray = field(default_factory=lambda: np.zeros(2))
    n_hits: int = 0
    first_stamp: float = 0.0
    last_stamp: float = 0.0

    def as_dict(self):
        return dict(id=self.id, decoded_id=self.decoded_id,
                   map_xy=[round(float(v), 4) for v in self.map_xy],
                   n_hits=self.n_hits,
                   first_stamp=round(self.first_stamp, 3), last_stamp=round(self.last_stamp, 3))


class CarrierScanMerger:
    """탐지 스트림을 받아 누적 클러스터를 유지한다. add() 를 프레임마다 호출한다."""

    def __init__(self, merge_radius_m=0.15, min_hits=2):
        self.merge_radius_m = merge_radius_m
        self.min_hits = min_hits
        self._clusters = []
        self._next_id = 0

    def add(self, detection: CarrierDetection):
        best, best_d = None, None
        for c in self._clusters:
            if detection.decoded_id and c.decoded_id and detection.decoded_id != c.decoded_id:
                continue        # ID 가 서로 다르면 절대 같은 물체가 아니다
            d = float(np.linalg.norm(c.map_xy - detection.map_xy))
            if d < self.merge_radius_m and (best is None or d < best_d):
                best, best_d = c, d

        if best is None:
            c = CarrierCluster(id=self._next_id, decoded_id=detection.decoded_id,
                               map_xy=detection.map_xy.copy(), n_hits=1,
                               first_stamp=detection.stamp, last_stamp=detection.stamp)
            self._next_id += 1
            self._clusters.append(c)
            return c

        # 누적 평균으로 위치를 갱신한다 (뒤로 갈수록 새 표본의 영향이 줄어든다)
        best.map_xy = (best.map_xy * best.n_hits + detection.map_xy) / (best.n_hits + 1)
        best.n_hits += 1
        best.last_stamp = detection.stamp
        if detection.decoded_id and not best.decoded_id:
            best.decoded_id = detection.decoded_id
        return best

    def clusters(self, min_hits=None):
        """min_hits 이상 걸린 클러스터만 돌려준다 (스치듯 한 번 잡힌 노이즈 제외)."""
        thresh = self.min_hits if min_hits is None else min_hits
        return [c for c in self._clusters if c.n_hits >= thresh]

    def as_dict_list(self, min_hits=None):
        return [c.as_dict() for c in self.clusters(min_hits)]


def rough_map_position(cam_map_xy, cam_heading_rad, range_m, bearing_offset_rad=0.0):
    """카메라 map 위치/방향 + 거리(깊이 또는 qr_decode_range.py 식 px 역산)로 대상의
    대략 map 위치를 만든다. carrier_detector 가 3D 를 몰라도(주행 중 프레임 한 장,
    QR 크기만으로 대략 거리를 낼 때) 쓸 수 있게 2D 로 충분히 단순화했다.
    """
    ang = cam_heading_rad + bearing_offset_rad
    return np.asarray(cam_map_xy, dtype=float) + range_m * np.array([math.cos(ang), math.sin(ang)])

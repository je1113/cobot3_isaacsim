"""
carrier_scan.CarrierScanMerger 단위테스트 — Isaac 불필요.

    python3 isaacpjt/tools/test_carrier_scan.py

실측 매거진 배치(layout_measured.yaml, shelf_1/top 4개, 피치 0.55~0.62 m)를
따라 만든 합성 주행 스트림으로 병합/중복제거를 확인한다. 위치 노이즈 2 cm,
ID 미decode 확률 15% 를 섞는다 (qr_decode_range.py 실측 — decode 반경이
detect 반경보다 좁다는 사실을 반영).
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent
                       / "src/cobot3_perception"))
from cobot3_perception.carrier_scan import CarrierDetection, CarrierScanMerger

MAGAZINES = [
    ("1", np.array([-6.59, 1.9295])),
    ("2", np.array([-6.0383, 1.8995])),
    ("1", np.array([-5.3888, 1.9295])),
    ("2", np.array([-4.815, 1.8995])),
]
DECODE_HALF_RANGE = 0.15
DETECT_HALF_RANGE = 0.35
BASE_X_STEP = 0.05


def run_stream(magazines, x_range, merge_radius_m=0.15, pos_noise_m=0.02,
              decode_miss_p=0.15, seed=0):
    rng = np.random.default_rng(seed)
    merger = CarrierScanMerger(merge_radius_m=merge_radius_m, min_hits=2)
    n_frames = 0
    for bx in np.arange(*x_range, BASE_X_STEP):
        for expected_id, mag_xy in magazines:
            dx = mag_xy[0] - bx
            if abs(dx) > DETECT_HALF_RANGE:
                continue
            n_frames += 1
            noisy_xy = mag_xy + rng.normal(0, pos_noise_m, 2)
            decoded = (expected_id if abs(dx) <= DECODE_HALF_RANGE
                      and rng.random() > decode_miss_p else "")
            merger.add(CarrierDetection(map_xy=noisy_xy, decoded_id=decoded, stamp=bx))
    return merger, n_frames


def test_real_layout_pitch():
    """실측 피치(0.55~0.62 m) — 정상 케이스"""
    merger, n_frames = run_stream(MAGAZINES, (-7.0, -4.3))
    clusters = sorted(merger.clusters(), key=lambda c: c.map_xy[0])
    assert len(clusters) == len(MAGAZINES), \
        f"클러스터 {len(clusters)}개, 기대 {len(MAGAZINES)}개"
    for (expected_id, mag_xy), c in zip(MAGAZINES, clusters):
        err_mm = float(np.linalg.norm(c.map_xy - mag_xy) * 1000)
        assert c.decoded_id == expected_id, f"ID 불일치 {c.decoded_id} != {expected_id}"
        assert err_mm < 50, f"위치 오차 {err_mm:.1f} mm 과다"
    print(f"[PASS] 실측 피치  프레임 {n_frames}  클러스터 {len(clusters)}개 전부 일치")


def test_tight_pitch_at_merge_radius():
    """병합 반경과 같은 간격(0.15 m) — 한계 근처, 정보용(assert 없음)"""
    close = [("1", np.array([-6.5, 1.93])), ("2", np.array([-6.35, 1.93]))]
    merger, n_frames = run_stream(close, (-6.9, -6.0))
    clusters = merger.clusters()
    tag = "PASS" if len(clusters) == 2 else "주의"
    print(f"[{tag}] 간격 0.15 m(=병합반경)  프레임 {n_frames}  클러스터 {len(clusters)}개 "
          f"(기대 2) — carrier_scan.py 한계 절 참고")


if __name__ == "__main__":
    test_real_layout_pitch()
    test_tight_pitch_at_merge_radius()

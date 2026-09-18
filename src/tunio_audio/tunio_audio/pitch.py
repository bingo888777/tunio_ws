"""
주파수 분석 코어. pitch_test.py에서 검증한 2단계 추정(FFT 유도 + 협대역 YIN).
audio_analyzer가 import 해서 사용. ROS 비의존.

TODO(정정인): pitch_test.py의 검증된 estimate_pitch/yin_frame/fft_peak를 이식.

목표 주파수는 이 모듈이 표를 들고 있지 않는다. 단일 출처는 서버이며
MeasurePitch.Goal.target_hz 로 전달된다(명세 4절).
"""
import numpy as np   # noqa: F401


def target_range(target_hz: float) -> tuple:
    """탐색 범위 = 목표 ±2반음. 옥타브 오인 차단용."""
    return target_hz * 2 ** (-2 / 12), target_hz * 2 ** (2 / 12)


def estimate(signal, sr, target_hz: float) -> tuple:
    """
    반환: (measured_hz 또는 None, failure_reason)

    failure_reason 은 MeasurePitch.Result 의 FAIL_* 값을 쓴다
    (FAIL_NONE / FAIL_LOW_SIGNAL / FAIL_NO_PITCH).
    """
    fmin, fmax = target_range(target_hz)
    # TODO: pitch_test.py 검증 코드 이식
    raise NotImplementedError

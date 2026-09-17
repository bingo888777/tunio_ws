"""
주파수 분석 코어. pitch_test.py에서 검증한 2단계 추정(FFT 유도 + 협대역 YIN).
audio_analyzer가 import 해서 사용. ROS 비의존.

TODO(정정인): pitch_test.py의 검증된 estimate_pitch/yin_frame/fft_peak를 이식.
탐색 범위는 대상 줄 목표 ±2반음으로 제한(옥타브 오인 차단).
"""
import numpy as np

TARGETS = {6: 82.4069, 5: 110.0, 2: 246.9417, 1: 329.6276}


def target_range(string_no):
    ft = TARGETS[string_no]
    return ft * 2 ** (-2 / 12), ft * 2 ** (2 / 12)


def estimate(signal, sr, string_no):
    """반환: (measured_hz 또는 None, fail_reason)"""
    fmin, fmax = target_range(string_no)
    # TODO: pitch_test.py 검증 코드 이식
    raise NotImplementedError

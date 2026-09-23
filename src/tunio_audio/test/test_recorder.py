"""콜백 블록 경계가 분석 파형을 훼손하지 않아야 한다."""
from types import SimpleNamespace

import numpy as np
import pytest

from tunio_audio.recorder import Recorder


@pytest.mark.parametrize('sr', [44100, 48000])
@pytest.mark.parametrize('block_size', [128, 512, 1024, 8192])
def test_repeated_reads_preserve_every_sample(sr, block_size):
    recorder = Recorder(sample_rate=sr)
    recorder._stream = SimpleNamespace(active=True)
    source = np.arange(block_size * ((sr + block_size - 1) // block_size), dtype=np.float32)
    for block in source.reshape(-1, block_size):
        recorder._callback(block[:, None], block_size, None, False)
    sizes = [.1, .01, .07, .003]
    actual = np.concatenate([recorder.read(sec) for sec in sizes])
    np.testing.assert_array_equal(actual, source[:len(actual)])


def test_flush_discards_pending_tail_from_previous_measurement():
    recorder = Recorder(sample_rate=1000)
    recorder._stream = SimpleNamespace(active=True)
    recorder._callback(np.ones((512, 1), np.float32), 512, None, False)
    recorder.read(.1)
    recorder.flush()
    recorder._callback(np.full((512, 1), 2, np.float32), 512, None, False)
    np.testing.assert_array_equal(recorder.read(.1), np.full(100, 2))

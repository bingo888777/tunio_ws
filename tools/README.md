# tools

- `pitch_test.py` — 마이크/주파수 분석 검증 스크립트 (ROS 무관, 단독 실행).
  여기서 표준편차 1센트 이내를 확인한 뒤, 검증된 로직을
  `src/tunio_audio/tunio_audio/pitch.py` 로 이식한다.

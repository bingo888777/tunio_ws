# AGENTS.md

이 저장소의 AI agent 작업 지침은 **CLAUDE.md** 에 정리돼 있다.
Claude Code, Cursor, 기타 코딩 agent 모두 먼저 `CLAUDE.md`를 읽고 그 규칙을 따른다.

핵심만 요약하면:
- 한국어로 대화, 표 대신 텍스트로 정리
- ROS 2 Jazzy / Python(rclpy) / 두산 M0609 + RG2
- 비전 없음. 조율 피드백은 마이크(주파수)로만. 힘 센서는 판정에 쓰지 않음
- 인터페이스(tunio_interfaces) 필드 확정이 최우선
- 임시값(재시도 횟수·허용 센트 등)을 확정값처럼 하드코딩하지 말 것

자세한 내용은 CLAUDE.md 참조.

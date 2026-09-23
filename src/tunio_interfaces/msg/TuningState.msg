# /tuning/state 로 발행하는 현재 작업 상태 전체.
# UI는 이 메시지 하나만 보고 실시간 탭을 그린다.
# QoS: reliable, depth 1, transient local (늦게 켠 UI도 마지막 상태를 즉시 받는다)

builtin_interfaces/Time stamp  # 발행 시각
string job_id                  # 진행 중인 작업 식별자. 대기 상태면 빈 문자열
uint8 phase                    # 현재 작업 단계
uint8 current_string           # 현재 처리 중인 줄 번호. 해당 없으면 0
float32 target_hz              # 현재 줄의 목표 주파수
float32 measured_hz            # 가장 최근 측정값
bool measurement_valid         # 위 값의 유효 여부. false면 화면에 '측정 불가' 표시
float32 error_hz               # 화면 표시용 오차 = 측정 - 목표. 양수면 높은 음
uint8 adjust_count             # 현재 줄의 조절 횟수
StringResult[] results         # 지금까지의 줄별 결과. 처리 전인 줄은 RESULT_PENDING
uint8 overall_result           # 전체 판정. 검증 패스 완료 전에는 OVERALL_PENDING
TuningSettings active_settings # 이번 작업에 적용 중인 설정 (작업 중 잠금)
string robot_notice            # 로봇이 보낸 최근 경고/오류. 새 작업 시작 시 비움

string error_message          # 작업 오류 사유. 새 작업 접수 시 비우며 ERROR 상태에서 표시

uint8 PHASE_IDLE=0             # 대기
uint8 PHASE_MEASURE_READY=1    # 측정 준비 (마이크 ready 대기)
uint8 PHASE_PLUCK=2            # 발현
uint8 PHASE_ANALYZE=3          # 분석
uint8 PHASE_ADJUST_PEG=4       # 페그 조절
uint8 PHASE_VERIFY=5           # 최종 검증 패스
uint8 PHASE_WIPE=6              # 넥 닦기 마무리 (옛 화음)
uint8 PHASE_RETURN=7           # 대기 자세 복귀
uint8 PHASE_DONE=8             # 완료
uint8 PHASE_PAUSED=9           # 일시정지
uint8 PHASE_CANCELED=10        # 취소
uint8 PHASE_ERROR=11           # 오류
uint8 PHASE_REGISTER_PEG=12    # 측정 전 미등록 페그 초기 등록
uint8 PHASE_PREPARE_PLUCK=13   # 피크 집기·발현 접근점 이동 (마이크 타이머 시작 전)

uint8 OVERALL_PENDING=0        # 판정 전
uint8 OVERALL_SUCCESS=1        # 4줄 모두 성공
uint8 OVERALL_PARTIAL=2        # 일부 성공
uint8 OVERALL_FAILED=3         # 모두 실패

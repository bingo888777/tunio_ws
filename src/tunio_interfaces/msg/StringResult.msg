# 줄 하나의 조율 결과.
# 조율 직후 값과 최종 검증 값을 함께 보존한다.

uint8 string_number        # 실제 줄 번호 (6, 5, 2, 1)
float32 target_hz          # 목표 주파수 (A4=440, 12평균율 기준 고정값)
float32 tuned_hz           # Phase 1 조율 루프에서 마지막으로 얻은 유효 측정값
bool has_tuned_hz          # 위 값의 유효 여부. false면 tuned_hz를 읽지 않는다 (DB에는 NULL)
float32 verified_hz        # 최종 검증 패스에서 얻은 측정값
bool has_verified_hz       # 위 값의 유효 여부
float32 final_error_cents  # 최종 판정에 쓴 센트 오차. 검증값이 있으면 그 기준
uint8 adjust_count         # 실제 수행한 페그 조절 횟수. 최초 측정은 포함하지 않는다
uint8 result               # 이 줄의 최종 판정
uint8 failure_reason       # result=RESULT_FAILED일 때의 사유

uint8 RESULT_PENDING=0     # 아직 처리 전
uint8 RESULT_SUCCESS=1     # 허용 센트 이내
uint8 RESULT_FAILED=2      # 실패

uint8 FAIL_NONE=0              # 실패 아님
uint8 FAIL_LOW_SIGNAL=1        # 소리 부족 (측정 재시도 소진)
uint8 FAIL_NO_PITCH=2          # 주파수 판별 불가 (측정 재시도 소진)
uint8 FAIL_TIMEOUT=3           # 응답 시간 초과 (측정 재시도 소진)
uint8 FAIL_ADJUST_EXHAUSTED=4  # 조절 횟수를 모두 쓰고도 허용 센트를 못 맞춤
uint8 FAIL_ROBOT_ERROR=5       # 로봇 동작 실패
uint8 FAIL_DETUNED_ON_VERIFY=6 # 검증 시 틀어짐

# 조율 파라미터 한 묶음.
# UI 설정 탭, 상태 Topic, DB 저장에 공통으로 쓴다.
# 작업 시작 시점의 값을 그대로 저장해, 이후 설정을 바꿔도 과거 작업의 기준이 바뀌지 않게 한다.

float32 tolerance_cents     # 허용 센트. |1200*log2(측정/목표)| 가 이 값 이내면 성공
uint8 max_adjust_count      # 줄당 최대 페그 조절 횟수 (임시 5)
uint8 max_measure_retry     # 측정 실패 시 페그를 돌리지 않고 다시 튕기는 최대 횟수 (임시 3)
float32 measure_timeout_sec # 측정 결과 응답 제한 시간 (임시 10.0)
float32 onset_timeout_sec   # onset 대기 상한. 이 안에 소리가 안 들어오면 실패 (임시 10.0)

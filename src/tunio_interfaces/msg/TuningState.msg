# /tuning/state 로 UI에 실시간 전달 (기획서 8.3)
string  phase              # 대기/이송/측정준비/발현/분석/페그조절/최종검증/복귀/완료/일시정지/취소/오류
uint8   current_string
float64 target_hz
float64 measured_hz
float64 error_hz
uint8   adjust_count
StringResult[] results     # 줄별 누적 결과
string  overall            # "" / 성공 / 부분성공 / 실패

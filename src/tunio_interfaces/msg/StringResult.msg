# 한 줄의 조율 결과
uint8   string_no          # 6,5,2,1
string  note               # "E2","A2","B3","E4"
float64 target_hz
float64 measured_hz        # 유효값 없으면 NaN
float64 error_cents
uint8   adjust_count       # 페그 조절 횟수
uint8   status             # 0=대기 1=성공 2=실패 3=검증시틀어짐
string  fail_reason        # "" / "소리부족" / "판별불가" / "시간초과" / "조절소진"

"""
OnRobot RG2 제어.

onrobot_rg_control 의 /onrobot/sendCommand 서비스(문자열 하나)로 조작한다.
명령 규약은 그 패키지의 genCommand 를 따른다.
  'c'        — 완전히 닫기
  'o'        — 완전히 열기
  'i' / 'd'  — 쥐는 힘을 2.5N 올리기 / 내리기
  숫자 문자열 — 목표 폭. 0.1mm 단위 정수 ("60" 이면 6.0mm)

힘을 숫자로 한 번에 못 준다. 서버가 최대힘(40N)으로 시작해서
'i'/'d' 로 2.5N씩만 움직이므로, 목표 힘까지 갈 단계 수를 계산해서 보낸다.
그 계산이 plan_commands 다. 로봇 없이 단위 테스트로 검증된다.

주의 — 런치할 때 gripper:=rg2 를 반드시 준다.
  onrobot_rg_control 의 기본값이 rg6 다. rg6 로 뜨면 한계값이
  힘 120N·폭 160mm 로 잡혀서, RG2 가 낼 수 없는 값을 걸러주지 못한다.

파지 확인 — 이 ROS 2 패키지는 grip detected 신호를 토픽으로 내보내지 않는다.
  하드웨어는 알려준다(OnRobotRGInput 의 gsta 비트 1). 그런데 그 토픽을
  발행하는 노드가 이 포팅본에 없다. 서버가 내부에서 읽어 /joint_states 의
  관절각으로만 흘린다. 파지 확인이 필요하면 둘 중 하나다 —
  (1) /joint_states 관절각을 폭으로 역산해 "덜 닫혔으면 물렸다"고 보거나
  (2) onrobot_rg_control 에 퍼블리셔를 추가하거나.
  확정 전까지 여기서는 파지 확인을 하지 않는다.
"""

# RG2 한계 (onrobot_rg_msgs 주석 기준)
MAX_FORCE_N = 40.0      # rgfr 0~400, 1/10N 단위
MAX_WIDTH_MM = 110.0    # rgwd 0~1100, 1/10mm 단위
FORCE_STEP_N = 2.5      # 'i' / 'd' 한 번에 움직이는 양 (rgfr 25)

# 서버가 시작할 때 잡는 힘. OnRobotRGControllerServer 가 max_force 로 초기화한다
INITIAL_FORCE_N = MAX_FORCE_N

SERVICE = "/onrobot/sendCommand"


class GripperError(Exception):
    """그리퍼 명령이 잘못됐거나 서비스가 응답하지 않을 때."""


def plan_commands(width_mm, force_n, current_force_n):
    """
    목표 폭·힘을 내기 위해 보낼 명령 문자열 순서를 만든다.

    반환: (명령 목록, 실제로 도달하는 힘 N)
    힘은 2.5N 단위로만 움직이므로 목표를 정확히 못 맞출 수 있다.
    실제 도달값을 같이 돌려주니 호출 쪽이 그 값을 다음 current_force_n 으로 쓴다.
    """
    if not 0.0 <= width_mm <= MAX_WIDTH_MM:
        raise GripperError(f"폭 {width_mm}mm 는 RG2 범위 0~{MAX_WIDTH_MM} 를 벗어난다")
    if not 0.0 <= force_n <= MAX_FORCE_N:
        raise GripperError(f"힘 {force_n}N 은 RG2 범위 0~{MAX_FORCE_N} 를 벗어난다")

    steps = round((force_n - current_force_n) / FORCE_STEP_N)
    cmds = ["i"] * steps if steps > 0 else ["d"] * (-steps)
    reached = current_force_n + steps * FORCE_STEP_N

    # 폭은 0.1mm 단위 정수로 보낸다. 힘을 먼저 맞추고 나서 닫는다
    cmds.append(str(int(round(width_mm * 10))))
    return cmds, reached


class RG2:
    """/onrobot/sendCommand 로 RG2 를 조작한다."""

    def __init__(self, node, service=SERVICE, timeout_sec=3.0):
        from onrobot_rg_msgs.srv import SetCommand
        self.node = node
        self.timeout_sec = timeout_sec
        self._force_n = INITIAL_FORCE_N
        self._cli = node.create_client(SetCommand, service)
        if not self._cli.wait_for_service(timeout_sec=timeout_sec):
            raise GripperError(
                f"그리퍼 서비스 {service} 가 없다. "
                f"onrobot_rg_control 을 gripper:=rg2 로 띄웠는지 확인하라")

    def _send(self, command):
        from onrobot_rg_msgs.srv import SetCommand
        import rclpy
        req = SetCommand.Request()
        req.command = command
        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_sec)
        res = future.result()
        if res is None:
            raise GripperError(f"그리퍼 명령 '{command}' 응답 없음")
        if not res.success:
            raise GripperError(f"그리퍼 명령 '{command}' 실패: {res.message}")

    def grip(self, width_mm, force_n):
        """지정한 폭까지 지정한 힘으로 닫는다."""
        cmds, reached = plan_commands(width_mm, force_n, self._force_n)
        for c in cmds:
            self._send(c)
        self._force_n = reached

    def release(self):
        """완전히 열어 놓는다."""
        self._send("o")

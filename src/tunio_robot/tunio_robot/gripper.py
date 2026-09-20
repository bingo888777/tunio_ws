"""
OnRobot RG2 제어 — 디지털 I/O + Compute Box 의 WebLogic.

제어 경로
  로봇 컨트롤러의 디지털 출력을 켜고 끄면, Compute Box(192.168.1.1)에서 돌고 있는
  WebLogic 프로그램이 그 조합에 맞춰 미리 설정된 파지를 실행한다.
  폭과 힘은 **웹 화면에서 정하는 값이고 코드가 못 바꾼다.**
  코드가 하는 일은 "몇 번 프리셋" 을 고르는 것뿐이다.

  Modbus 로 폭·힘을 직접 보내는 길(onrobot_rg_control 의 /onrobot/sendCommand)도
  있지만 이 현장은 WebLogic 을 쓴다. 두 경로를 같이 열어두면 어느 쪽이 실제로
  먹었는지 헷갈리므로 여기서는 I/O 만 쓴다.

프리셋을 바꾸려면
  1. 웹에서 폭·힘을 고친다
  2. teach_points.yaml 의 gripper.presets 기록을 같이 고친다
  기록이 어긋나도 로봇은 그대로 움직인다. 코드가 알 방법이 없다. 사람이 맞춰야 한다.

출력을 거는 순서
  먼저 쓰는 출력을 전부 0 으로 내리고, 그다음 1 로 올릴 것만 올린다.
  한 번에 하나씩 바꾸면 중간 조합이 순간적으로 다른 프리셋과 맞아떨어져
  엉뚱한 파지가 한 번 실행될 수 있다. 전부 0 인 조합은 어느 규칙과도 안 맞아
  그리퍼가 직전 상태를 유지한다.
"""
import time


class GripperError(Exception):
    """프리셋이 없거나 그리퍼 조작이 실패했을 때."""


class Preset:
    """WebLogic 규칙 하나. 출력 조합과 그 조합이 내는 폭·힘 기록."""

    def __init__(self, name, outputs, width_mm=None, force_n=None):
        self.name = name
        self.outputs = {int(k): int(v) for k, v in outputs.items()}
        self.width_mm = width_mm
        self.force_n = force_n
        for idx, val in self.outputs.items():
            if val not in (0, 1):
                raise GripperError(f"프리셋 '{name}' 의 출력 {idx} 값이 0/1 이 아니다: {val}")

    def __repr__(self):
        return f"<Preset {self.name} {self.outputs} {self.width_mm}mm {self.force_n}N>"


def parse_presets(config):
    """teach_points.yaml 의 gripper.presets 절을 읽는다."""
    if not config:
        raise GripperError("교시점 파일에 gripper.presets 가 없다")
    out = {}
    for name, item in config.items():
        outputs = item.get("outputs")
        if not outputs:
            raise GripperError(f"프리셋 '{name}' 에 outputs 가 없다")
        out[name] = Preset(name, outputs,
                           width_mm=item.get("width_mm"), force_n=item.get("force_n"))
    return out


def output_plan(preset, all_indexes):
    """
    디지털 출력을 세팅할 순서를 만든다.

    반환: [(출력번호, 값), ...]
    쓰는 출력을 전부 0 으로 내린 뒤 1 로 올릴 것만 올린다. 중간 조합이
    다른 프리셋과 맞아떨어지는 것을 막기 위해서다.
    """
    plan = [(i, 0) for i in sorted(all_indexes)]
    plan += [(i, 1) for i in sorted(preset.outputs) if preset.outputs[i] == 1]
    return plan


class RG2:
    """디지털 출력으로 WebLogic 프리셋을 고른다."""

    def __init__(self, api, presets, settle_sec=0.3):
        """
        프리셋 표를 들고 그리퍼 조작 준비를 한다.

        api        — DSR_ROBOT2 모듈 (set_digital_output 을 쓴다)
        presets    — parse_presets 결과
        settle_sec — 출력을 건 뒤 그리퍼 동작이 끝나기를 기다리는 시간.
                     RG2 는 폭에 따라 0.1~0.5초쯤 걸린다. 짧으면 아직 움직이는
                     중에 팔이 출발한다
        """
        self.api = api
        self.presets = presets
        self.settle_sec = settle_sec
        self._indexes = sorted({i for p in presets.values() for i in p.outputs})
        self.current = None

    def select(self, name):
        """이름으로 프리셋을 건다."""
        preset = self.presets.get(name)
        if preset is None:
            raise GripperError(
                f"'{name}' 프리셋이 없다. 있는 것: {sorted(self.presets)}. "
                f"웹에서 규칙을 만들고 teach_points.yaml 에도 적어야 한다")
        for idx, val in output_plan(preset, self._indexes):
            self.api.set_digital_output(idx, val)
        time.sleep(self.settle_sec)
        self.current = name
        return preset

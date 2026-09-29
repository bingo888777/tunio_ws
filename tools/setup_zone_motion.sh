#!/usr/bin/env bash
# 로봇 연결 없이 시뮬레이션 의존성과 생성 URDF만 준비한다.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
/usr/bin/python3 -m venv --system-site-packages .venv-zone
touch .venv-zone/COLCON_IGNORE
.venv-zone/bin/python -m pip install --only-binary=:all: \
  python-fcl==0.7.0.11 trimesh==5.1.0 pycollada==0.9.3 \
  numpy scipy PyYAML vtk PyQt5
source /opt/ros/jazzy/setup.bash
source /home/mook/ws_cobot_pjt/ws_dsr/install/setup.bash
.venv-zone/bin/python - <<'PYTHON'
import json
from pathlib import Path
import xacro
config = json.loads(Path('docs/simulation/work_motion_model.json').read_text())['tool_model']
xml = xacro.process_file(config['source_xacro']).toxml()
output = Path(config['urdf'])
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(xml)
print(f'RG2 모델 준비: {output}')
PYTHON
OPENBLAS_NUM_THREADS=1 .venv-zone/bin/python tools/work_motion_test.py --high-only --check

# flake8 스타일 검사
import os

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    config = os.path.join(os.path.dirname(__file__), '..', 'setup.cfg')
    rc, errors = main_with_errors(argv=['--config', config])
    assert rc == 0, f'flake8 errors:\n{errors}'

"""
Copyright (c) 2022 Teslabs Engineering S.L.
SPDX-License-Identifier: Apache 2.0
"""

from pathlib import Path
import sys

import pytest


_SCRIPT_DIR = Path(__file__).absolute().parent
sys.path.insert(0, str(_SCRIPT_DIR.parents[2]))


@pytest.fixture()
def data():
    """Pytest fixture to load test data files"""
    return _SCRIPT_DIR / "data"


@pytest.fixture()
def hal_path(data):
    """Pytest fixture to load test HAL files"""
    return data / "hal"

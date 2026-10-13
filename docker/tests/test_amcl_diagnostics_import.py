"""The standalone compiler check must also be safe for pytest collection."""
from pathlib import Path
import runpy
import sys
from unittest.mock import patch


def test_compiler_check_import_does_not_consume_pytest_arguments():
    path=Path(__file__).with_name('test_amcl_seed_diagnostics.py')
    with patch.object(sys,'argv',['pytest','-q']):
        namespace=runpy.run_path(str(path))
    assert callable(namespace['main'])

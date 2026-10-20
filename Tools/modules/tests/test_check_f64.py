# AP_FLAKE8_CLEAN
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import check_f64  # noqa: E402


@pytest.mark.parametrize('name', [
    '__aeabi_dadd', '__aeabi_dsub', '__aeabi_dmul', '__aeabi_ddiv', '__aeabi_dcmpeq', '__aeabi_dcmplt',
    '__aeabi_d2f', '__aeabi_d2iz', '__aeabi_d2uiz', '__aeabi_f2d', '__aeabi_i2d', '__aeabi_ui2d',
    '__aeabi_l2d', '__aeabi_ul2d', '__adddf3', '__subdf3', '__muldf3', '__divdf3', '__negdf2',
    '__extendsfdf2', '__truncdfsf2', '__fixdfsi', '__fixunsdfsi', '__fixdfdi', '__floatsidf',
    '__floatundidf', '__eqdf2', '__ltdf2', '__gedf2', '__unorddf2',
])
def test_f64_routines_are_found(name):
    assert check_f64.is_f64_routine(name)


@pytest.mark.parametrize('name', [
    '__aeabi_fadd', '__aeabi_fmul', '__aeabi_f2iz', '__aeabi_i2f', '__aeabi_idiv', '__aeabi_uldivmod',
    '__aeabi_memcpy', 'sinf', 'main', '__addsf3', '__divsf3', '__aeabi_ldiv0',
])
def test_other_symbols_pass(name):
    assert not check_f64.is_f64_routine(name)

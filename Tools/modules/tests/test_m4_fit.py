# AP_FLAKE8_CLEAN
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import m4_fit  # noqa: E402

TOP = 0x30040000


def test_window_for_one_byte_is_the_smallest_region():
    assert m4_fit.code_window(1, 4) == dict(base=TOP - 256, size=256, rx_base=TOP - 32)


def test_window_for_100k_disables_one_subregion():
    assert m4_fit.code_window(102400, 8) == dict(base=TOP - 131072, size=131072, rx_base=TOP - 114688)


def test_window_for_128k_exactly_and_over():
    assert m4_fit.code_window(131072, 8) == dict(base=TOP - 131072, size=131072, rx_base=TOP - 131072)
    assert m4_fit.code_window(131073, 8) is None


def test_window_waste_is_under_one_subregion_for_every_size():
    for rx_size in range(1, 131073, 61):
        window = m4_fit.code_window(rx_size, 8)
        assert window['base'] % window['size'] == 0
        assert window['base'] <= window['rx_base'] and window['rx_base'] + rx_size <= TOP
        assert TOP - window['rx_base'] - rx_size < max(window['size'] // 8, 8)


def test_reference_layout():
    fit = m4_fit.layout(file_size=100000, rx_size=90000, rx_alignment=8, rw_memory_size=3000, rw_alignment=8,
                        stack_size=32768, heap_size=32768)
    assert fit['result'] == 'ok'
    assert fit['rw_base'] == 0x30027448
    assert fit['load_peak'] == 201304 and fit['load_margin'] == 28072
    assert fit['run_need'] == 166872 and fit['run_margin'] == 62504


def test_staging_that_reaches_rw_does_not_fit():
    fit = m4_fit.layout(file_size=130000, rx_size=110000, rx_alignment=8, rw_memory_size=4000, rw_alignment=8,
                        stack_size=32768, heap_size=32768)
    assert fit['result'] == 'no-room-to-stage'


def test_heap_that_reaches_rw_does_not_fit():
    fit = m4_fit.layout(file_size=60000, rx_size=50000, rx_alignment=8, rw_memory_size=2000, rw_alignment=8,
                        stack_size=65536, heap_size=120000)
    assert fit['result'] == 'no-room-to-run'

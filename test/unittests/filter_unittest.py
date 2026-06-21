"""
Unit tests for calibration filter
TODO:
"""

import numpy as np
from os import path
import pytest
import sys
sys.path.append(path.join(path.dirname(path.dirname(path.dirname(__file__))), "calibrate"))
from filter import FilterParams, filter_data, moving_avg, continuous_threshold, continuous_sign, remove_spikes


class TestFilterParams:
    test_N = 200
    filter_params = FilterParams()
    def test_init_no_T(self):
        self.filter_params.set_N(self.test_N)
        assert self.filter_params.N == self.test_N

    def test_init_T_no_t(self):
        self.filter_params.T = 0.1
        self.filter_params.set_N(self.test_N)
        assert self.filter_params.N == self.test_N

    def test_init_T_and_t(self):
        self.filter_params.T = 0.1  # N=100
        t = np.linspace(0, 10, num=10001)  # 10s, 1000 Hz
        self.filter_params.set_N(self.test_N, t)  # N is ignored
        assert self.filter_params.N == 100

    def test_init_bad_fcn(self):
        with pytest.raises(ValueError):
            FilterParams(fcn="bad")


def test_continuous_threshold():
    output = continuous_threshold(np.linspace(0, 100, num=101), thresh=50, steepness=10)
    assert output[0] < 0.1
    assert output[-1] > 0.95


class TestFilterSignal:
    t = np.linspace(0, 10, num=10000)  # 10s, 1000 Hz
    lo_freq = np.sqrt(2) * np.sin(2 * np.pi * t)  # 1 Hz, avg_mag = 1
    hi_freq = np.sqrt(2) * np.sin(200 * np.pi * t)  # 100 Hz, avg_mag = 1
    def measure_response(self, signal: np.array, type: str, expect_reduced: bool):
        for fcn in FilterParams.fcn_options:
            filter_params = FilterParams(T=0.1, fcn=fcn)  # 10 Hz
            filter_params.set_N(None, t_data=self.t)
            filtered_mag = np.sqrt(np.sum(filter_data(signal, filter_params, type=type) ** 2) / len(signal))
            if expect_reduced:
                assert filtered_mag < 0.01
            else:
                assert filtered_mag > 0.95

    def test_lopass_reduce_hi_freq(self):
        self.measure_response(self.hi_freq, "lowpass", expect_reduced=True)

    def test_lopass_pass_lo_freq(self):
        self.measure_response(self.lo_freq, "lowpass", expect_reduced=False)

    def test_hipass_reduce_lo_freq(self):
        self.measure_response(self.lo_freq, "highpass", expect_reduced=True)

    def test_hipass_pass_hi_freq(self):
        self.measure_response(self.hi_freq, "highpass", expect_reduced=False)


class TestRemoveSpikes:
    filtered_out = pytest.approx(np.array([0,0,0,0,0,0,0,0,0]))
    def calc_all(self, raw):
        self.remove_1_up = remove_spikes(raw, [False], max_up_spike_width=1)
        self.remove_2_up = remove_spikes(raw, [False], max_up_spike_width=2)
        self.remove_1_down = remove_spikes(raw, [True], max_down_spike_width=1)
        self.remove_2_down = remove_spikes(raw, [True], max_down_spike_width=2)
        self.same_as_input = pytest.approx(raw)

    def test_width_1_up(self):
        self.calc_all(np.array([0,0,0,0,1,0,0,0,0]))
        assert self.remove_1_up == self.filtered_out
        assert self.remove_2_up == self.filtered_out
        assert self.remove_1_down == self.same_as_input
        assert self.remove_2_down == self.same_as_input

    def test_width_2_up(self):
        self.calc_all(np.array([0,0,0,0,1,1,0,0,0]))
        assert self.remove_1_up == self.same_as_input
        assert self.remove_2_up == self.filtered_out
        assert self.remove_1_down == self.same_as_input
        assert self.remove_2_down == self.same_as_input

    def test_width_1_down(self):
        self.calc_all(np.array([0,0,0,0,-1,0,0,0,0]))
        assert self.remove_1_up == self.same_as_input
        assert self.remove_2_up == self.same_as_input
        assert self.remove_1_down == self.filtered_out
        assert self.remove_2_down == self.filtered_out

    def test_width_2_down(self):
        self.calc_all(np.array([0,0,0,0,-1,-1,0,0,0]))
        assert self.remove_1_up == self.same_as_input
        assert self.remove_2_up == self.same_as_input
        assert self.remove_1_down == self.same_as_input
        assert self.remove_2_down == self.filtered_out



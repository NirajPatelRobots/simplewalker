"""
Unit tests for calibration
TODO:
    Test FilterParams initialization with N and T
"""

import numpy as np
from os import path
import pytest
import sys
sys.path.append(path.join(path.dirname(path.dirname(path.dirname(__file__))), "calibrate"))
from motor_model import ModelFcn


class MultiplierModelFcn(ModelFcn):
    def __init__(self, fcn_input, **kwargs):
        super().__init__(fcn_input, {"mult": 1.0}, kwargs)
    def __call__(self, results, nonlinparams) -> np.ndarray:
        return super().__call__(results, nonlinparams) * self.mult


class TestModelFcn:
    results = {"vel_f": np.ndarray([1, 2, 3])}

    def test_init_default(self):
        m = ModelFcn("vel_f", {"softness": 4, "fungibility": 2.3}, {})
        assert m.variable_param_names == ["softness", "fungibility"]
        assert m.softness == 4
        assert m.fungibility == 2.3

    def test_init_default_and_const(self):
        m = ModelFcn("vel_f", {"softness": 4, "fungibility": 2.3}, {"fungibility": 9.81})
        assert m.variable_param_names == ["softness"]
        assert m.softness == 4
        assert m.fungibility == 9.81

    def test_init_inherit_variable_params(self):
        m = ModelFcn("vel_f", {"softness": 4, "fungibility": 2.3}, {"fungibility": 9.81})
        m2 = ModelFcn(m, {"box_height": 5}, {})
        assert m2.variable_param_names == ["box_height", ["softness"]]
        assert m.softness == 4
        assert m.fungibility == 9.81

    def test_set_params(self):
        m = ModelFcn("vel_f", {"softness": 4, "fungibility": 2.3}, {"fungibility": 9.81})
        m.set_params({"softness": 100, "fungibility": 3.14})
        assert m.variable_param_names == ["softness"]
        assert m.softness == 100
        assert m.fungibility == 3.14

    def test_call_str(self):
        m = ModelFcn("vel_f", {}, {})
        c = m(self.results, {})
        assert np.all(c == self.results["vel_f"])

    def test_call_recursive(self):
        m = ModelFcn("vel_f", {}, {})
        m2 = ModelFcn(m, {}, {})
        c = m2(self.results, {})
        assert np.all(c == self.results["vel_f"])

    def test_call_lambda(self):
        m = ModelFcn((lambda results: results["vel_f"]), {}, {})
        c = m(self.results, {})
        assert np.all(c == self.results["vel_f"])

    def test_call_recursive_with_params(self):
        m = MultiplierModelFcn("vel_f", mult=3.0)
        m2 = ModelFcn(m, {}, {})
        c = m2(self.results, {})
        assert np.all(c == self.results["vel_f"] * 3.0)
        c = m2(self.results, {"input": {"mult": 10.0}})
        assert np.all(c == self.results["vel_f"] * 10.0)

    def test_call_bad_input(self):
        with pytest.raises(TypeError) as excinfo:
            m = ModelFcn(self.results["vel_f"], {}, {})
            m(self.results, {})
        assert "ndarray" in str(excinfo.value)



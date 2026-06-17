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
from motor_model import ModelFcn, get_nonlin_paramArr, assign_nonlin_parameters, model_fcns


class MultiplierModelFcn(ModelFcn):
    def __init__(self, fcn_input, **kwargs):
        super().__init__(fcn_input, {"mult": 1.0}, kwargs)
    def __call__(self, results, nonlinparams) -> np.ndarray:
        return super().__call__(results, nonlinparams) * self.mult


class AdderModelFcn(ModelFcn):
    def __init__(self, fcn_input, **kwargs):
        super().__init__(fcn_input, {"to_add": 0.0}, kwargs)
    def __call__(self, results, nonlinparams) -> np.ndarray:
        return super().__call__(results, nonlinparams) + self.to_add

def get_mock_model_fcns():
    return {
        "mult": MultiplierModelFcn("vel_f"),
        "mult_copy": MultiplierModelFcn("vel_f"),
        "mult_10": MultiplierModelFcn("vel_f", mult=10),
        "nested_mult": MultiplierModelFcn(MultiplierModelFcn("vel_f")),
        "10_nested_mult": MultiplierModelFcn(MultiplierModelFcn("vel_f"), mult=10),
        "add_mult": MultiplierModelFcn(AdderModelFcn("vel_f"))
    }


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
        assert m2.variable_param_names == ["box_height"]
        assert m2.fcn_input.variable_param_names == ["softness"]
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

    def test_init_bad_param_name(self):
        with pytest.raises(ValueError) as excinfo:
            ModelFcn(self.results["vel_f"], {"input": 1.1}, {})
        assert "input" in str(excinfo.value)
        with pytest.raises(ValueError) as excinfo:
            ModelFcn(self.results["vel_f"], {"normal_stuff": 0.8}, {"input": 1.1})
        assert "input" in str(excinfo.value)

    # Test call with model_fcns[mod](results, nonlinearparams[mod])
    def test_call_fcn_pass_nonlinparams(self, mocker):
        print("FIRST one")
        new_model_fcns = get_mock_model_fcns()
        mocker.patch("motor_model.model_fcns", new=new_model_fcns)
        model = ["mult", "mult_copy", "mult_10", "add_mult", "nested_mult", "10_nested_mult"]
        param_values = np.array([2.0, 3.0,  # mult, mult_copy, none from mult_10,
                                 4.0, 5.0,  # add_mult
                                 6.0, 7.0,  # nested_mult
                                 8.0        # 10_nested_mult
                                 ])
        nonlinear_params = assign_nonlin_parameters(model, param_values)
        expected_contributions = {
            "mult": self.results["vel_f"] * 2.0,
            "mult_copy": self.results["vel_f"] * 3.0,
            "mult_10": self.results["vel_f"] * 10,
            "add_mult": (self.results["vel_f"] + 5.0) * 4.0,  # params for mult before add
            "nested_mult": self.results["vel_f"] * 6 * 7,
            "10_nested_mult": self.results["vel_f"] * 8 * 10,
        }
        for mod in model:
            assert np.all(new_model_fcns[mod](self.results, nonlinear_params[mod]) == expected_contributions[mod])


class TestNonlinParamsConversion:
    def test_get_nonlin_paramArr_simple(self, mocker):
        print("OTHERONE")
        mocker.patch("motor_model.model_fcns", new=get_mock_model_fcns())
        nonlin_params = get_nonlin_paramArr(["mult", "mult_copy", "mult_10"])
        assert all(nonlin_params == [1.0, 1.0])

    def test_get_nonlin_paramArr_nested(self, mocker):
        mocker.patch("motor_model.model_fcns", new=get_mock_model_fcns())
        nonlin_params = get_nonlin_paramArr(["mult", "mult_copy", "mult_10", "add_mult", "nested_mult", "10_nested_mult"])
        assert all(nonlin_params == [1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0])

    def test_get_nonlin_dict(self, mocker):
        mocker.patch("motor_model.model_fcns", new=get_mock_model_fcns())
        model = ["mult", "mult_copy", "mult_10", "add_mult", "nested_mult", "10_nested_mult"]
        param_values = np.array([2.0, 3.0,  # mult, mult_copy, none from mult_10,
                                 4.0, 5.0,  # add_mult
                                 6.0, 7.0,  # nested_mult
                                 8.0        # 10_nested_mult
                                 ])
        nonlinear_params = assign_nonlin_parameters(model, param_values)
        assert nonlinear_params == {"mult": {"mult": 2.0},
                                    "mult_copy": {"mult": 3.0},
                                    "mult_10": {},
                                    "add_mult": {"mult": 4.0, "input": {"to_add": 5.0}},
                                    "nested_mult": {"mult": 6.0, "input": {"mult": 7.0}},
                                    "10_nested_mult": {"input": {"mult": 8.0}},
                                    }

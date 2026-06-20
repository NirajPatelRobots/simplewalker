# /// script
# dependencies = ["calibrate"]
# ///
""" motor model for calibration

Fun facts:
  - Need to think about current/inductance. It might be necessary.
  - theta_3dot is an alternate way to model inductance. Would be convenient but the signal is way too noisy.
    I assume it's ok if the 2nd derivative is the predicted variable and the 3rd derivative is one of the model fcns.
  - Tried using V and vel instead of V_f and vel_f, and filtering each fcn in make_independent_variable.
    Makes sense because we're comparing to acc_f, which can't change quickly. But it constrains what acc_pred can do.
    R^2 ~.78->.605. Using filtered signals for slowness and np.sign improved to .65.
  - Tried spring-mass-damper model that holds hidden spring pos and vel variables, gives acceleration as spring force
    pulling from true pos to spring pos. The hidden spring variables are numerically integrated from that accel.
    But, calibration optimizes it to pull in the opposite direction, setting accel to pull to the true pos. This gives
    it perfect prediction if the spring constant is high enough. The problem is that the 2nd order dynamics can't be
    predicted in the current framework. The spring is driven by the motor motion, so it's impossible to know the
    accel caused by the spring without first knowing the accel without the spring. Attempt saved in try_spring branch.
    Need multi-part calibration? Simpler model? More complex measurement matrix?
    -> Created ModelSpring which takes another model as input
  - Adding slow_omega makes the prediction look more wrong, even if it matches the spikes into motion more closely.
    It made the prediction of the return to 0 speed worse! And it fails validation because da/dV < 0 for vel=0!
TODO:
    Validate that d(acc_pred)/dV > 0
    Nonlinear params
        nonlinparam_ranges: list[(float | None, float | None)]
        Class for nonlinparams with names, current values, defaults, dict<->array conversion?
    Create model from string
    Move validate_params to model_functions.py and validate nonlin parameters
"""
import dataclasses
import json
import numpy as np
from filter import moving_avg, continuous_threshold, continuous_sign
from types import FunctionType

class ModelFcn:
    # fcn_input can be one of: a string key for results, another ModelFcn, or a (lambda) function(results) -> ndarray
    def __init__(self, fcn_input, default_params: dict, const_params: dict):
        if "input" in default_params.keys() or "input" in const_params.keys():
            raise ValueError("'input' is not a valid param name")
        self.fcn_input = fcn_input
        self.variable_param_names = [p for p in default_params.keys() if p not in const_params.keys()]
        self.set_params(default_params)
        self.set_params(const_params)  # override defaults
    def set_params(self, new_params: dict):
        for param_name, param_value in new_params.items():
            if param_name == "input":
                self.fcn_input.set_params(param_value)
            else:
                self.__dict__[param_name] = param_value
    def __call__(self, results) -> np.ndarray:
        if isinstance(self.fcn_input, str):
            return results[self.fcn_input]
        elif isinstance(self.fcn_input, ModelFcn):
            return self.fcn_input(results)
        elif isinstance(self.fcn_input, FunctionType):
            return self.fcn_input(results)
        raise TypeError(type(self.fcn_input))

@dataclasses.dataclass
class Params:
    lin: dict = dict
    nonlin: dict = dict


"""make array used for the independent variable in regression"""
def make_independent_variable(results, model, model_fcns, nonlinear_paramArr=None):
    X = np.hstack((results["V_f"].reshape(-1,1), results["vel_f"].reshape(-1,1)))
    results["slowness"] = 1 - continuous_threshold(moving_avg(np.abs(results["vel_f"]), 20), thresh=0.06, steepness=6)
    results["sign_vel_f"] = continuous_sign(moving_avg(results["vel_f"], 20), thresh=0.06, steepness=10)
    if nonlinear_paramArr is not None:
        nonlinearparams = assign_nonlin_parameters(model, nonlinear_paramArr, model_fcns)
        for mod in model:
            model_fcns[mod].set_params(nonlinearparams[mod])
    for mod in model:
        X = np.hstack((X, model_fcns[mod](results).reshape(-1, 1)))
    return X


def assign_lin_parameters(param_array, model):
    """assigns parameters based on regression outputs and model"""
    params = {}
    for mod, param in zip(["V", "omega"] + model, param_array, strict=True):
        params[mod] = float(param)
    return params

def get_nonlin_paramArr(model: list[str], model_fcns: dict) -> np.ndarray:
    def extract(fcn: ModelFcn, arr: list):
        arr.extend([fcn.__dict__[var] for var in fcn.variable_param_names])
        if isinstance(fcn.fcn_input, ModelFcn):
            extract(fcn.fcn_input, arr)
    arr = []
    for mod in model:
        extract(model_fcns[mod], arr)
    return np.array(arr)

def assign_nonlin_parameters(model: list[str], values: np.ndarray, model_fcns: dict) -> dict[str, dict | float]:
    def assign(fcn: ModelFcn, idx: int) -> [dict, int]:
        params = {param_name: float(values[idx + i]) for i, param_name in enumerate(fcn.variable_param_names)}
        idx += len(fcn.variable_param_names)
        if isinstance(fcn.fcn_input, ModelFcn):
            params["input"], idx = assign(fcn.fcn_input, idx)
        return params, idx
    idx = 0
    nonlinparams = {}
    for mod in model:
        nonlinparams[mod], idx = assign(model_fcns[mod], idx)
    return nonlinparams

def calc_model_contributions(X, paramArr, model):
    return {mod: param * X[:, i].transpose()
            for mod, param, i in zip(["V", "omega"] + model, paramArr, range(len(paramArr)), strict=True)}


def validate_params(params):
    # Voltage should push forward
    assert(params.lin["V"] > 0)
    # Friction should oppose motion
    assert(params.lin["omega"] < 0)
    assert("const_fric" not in params.lin or params.lin["const_fric"] < 0)
    assert("const_opposing_fric" not in params.lin or params.lin["const_opposing_fric"] < 0)
    if "static_fric" in params.lin:
        assert(params.lin["static_fric"] + params.lin["V"] > 0)


def sanitize_dict(d):
    return {k: (sanitize_dict(v) if type(v) is dict else float(v)) for (k, v) in d.items()}

def saveParams(params, filename = "new"):
    with open(filename if "." in str(filename) else str(filename) + ".json", "w") as file:
        json.dump(obj={"lin": sanitize_dict(params.lin), "nonlin": sanitize_dict(params.nonlin)}, fp=file, indent=2)


def loadParams(filename = "motorparams"):
    with open(filename if "." in str(filename) else str(filename) + ".json") as file:
        loaded = json.load(file)
    return Params(sanitize_dict(loaded["lin"]), sanitize_dict(loaded["nonlin"]))


if __name__ == "__main__":
    import sys
    params = loadParams(sys.argv[1])
    validate_params(params)
    print("Success, No Validation Errors")

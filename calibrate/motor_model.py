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
    Nonlinear params
        nonlinparam_ranges: list[(float | None, float | None)]
            minimize::Constraint. Combined with param validation?
        Class for nonlinparams with names, current values, defaults, dict<->array conversion?
        Inherit params up/down fcn signal chain, or have a way to set them equal
        Model fcns with 2 inputs. Ex: Punch using vel_f to find when to punch and another input for values during punch.
    Create model from string
    Set const and default params from input
    Differentiability? Difficult.
"""
import dataclasses
import numpy as np

class ModelFcn:
    # fcn_input can be one of: a string key for results, another ModelFcn, or a (lambda) function(results) -> ndarray
    def __init__(self, fcn_input, default_params: dict, const_params: dict):
        if "input" in default_params.keys() or "input" in const_params.keys():
            raise ValueError("'input' is not a valid param name")
        self.fcn_input = fcn_input
        self.const_param_names = list(const_params.keys())
        self.variable_param_names = [p for p in default_params.keys() if p not in self.const_param_names]
        self.set_params(default_params)
        self.set_params(const_params)  # override defaults
    def set_params(self, new_params: dict, set_const=True):
        for param_name, param_value in new_params.items():
            if param_name == "input":
                self.fcn_input.set_params(param_value)
            elif set_const or param_name not in self.const_param_names:
                self.__dict__[param_name] = param_value
    def __call__(self, results) -> np.ndarray:
        if isinstance(self.fcn_input, str):
            return results[self.fcn_input]
        else:
            return self.fcn_input(results)

@dataclasses.dataclass
class Params:
    lin: dict = dict
    nonlin: dict = dict
    def __str__(self):
        def strVal(d: dict):
            return ", ".join([(f"{k}: ({strVal(v)})" if len(v) > 0 else k) if type(v) is dict
                              else f"{k}={v:.3g}" for k, v in d.items()])
        return f"Lin: ({strVal(self.lin)}); Nonlin: ({strVal(self.nonlin)})"


"""make array used for the independent variable in regression"""
def make_independent_variable(results, model, model_fcns, nonlinear_paramArr=None):
    X = np.hstack((results["V_f"].reshape(-1,1), results["vel_f"].reshape(-1,1)))
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

def assign_nonlin_parameters(model: list[str], values: np.ndarray, model_fcns: dict, include_consts=False) -> dict[str, dict | float]:
    def assign(fcn: ModelFcn, idx: int) -> [dict, int]:
        params = {param_name: float(values[idx + i]) for i, param_name in enumerate(fcn.variable_param_names)}
        idx += len(fcn.variable_param_names)
        if include_consts:
            params |= {n: fcn.__dict__[n] for n in fcn.const_param_names}
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

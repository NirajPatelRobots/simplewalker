""" motor model for calibration

Fun facts:
  - Expect param["static_fric"] + param["V"] > 0 or else friction reverses
  - Need to think about current/inductance. It might be necessary.
  - theta_3dot is an alternate way to model inductance. Would be convenient but the signal is way too noisy.
    I *think* it's fine if the 2nd derivative is the predicted variable and the 3rd derivative is one of the model fcns.
  - Tried using V and vel instead of V_f and vel_f, and filtering each fcn in make_independent_variable.
    Makes sense because we're comparing to acc_f, which can't change quickly. But it constrains what acc_pred can do.
    R^2->.605. Using filtered signals for slowness and np.sign improved to .65.
TODO:
    Validate that d(acc_pred)/dV > 0
    Move slowness over here, and other sub-parameters if needed
    Some way to deal with nonlinear parameters like slowness threshold
    Better model sticky stops. Spring that stores and releases energy?
"""

import numpy as np
from filter import *


model_fcns = {
    "V": lambda results:                 results["V_f"],
    "omega": lambda results:             results["vel_f"],
    "const_fric": lambda results:        (1 - results["slowness"]) * np.sign(results["vel_f"]),
    "const_opposing_fric": lambda results:
    (1 - results["slowness"]) * np.where(np.sign(results["vel_f"]) == np.sign(results["V_f"]), np.sign(results["vel_f"]), 0),

    "static_fric": lambda results:       results["slowness"] * results["V_f"],
    "offset": lambda results:            1 - results["slowness"],
    "Vsq": lambda results:               np.abs(results["V_f"]) * results["V_f"],

    "theta_3dot": lambda results:        derivative(results, "acc"),
    "knee_leg_weight_2": lambda results: -np.sin(results["angle"]),
    "hip_leg_weight_12": lambda results: np.sin(results["knee_angle"]),
}


def Model(model, params):
    if params is None:
        model = model or []
    else:
        model = [m for m in params.keys() if m not in ["V", "omega"]]
    return model


"""make array used for the independent variable in regression"""
def make_independent_variable(results, model):
    X = np.hstack((results["V_f"].reshape(-1,1), results["vel_f"].reshape(-1,1)))
    results["slowness"] = continuous_threshold(moving_avg(np.abs(results["vel_f"]), 20), thresh=0.06, steepness=6)
    for mod in model:
        X = np.hstack((X, model_fcns[mod](results).reshape(-1, 1)))
    return X

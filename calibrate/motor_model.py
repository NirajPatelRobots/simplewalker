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
    Sub-parameters other than slowness if needed
    Some way to deal with nonlinear parameters like slowness threshold
    look at slowness factor (coulomb (and/or static?) fric too high)
    Better model sticky stops. Spring that stores and releases energy?
        Springs into action so it starts moving quickly, then oscillates.
        Observed oscillation T = 0.25 to 0.27s
            ^ Is that just the filter?
    Direct counter to low voltage
"""

import numpy as np
from filter import moving_avg, derivative, continuous_threshold, continuous_sign, integration


def Punch(signal, num_points=100, on_thresh=0.1, off_thresh=None, early=True) -> np.ndarray:
    off_thresh = off_thresh or on_thresh  # default to same
    old_signal = signal if early else np.roll(signal, num_points)
    new_signal = np.roll(signal, -num_points) if early else signal
    punch_force = np.where((np.abs(old_signal) < off_thresh) & (np.abs(new_signal) > on_thresh),
                           (new_signal - old_signal), 0)
    punch_force[0:num_points] = np.zeros(num_points)
    return punch_force


def Spring(unsprung_acc, results, freq, damping_eta):
    w = 2 * np.pi * freq;   k = w**2;   c = 2 * w * damping_eta
    spring_angle = np.zeros(unsprung_acc.shape)  # the secondary is pulled to the primary by the spring
    spring_vel = np.zeros(unsprung_acc.shape)
    spring_acc = np.zeros(unsprung_acc.shape)
    primary_angle = np.zeros(unsprung_acc.shape)  # the primary has both the unsprung_acc and the spring force on it
    primary_vel = np.zeros(unsprung_acc.shape)
    for i in range(1, len(unsprung_acc)):
        dt = results["t"][i] - results["t"][i - 1]
        if results["t"][i] in results["log_starts"].keys():
            for hidden_signal in [spring_angle, spring_vel, primary_angle, primary_vel]:
                hidden_signal[i] = 0
        else:
            spring_acc[i] = k * (primary_angle[i-1] - spring_angle[i-1]) + c * (primary_vel[i-1] - spring_vel[i-1])
            primary_acc = unsprung_acc[i] - spring_acc[i]
            spring_angle[i], spring_vel[i] = integration(spring_angle[i - 1], spring_vel[i - 1], spring_acc[i], dt)
            primary_angle[i], primary_vel[i] = integration(primary_angle[i - 1], primary_vel[i - 1], primary_acc, dt)
    return spring_acc


model_fcns = {
    "V": lambda results:                 results["V_f"],
    "omega": lambda results:             results["vel_f"],
    "const_fric": lambda results:        results["sign_vel_f"],
    "const_opposing_fric": lambda results:
    results["sign_vel_f"] * np.where(np.sign(results["vel_f"]) == np.sign(results["V_f"]), 1, 0),
    "static_fric": lambda results:       results["slowness"] * results["V_f"],

    # speed or Voltage punch "kicks-in" as signal is starting up
    "s_punch": lambda results:           Punch(results["vel_f"], num_points=95, on_thresh=0.05, early=False),
    "V_punch": lambda results:           Punch(results["V_f"],   num_points=130, on_thresh=0.5, off_thresh=0.4, early=False),
    "sharp_punch": lambda results:       Punch(results["vel_f"], num_points=50, on_thresh=0.05, early=False),
    "early_punch": lambda results:       Punch(results["vel_f"], num_points=80, on_thresh=0.05, early=True),
    "spring_punch": lambda results:      Spring(Punch(results["vel_f"], num_points=10, on_thresh=0.05), results, 3, 0.1),

    "slow_omega": lambda results:        results["slowness"] * results["vel_f"],
    "sign_V": lambda results:            continuous_sign(results["V_f"], 0.15, 12),
    "offset": lambda results:            1 - results["slowness"],
    "Vsq": lambda results:               np.abs(results["V_f"]) * results["V_f"],

    "theta_3dot": lambda results:        derivative(results, "acc"),
    "knee_leg_weight_2": lambda results: -np.sin(results["angle"]),
    "hip_leg_weight_12": lambda results: np.sin(results["knee_angle"]),
}

##################################################################################################

def Model(model, params):
    if params is None:
        model = model or []
    else:
        model = [m for m in params.keys() if m not in ["V", "omega"]]
    return model


"""make array used for the independent variable in regression"""
def make_independent_variable(results, model):
    X = np.hstack((results["V_f"].reshape(-1,1), results["vel_f"].reshape(-1,1)))
    results["slowness"] = 1 - continuous_threshold(moving_avg(np.abs(results["vel_f"]), 20), thresh=0.06, steepness=6)
    results["sign_vel_f"] = continuous_sign(moving_avg(results["vel_f"], 20), thresh=0.06, steepness=10)
    for mod in model:
        X = np.hstack((X, model_fcns[mod](results).reshape(-1, 1)))
    return X


def assign_parameters(param_array, model):
    """assigns parameters based on regression outputs and model"""
    params = {}
    for mod, param in zip(["V", "omega"] + model, param_array, strict=True):
        params[mod] = float(param)
    return params


def calc_model_contributions(X, paramArr, model):
    return {mod: param * X[:, i].transpose()
            for mod, param, i in zip(["V", "omega"] + model, paramArr, range(len(paramArr)), strict=True)}


def validate_params(params):
    # Voltage should push forward
    assert(params["V"] > 0)
    # Friction should oppose motion
    assert(params["omega"] < 0)
    assert("const_fric" not in params or params["const_fric"] < 0)
    assert("const_opposing_fric" not in params or params["const_opposing_fric"] < 0)
    if "static_fric" in params:
        assert(params["static_fric"] + params["V"] > 0)


def saveParams(params, filename = "new"):
    with open(filename if "." in filename else filename + ".learnedparams", "wb") as file:
        np.savez(file, **params)


def loadParams(filename = "motorparams"):
    filename = filename if "." in filename else filename + ".learnedparams"
    return {k: float(v) for (k, v) in np.load(filename).items()}


if __name__ == "__main__":
    import sys
    params = loadParams(sys.argv[1])
    validate_params(params)
    print("Success, No Validation Errors")

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
    Better model sticky stops. Spring that stores and releases energy?
        Springs into action so it starts moving quickly, then oscillates.
    Direct counter to low voltage
    Punch is great for deadband but way too hard for squares. Find a compromise.
TODO: infra
    Validate that d(acc_pred)/dV > 0
    Make slowness into a ModelFcn, use it to compose other Fcns
    New file for model_functions implementations?
    Nonlinear params
        nonlinparam_ranges: list[(float | None, float | None)]
        How should inherited variable_param_names be stored?
    Create model from string
"""

import numpy as np
from filter import moving_avg, derivative, continuous_threshold, continuous_sign, integration
from types import FunctionType

class ModelFcn:
    # fcn_input can be one of: a string key for results, another ModelFcn, or a (lambda) function(results) -> ndarray
    def __init__(self, fcn_input, default_params: dict, const_params: dict):
        self.fcn_input = fcn_input
        self.variable_param_names = [p for p in default_params.keys() if p not in const_params.keys()]
        self.set_params(default_params)
        self.set_params(const_params)  # override defaults
        if isinstance(self.fcn_input, ModelFcn):
            self.variable_param_names.append(fcn_input.variable_param_names)
    def set_params(self, new_params: dict):
        for param_name, param_value in new_params.items():
            self.__dict__[param_name] = param_value
    def __call__(self, results, nonlinparams) -> np.ndarray:
        self.set_params(nonlinparams)
        if isinstance(self.fcn_input, str):
            return results[self.fcn_input]
        elif isinstance(self.fcn_input, ModelFcn):
            return self.fcn_input(results, nonlinparams["input"] if "input" in nonlinparams else {})
        elif isinstance(self.fcn_input, FunctionType):
            return self.fcn_input(results)
        raise TypeError(type(self.fcn_input))


class Punch(ModelFcn):
    def __init__(self, fcn_input, early=False, **kwargs):
        super().__init__(fcn_input, {"num_points": 100, "on_thresh": 0.1, "off_thresh": 0.1}, kwargs)
        self.early = early
    # off_thresh is None -> off_thresh = on_thresh
    def __call__(self, results, nonlinparams) -> np.ndarray:
        input_acc = super().__call__(results, nonlinparams)
        off_thresh = self.off_thresh or self.on_thresh
        old_signal = input_acc if self.early else np.roll(input_acc, self.num_points)
        new_signal = np.roll(input_acc, -self.num_points) if self.early else input_acc
        punch_force = np.where((np.abs(old_signal) < off_thresh) & (np.abs(new_signal) > self.on_thresh),
                               (new_signal - old_signal), 0)
        punch_force[0:self.num_points] = np.zeros(self.num_points)
        return punch_force


class Delay(ModelFcn):
    def __init__(self, fcn_input, **kwargs):
        super().__init__(fcn_input, {"delay_samples": 0}, kwargs)
    def __call__(self, results, nonlinparams) -> np.ndarray:
        input_acc = super().__call__(results, nonlinparams)
        return np.roll(input_acc, self.delay_samples)


class Spring(ModelFcn):
    def __init__(self, fcn_input, **kwargs):
        super().__init__(fcn_input, {"freq": 3, "damping_eta": 0.1}, kwargs)
    def __call__(self, results, nonlinparams) -> np.ndarray:
        unsprung_acc = super().__call__(results, nonlinparams)
        w = 2 * np.pi * self.freq;   k = w**2;   c = 2 * w * self.damping_eta
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


# Model function classes are defined above. In model_fcns, the fcns are created with inputs and const params set.
model_fcns = {
    "V": lambda results:                 results["V_f"],
    "omega": lambda results:             results["vel_f"],
    "const_fric": lambda results:        results["sign_vel_f"],
    "const_opposing_fric": lambda results:
    results["sign_vel_f"] * np.where(np.sign(results["vel_f"]) == np.sign(results["V_f"]), 1, 0),
    "static_fric": lambda results:       results["slowness"] * results["V_f"],

    # speed or Voltage punch "kicks-in" as signal is starting up
    # "s_punch":           Punch("vel_f", num_points=95, on_thresh=0.02, off_thresh=0.17, early=False),
    "s_punch":           Punch("vel_f", num_points=95, on_thresh=0.05, off_thresh=None, early=False),
    "V_punch":           Punch("V_f",   num_points=130, on_thresh=0.5, off_thresh=0.4, early=False),
    "sharp_punch":       Punch("vel_f", num_points=30, on_thresh=0.02, off_thresh=0.05, early=False),
    "early_punch":       Punch("vel_f", num_points=80, on_thresh=0.05, early=True),
    "spring_punch":      Spring(Punch("vel_f", num_points=10, on_thresh=0.05), freq=3, damping_eta=0.1),
    "delayed_punch":     Delay(Punch("vel_f", num_points=95, on_thresh=0.05, early=False), delay_samples=200),

    "slow_omega": lambda results:        results["slowness"] * results["vel_f"],
    "sign_V": lambda results:            continuous_sign(results["V_f"], 0.15, 12),
    "offset": lambda results:            1 - results["slowness"],
    "Vsq": lambda results:               np.abs(results["V_f"]) * results["V_f"],

    "theta_3dot": lambda results:        derivative(results, "acc"),
    "knee_leg_weight_2": lambda results: -np.sin(results["angle"]),
    "hip_leg_weight_12": lambda results: np.sin(results["knee_angle"]),
}

for name, m in model_fcns.items():
    if not isinstance(m, ModelFcn):
        model_fcns[name] = ModelFcn(m, {}, {})

##################################################################################################

def Model(model, params):
    if params is None:
        model = model or []
    else:
        model = [m for m in params.keys() if m not in ["V", "omega"]]
    return model


"""make array used for the independent variable in regression"""
def make_independent_variable(results, model, nonlinearparams=None):
    nonlinearparams = None or {}
    X = np.hstack((results["V_f"].reshape(-1,1), results["vel_f"].reshape(-1,1)))
    results["slowness"] = 1 - continuous_threshold(moving_avg(np.abs(results["vel_f"]), 20), thresh=0.06, steepness=6)
    results["sign_vel_f"] = continuous_sign(moving_avg(results["vel_f"], 20), thresh=0.06, steepness=10)
    for mod in model:
        X = np.hstack((X, model_fcns[mod](results, nonlinearparams).reshape(-1, 1)))
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

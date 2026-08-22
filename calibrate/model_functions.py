# /// script
# dependencies = ["calibrate"]
# ///
""" Specific motor model functions for calibration
TODO:
    Validate that d(acc_pred)/dV > 0
    Punch (s_punch) matches db and big squares, but is too hard for smaller squares.
"""

import numpy as np
from filter import moving_avg, derivative, continuous_threshold, continuous_sign, integration
from motor_model import ModelFcn
from calibrate_fileIO import loadParams


class Punch(ModelFcn):
    def __init__(self, fcn_input, early=False, rise_only=True, start=None, **kwargs):
        start_params = {"on_thresh": 0.1, "off_thresh": 0.1} | (start or {})
        super().__init__(fcn_input, start_params, {"num_points": 100} | kwargs)
        self.early = early
        self.rise_only = rise_only
    # off_thresh is None -> off_thresh = on_thresh
    def __call__(self, results) -> np.ndarray:
        input_acc = super().__call__(results)
        off_thresh = self.off_thresh or self.on_thresh
        old_signal = input_acc if self.early else np.roll(input_acc, self.num_points)
        new_signal = np.roll(input_acc, -self.num_points) if self.early else input_acc
        punch_here = (np.abs(old_signal) < off_thresh) & (np.abs(new_signal) > self.on_thresh)
        if self.rise_only:
            punch_here &= (np.abs(new_signal) > np.abs(old_signal))
        punch_force = np.where(punch_here, (new_signal - old_signal), 0)
        punch_force[0:self.num_points] = np.zeros(self.num_points)
        return punch_force


class DifferenceSince(ModelFcn):
    def __init__(self, fcn_input, delay_samples=0):
        super().__init__(fcn_input, {}, {"delay_samples": delay_samples})
    def __call__(self, results) -> np.ndarray:
        input_acc = super().__call__(results)
        out_acc = input_acc - np.roll(input_acc, self.delay_samples)
        out_acc[0:self.delay_samples] *= 0
        return out_acc


class CutoffAbove(ModelFcn):
    def __init__(self, fcn_input, start=None, **kwargs):
        super().__init__(fcn_input, {"threshold": 0.0} | (start or {}), kwargs)
    def __call__(self, results) -> np.ndarray:
        input_signal = super().__call__(results)
        return np.where(input_signal > self.threshold, 0, input_signal)


class Delay(ModelFcn):
    def __init__(self, fcn_input, delay_samples=0):
        super().__init__(fcn_input, {}, {"delay_samples": delay_samples})
    def __call__(self, results) -> np.ndarray:
        input_acc = super().__call__(results)
        out_acc = np.roll(input_acc, self.delay_samples)
        out_acc[0:self.delay_samples] *= 0
        return out_acc


class Spring(ModelFcn):
    def __init__(self, fcn_input, **kwargs):
        super().__init__(fcn_input, {"freq": 3, "damping_eta": 0.1}, kwargs)
    def __call__(self, results) -> np.ndarray:
        unsprung_acc = super().__call__(results)
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


def slowness(vel_f: np.ndarray):
    return 1 - continuous_threshold(moving_avg(np.abs(vel_f), 20), thresh=0.06, steepness=6)
def sign_vel_f(vel_f: np.ndarray):
    return continuous_sign(moving_avg(vel_f, 20), thresh=0.06, steepness=10)

#############################################################################################


# Model function classes are defined above. In model_fcns, the fcns are created with inputs and const params set.
model_fcns = {
    "V": lambda results:                 results["V_f"],
    "omega": lambda results:             results["vel_f"],
    "const_fric": lambda results:        sign_vel_f(results["vel_f"]),
    "const_opposing_fric": lambda results:
    sign_vel_f(results["vel_f"]) * np.where(np.sign(results["vel_f"]) == np.sign(results["V_f"]), 1, 0),
    "static_fric": lambda results:       slowness(results["vel_f"]) * results["V_f"],
    "low_V":                             CutoffAbove("V_f", start={"threshold": 0.4}),

    # speed or Voltage punch "kicks-in" as signal is starting up
    # "s_punch":           Punch("vel_f", num_points=95, on_thresh=0.05, off_thresh=None, early=False),
    # "s_punch":           Punch("vel_f", num_points=95, on_thresh=0.17, off_thresh=0.25, early=False),
    "s_punch":           Punch("vel_f", num_points=180, early=False, rise_only=True,
                               start={"on_thresh": 0.0, "off_thresh": 0.03}),
    "diff_since":        DifferenceSince("vel_f", delay_samples=180),
    "V_punch":           Punch("V_f",   num_points=130, on_thresh=0.5, off_thresh=0.4, early=False),
    "sharp_punch":       Punch("vel_f", num_points=30, on_thresh=0.02, off_thresh=0.05, early=False),
    "early_punch":       Punch("vel_f", num_points=80, on_thresh=0.05, early=True),
    "spring_punch":      Spring(Punch("vel_f", num_points=10, on_thresh=0.05), freq=3, damping_eta=0.1),
    "delayed_punch":     Delay(Punch("vel_f", num_points=95, on_thresh=0.05, early=False), delay_samples=200),

    "slow_omega": lambda results:        slowness(results["vel_f"]) * results["vel_f"],
    "sign_V": lambda results:            continuous_sign(results["V_f"], 0.15, 12),
    "offset": lambda results:            1 - slowness(results["vel_f"]),
    "Vsq": lambda results:               np.abs(results["V_f"]) * results["V_f"],

    "theta_3dot": lambda results:        derivative(results, "acc"),
    "knee_leg_weight_2": lambda results: -np.sin(results["angle"]),
    "hip_leg_weight_12": lambda results: np.sin(results["knee_angle"]),
}

for name, m in model_fcns.items():
    if not isinstance(m, ModelFcn):
        model_fcns[name] = ModelFcn(m, {}, {})


#############################################################################################

def validate_params(params):
    # Voltage should push forward
    assert(params.lin["V"] > 0)
    # Friction should oppose motion
    assert(params.lin["omega"] < 0)
    assert("const_fric" not in params.lin or params.lin["const_fric"] < 0)
    assert("const_opposing_fric" not in params.lin or params.lin["const_opposing_fric"] < 0)
    if "static_fric" in params.lin:
        assert(params.lin["static_fric"] + params.lin["V"] > 0)


if __name__ == "__main__":
    import sys
    params = loadParams(sys.argv[1])
    validate_params(params)
    print("Success, No Validation Errors")

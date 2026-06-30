# /// script
# dependencies = ["calibrate"]
# ///
""" Specific motor model functions for calibration
TODO:
    Validate that d(acc_pred)/dV > 0
    Better model sticky stops. Spring that stores and releases energy?
        Springs into action so it starts moving quickly, then oscillates.
    - Direct counter to low voltage
    - Punch is great for deadband but way too hard for squares. Find a compromise.
"""

import numpy as np
from filter import moving_avg, derivative, continuous_threshold, continuous_sign, integration
from motor_model import ModelFcn, loadParams

class Punch(ModelFcn):
    def __init__(self, fcn_input, early=False, start=None, **kwargs):
        start_params = {"on_thresh": 0.1, "off_thresh": 0.1} | (start or {})
        super().__init__(fcn_input, start_params, {"num_points": 100} | kwargs)
        self.early = early
    # off_thresh is None -> off_thresh = on_thresh
    def __call__(self, results) -> np.ndarray:
        input_acc = super().__call__(results)
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
    def __call__(self, results) -> np.ndarray:
        input_acc = super().__call__(results)
        return np.roll(input_acc, self.delay_samples)


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

    # speed or Voltage punch "kicks-in" as signal is starting up
    # "s_punch":           Punch("vel_f", num_points=95, on_thresh=0.05, off_thresh=None, early=False),
    # "s_punch":           Punch("vel_f", num_points=95, on_thresh=0.17, off_thresh=0.25, early=False),
    "s_punch":           Punch("vel_f", num_points=95, early=False,
                               start={"on_thresh": 0.035, "off_thresh": 0.18}),
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

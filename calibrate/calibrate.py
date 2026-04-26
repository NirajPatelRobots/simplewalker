# -*- coding: utf-8 -*-
"""
Library and cli for motor calibration
TODO:
    more digital filter tuning (IIR vs FIR?)
        organize filter data
    slowness:
        look at slowness factor (coulomb (and/or static?) fric too high)
        graph slowness factor and acceleration?
        some way to compare slowness functions for static and const friction? (sub-model)
    some way to deal with nonlinear parameters, like slowness threshold
    angle noise wrong, high-freq graph big oscillations because filter after stitch. Do high-freq filter before stitch.
    measure battery voltage during calibration
    better model sticky stops. Spring that stores and releases energy?
    clean up and organize
        organized model definition
            Dict[model_name: (function(results: Dict) -> np.array)] ?
                ex "Vsq": (lambda results: return np.abs(results["V_f"]) * results["V_f"])
        more detailed cmd line main() with model, parameter output (new file?)
    graph individual variable contributions to model (use X matrix)
    think more about inductance (can model with 3rd derivative of angle)
    way to run with saved parameters
    electrically isolate angle sensors to reduce noise?

Created Jun 2021
@author: Niraj
"""

import numpy as np
from scipy import signal
from scipy.fft import rfftfreq
import time

FILT_N = 60
FILT_ORDER = 6
FILT_FCN = 'butter'  # 'cheby'
FILT_RIPPLE = 0.5  # only applies to cheby filters
FILT_MOVING_AVG_N = 10  # mostly to counter value quantization. After spike removal, before lowpass.
assert(FILT_MOVING_AVG_N < FILT_N)
if FILT_FCN == 'cheby':
    lowpass_sos = signal.cheby1(FILT_ORDER, FILT_RIPPLE, 1/FILT_N, 'lowpass', output='sos')
    hipass_sos  = signal.cheby1(FILT_ORDER, FILT_RIPPLE, 1/FILT_N, 'highpass',  output='sos')
else:
    lowpass_sos = signal.butter(FILT_ORDER, 1/FILT_N, 'lowpass', output='sos')
    hipass_sos  = signal.butter(FILT_ORDER, 1/FILT_N, 'highpass', output='sos')


def examineMotor(testdata, model = None, params = None):
    """return the model parameters (dict) of a motor.
    and the results of the examination (dict)
    V is 1D array of driving voltage, angle is same size array of sensed angle.
    V and angle can be one array or lists of arrays.
    params is optional existing parameters which may be edited.
    Motor model d2(angle)/dt2 = param[V] * V + param[omega] * d(angle)/dt
    model is a list of strings naming additional model dynamics"""

    def filter_and_derivative(testdata):
        results = {m: np.empty(0) for m in [m + f for m in ["V", "angle", "vel", "acc"] for f in ("", "_f")] + ["t"]}
        results["filter_period"] = testdata[0]["t"][FILT_N] - testdata[0]["t"][0]
        for this_data in testdata:
            clean_up_test_data(this_data, results)
            this_data["vel"] = derivative(this_data, "angle")
            this_data["acc"] = derivative(this_data, "vel")
            this_t = cut_first(this_data["t"])
            new_start_t = results["t"][-1] + (this_t[1] - this_t[0]) if len(results["t"]) > 0 else 0.
            results["t"] = np.concatenate((results["t"], this_t - this_t[0] + new_start_t))
            for m in ["V", "angle", "vel", "acc"]:
                results[m] = np.concatenate((results[m], cut_first(this_data[m])))
                results[m+"_f"] = np.concatenate((results[m+"_f"], filter_data(this_data[m])))
        results["diff_t"] = np.diff(results["t"])
        if np.any(results["diff_t"] <= 0):
            raise ValueError("ERROR: time stops or goes backwards", np.flatnonzero(results["diff_t"] <= 0))
        results["spiky_angle_hf"] = filter_data(results["spiky_angle"], type='high')
        results["angle_hf"] = filter_data(results["angle"], type='high')
        spiky_angle_noise = round(np.std(results["spiky_angle_hf"]) * 1000, 3)
        clean_angle_noise = round(np.std(results["angle_hf"]) * 1000, 3)
        print(f"Angle noise std_dev [mrad]: Spiky: {spiky_angle_noise} Clean: {clean_angle_noise}",
              f"Ratio: {round(clean_angle_noise/spiky_angle_noise, 3)}")
        results["N"] = np.size(results["V_f"])
        return results

    def derivative(data, name):
        return np.concatenate(([0], np.diff(data[name]) / np.diff(data["t"])))

    def make_independent_variable(results, model):
        """make array used for the independent variable in regression"""
        V_f = results["V_f"]
        vel_f = results["vel_f"]
        X = np.hstack((V_f.reshape(-1,1), vel_f.reshape(-1,1)))
        for mod in model:
            if mod == "const_fric":
                static_direction = np.sign(vel_f)
                X = np.hstack((X, ((1 - slowness_factor_continuous(vel_f)) * static_direction).reshape(-1,1)))
            if mod == "const_opposing_fric":
                opposing_direction = np.where(np.sign(vel_f) == np.sign(V_f), np.sign(vel_f), 0)
                X = np.hstack((X, ((1 - slowness_factor_continuous(vel_f)) * opposing_direction).reshape(-1,1)))
            # expect param["static_fric"] + param["V"] > 0 or else friction reverses
            if mod == "static_fric":
                X = np.hstack((X, (slowness_factor_continuous(vel_f) * V_f).reshape(-1,1)))
            if mod == "offset":
                X = np.hstack((X, (1 - slowness_factor_continuous(vel_f)).reshape(-1,1)))
            elif mod == "Vsq":
                X = np.hstack((X, (np.abs(V_f) * V_f).reshape(-1,1)))
            elif mod == "theta_3dot":   # alternate strategy to modeling current
                X = np.hstack((X, derivative(results, "acc_f").reshape(-1,1)))
            elif mod == "knee_leg_weight_2":
                X = np.hstack((X, -np.sin(results["angle_f"]).reshape(-1,1)))
            elif mod == "hip_leg_weight_1":
                X = np.hstack((X, -np.sin(results["angle_f"]).reshape(-1,1)))
            elif mod == "hip_leg_weight_12":
                X = np.hstack((X, np.sin(results["knee_angle_f"]).reshape(-1,1)))
        return X
    
    def determine_params(X, acc_f, results, remove_outliers=True):
        paramArr = np.linalg.inv(X.T @ X) @ X.T @ acc_f
        results["accel_predic"] = X @ paramArr
        if remove_outliers:
            accel_error = acc_f - results["accel_predic"]
            results["outlier_thresh"] = 3 * np.std(accel_error)
            outliers = np.abs(accel_error) > results["outlier_thresh"]
            acc_f_clean = np.where(outliers, 0., acc_f)
            X_clean = np.where(outliers.reshape((-1,1)) * np.ones((1,X.shape[1])), 0., X)
            results["num_outliers"] = np.sum(outliers)
            print("Removed", results["num_outliers"], "outliers out of", results["N"],
                  "(" + str(round(results["num_outliers"] / results["N"] * 100, 2))
                  + "%) where Error >", round(results["outlier_thresh"], 1), "rad/s^2")
            return determine_params(X_clean, acc_f_clean, results, remove_outliers=False)
        else:
            return paramArr
        
    def assign_parameters(param_array, params, model):
        """assigns parameters based on regression outputs and model"""
        for mod, param in zip(["V", "omega"] + model, param_array, strict=True):
            params[mod] = float(param)
        return params

    def calc_result_stats(results):
        accel_error = results["acc_f"] - results["accel_predic"]
        results["accel_error"] = accel_error
        results["error_std_dev"] = np.std(accel_error)
        results["R^2_with_outliers"] = 1 - np.sum(accel_error**2) / np.sum(results["acc_f"]**2)
        results["accel_error_nan"] = np.where(np.abs(accel_error) < results["outlier_thresh"], accel_error, np.nan)
        acc_f_nan = np.where(np.abs(accel_error) < results["outlier_thresh"], results["acc_f"], np.nan)
        results["avg_acc_no_outliers"] = np.nanmean(np.abs(acc_f_nan))
        results["R^2_no_outliers"] = 1 - np.nansum(results["accel_error_nan"]**2) / np.nansum(acc_f_nan**2)
        results["dt"] = float(np.mean(results["diff_t"]))
        results["dt_std_dev"] = float(np.std(np.where(results["diff_t"] > 10 * results["dt"], results["dt"], results["diff_t"])))
        results["fft_freqs"] = None
        if results["dt_std_dev"] / results["dt"] < 0.01:  # if dt is stable to within 1% precision
            results["fft_freqs"] = rfftfreq(results["N"], results["dt"])

    if len(testdata) == 0:
        raise ValueError("No testdata")
    if params is None:
        params = {}
    if model is None:
        model = []
    results = filter_and_derivative(testdata)
    
    # use Linear Least Squares regression to determine parameters
    X = make_independent_variable(results, model)
    startTime = time.perf_counter()
    paramArr = determine_params(X, results["acc_f"], results, remove_outliers=True)
    print("Parameter determination took", time.perf_counter() - startTime, "s")
    params = assign_parameters(paramArr, params, model)
    calc_result_stats(results)
    return params, results


def printMotorResults(params, results):
    print(f'dt: {results["dt"]:.3} s; dt std dev: {results["dt_std_dev"]:.2e} s;'
          f' total data runtime: {np.max(results["t"]):.0f} s')
    print("Parameters:", params, end="\n\n")
    print(f'Filter nyquist period [s]: {2 * results["filter_period"]:.3}')
    print("Average error =", round(np.average(np.abs(results["accel_error"])), 3), "rad/s^2, Average acceleration",
          round(np.sum(np.abs(results["acc_f"] )) / results["N"], 3), "rad/s^2")
    print("Error std dev:", round(results["error_std_dev"], 3), "rad/s^2")
    print("Regression R^2:", round(results["R^2_with_outliers"], 3))
    print(f'\tWithout {results["num_outliers"]} outliers:')
    print("Average error =", round(np.nanmean(np.abs(results["accel_error_nan"])), 3), "rad/s^2, Average acceleration",
          round(results["avg_acc_no_outliers"], 3), "rad/s^2")
    results["error_std_dev_clean"] = np.nanstd(np.abs(results["accel_error_nan"]))
    print("Error std dev:", round(results["error_std_dev_clean"], 3), "rad/s^2")
    print("Regression R^2:", round(results["R^2_no_outliers"], 3))


def loadRun(filename):
    if not "motortest" in filename:
        filename = filename + ".motortest"
    try:
        with open(filename, 'rb') as file:
            data = dict(np.load(file, allow_pickle=True))
            data["filename"] = filename
            if "t" not in data:
                data["t"] = np.linspace(0., len(data["V"]) * data["dt"], num=len(data["V"]))
            return data
    except Exception as e:
        print("Load failed:", filename, " because \n", e)

def cut_first(data, n=FILT_N * 4):
    return data[n:]

def filter_data(data, type='lowpass'):
    n_suspicious_filtered = FILT_N * 2
    padded_data = np.concatenate((np.average(data[:n_suspicious_filtered]) * np.ones(n_suspicious_filtered), data))
    if type == 'lowpass' or type == 'low':
        filtered = signal.sosfilt(lowpass_sos, padded_data)
    else:
        filtered = signal.sosfilt(hipass_sos, padded_data)
    filtered = filtered[n_suspicious_filtered:]
    if type == 'lowpass' or type == 'low':
        filtered = cut_first(filtered)
    return filtered


def clean_up_test_data(this_data, results):
    """ the angle is fuzzy, with noticeable spikes down from a probable true value. Remove those.
    down: [True] to remove downspikes only, [True, False] to remove down then up for each width.
    recalc change is whether to recalculate changes after every spike removal. might remove this."""
    def remove_spikes(angle: np.ndarray, down: list[bool], recalc_change=False,
                      min_spike_start_size=0.001, max_up_spike_width=4, max_down_spike_width=4):
        future_length = max(max_up_spike_width, max_down_spike_width) + 1
        prev_angle = np.roll(angle, 1)
        prev_angle[0] = prev_angle[1]
        # distance from previous angle to this angle and several future angles. [:,i] is i samples in the future.
        change_from_prev_angle = np.empty((len(angle), future_length))
        if not recalc_change:
            for i in range(future_length):
                change_from_prev_angle[:, i] = np.roll(angle, -i) - prev_angle
        for width in range(1, future_length):
            for is_down in down:
                if width > (max_down_spike_width if is_down else max_up_spike_width):
                    continue
                if recalc_change:
                    for i in range(future_length):
                        change_from_prev_angle[:, i] = np.roll(angle, -i) - prev_angle
                change_across_spike = change_from_prev_angle[:, width]
                if is_down:
                    is_away_from_prev = change_from_prev_angle < -min_spike_start_size
                    ends_going_back = (change_from_prev_angle[:, width-1] < change_across_spike)
                else:
                    is_away_from_prev = change_from_prev_angle > min_spike_start_size
                    ends_going_back = (change_from_prev_angle[:, width-1] > change_across_spike)
                all_samples_away_from_prev = np.all(is_away_from_prev[:, 0:width], axis=1)
                is_start_of_spike = (all_samples_away_from_prev & ends_going_back)
                for index_in_spike in range(width):
                    is_this_index_in_spike = np.roll(is_start_of_spike, index_in_spike)
                    interpolated_value_unshifted = (index_in_spike + 1) / (width + 1) * change_across_spike + prev_angle
                    interpolated_value = np.roll(interpolated_value_unshifted, index_in_spike)
                    angle = np.where(is_this_index_in_spike, interpolated_value, angle)
        return angle

    def remove_idle_messages(this_data, t_max=100):
        index_first = np.flatnonzero(this_data["t"] < t_max)[0]
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][index_first:]
        high_t = np.flatnonzero(this_data["t"] > t_max)
        index_last = high_t[0] if len(high_t) > 0 else len(this_data["t"])
        if index_first > 0 or len(this_data['angle']) > index_last:
            print(f"Cut off first {index_first}, last {len(this_data['angle']) - index_last}, now {index_last}")
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][:index_last]

    startTime = time.time()
    remove_idle_messages(this_data)
    # save "spiky_angle" so we can overwrite testdata["angle"] with clean_angle
    if "spiky_angle" in results:
        results["spiky_angle"] = np.concatenate((results["spiky_angle"], cut_first(this_data["angle"])))
    else:
        results["spiky_angle"] = cut_first(this_data["angle"])
    this_data["angle"] = remove_spikes(this_data["angle"], [False], max_up_spike_width=2)
    this_data["angle"] = remove_spikes(this_data["angle"], [True], max_down_spike_width=4)
    # this_data["angle"] = remove_spikes(this_data["angle"], [False, True], recalc_change=True)
    this_data["angle"] = moving_avg(this_data["angle"], n_samples=FILT_MOVING_AVG_N, pad_val=this_data["angle"][0])
    duration = time.time() - startTime
    results["data clean time"] = results["data clean time"] + duration if "data clean time" in results else duration


def moving_avg(data, n_samples=1, pad_val=0):
    return np.concatenate((np.ones(n_samples-1) * pad_val, np.convolve(data, np.ones(n_samples), 'valid') / n_samples))

# continuous number [0,1] where 1 means it's slow enough for static friction
def slowness_factor_continuous(vel, speed_thresh=0.06, steepness=6, moving_avg_samples=20):
    # arctangent is useful because it has a flat response in both limits
    # return np.arctan((np.abs(vel) < speed_thresh) * 20 - 10) / np.pi + 0.5
    # return np.arctan(speed_thresh / np.abs(vel)) / np.pi * 2
    speed = moving_avg(np.abs(vel), moving_avg_samples)
    return np.arctan((speed_thresh - speed) * steepness / speed_thresh) / np.pi + 0.5
    # return np.clip(speed_thresh / np.abs(vel), 0., 1.)


def saveParams(params, filename = "new"):
    with open(filename +".learnedparams", "wb") as file:
        np.savez(file, **params)
    
def loadParams(filename = "motorparams"):
    try:
        with open(filename+".learnedparams", "rb") as file:
            return dict(np.load(file))
    except FileNotFoundError:
        print("Couldn't load parameters")
        return None
        

def main():
    import sys
    if len(sys.argv) < 2:
        raise ValueError("First arg should be file pattern")
    import glob
    testdata = [loadRun(f) for f in sorted([f for i in range(1, len(sys.argv)) for f in glob.glob(sys.argv[i])])]  # :P
    print("Files:", [data["filename"] for data in testdata])
    params, results = examineMotor(testdata, model=['static_fric', 'const_opposing_fric'])
    printMotorResults(params, results)


if __name__ == "__main__":
    main()

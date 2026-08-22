# -*- coding: utf-8 -*-
"""
Library for motor calibration
TODO:
    Nonlinear integer optimization
    Indicate when parameter has little effect on optimization
    deadband model: modify angle measurements?
    measure battery voltage during calibration
    why angle jumps? Just remove them from test data?
    electrically isolate angle sensors to reduce noise?
    structure: clear flow of sensor data -> /(sensor model) -> believed true values -> /(motor model) -> prediction
    Way of representing how much data is lost when filtering is increased causing R^2 to increase
    prediction accuracy as fcn of vel?
    fourier analysis (compare F_V(w) with F_acc_pred(w)) to find frequency response of system
    Screw a second motor to the calibration motor, short its motor wires, and use it to measure with (2x) backlash?
    Stop functions from "cheating" with perfect prediction by using true acc derivative of results
        Perturb with noise?
        Enforce Causality: Don't allow fcns to use vel_f values more recent than now (or now - start of filter period)?
        vel_f used by model fcns has different filter?

Created Jun 2021
@author: Niraj
"""

import numpy as np
from scipy.optimize import minimize
import time
from filter import FilterParams, filter_data, cut_first, moving_avg, derivative, remove_spikes
import motor_model
from model_functions import model_fcns


def examineMotor(testdata, model, params, filter_params=FilterParams(), test_only=False):
    """return the model parameters (dict) of a motor.
    and the results of the examination (dict)
    V is 1D array of driving voltage, angle is same size array of sensed angle.
    V and angle can be one array or lists of arrays.
    params is optional existing parameters which may be edited.
    Motor model d2(angle)/dt2 = param[V] * V + param[omega] * d(angle)/dt
    model is a list of strings naming additional model dynamics"""

    results_t = dict[str, float | int | np.ndarray | np.float64 | np.int64 | dict | str]

    def filter_and_derivative_and_assemble(testdata, filter_params) -> results_t:
        results: results_t = {m: np.empty(0) for m in ["t", "angle_hf", "spiky_angle_hf"]
                   + [n + f for n in ["V", "angle", "vel", "acc", "spiky_angle"] for f in ("", "_f")]}
        for this_data in testdata:
            if filter_params.T is not None:
                filter_params.set_N(N=None, t_data=this_data["t"])
            clean_up_test_data(this_data, results, filter_params)
            this_data["vel"] = derivative(this_data, "angle")
            this_data["acc"] = derivative(this_data, "vel")
            assemble_t(this_data, results, filter_params)
            for m in ["V", "angle", "vel", "acc", "spiky_angle"]:
                results[m] = np.concatenate((results[m], cut_first(this_data[m], filter_params)))
                results[m+"_f"] = np.concatenate((results[m+"_f"], filter_data(this_data[m], filter_params)))
            for m in ["spiky_angle", "angle"]:
                results[m + "_hf"] = np.concatenate((results[m+"_hf"], filter_data(this_data[m], filter_params, type='high')))
        results["diff_t"] = np.diff(results["t"])
        if np.any(results["diff_t"] <= 0):
            raise ValueError("ERROR: time stops or goes backwards", np.flatnonzero(results["diff_t"] <= 0))
        results["N"] = np.size(results["V_f"])
        results["filter_period"] = results["t"][filter_params.N] - results["t"][0]
        results["filter_desc"] = str(filter_params)
        return results

    def assemble_t(this_data, results, filter_params):
        this_t = cut_first(this_data["t"], filter_params)
        new_start_t = results["t"][-1] + (this_t[1] - this_t[0]) if len(results["t"]) > 0 else 0.
        results["t"] = np.concatenate((results["t"], this_t - this_t[0] + new_start_t))
        if "log_starts" not in results:
            results["log_starts"] = {}
        results["log_starts"][new_start_t] = this_data["filename"]

    def determine_params(X, acc_f, results, remove_outliers=True):
        paramArr = np.linalg.inv(X.T @ X) @ X.T @ acc_f  # linear least squares
        if remove_outliers:
            accel_error = acc_f - X @ paramArr
            results["outlier_thresh"] = 3 * np.std(accel_error)
            outliers = np.abs(accel_error) > results["outlier_thresh"]
            acc_f_clean = np.where(outliers, 0., acc_f)
            X_clean = np.where(outliers.reshape((-1,1)), 0., X)
            return determine_params(X_clean, acc_f_clean, results, remove_outliers=False)
        else:
            return paramArr

    def calc_result_stats(results, calc_all=False):
        accel_error = results["accel_predic"] - results["acc_f"]
        results["error_std_dev"] = np.std(accel_error)
        results["outlier_thresh"] = 3 * results["error_std_dev"]
        results["accel_error_nan"] = np.where(np.abs(accel_error) < results["outlier_thresh"], accel_error, np.nan)
        acc_f_nan = np.where(np.abs(accel_error) < results["outlier_thresh"], results["acc_f"], np.nan)
        results["R^2_no_outliers"] = 1 - np.nansum(results["accel_error_nan"]**2) / np.nansum(acc_f_nan**2)
        if calc_all:
            results["accel_error"] = accel_error
            results["num_outliers"] = np.sum(np.abs(accel_error) > results["outlier_thresh"])
            results["outlier%"] = results["num_outliers"] / results["N"] * 100
            results["avg_error"] = np.average(np.abs(accel_error))
            results["R^2_with_outliers"] = 1 - np.sum(accel_error**2) / np.sum(results["acc_f"]**2)
            results["avg_acc_no_outliers"] = np.nanmean(np.abs(acc_f_nan))
            results["avg_error_no_outs"] = np.nanmean(np.abs(results["accel_error_nan"]))
            results["error_std_dev_no_outs"] = np.nanstd(np.abs(results["accel_error_nan"]))
            results["dt"] = float(np.mean(results["diff_t"]))
            results["dt_std_dev"] = float(np.std(np.where(results["diff_t"] > 10 * results["dt"], results["dt"], results["diff_t"])))
            minutes, seconds = divmod(int(np.max(results["t"])), 60)
            results["runtime"] = f'{minutes:02d}:{seconds:02d}' if minutes > 0 else f'{seconds} s'

    def calc_residual(nonlin_paramArr, results, model, model_fcns):
        X = motor_model.make_independent_variable(results, model, model_fcns, nonlin_paramArr)
        paramArr = determine_params(X, results["acc_f"], results, remove_outliers=True)
        results["accel_predic"] = X @ paramArr
        calc_result_stats(results)
        return 1 - results["R^2_no_outliers"]

    def optimize(initial_nonlin_paramArr, results, model, model_fcns, xatol=1e-6, fatol=1e-5) -> np.ndarray:
        startTime = time.perf_counter()
        result = minimize(lambda x: calc_residual(x, results, model, model_fcns),
                          x0=initial_nonlin_paramArr,
                          method="Nelder-Mead", options={"xatol": xatol, "fatol": fatol})
        optimize_time = round(time.perf_counter() - startTime, 3)
        results["optimize_desc"] = f"N-M {xatol=} {fatol=}"
        print(f"- Optimization ({results['optimize_desc']}) took {result.nfev} tries in {optimize_time}s")
        print("Start:", initial_nonlin_paramArr.transpose(), "End:", result.x.transpose())
        if not result.success:
            print(result)
            exit(result.status)
        return result.x

    if len(testdata) == 0:
        raise ValueError("No testdata")
    results = filter_and_derivative_and_assemble(testdata, filter_params)

    if params is None:
        params = motor_model.Params()
        model = model or []
    else:
        if test_only:
            model = [m for m in params.lin.keys() if m not in ["V", "omega"]]
        print("Loaded nonlinear params:", params.nonlin)
        for mod in model:
            if mod in params.nonlin:
                model_fcns[mod].set_params(params.nonlin[mod], set_const=False)
    # now set nonlin_paramArr from the model fcns, which are either defaults or just set from params.nonlin above^
    nonlin_paramArr = motor_model.get_nonlin_paramArr(model, model_fcns)

    if (not test_only) and (len(nonlin_paramArr) > 0):
        nonlin_paramArr = optimize(nonlin_paramArr, results, model, model_fcns)
    X = motor_model.make_independent_variable(results, model, model_fcns, nonlin_paramArr)
    paramArr = determine_params(X, results["acc_f"], results, remove_outliers=True)
    results["accel_predic"] = X @ paramArr
    calc_result_stats(results, True)
    results["model_contributions"] = motor_model.calc_model_contributions(X, paramArr, model)
    params.lin = motor_model.assign_lin_parameters(paramArr, model)
    params.nonlin = motor_model.assign_nonlin_parameters(model, nonlin_paramArr, model_fcns, include_consts=True)
    return params, results


def printMotorResults(params, results):
    print(f'\ndt: {results["dt"]:.3} s; dt std dev: {results["dt_std_dev"]:.2e} s; total runtime:', results["runtime"])
    spiky_angle_noise = np.std(results["spiky_angle_hf"]) * 1000
    clean_angle_noise = np.std(results["angle_hf"]) * 1000
    print(f"Angle noise std_dev [mrad]: Spiky: {round(spiky_angle_noise, 3)} Clean: {round(clean_angle_noise, 3)}",
          f"Ratio: {round(clean_angle_noise/spiky_angle_noise, 3)}")
    print(f'{results["filter_desc"]} filter; Period = {round(results["filter_period"] * 1000)}ms')
    print("Parameters:", params, end="\n\n")
    print("Average error =", round(results["avg_error"], 3), "rad/s^2, Average acceleration =",
          round(np.sum(np.abs(results["acc_f"] )) / results["N"], 3), "rad/s^2")
    print("Error std dev:", round(results["error_std_dev"], 3), "rad/s^2")
    print("Regression R^2:", round(results["R^2_with_outliers"], 3))
    print(f'- Without {results["num_outliers"]} outliers ({round(results["outlier%"], 2)}%):')
    print("Average error =", round(results["avg_error_no_outs"], 3), "rad/s^2, Average acceleration",
          round(results["avg_acc_no_outliers"], 3), "rad/s^2")
    print("Error std dev:", round(results["error_std_dev_no_outs"], 3), "rad/s^2")
    print("Regression R^2:", round(results["R^2_no_outliers"], 3))


def clean_up_test_data(this_data, results, filter_params):
    def remove_idle_messages(this_data, t_max=100):
        index_first = np.flatnonzero(this_data["t"] < t_max)[0]
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][index_first:]
        high_t = np.flatnonzero(this_data["t"] > t_max)
        index_last = high_t[0] if len(high_t) > 0 else len(this_data["t"])
        for m in ["V", "angle", "t"]:
            this_data[m] = this_data[m][:index_last]

    remove_idle_messages(this_data)
    # save "spiky_angle" so we can overwrite testdata["angle"] with clean_angle
    this_data["spiky_angle"] = this_data["angle"]
    # the angle is fuzzy, with noticeable spikes down from a probable true value. Remove those.
    this_data["angle"] = remove_spikes(this_data["angle"], [False], max_up_spike_width=2)
    this_data["angle"] = remove_spikes(this_data["angle"], [True], max_down_spike_width=4)
    # moving average, mostly to counter value quantization. After spike removal, before lowpass.
    this_data["angle"] = moving_avg(this_data["angle"], n_samples=filter_params.angle_moving_avg_N,
                                    pad_val=this_data["angle"][0])

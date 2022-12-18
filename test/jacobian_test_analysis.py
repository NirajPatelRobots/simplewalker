""" Analyze Jacobian unit test through logged data
October 2022
TODO:
    cmd line settings
    read settings from file?
    save individual jacobian results so there's only one copy of base_input and base_out for each jacbian
    maybe main should be a jupyter notebook
"""

import numpy as np
from display.logReader import LoadedLog
import matplotlib.pyplot as plt
import sys


def reduce_to_1D(array: np.ndarray):
    new_array = array
    while len(new_array.shape) > 1:
        new_array = np.linalg.norm(new_array, axis=0)
    return new_array


def angle_between(vec1: np.ndarray, vec2: np.ndarray) -> np.float64:
    vec1_unit = vec1 / reduce_to_1D(vec1)
    vec2_unit = vec2 / reduce_to_1D(vec2)
    return np.arccos(np.clip(np.sum(vec1_unit * vec2_unit, axis=0), -1.0, 1.0))


def calc_error(results: LoadedLog) -> None:
    def clip_errors():
        results.add_field("clipped_abs_err_mag", np.clip(results.abs_err_mag, 0, 4 * np.mean(results.abs_err_mag)))
        results.add_field("clipped_frac_err_mag", np.clip(results.frac_err_mag, 0, 0.2))
        results.add_field("clipped_err_angle", np.clip(results.err_angle, 0, 4 * np.mean(results.err_angle)))
    results.add_field("true_output_change", results.true_out - results.base_out)
    no_change_idxs = (np.linalg.norm(results.true_output_change, axis=0) < 1e-12)
    num_no_change = np.sum(no_change_idxs)
    if num_no_change > 0:
        print(num_no_change, "removed due to small true output change")
    results.remove_entry(no_change_idxs)
    approx_output_change = results.approx_out - results.base_out
    results.add_field("abs_err_mag", reduce_to_1D(results.true_out - results.approx_out))
    results.add_field("frac_err_mag", results.abs_err_mag / reduce_to_1D(results.true_output_change))
    results.add_field("input_alt_angle", angle_between(results.input, results.alt_input))
    results.add_field("err_angle", angle_between(results.true_output_change, approx_output_change))
    clip_errors()


def plot_3D_error(log: LoadedLog, pos_name: str = None, color_name: str = None, figure_num: int = 1) -> None:
    pos_name = pos_name or ""
    color_name = color_name or ""
    position = log.fields[pos_name]
    fig = plt.figure(figure_num)
    ax = fig.add_subplot(projection='3d')
    sc = ax.scatter(position[0, :], position[1, :], position[2, :], c=reduce_to_1D(log.fields[color_name]))
    fig.colorbar(sc)
    plt.title(log.filename + " " + color_name + " over 3D " + pos_name)
    ax.set_xlabel(pos_name + "[0]")
    ax.set_ylabel(pos_name + "[1]")
    ax.set_zlabel(pos_name + "[2]")
    plt.grid(True)


def plot_1D_error(log: LoadedLog, x_name: str, y_name: str, figure_num: int = 2):
    plt.figure(figure_num)
    plt.plot(reduce_to_1D(log.fields[x_name]), reduce_to_1D(log.fields[y_name]), '.', alpha=0.3)
    plt.grid(True)
    plt.xlabel(x_name)
    plt.ylabel(y_name)


def get_failing_results(results: LoadedLog, max_frac_error: float = None, max_angle_error: float = None) -> (LoadedLog, LoadedLog):
    fails = results.__deepcopy__()
    fails.filename += "_fails"
    passes = results.__deepcopy__()
    passes.filename += "_passes"
    max_frac_error = max_frac_error or 0.2
    max_angle_error = max_angle_error or 0.2
    print("Sorting failing results (fractional error >", max_frac_error, " or angle error > ", max_angle_error, ")")
    print("Start with ", results.frac_err_mag.size)
    pass_index = 0
    fail_index = 0
    while True:
        try:
            if fails.frac_err_mag[fail_index] < max_frac_error and fails.err_angle[fail_index] < max_angle_error:
                pass_index += 1
                fails.remove_entry(fail_index)
            else:
                fail_index += 1
                passes.remove_entry(pass_index)
        except IndexError:
            print("Passing results:", pass_index, "fails:", fail_index)
            return passes, fails


def print_mag_stats(log: LoadedLog, name: str, intro: str = None):
    mag = reduce_to_1D(log.fields[name])
    mean = np.mean(mag)
    std_dev = np.std(mag)
    intro = intro or ""
    print(intro, name, "shape", mag.shape, "mean magnitude:", mean, "std_dev:", std_dev)


def print_log_error(log: LoadedLog, name: str = None):
    if name is not None:
        print(name, "Jacobian results, size=", log.input.shape)
    print("\tAverage abs error mag:", np.mean(log.abs_err_mag))
    print("\tAverage fractional error mag:", np.mean(log.frac_err_mag))
    print("\tAverage error angle:", np.mean(log.err_angle))
    print("\tAbs error mag shape:", log.abs_err_mag.shape)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        print("Loading", sys.argv[1], "... ")
        log = LoadedLog(sys.argv[1], ["input", "input_diff", "alt_input", "base_out", "true_out", "approx_out"])
        print("Loaded Jacobian results, size=", log.input.shape)
        calc_error(log)
        print_log_error(log)
        passes, fails = get_failing_results(log)
        print_log_error(passes, "passes")
        print_log_error(fails, "fails")
        print_mag_stats(fails, "input", "fails")
        print_mag_stats(passes, "input", "passes")
        plot_1D_error(log, "true_out", "clipped_frac_err_mag")
        plot_1D_error(log, "input_alt_angle", "clipped_frac_err_mag", 4)
        plot_3D_error(passes, "input", "clipped_frac_err_mag")
        plot_3D_error(passes, "input", "err_angle", 3)
        plt.show()
    else:
        print("Usage:", sys.argv[0], "log_file_name [--max-frac-error <float>] [--max-angle-error <float>]")


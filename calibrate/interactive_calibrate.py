"""
UI for experimenting with calibration
Interactive command line and graphs
TODO:
    graph slowness factor and acceleration / accel error
    fancy modern interactive data frontend?
"""
import matplotlib.pyplot as plt
import numpy as np
from os import listdir
from os.path import isfile, join, dirname, splitext
import sys

from calibrate import loadRun, examineMotor, printMotorResults
from filter import FilterParams, FourierAnalysis, FourierSignal
from motor_model import saveParams, loadParams


def graphMotorResults(results, filter_params):
    # plt.style.use('dark_background')
    graph_V_and_angle_and_derivatives(results)
    # graph_3d_acc_V_vel(results)
    #graph_torque_ripple(results)
    #graph_slowness(results)
    # graph_sign_vel_f(results)
    #graph_acc_predict(results)
    #graph_time_error(results)
    # graph_error_stats(results)
    fourier = FourierAnalysis(results, filter_params)
    graph_high_freq(results)
    graph_signal_filter_frequencies(results, "angle", 'rad', fourier, num=10, x_max_filt_cutoff_mult=1.5, also="spiky_angle")
    graph_signal_filter_frequencies(results, "acc", 'rad/s^2', fourier, num=11, x_max_filt_cutoff_mult=1.5)
    graph_signal_filter_frequencies(results, "V", 'Volts', fourier, num=12, x_max_filt_cutoff_mult=1.5)
    graph_signal_filter_frequencies(results, "accel_error", 'rad/s^2', fourier, num=13, x_max_filt_cutoff_mult=1.5)
    graph_contributions(results)
    plt.show()


# graph voltage, angle, velocity, and acceleration filtered and unfiltered over time
def graph_V_and_angle_and_derivatives(results):
    plt.figure(1, clear=True)
    ax = plt.subplot(411)
    plt.suptitle("Raw and Filtered Data")
    plt.ylabel("Driving Voltage (V)")
    lines = plt.plot(results["t"], results["V"], results["t"], results["V_f"])
    plt.legend(lines, ["Raw", "Filtered"])
    plt.grid()
    plt.subplot(412, sharex=ax)
    plt.ylabel("Angle [rad]")
    if "spiky_angle" in results:
        plt.plot(results["t"], results["spiky_angle"], 'c')
    plt.plot(results["t"], results["angle"], results["t"], results["angle_f"])
    plt.grid()
    plt.subplot(413, sharex=ax)
    plt.ylabel("Velocity [rad/s]")
    plt.plot(results["t"], results["vel"], '.', results["t"], results["vel_f"])
    plt.ylim(np.min(results["vel_f"]) * 1.1, np.max(results["vel_f"]) * 1.1)
    plt.grid()
    ax = plt.subplot(414, sharex=ax)
    plt.ylabel("Acceleration [rad/s^2]")
    plt.xlabel("Time [s]")
    lines = plt.plot(results["t"], results["acc"], '.', results["t"], results["acc_f"],
                     results["t"], results["accel_predic"])
    plt.legend(lines, ["Raw", "Filtered", "Predicted"])
    ax.set_ylim(np.min(results["acc_f"]) * 1.1, np.max(results["acc_f"]) * 1.1)
    plt.grid()

# 3D graph acceleration and acceleration error against voltage and speed
def graph_3d_acc_V_vel(results):
    fig = plt.figure(2, clear=True)
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(results["V_f"], results["vel_f"], zs=results["acc_f"], c=results["t"])
    plt.title("Acceleration")
    ax.set_zlabel("Acceleration [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")

    fig = plt.figure(3, clear=True)
    ax = fig.add_subplot(projection = '3d')
    ax.scatter(results["V_f"], results["vel_f"], zs=results["accel_error_nan"], c=results["t"])  # c=results["accel_error"])
    plt.title("Acceleration Error")
    ax.set_zlabel("Acceleration Error [rad/s^2]")
    plt.xlabel("Voltage [V]")
    plt.ylabel("Velocity [rad/s]")

# investigate torque ripple: plot acceleration error against angle
def graph_torque_ripple(results):
    plt.figure(4, clear=True)
    plt.scatter(results["angle_f"], results["accel_error"], c=results["t"])
    plt.title("Torque Ripple")
    plt.xlabel("Angle [rad]")
    plt.ylabel("Acceleration error [rad/s^2]")
    plt.grid()

def graph_slowness(results):
    plt.figure(5, clear=True)
    plt.plot(results["t"], np.abs(results["vel_f"]), results["t"], results["slowness"])
    plt.legend(["speed", "slowness"])
    plt.title("Speed and Slowness Factor")
    plt.xlabel("time [s]")
    plt.ylabel("Speed [rad/s] and Slowness [arb. units]")
    plt.grid()

def graph_sign_vel_f(results):
    plt.figure(16, clear=True)
    plt.plot(results["t"], results["vel_f"], results["t"], results["sign_vel_f"])
    plt.legend(["vel_f", "sign_vel_f"])
    plt.title("Velocity and sign_vel_f Factor")
    plt.xlabel("time [s]")
    plt.ylabel("Velocity [rad/s] and sign_vel_f [arb. units]")
    plt.grid()

def graph_acc_predict(results):
    plt.figure(6, clear=True)
    lines = plt.plot(results["t"], results["acc_f"], results["t"], results["accel_predic"])
    plt.title("Acceleration Prediction")
    plt.legend(lines, ["Filtered", "Predicted"])
    plt.xlabel("time [s]")
    plt.ylabel("Acceleration [rad/s^2]")
    plt.grid()

def graph_error_stats(results):
    from scipy import stats
    plt.figure(7, clear=True)
    accel_error = results["accel_error"]
    _, x, _ = plt.hist([accel_error, results["accel_error_nan"]], bins="auto", label=["all", "clean"], log=True)
    pdf_scale = len(accel_error) * (np.max(x) - np.min(x)) / (len(x) - 1)  # n_points * x_domain_length / n_bins
    plt.plot(x, pdf_scale * stats.norm.pdf(x, loc=np.mean(accel_error), scale=results["error_std_dev"]), label="norm w/ outliers")
    x = np.array([x_i for x_i in x if abs(x_i) < results["outlier_thresh"]])
    pdf_scale = len(accel_error) * (np.max(x) - np.min(x)) / len(x)
    plt.plot(x, pdf_scale * stats.norm.pdf(x, loc=np.mean(accel_error), scale=results["error_std_dev_clean"]), 'k', label="clean normal")
    plt.ylim(bottom=0.7 if plt.gca().get_yscale() == "log" else 0, top=plt.ylim()[1])
    plt.title("Acceleration Error")
    plt.xlabel("Acceleration Error [rad/s^2]")
    plt.ylabel("count")
    plt.legend()
    plt.grid()

def graph_time_error(results):
    plt.figure(8, clear=True)
    plt.plot(results["diff_t"], '.')
    plt.title("Time Jitter")
    plt.ylabel("Change in time [s]")
    plt.grid()

def graph_signal_filter_frequencies(results, name, unit, fourier: FourierAnalysis, num=None,
                                    x_max_filt_cutoff_mult=None, also=None, moving_avg_pts=10):
    def plot_mag(axs, freqs, signal_mag, fmt, label, zorder=2):
        axs[0].plot(freqs, signal_mag, fmt, zorder=zorder, label=label)
        axs[1].plot(freqs, 20 * np.log10(signal_mag), fmt, zorder=zorder, label=label)

    def plot_filter(ax, f, desc, filter_desc=None):
        twin_ax = ax.twinx()
        twin_ax.plot(fourier.w, f, 'b', label='filter')
        twin_ax.set_ylabel(f"Filter {filter_desc or desc} (blue)")
        ax.set_ylabel(f"Signal {desc}")

    if not fourier.valid:
        return
    freqs = fourier.freqs
    name_f = name + "_f"
    data_fourier = FourierSignal(results, name, fourier, moving_avg_pts)

    fig, axs = plt.subplots(num=num, nrows=3, sharex='all', figsize=(8, 10), clear=True)
    plot_mag(axs, freqs, data_fourier.smooth_mag, 'C1', "moving avg", zorder=4)
    axs[0].set_ylim(axs[0].get_ylim())
    axs[1].set_ylim(axs[1].get_ylim())
    plot_mag(axs, freqs, data_fourier.signal_mag, '.C2', name)

    plot_filter(axs[0], np.abs(fourier.h),                f"Magnitude [{unit}]", "Magnitude")
    plot_filter(axs[1], 20 * np.log10(np.abs(fourier.h)), "Amplitude [dB]")
    plot_filter(axs[2], np.unwrap(np.angle(fourier.h)),   "Angle [rad]")

    axs[2].plot(freqs, np.unwrap(np.angle(data_fourier.fourier_signal)), 'g', label=name)
    axs[2].set_ylim(axs[2].get_ylim())
    if name_f in results:
        filtered_fourier = FourierSignal(results, name_f, fourier)
        plot_mag(axs, freqs, filtered_fourier.signal_mag, '.C3', name_f)
        axs[2].plot(freqs, np.unwrap(np.angle(filtered_fourier.fourier_signal)), 'r', label=name_f)  # goodbye
    if also is not None:
        also_fourier = FourierSignal(results, also, fourier, moving_avg_pts)
        plot_mag(axs, freqs, also_fourier.signal_mag, '.C4', also, zorder=1)
        plot_mag(axs, freqs, also_fourier.smooth_mag, 'C5', also+'_avg', zorder=3)
    for i in range(3):
        axs[i].grid()
        axs[i].axvline(1 / results["filter_period"], color='k', linestyle='--', label="filter cutoff")
        if x_max_filt_cutoff_mult:
            axs[i].set_xlim(left=0, right=x_max_filt_cutoff_mult / results["filter_period"])
    axs[2].set_xlabel("frequency [Hz]")
    axs[0].set_title(f"Fourier Transform ({name})")
    axs[0].legend()

def graph_high_freq(results):
    plt.figure(9, clear=True)
    plt.plot(results["t"], results["spiky_angle_hf"], '.', label="spiky_angle")
    plt.plot(results["t"], results["angle_hf"], '.', label="angle")
    plt.title("High Frequency")
    plt.ylabel("Angle [rad]")
    plt.legend()
    plt.grid()

def graph_contributions(results, combine_V_fric=True):
    plt.figure(15, clear=True)
    if combine_V_fric:
        results["model_contributions"]["V + fric"] = results["model_contributions"]["V"].copy()
        for fric_type in ["static_fric", "const_opposing_fric"]:
            if fric_type in results["model_contributions"]:
                results["model_contributions"]["V + fric"] += results["model_contributions"][fric_type]
    for mod, contribution in results["model_contributions"].items():
        plt.plot(results["t"], contribution, label=mod)
    plt.plot(results["t"], results["accel_error"], 'k', zorder=1, label="accel error")
    plt.plot(results["t"], results["acc_f"], '--', color='gray', zorder=1, label="acc_f")
    plt.title("Model Contributions")
    plt.ylabel("Acc [rad/s^2]")
    plt.legend()
    plt.grid()


def main():
    """run the tests and analysis with a text UI"""
    print("Motor test and calibration. Choose one of:",
          "examine [file_prefix]",
          "saveparams ['filename']",
          "loadparams <'filename'>",
          "model [[no] model_name]",
          "code", sep="\n ")
    testdata = []
    model = ['static_fric', 'const_fric', 's_punch']
    filter_params = FilterParams(fcn="butter")
    params = None
    data_path = join(dirname(dirname(__file__)), "data")
    test_type = ""
    cmd_args = None if (len(sys.argv) < 2) else sys.argv[1:]

    while True:
        if cmd_args:
            args = cmd_args
            print("From cmd args:", args)
            cmd_args = None
        else:
            args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        command = command.strip().lower()
        if command == "":
            continue
        elif command == "examine":
            if len(args) > 1:
                filenames = sorted([f for f in listdir(data_path) if isfile(join(data_path,f)) and f.startswith(args[1])])
                print("Loading", ', '.join([splitext(f)[0] for f in filenames]), "\n")
                testdata = []
                for i, f in enumerate(filenames):
                    newdata = loadRun(join(data_path,f))
                    if newdata is not None:
                        testdata.append(newdata)
                        if len(testdata) == 1:
                            test_type = testdata[0]["test_type"]
                        else:
                            if not testdata[i]["test_type"] == test_type:
                                print("mismatched test_type", test_type, testdata[i]["test_type"], "for", f)
            if len(testdata) > 0:
                if test_type == "motor":
                    params, results = examineMotor(testdata, model, params, filter_params)
                    printMotorResults(params, results)
                    graphMotorResults(results, filter_params)
        elif command == "saveparams":
            filename = args[1] if len(args) > 1 else "new"
            saveParams(params, join(data_path, filename))
        elif command == "loadparams":
            filename = args[1] if len(args) > 1 else "new"
            try:
                ret = loadParams(join(data_path, filename))
            except FileNotFoundError:
                print("File not found")
            else:
                params = ret
                print("Loaded parameters:", params)
        elif command == "model":
            while len(args) > 1:
                if args[1] == "no":
                    model.remove(args[2])
                    args = [args[0]] + args[3:]
                else:
                    model.append(args[1])
                    args = [args[0]] + args[2:]
            print(model)
        elif command.startswith("exit"):
            break
        elif command == "code":
            while not command == "exit":
                command = input(">>> ")
                try:
                    print(eval(command))
                except Exception as e:
                    print(e)


if __name__ == "__main__":
    main()

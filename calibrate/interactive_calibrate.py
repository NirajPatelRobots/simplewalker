"""
UI for experimenting with calibration
Interactive command line and graphs
TODO:
    change filter parameters
"""
from scipy import signal, stats
from scipy.fft import rfft
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join, dirname
import sys

from calibrate import *


def graphMotorResults(results):
    graph_V_and_angle_and_derivatives(results)
    graph_3d_acc_V_vel(results)
    graph_torque_ripple(results)
    graph_slowness(results)
    graph_acc_predict(results)
    graph_time_error(results)
    graph_error_stats(results)
    graph_high_freq(results)
    graph_signal_filter_frequencies(results, "angle", 'rad', 10, x_max_filter_cutoff_multipler=1.5)
    graph_signal_filter_frequencies(results, "acc", 'rad/s^2', 11, x_max_filter_cutoff_multipler=1.5)
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
    plt.plot(results["t"], np.abs(results["vel_f"]), results["t"],
             slowness_factor_continuous(results["vel_f"]))
    plt.title("Speed and Slowness Factor")
    plt.xlabel("time [s]")
    plt.ylabel("Speed [rad/s] and Slowness [arb. units]")
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

def graph_signal_filter_frequencies(results, name, unit, num=None, x_max_filter_cutoff_multipler=None):
    freqs = results["fft_freqs"]
    if results["fft_freqs"] is None:
        print(f'timestep is too variable for fft. Implement slow fourier transforms. {results["dt_std_dev"]=}')
        return
    name_f = name + "_f"
    window = signal.windows.blackman(results["N"])
    data_fourier = rfft(results[name] * window)
    filtered_fourier = rfft(results[name_f] * window)
    data_mag = 2.0/results["N"] * np.abs(data_fourier)
    filtered_mag = 2.0/results["N"] * np.abs(filtered_fourier)
    num_avg = 100
    data_mag_smooth = np.convolve(data_mag, np.ones(num_avg), 'same') / num_avg
    try:
        w, h = signal.freqz_sos(lowpass_sos, fs=1/results["dt"])
    except AttributeError:
        w, h = signal.sosfreqz(lowpass_sos, fs=1/results["dt"])

    fig, axs = plt.subplots(num=num, nrows=3, sharex='all', figsize=(8, 10), clear=True)
    axs[0].plot(freqs, data_mag_smooth, 'C1', zorder=4, label="moving avg")
    axs[0].set_ylim(axs[0].get_ylim())
    axs[0].plot(freqs, data_mag, '.', color='C2', label=name)
    axs[0].plot(freqs, filtered_mag, '.', color='C3', label=name_f)
    twin_ax0 = axs[0].twinx()
    twin_ax0.plot(w, np.abs(h), 'b', label='filter')
    twin_ax0.set_ylabel("Filter Magnitude (blue)")
    axs[0].set_ylabel(f"Signal Magnitude [{unit}]")

    axs[1].plot(freqs, 20 * np.log10(data_mag_smooth), 'C1', zorder=4, label="moving avg")
    axs[1].set_ylim(axs[1].get_ylim())
    axs[1].plot(freqs, 20 * np.log10(data_mag), '.', color='C2', label=name)
    axs[1].plot(freqs, 20 * np.log10(filtered_mag), '.', color='C3', label=name_f)
    twin_ax1 = axs[1].twinx()
    twin_ax1.plot(w, 20 * np.log10(np.abs(h)), 'b', label='filter')
    twin_ax1.set_ylabel("Filter Amplitude (blue) [dB]")
    axs[1].set_ylabel("Signal Amplitude [dB]")

    axs[2].plot(freqs, np.unwrap(np.angle(data_fourier)), 'g', label=name)
    axs[2].set_ylim(axs[2].get_ylim())
    axs[2].plot(freqs, np.unwrap(np.angle(filtered_fourier)), 'r', label=name_f)  # goodbye
    twin_ax2 = axs[2].twinx()
    twin_ax2.plot(w, np.unwrap(np.angle(h)), 'b', label='filter')
    twin_ax2.set_ylabel("Filter Angle (blue) [rad]")
    axs[2].set_ylabel("Signal Angles [rad]")

    axs[2].set_xlabel("frequency [Hz]")
    axs[0].set_title(f"Fourier Transform ({name})")
    axs[0].legend()
    for i in range(3):
        axs[i].grid()
        axs[i].axvline(1 / results["filter_period"], color='k', linestyle='--', label="filter cutoff")
        if x_max_filter_cutoff_multipler:
            axs[i].set_xlim(left=0, right=x_max_filter_cutoff_multipler / results["filter_period"])

def graph_high_freq(results):
    plt.figure(9, clear=True)
    plt.plot(results["t"], results["spiky_angle_hf"], '.', label="spiky_angle")
    plt.plot(results["t"], results["angle_hf"], '.', label="angle")
    plt.title("High Frequency")
    plt.ylabel("Angle [rad]")
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
    model = ['static_fric', 'const_opposing_fric']
    params = {}
    data_path = join(dirname(dirname(__file__)), "data")
    settings_path = join(dirname(dirname(__file__)), "settings")
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
                filenames = [f for f in listdir(data_path) if isfile(join(data_path,f)) and f.startswith(args[1])]
                print("Loading", filenames, "\n")
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
                    params, results = examineMotor(testdata, model)
                    printMotorResults(params, results)
                    graphMotorResults(results)
        elif command == "saveparams":
            filename = args[1] if len(args) > 1 else "new"
            saveParams(params, join(settings_path, filename))
        elif command == "loadparams":
            filename = args[1] if len(args) > 1 else "new"
            ret = loadParams(join(settings_path, filename))
            if not ret is None:
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

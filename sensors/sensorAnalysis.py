""" Estimate the bias of the sensors
September 2022
TODO:
    detect bad samples (got old (very different) data from buffer for data[1:6])
    automatically detect logs with bumps
    temperature
"""

from dataclasses import dataclass
from typing import List
import numpy as np
from display import logReader
from scipy.optimize import minimize
from scipy.fft import rfft, rfftfreq
from scipy.signal.windows import blackman
import os
import matplotlib.pyplot as plt
# plt = None


@dataclass
class AnalyzedIMULog:
    name: str = ""
    duration: float = 0
    mean: np.array = np.zeros(3)
    std_dev: float = 0
    covariance: np.array = np.identity(3)
    num_points: int = 0

    def __repr__(self):
        return "Log " + self.name + ": n=" + str(self.num_points) \
               + " mean = " + str(self.mean) + " mag = " + str(np.linalg.norm(self.mean)) \
               + " cov mag = " + str(np.linalg.norm(self.covariance)) \
               + " std dev = " + str(self.std_dev)


def analyze_stationary_imu_log(time: np.ndarray, data: np.array, name: str = None) -> AnalyzedIMULog:
    if data.size == 0:
        raise IndexError
    result = AnalyzedIMULog(duration=np.max(time))
    if name is not None:
        result.name = name
    result.mean = np.mean(data, axis=1)
    noise = data - result.mean.reshape(3, 1)
    result.std_dev = np.mean(np.std(noise, axis=1))
    result.covariance = np.cov(noise, rowvar=False)
    result.num_points = data.shape[1]
    return result


def calc_residual(bias: np.array, log_means: List[np.array], expected_magnitude: float) -> (np.float64, np.ndarray):
    """ calculate the residual and the 3-vector Jacobian of the residual
    the residual represents the difference between the expected mean and the log means biased by the bias.
    the total residual is the sum of the residuals for each log.
    The residual for one log is ((log_mean - bias)^2 - expected_magnitude^2)^2 """
    res = 0.0
    res_jac = np.zeros(3)
    for log_mean in log_means:
        sqrt_res = np.sum((log_mean - bias)**2) - expected_magnitude**2
        res += sqrt_res**2
        res_jac += 4 * sqrt_res * (bias - log_mean)
    return res, res_jac


class SensorBiasCalculator:
    def __init__(self, sensor_name: str, expected_magnitude: float, tolerance: float = None):
        self.sensor_name = sensor_name
        self.expected_magnitude = expected_magnitude
        self.analyzed: List[AnalyzedIMULog] = []
        self.bias = np.zeros(3)
        self.result = None
        self.solver_tolerance = tolerance

    def calc_sensor_bias(self) -> np.array:
        log_means = [log.mean for log in self.analyzed]
        self.result = minimize(calc_residual, np.zeros(3), args=(log_means, self.expected_magnitude),
                               jac=True, tol=self.solver_tolerance)
        if not self.result.success:
            raise RuntimeError("Solver couldn't solve bias for " + self.sensor_name)
        self.bias = self.result.x
        return self.bias

    def analyze_log(self, log: logReader.LoadedLog):
        self.analyzed.append(analyze_stationary_imu_log(log.timestamp, log.fields[self.sensor_name],
                                                        log.filename + '.' + self.sensor_name))
        graph_log(log.timestamp, log.fields[self.sensor_name], self.analyzed[-1])

    def print_logs(self):
        print(len(self.analyzed), " analyzed logs:")
        for log in self.analyzed:
            print(log)

    def print_results(self):
        std_dev_total = 0
        print("Success:", self.result.success, "i=", self.result.nit, "residual=", self.result.fun, self.result.message)
        print(self.sensor_name, "Bias:", self.bias, "unbiased means:")
        for log in self.analyzed:
            print(log.mean - self.bias, "mag =", np.linalg.norm(log.mean - self.bias))
            std_dev_total += log.std_dev
        print("Std dev:", std_dev_total / len(self.analyzed))

    @staticmethod
    def stationary_log_names() -> List[str]:
        return [f.replace(".log", "") for f in sorted(os.listdir("data/"))
                if f.startswith("stationary_calibration_") and f.endswith(".log")]


def load_sensor_log(log_name: str, sensor_name: str) -> logReader.LoadedLog:
    try:
        log = logReader.LoadedLog("data/" + log_name + ".log", ["timestamp", sensor_name])
    except KeyError:
        try:
            log = logReader.LoadedLog("data/" + log_name + ".log", ["utime", sensor_name])
            log.add_field("timestamp", log.utime * 1e-6)
            log.remove_field("utime")
        except KeyError:
            log = logReader.LoadedLog("data/" + log_name + ".log", [sensor_name])
            log.add_field("timestamp", np.arange(0, stop=log.fields[sensor_name].shape[1]))  # intentionally looks fake
    return log


def graph_log(time: np.ndarray, data: np.array, analyzed: AnalyzedIMULog, ) -> None:
    if not plt:
        return
    fig, axs = plt.subplots(3, 3)
    fig.suptitle(analyzed.name)
    for i in range(3):
        axs[i][0].plot(time, data[i, :])
        axs[i][0].plot([np.min(time), np.max(time)], [analyzed.mean[i], analyzed.mean[i]])
        axs[i][0].grid()
        axs[i][0].set_ylabel('xyz'[i])
    axs[2][0].set_xlabel("time (s)")
    # plot fft
    N = analyzed.num_points
    window = blackman(N)
    freqs = rfftfreq(N, np.mean(np.diff(time)))
    DC_FREQS_REMOVE = 3
    for i in range(3):
        data_fourier = rfft(data[i, :] * window)
        axs[i][1].semilogy(freqs[DC_FREQS_REMOVE:], 2.0/N * np.abs(data_fourier[DC_FREQS_REMOVE:]), 'k.')
        axs[i][1].grid()
        axs[i][2].plot(freqs, np.unwrap(np.angle(data_fourier)), 'k')  # is this just noise?
        axs[i][2].grid()
    axs[2][1].set_xlabel("frequency (Hz)")
    axs[2][2].set_xlabel("frequency (Hz)")


if __name__ == '__main__':
    for sensor_name, expected_mag, tolerance in [("accel", 9.81, 1e-3), ("gyro", 0., 1e-14)]:
        calc = SensorBiasCalculator(sensor_name, expected_mag, tolerance)
        for log_name in SensorBiasCalculator.stationary_log_names():
            calc.analyze_log(load_sensor_log(log_name, sensor_name))
        calc.print_logs()
        calc.calc_sensor_bias()
        calc.print_results()
        print("\n")
    if plt:
        plt.show()

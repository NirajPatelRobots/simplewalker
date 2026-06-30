""" filtering for offline calibration
TODO:
    remove spikes: better removal of spikes that swing positive and negative
    Better quantization removal. Markov graph to do this and spike removal?
    - continuous_threshold get closer to 0: continuous_threshold_to_zero() which goes to 0 at a certain value
    filter_data() compensate for delay? FIR delay = order / 2
    organize chain of filters: spike removal -> moving_avg -> lowpass. DataCleaner?
"""
import numpy as np
from scipy import signal
from scipy.fft import rfft, rfftfreq
from dataclasses import dataclass

DEFAULT_FILTER_CUTOFF_N = 180
DEFAULT_IIR_ORDER = 5
DEFAULT_FIR_ORDER = 250
CUT_FIRST_N_MULT = 2
DEFAULT_ANGLE_MOVING_AVG_SAMPLES = 30


@dataclass
class FilterParams:
    T: float = None
    N: int = None
    order: int = None
    fcn: str = "butter"   # 'cheby'
    ripple: float = 0.5  # only applies to cheby filters
    angle_moving_avg_N: int = None
    fcn_options = ["butter", "fir", "cheby"]
    def __post_init__(self):
        if self.order is None:
            self.order = DEFAULT_FIR_ORDER if self.fcn == "fir" else DEFAULT_IIR_ORDER
        self.set_N(self.N if self.N is not None else DEFAULT_FILTER_CUTOFF_N)
        self.angle_moving_avg_N = self.angle_moving_avg_N or DEFAULT_ANGLE_MOVING_AVG_SAMPLES
        assert(self.angle_moving_avg_N < self.N)

    def set_N(self, N: int | None, t_data: np.ndarray = None):
        self.N = N if (self.T is None or t_data is None) else self.N_from_T(t_data)
        if self.fcn == 'cheby':
            self.loPass_polynom = signal.cheby1(self.order, self.ripple, 2 / self.N, 'lowpass')
            self.hiPass_polynom = signal.cheby1(self.order, self.ripple, 2 / self.N, 'highpass')
        elif self.fcn == "butter":
            self.loPass_polynom = signal.butter(self.order, 2 / self.N, 'lowpass')
            self.hiPass_polynom = signal.butter(self.order, 2 / self.N, 'highpass')
        elif self.fcn == "fir":
            self.loPass_polynom = signal.firwin(self.order + 1, 2 / self.N), 1.0
            self.hiPass_polynom = signal.firwin(self.order + 1, 2 / self.N, pass_zero='highpass'), 1.0
        else:
            raise ValueError("Function must be one of: " + str(self.fcn_options))
    def N_from_T(self, t_data):
        return np.argmin(np.abs(t_data - self.T))

    def __str__(self):
        return f"N={self.N} order={self.order} {self.fcn}"


class FourierAnalysis:
    def __init__(self, results: dict, filter_params: FilterParams):
        self.valid = (results["dt_std_dev"] / results["dt"] < 0.01)  # dt is stable to within 1% precision
        if not self.valid:
            print(f'timestep is too variable for fft. Implement slow fourier transforms. {results["dt_std_dev"]=}')
            return
        self.freqs = rfftfreq(results["N"], results["dt"])
        self.window = signal.windows.blackman(results["N"])
        self.w, self.h = signal.freqz(filter_params.loPass_polynom[0], filter_params.loPass_polynom[1],
                                      fs=1/results["dt"])


class FourierSignal:
    def __init__(self, results, name, fourier: FourierAnalysis, moving_avg_points=None):
        self.fourier_signal = rfft(results[name] * fourier.window)
        self.signal_mag = 2.0 / results["N"] * np.abs(self.fourier_signal)
        if moving_avg_points is None:
            self.smooth_mag = None
        else:
            self.smooth_mag = moving_avg(self.signal_mag, moving_avg_points, np.nan)


def filter_data(data, filter_params=FilterParams(), type='lowpass', cut=True):
    n_suspicious_filtered = filter_params.N * 2
    padded_data = np.concatenate((np.average(data[:n_suspicious_filtered]) * np.ones(n_suspicious_filtered), data))
    filt_polynom = filter_params.loPass_polynom if type == 'lowpass' or type == 'low' else filter_params.hiPass_polynom
    filtered = signal.lfilter(filt_polynom[0], filt_polynom[1], padded_data)[n_suspicious_filtered:]
    if cut:
        filtered = cut_first(filtered, filter_params)
    return filtered


def cut_first(data, filter_params):
    return data[filter_params.N * CUT_FIRST_N_MULT:]

# Moving average of the last N points. Starts with pad_val for the first N-1 points.
def moving_avg(data, n_samples=1, pad_val=0):
    return np.concatenate((np.ones(n_samples-1) * pad_val, np.convolve(data, np.ones(n_samples), 'valid') / n_samples))


def derivative(data, name):  # this isn't really a filter but model functions need it
    return np.concatenate(([0], np.diff(data[name]) / np.diff(data["t"])))

# naive numerical integration. Return (new_pos, new_vel)
def integration(pos_in: float, v_in: float, a_in: float, dt: float) -> (float, float):
    new_vel = v_in + a_in * dt
    return pos_in + (new_vel * dt) + (0.5 * a_in * dt**2), new_vel


# Returns smooth continuous threshold with a flat response in both limits (0 and 1)
# As values -> +inf, return -> 1. As values -> -inf, return -> 0.
# Mirror around y axis if thresh < 0. Bad near thresh = 0.
def continuous_threshold(values, thresh, steepness):
    return np.arctan((values - thresh) * steepness / thresh) / np.pi + 0.5

# Smooth continuous replacement for np.sign. Flat response in both limits and around values = 0.
# As values -> +inf, return -> 1. As values -> -inf, return -> -1. As abs(values) -> 0, return -> 0.
def continuous_sign(values, thresh: float, steepness: float):
    return np.where(values > 0, continuous_threshold(values, abs(thresh), steepness),
                               -continuous_threshold(values, -abs(thresh), steepness))


""" Remove spikes smaller than a certain width. Interpolates replacement data.
    "angle" can be any signal.
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

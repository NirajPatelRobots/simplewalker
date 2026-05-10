""" filtering for offline calibration
TODO:
    remove spikes: better removal of spikes that swing positive and negative
    more digital filter tuning (IIR vs FIR?)
"""
import numpy as np
from scipy import signal
from dataclasses import dataclass


@dataclass
class FilterParams:
    T: int = None
    N: int = 60
    order: int = 6
    fcn: str = "butter"   # 'cheby'
    ripple: float = 0.5  # only applies to cheby filters
    def __post_init__(self):
        self.set_N(self.N if self.N is not None else 60)
    def set_N(self, N: int | None, t_data: np.ndarray = None):
        if N is None:
            N = np.argmax(t_data > self.T) - 1
        self.N = N
        if self.fcn == 'cheby':
            self.lowpass_sos = signal.cheby1(self.order, self.ripple, 1/N, 'lowpass',  output='sos')
            self.hipass_sos  = signal.cheby1(self.order, self.ripple, 1/N, 'highpass', output='sos')
        else:
            self.lowpass_sos = signal.butter(self.order, 1/N, 'lowpass', output='sos')
            self.hipass_sos  = signal.butter(self.order, 1/N, 'highpass', output='sos')


def filter_data(data, filter_params=FilterParams(), type='lowpass', cut=True):
    n_suspicious_filtered = filter_params.N * 2
    padded_data = np.concatenate((np.average(data[:n_suspicious_filtered]) * np.ones(n_suspicious_filtered), data))
    if type == 'lowpass' or type == 'low':
        filtered = signal.sosfilt(filter_params.lowpass_sos, padded_data)
    else:
        filtered = signal.sosfilt(filter_params.hipass_sos, padded_data)
    filtered = filtered[n_suspicious_filtered:]
    if cut:
        filtered = cut_first(filtered, filter_params)
    return filtered


def cut_first(data, filter_params):
    return data[filter_params.N * 4:]


def moving_avg(data, n_samples=1, pad_val=0):
    return np.concatenate((np.ones(n_samples-1) * pad_val, np.convolve(data, np.ones(n_samples), 'valid') / n_samples))


def derivative(data, name):  # this isn't really a filter but motor_model needs it
    return np.concatenate(([0], np.diff(data[name]) / np.diff(data["t"])))


# Returns smooth continuous threshold with a flat response in both limits (0 and 1)
# As values -> +inf, return -> 0. As values -> -inf, return -> 1
def continuous_threshold(values, thresh, steepness):
    return np.arctan((thresh - values) * steepness / thresh) / np.pi + 0.5


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

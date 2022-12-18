""" unittests for sensor static analysis
September 2022
TODO:
    time solver
    test load_sensor_log
"""

import numpy as np
import unittest
from sensorAnalysis import analyze_stationary_imu_log, calc_residual, AnalyzedIMULog, SensorBiasCalculator
import logReader


class TestSensorAnalysis(unittest.TestCase):
    @staticmethod
    def uncorrelated_disturb_data(data: np.ndarray, disturbance_mag: float = 1e-9) -> None:
        """for 3xn data, perturb by disturbance_mag alternating + and -, alternating 3D index """
        length = data.shape[1]
        for i in range(length):  # TODO: can probably make this a np operation
            data[i % 3, i] += (-1) ** (i % 2) * disturbance_mag


    def test_analyze_stationary_imu_log_no_data(self):
        with self.assertRaises(IndexError):
            analyze_stationary_imu_log(np.array([]), np.array([]))

    def test_analyze_stationary_imu_log_almost_const_data(self):
        num_data = 120
        time = np.linspace(0, num_data / 10, num=num_data)
        expected_mean = np.arange(3).reshape(3, 1)
        data = np.ones((3, num_data)) * expected_mean
        self.uncorrelated_disturb_data(data, 1e-9)

        result = analyze_stationary_imu_log(time, data, "const_data")
        # print("mean", result.mean)
        # print("covariance", result.covariance)
        # print("abs noise covariance", result.abs_covariance)
        for i in range(3):
            self.assertAlmostEqual(float(expected_mean[i]), float(result.mean[i]), msg="const data mean "+str(i))
        self.assertAlmostEqual(0.0, np.linalg.norm(result.abs_covariance), msg="const data small covariance")

    def test_calc_residual_all_zero(self):
        num_logs = 5
        log_means = [np.zeros(3) for _ in range(num_logs)]
        bias = np.zeros(3)
        res, jac = calc_residual(bias, log_means, 0.0)
        self.assertAlmostEqual(0.0, res)
        self.assertEqual((3,), jac.shape)
        for jac_elem in jac.flat:
            self.assertEqual(0.0, jac_elem)

    def test_calc_residual_zero_data_small_bias(self):
        num_logs = 5
        log_means = [np.zeros(3) for _ in range(num_logs)]
        for i in range(2):
            with self.subTest(i=i):
                bias_sign = (-1)**i
                bias = np.ones(3) * bias_sign

                res, jac = calc_residual(bias, log_means, 0.0)
                self.assertTrue(res > 0.0)
                self.assertEqual((3,), jac.shape)
                self.assertTrue(np.all(jac * bias_sign > 0))

    def test_calc_sensor_bias_identical_logs_zero_mag(self):
        test_biases = [np.array([1., -2., 3.]), np.array([-3., 1., -4.])]
        num_logs = 6
        for bias_num, test_bias in enumerate(test_biases):
            with self.subTest(i=bias_num, msg="test_bias=" + str(test_bias)):
                calc = SensorBiasCalculator("test_zero", 0.0)
                calc.analyzed = [AnalyzedIMULog(mean=test_bias) for _ in range(num_logs)]
                bias = calc.calc_sensor_bias()
                for i in range(3):
                    self.assertAlmostEqual(test_bias[i], bias[i], delta=0.01)

    def test_calc_sensor_bias_variable_logs(self):
        test_biases = [np.array([1., -2., 3.]), np.array([-3., 1., -4.])]
        num_logs = 6  # 6 directions of the 3D mean, perturbed in 6 directions by uncorrelated_disturb_data
        log_difference_mags = [0.0, 1.0]
        for log_difference_mag in log_difference_mags:
            for test_bias in test_biases:
                test_means = np.ones((3, num_logs)) * test_bias.reshape(3, 1)
                self.uncorrelated_disturb_data(test_means, log_difference_mag)
                with self.subTest(msg="test_bias=" + str(test_bias) + "test_bias=" + str(test_bias)):
                    calc = SensorBiasCalculator("test_zero", 0.0)
                    calc.analyzed = [AnalyzedIMULog(mean=test_means[:, log_num]) for log_num in range(num_logs)]
                    bias = calc.calc_sensor_bias()
                    for i in range(3):
                        self.assertAlmostEqual(test_bias[i], bias[i], delta=0.01)

    def test_analyze_log_basics_const_data(self):
        duration = 5.0
        num_points = 250
        stationary_log = logReader.LoadedLog("test_log", load_now=False)
        stationary_log.add_field("timestamp", np.linspace(0.0, duration, num=num_points))
        stationary_log.add_field("test_sensor", np.ones((3, num_points)))
        calc = SensorBiasCalculator("test_sensor", 0.0)
        calc.analyze_log(stationary_log)
        self.assertEqual("test_log.test_sensor", calc.analyzed[0].name)
        self.assertEqual(duration, calc.analyzed[0].duration)
        for i in range(3):
            self.assertEqual(1.0, calc.analyzed[0].mean[i])



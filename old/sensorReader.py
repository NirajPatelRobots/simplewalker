# -*- coding: utf-8 -*-
"""
For reading sensors for simplewalker
TODO:
    way of storing angle manual calibration values: zero, 90 deg, max, min.
        -use with motorControl.py

Dependencies:
    https://github.com/m-rtijn/mpu6050
    https://github.com/luxedo/RPi_mcp3008

Created Jun 2021
@author: Niraj
"""

import numpy as np
try:
    import mpu6050
    import mcp3008
except:
    print("sensorReader loaded without dependencies, no functionality")


class SensorReader:
    """ Class for reading sensors"""
    def __init__(self, angleOffset = np.array([0, 0, 0, 0]), rightAngleValue = np.array([512, 512, 512, 512])):
        """angleOffset is sensor value when angle is zero
        rightAngleValue is sensor value when angle is pi/2"""
        #self.imu = mpu6050.mpu6050(0x68) # initialize at default address
        self.adc = mcp3008.MCP3008(device = 0) # potentiometers and voltage
        self.angleOffset = angleOffset
        self.angleGain = np.pi / 2 / (rightAngleValue - self.angleOffset)
    
    def readIMU(self):
        """ returns acceleration [m/s^2], angular_velocity [rad/s]
        both are length 3 arrays for 3D acceleration and angular velocity. """
        accel_data = self.imu.get_accel_data()
        gyro_data = self.imu.get_gyro_data()
        accel = np.array([accel_data['x'], accel_data['y'], accel_data['z']])
        angVel = np.array([gyro_data['x'], gyro_data['y'], gyro_data['z']])
        return accel, angVel
    
    def readBatteryVoltage(self):
        """return battery voltage (float)."""
        """ R_high = 329 kOhm R_low = 993 kOhm
        O-------/\/\/\/---o----/\/\/\/----O
        V_Batt  R_high  sensor  R_low   GND
        V_sens = V_Batt*R_low/(R_high+R_low)
        scale = V_ref/V_sens/max_sense = 3.3/0.249/1023"""
        return 0.0130 * self.adc.read([mcp3008.CH0])[0]
    
    def readAngles(self):
        """reads sensors and returns array of 4 angles [rad].
        order is (0: right motor 1) (1: right motor 2) (2: left motor 1) (3: left motor 2)"""
        readings = self.adc.read([mcp3008.CH1, mcp3008.CH2, mcp3008.CH3, mcp3008.CH4])
        angles = (np.array(readings) - self.angleOffset) * self.angleGain
        return angles

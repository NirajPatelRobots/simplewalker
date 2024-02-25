# -*- coding: utf-8 -*-
"""
Work with the motor controller to take data to calibrate the motor.
The motor controller has to run calibrate_motor.
TODO:
    remove local version?
    receive data from controller

Created Jun 2021, reworked late 2023
@author: Niraj
"""

import numpy as np
import time
from os.path import sep
import types
import re

RUN_LOCAL = False # if RUN_LOCAL, then the computer running this code is controlling the motors. IF RUN_LOCAL is False, then this code sends and receives information from a microcontroller

if RUN_LOCAL:
    import motorControl
    import sensorReader
else:
    import serial
    import struct

def checkMotorPerformance(motorNum, dt, filename = None, frequency_scale = 1, amplitude_scale = 1):
    """test the motor by setting voltages and sensing position.
    motorNum 0 to 3 is which motor to read from and send to
    dt is the time [seconds]
    filename is file name to save the data to.
    frequency_scale and amplitude_scale scale the input voltage waveform"""
    centerVal = np.pi/2 # rad
    kp = 0.002
    maxDisplacement = 0.45 * np.pi # rad
    
    motors = motorControl.MotorController()
    sensors = sensorReader.SensorReader()
    V = excitationVoltage(frequency_scale*dt, amplitude_scale)
    N = np.size(V)
    angle = np.empty(N-1)
    
    # return to start
    numInRange = 0
    while numInRange < 10:
        angle[0] = sensors.readAngles()[motorNum]
        V_set = kp * (centerVal - angle[0])
        motors.setMotor(V_set, motorNum)
        if np.abs(centerVal - angle[0]) < 0.01:
            numInRange += 1
        else:
            numInRange = 0
        time.sleep(dt)
    
    print("At start position,", N, "samples,", N*dt, "s")
    startTime = time.perf_counter()
    for i in range(1, N):
        V_bat = sensors.readBatteryVoltage()
        angle[i-1] = sensors.readAngles()[motorNum] # shift i-1 because causality. So V[i] affects angle[i]
        if V[i] > V_bat: # account for voltage clipping
            V[i] = V_bat
        elif V[i] < -V_bat:
            V[i] = -V_bat
        motors.setMotor(V[i] / V_bat, motorNum)
        if np.abs(angle[i-1] - centerVal) > maxDisplacement:
            angle = angle[:i]
            V = V[:i+1]
            print("Angle out of range, terminating test")
            break
        time.sleep(dt)
    V = V[:-1] # unused because of shift
    runTime = time.perf_counter() - startTime
    print("Test run time:", round(runTime, 2), "s, loop takes ", round(runTime/N, 2), "s, code takes", round(1000*(runTime/N - dt), 3), "ms)")
    
    if not filename is None:
        saveRun(filename, V, angle, motorNum, dt)
    return V, angle


def saveRun(filename, V_, angle_, motorNum, dt, test_type="motor"):
    with open("data" + sep + "m" +str(motorNum) + '_' + filename +".motortest", 'wb') as file:
        np.savez(file, V = V_, angle = angle_, dt=dt, test_type=test_type)
    print("saved V", V_.shape, "angle", angle_.shape)

def loadRun(filename):
    try:
        with open("data" + sep + filename +".motortest", 'rb') as file:
            data = np.load(file, allow_pickle=True)
            V = data['V']
            angle = data['angle']
            return V, angle
    except:
        print("Load failed")
        return np.array([]), np.array([])

def excitationVoltage(frequency_scale, amplitude_scale):
    """return an array of voltages.
    The voltages are sent to the motor and used to determine parameters."""
    def _voltageLoop(maxChange, maxVal):
        """returns an array of voltages starting and ending at zero"""
        freq = maxChange / abs(maxVal)
        t = np.arange(0., np.pi/freq, 0.01)
        return maxVal * np.sin(freq*t)
    
    num_loops = 20
    num_square = 5
    freq = 1000. * frequency_scale
    amp = 1. * amplitude_scale
    
    length = int(200./freq)
    V = np.zeros((2*length))
    for i in range(num_loops):
        loop_amp = (i+1)/(num_loops+1) * amp * (-1. if i%2 == 1 else 1.)
        V = np.concatenate((V, _voltageLoop(freq, loop_amp)))
    V = np.concatenate((V, -V[::-1]))
    for i in range(num_square):
        V = np.concatenate((V, amp*np.ones(length), -amp*np.ones(2*length), amp*np.ones(length)))
    V = np.concatenate((V, np.zeros((length))))
    return V


def make_MotorCalibrationTriggerMessage(motorNum, amp_scale, freq_scale, dt, send_skip_iterations,
                                        max_displacement, min_displacement, text_output) -> bytes:
    return struct.pack("<hhfffffhh", 0x0D11, motorNum, amp_scale, freq_scale, dt, max_displacement, min_displacement,
                       send_skip_iterations, text_output)


def parse_MotorCalibrationStateMessage(message:bytes):
    fields = struct.unpack("<hhfff", message)  # fields[0] is ID, not useful
    return types.SimpleNamespace(timestamp_us=fields[1], angle=fields[2], angvel=fields[3], voltage=fields[4])


def print_from_micro(ser: serial.Serial, max_lines: int):
    text = ""
    regex = re.compile(r"(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)$", flags=re.MULTILINE)
    voltage, angle = ([], [])
    last_time = 0
    is_finished = False
    for lines_read in range(max_lines):
        text += ser.read(2048).decode().replace("\r", "")
        while len(text) and "\n" in text:
            regex_match = regex.match(text)
            if regex_match:
                text = text[regex_match.end()+1:]
                line_data = regex_match.group().split(',')
                print("got [time, voltage, angle, angvel]:", line_data)
                voltage.append(float(regex_match.group(2)))
                angle.append(float(regex_match.group(3)))
                # this_time = float(regex_match.group(1))
                # if this_time - last_time > 2:
                #     voltage, angle = ([], [])
                # last_time = this_time
            else:
                newlineplace = text.find('\n')
                print("|Other text|>", text[:newlineplace], "<|Other text|")
                is_finished = "finished calibration" in str(text[:newlineplace]) or "was terminated" in str(text[:newlineplace])
                text = text[newlineplace+1:]
        if is_finished:
            break
    print(text)
    return np.array(voltage), np.array(angle)


def main():
    """run the tests with a text UI"""
    if not RUN_LOCAL:
        ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        if ser.is_open:
            print("Connected to", ser.name)
        else:
            print("Could not connect to", ser.name)
            return
    print("Motor test. Choose one of:",
          "run ['frequency_scale', ['amplitude_scale', ['filename']]]",
          "motornum number (0: R motor 1, 1: R motor 2, 2: L motor 1, 3: L motor 2)",
          "max_angle (float) or min_angle (float)",
          "save [filename]",
          "code", sep = "\n ")
    freq_scale = 1.
    amp_scale = 1.
    motorNum = 0
    max_angle = 1
    min_angle = -1
    V = np.array([])
    angle = np.array([])
    dt = 0.01  # seconds
    filename = None
    
    while True:
        args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        command = command.strip().lower()
        if command == "":
            if not RUN_LOCAL:
                ser.reset_input_buffer()
                print_from_micro(ser, 6)
        elif command == "run":
            if len(args) > 1:
                freq_scale = float(args[1])
                if len(args) > 2:
                    amp_scale = float(args[2])
                    if len(args) > 3:
                        filename = args[3]
                    else:
                        filename = None
            if RUN_LOCAL:
                V, angle = checkMotorPerformance(motorNum, dt, filename, freq_scale, amp_scale)
            else:
                message = make_MotorCalibrationTriggerMessage(motorNum, amp_scale, freq_scale, dt, 0,
                                                              max_angle, min_angle, True)
                ser.reset_input_buffer()
                ser.write(message)
                print("sent:", message)
                V, angle = print_from_micro(ser, 60)
                print("got V", V.shape, "angle", angle.shape)
        elif command == "motornum":
            try:
                motorNum = int(args[1])
            except:
                print("Invalid motor number")
        elif command == "save" and len(args) > 1:
            saveRun(args[1], V, angle, motorNum, dt)
        elif command == "dt":
            if len(args) > 1:
                dt = float(args[1])
            print("dt:", dt)
        elif command == "max_angle":
            if len(args) > 1:
                max_angle = float(args[1])
            print("max angle:", max_angle)
        elif command == "min_angle":
            if len(args) > 1:
                min_angle = float(args[1])
            print("min angle:", min_angle)
        elif command.startswith("exit"):
            ser.close()
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

# -*- coding: utf-8 -*-
"""
Work with the motor controller to take data to calibrate the motor.
The motor controller has to run calibrate_motor.
Can run interactively or read from a json file.
TODO:
    receive binary data from controller
    remove angvel from text comm (removed from MotorCalibrationStateMsg)
    measure battery voltage during calibration
    different excitation voltage patterns
    compare reported controller time with system time?

Created Jun 2021, reworked late 2023
@author: Niraj
"""

import numpy as np
from os.path import sep
import time
import sys
import json
import types
import re
import serial
import struct


def saveRun(filename, V_, angle_, motorNum, t_, test_type="motor"):
    with open("data" + sep + "m" +str(motorNum) + '_' + filename +".motortest", 'wb') as file:
        np.savez(file, V=V_, angle=angle_, t=t_, test_type=test_type)
        print("saved V, angle, t", t_.shape, "as", file.name)

class ControllerSerial(serial.Serial):
    def __init__(self, connect=True):
        super(ControllerSerial, self).__init__()
        if connect:
            self.connect()

    def connect(self) -> None:
        super(ControllerSerial, self).__init__(port='/dev/ttyACM0', baudrate=115200, parity=serial.PARITY_NONE,
                                               stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        if self.is_open:
            print("Connected to", self.name)
        else:
            print("Could not connect to", self.name)

    def readstr(self) -> str:
        return self.read(2048).decode().replace("\r", "")


def make_MotorCalibrationTriggerMessage(motorNum, amp_scale, freq_scale, dt, send_skip_iterations,
                                        max_displacement, min_displacement, text_output) -> bytes:
    return struct.pack("<hhfffffhh", 0x0D11, motorNum, amp_scale, freq_scale, dt, max_displacement, min_displacement,
                       send_skip_iterations, text_output)


def parse_MotorCalibrationStateMessage(message:bytes):
    fields = struct.unpack("<hhfff", message)  # fields[0] is ID, not useful
    return types.SimpleNamespace(timestamp_us=fields[1], angle=fields[2], angvel=fields[3], voltage=fields[4])


def print_from_micro(ser: ControllerSerial):
    text = ""
    regex = re.compile(r"(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)$", flags=re.MULTILINE)
    voltage, angle, timestamp = ([], [], [])
    is_finished = False
    lines_read = 0
    while not is_finished:
        text += ser.readstr()
        lines_read += 1
        while len(text) and "\n" in text:
            regex_match = regex.match(text)
            if regex_match:
                text = text[regex_match.end()+1:]
                this_time = float(regex_match.group(1))
                print("t = ", this_time, "   ", end='\r')
                is_finished = (this_time > 1000)
                voltage.append(float(regex_match.group(2)))
                angle.append(float(regex_match.group(3)))
                timestamp.append(this_time)
            else:
                newlineplace = text.find('\n')
                print("|Other text|>", text[:newlineplace], "<|Other text|")
                is_finished = "finished calibration" in str(text[:newlineplace]) or "was terminated" in str(text[:newlineplace])
                text = text[newlineplace+1:]
    print(text)
    return np.array(voltage), np.array(angle), np.array(timestamp)

def motortest_filename_tag(amp_scale, freq_scale) -> str:
    return f"_a{amp_scale:g}_f{freq_scale:g}".replace(".", "p")


def run_tests_from_file_input(infile_name: str, series_name: str):
    ser = ControllerSerial()
    with open(infile_name) as infile:
        series_config = json.loads(infile.read())
    ser.reset_input_buffer()
    for run in series_config["runs"]:
        print(run["type"], "run")
        for amp_scale in run["amp_scales"]:
            for freq_scale in run["freq_scales"]:
                filename = series_name + motortest_filename_tag(amp_scale, freq_scale)
                print("\tRunning:", filename)
                if "dry_run" in series_config and series_config["dry_run"] is True:
                    continue
                time.sleep(0.5)
                ser.reset_input_buffer()
                ser.write(make_MotorCalibrationTriggerMessage(series_config["motorNum"], amp_scale, freq_scale,
                                      series_config["dt"], 0, series_config["max_angle"], series_config["min_angle"], True))
                V, angle, timestamp = print_from_micro(ser)
                print("got V", V.shape, "angle", angle.shape, "time", timestamp.shape)
                saveRun(filename, V, angle, series_config["motorNum"], timestamp)
                time.sleep(0.5)
                ser.reset_input_buffer()


def interactive_main():
    """run the tests with a text UI"""
    ser = ControllerSerial()
    print("Motor test. Choose one of:",
          "run ['frequency_scale', ['amplitude_scale', ['filename']]]",
          "motornum number (0: R motor 1, 1: R motor 2, 2: L motor 1, 3: L motor 2)",
          "max_angle (float) or min_angle (float)",
          "dt (float)",
          "ls [file_prefix]",
          "save [filename]",
          "code", sep = "\n ")
    freq_scale = 1.
    amp_scale = 1.
    motorNum = 0
    max_angle = 1
    min_angle = -1
    V = np.array([])
    angle = np.array([])
    dt = 0.002  # seconds
    filename = None
    timestamp = np.array([])
    
    while True:
        args = input(">>> ").split()
        command = args[0] if len(args) > 0 else "" #command is first, the rest of the args are available if needed
        command = command.strip().lower()
        if command == "":
            ser.reset_input_buffer()
            for _ in range(6):
                print_from_micro(ser)
        elif command == "run":
            if len(args) > 1:
                freq_scale = float(args[1])
                if len(args) > 2:
                    amp_scale = float(args[2])
                    if len(args) > 3:
                        filename = args[3]
            message = make_MotorCalibrationTriggerMessage(motorNum, amp_scale, freq_scale, dt, 0,
                                                          max_angle, min_angle, True)
            ser.reset_input_buffer()
            ser.write(message)
            print("sent:", message)
            V, angle, timestamp = print_from_micro(ser)
            print("got V", V.shape, "angle", angle.shape, "time", timestamp.shape)
            if filename:
                saveRun(filename + motortest_filename_tag(amp_scale, freq_scale), V, angle, motorNum, timestamp)
        elif command == "motornum":
            try:
                motorNum = int(args[1])
            except:
                print("Invalid motor number")
        elif command == "save":
            filename = (args[1] if len(args) > 1 else "")
            saveRun(filename + motortest_filename_tag(amp_scale, freq_scale), V, angle, motorNum, timestamp)
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
        elif command == "ls":
            import glob
            ls_path = "data" + sep + (args[1] if len(args) > 1 else "") + "*.motortest"
            print([d[5:-10] for d in glob.glob(ls_path)])
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
    if len(sys.argv) == 3:
        run_tests_from_file_input(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 1:
        interactive_main()
    else:
        raise ValueError("Run without args for interactive mode or with 2 args: input_filename, output_filename_tag")

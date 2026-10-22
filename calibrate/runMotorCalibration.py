"""
Work with the motor controller to take data to calibrate the motor.
The motor controller has to run calibrate_motor.
Can run interactively or read from a json file.
TODO:
    receive binary data from controller
    remove angvel from text comm (removed from MotorCalibrationStateMsg)
    send reference path
    measure battery voltage during calibration
    improve stdout display
    compare reported controller time with system time?

Created Jun 2021, reworked late 2023
@author: Niraj
"""

import json
from os.path import sep
import re
import struct
import sys
import time
import types

import numpy as np
import serial

def saveRun(filename, data):
    data["test_type"] = "motor"
    with open("data" + sep + "m" +str(data["motorNum"]) + '_' + filename +".motortest", 'wb') as file:
        print("Saving V, angle, t", data["t"].shape, "as", file.name)
        np.savez(file, **data)

class ControllerSerial(serial.Serial):
    def __init__(self, connect=True):
        super().__init__()
        if connect:
            self.connect()

    def __del__(self):
        super().close()
    
    def connect(self) -> None:
        print("Connecting...", end="\r")
        super().__init__(port='/dev/ttyACM0', baudrate=115200, parity=serial.PARITY_NONE,
                                               stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
        if self.is_open:
            print("Connected to", self.name)
        else:
            print("Could not connect to", self.name)

    def readstr(self) -> str:
        return self.read(2048).decode().replace("\r", "")


def make_MotorCalibrationTriggerMessage(motorNum, amp_scale, freq_scale, dt, input_type, command_signal_type,
                                        max_displacement, min_displacement, text_output) -> bytes:
    return struct.pack("<hhfffffcch", 0x0D11, motorNum, amp_scale, freq_scale, dt, max_displacement, min_displacement,
                       bytes([input_type]), bytes([command_signal_type]), text_output)


def parse_MotorCalibrationStateMessage(message:bytes):
    fields = struct.unpack("<hhfff", message)  # fields[0] is ID, not useful
    return types.SimpleNamespace(timestamp_us=fields[1], angle=fields[2], angvel=fields[3], voltage=fields[4])


input_type_names = ["sin", "squ", "db", "const"] # must match messages.h
signal_type_names = ["voltage", "accel", "angle"]

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
                print("Angle =", regex_match.group(3), "  V =", regex_match.group(2), "    ", end='\r')
                voltage.append(float(regex_match.group(2)))
                angle.append(float(regex_match.group(3)))
                timestamp.append(float(regex_match.group(1)))
                is_finished = (timestamp[-1] > 1000)
            else:
                newlineplace = text.find('\n')
                print("|Other text|>", text[:newlineplace], "<|Other text|")
                is_finished = "finished calibration" in str(text[:newlineplace]) or "was terminated" in str(text[:newlineplace])
                text = text[newlineplace+1:]
    print(text)
    return np.array(voltage), np.array(angle), np.array(timestamp)


def motortest_filename_tag(amp_scale, freq_scale, sig_type, input_type) -> str:
    return f"_{sig_type}_{input_type}_a{amp_scale:g}_f{freq_scale:g}".replace(".", "p")


def run_tests_from_file_input(infile_name: str, series_name: str):
    with open(infile_name) as infile:
        series_config = json.loads(infile.read())
    ser = ControllerSerial()
    ser.reset_input_buffer()
    data = {k: series_config[k] for k in ("signal_type", "motorNum", "dt")}
    for run in series_config["runs"]:
        print(run["name"]) if "name" in run else None
        for run_type in run["types"]:
            data["run_type"] = run_type
            run_type_num = input_type_names.index(run_type)
            signal_type_num = signal_type_names.index(series_config["signal_type"])
            print(f'{run_type} (type {run_type_num}) run')
            for amp_scale in run["amp_scales"]:
                data["amp_scale"] = amp_scale
                for freq_scale in run["freq_scales"]:
                    filename = series_name + motortest_filename_tag(
                                    amp_scale, freq_scale, series_config["signal_type"], run_type)
                    data["freq_scale"] = freq_scale
                    if (series_config.get("dry_run")) or (run.get("skip")):
                        print("Skipping", filename)
                        continue
                    print("\tRunning:", filename)
                    time.sleep(0.5)
                    ser.reset_input_buffer()
                    ser.write(make_MotorCalibrationTriggerMessage(data["motorNum"], amp_scale, freq_scale,
                                                        data["dt"], run_type_num, signal_type_num,
                                                        series_config["max_angle"], series_config["min_angle"], True))
                    data["V"], data["angle"], data["t"] = print_from_micro(ser)
                    print("got", ", ".join([k + ": " + str(data[k].shape) for k in ("V", "angle", "t")]))
                    saveRun(filename, data)
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
          "type [int(MotorCalibrationInputType)]",
          "code", sep = "\n ")
    max_angle = 1
    min_angle = -1
    filename = None
    data = {"V": np.array([]), "angle": np.array([]), "timestamp": np.array([]), "dt": 0.002, "motorNum": 0,
            "signal_type": "voltage", "input_type": "sin", "freq_scale": 1., "amp_scale": 1.}
    
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
                data["freq_scale"] = float(args[1])
                if len(args) > 2:
                    data["amp_scale"] = float(args[2])
                    if len(args) > 3:
                        filename = (args[3] + motortest_filename_tag(
                                    data["amp_scale"], data["freq_scale"], data["signal_type"], data["input_type"]))
            message = make_MotorCalibrationTriggerMessage(data["motorNum"], data["amp_scale"], data["freq_scale"], data["dt"],
                            input_type_names.index(data["input_type"]), signal_type_names.index(data["signal_type"]),
                            max_angle, min_angle, True)
            ser.reset_input_buffer()
            ser.write(message)
            print("sent:", message)
            data["V"], data["angle"], data["t"] = print_from_micro(ser)
            print("got", ", ".join([k + ": " + str(data[k].shape) for k in ("V", "angle", "t")]))
            if filename:
                saveRun(filename, data)
        elif command == "motornum":
            try:
                data["motorNum"] = int(args[1])
            except ValueError:
                print("Invalid motor number")
        elif command == "save":
            filename = (args[1] if len(args) > 1 else "") + motortest_filename_tag(
                            data["amp_scale"], data["freq_scale"], data["signal_type"], data["input_type"])
            saveRun(filename, data)
        elif command == "dt":
            if len(args) > 1:
                data["dt"] = float(args[1])
            print("dt:", data["dt"])
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
        elif command == "type":
            if len(args) > 1:
                if args[1] in input_type_names:
                    data["input_type"] = args[1]
                elif args[1] in signal_type_names:
                    data["signal_type"] = args[1]
                else:
                    data["input_type"] = input_type_names[int(args[1])]
            print(f'Input type: {data["input_type"]} ({input_type_names.index(data["input_type"])})')
            print(f'Signal type: {data["signal_type"]} ({signal_type_names.index(data["signal_type"])})')
        elif command.startswith("exit"):
            break
        elif command == "code":
            while command != "exit":
                command = input(">>> ")
                try:
                    print(eval(command))
                except Exception as e:  # noqa: BLE001
                    print(e)


if __name__ == "__main__":
    if len(sys.argv) == 3:
        run_tests_from_file_input(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 1:
        interactive_main()
    else:
        raise ValueError("Run without args for interactive mode or with 2 args: input_filename, output_filename_tag")

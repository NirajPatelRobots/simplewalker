# Simplewalker
A two-legged walking robot
---
A raspberry pi zero, the Main Computer, calculates state, balance and dynamics. 
A pi pico is the low level motor controller and sensor reader.

## Download Instructions
Download the repo to a raspberry pi zero (or another SBC).

Requirements:
- C++ 17
- cmake 3.23+
- Eigen 3.4.8
- python 3.11
    - main computer requires pyserial
    - base python dependencies handled by uv
- rapidxml
- googletest 1.17 (installed by cmake)
- wiringPi (`apt-get install libwiringpi-dev` to build on ubuntu)
- MPU6050 pico code: https://github.com/NirajPatelRobots/pico-examples

To develop for pico on a non-pi computer: https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf

### Microcontroller Instructions
1. To develop for pico on the Zero, go to https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf Chapter 1 "Quick Pico Setup"
2. In the microcontroller/ directory, build the pico program:
    ```
    mkdir microcontroller/build
    export PICO_SDK_PATH=~/pico/pico-sdk/
    cp ~/pico/pico-sdk/external/pico_sdk_import.cmake microcontroller/
    cd microcontroller/build
    cmake ..
    make
    ```

3. Unplug the pico, then hold down the BOOTSEL button on the pico. 
Plug in the pico and keep the button held down until the green light on the zero stabilizes. 

4. Check the pico is connected, then load the built .uf2 file onto the pico:
    ```
    sudo picotool info -a
    sudo picotool load <name>.uf2 -v -x
    ```

To read output from printf, use USB. Can change to UART in CMakeLists.txt.
`minicom -b 115200 -o -D /dev/ttyACM0` to listen to the output. To exit minicom, use CTRL-A followed by X.


### Single Board Computer instructions
In the `simplewalker/build` folder of the repository, run
```
cmake ..    # once, to set up cmake
make simplewalker 
cd ..
build/simplewalker
```
to make and run the main program.

The built executables are simplewalker, test_localization, unittests, and collect_sensor_cal_data.


## Motor Models
A motor model predicts the change in state (acceleration) for any current state (velocity).

A motor model has two linear groups of terms: state terms and input terms.
Motor models have the form:
```
accel = state_term_1(state) + state_term_2(state) + ... + Voltage * (input_term_1(state) + ...)
accel = state_group(state) + Voltage * input_group(state)
```

Each term is a chain of model functions.
A function chain has the form:
```
state_term_1(state) = weight * f_1(f_2(f_base(state)))
```
In this example, f_2 is f_1's parent and f_base is f_2's parent. Base functions are special.

### Motor Model API
Write API:

| Name          | term_idx | fcn_field           | value  |
|---------------|----------|---------------------|--------|
| Create_Term   | -        | base fcn ID         | weight |
| Delete_Term   | term_idx | -                   | -      |
| Add_Function  | term_idx | nonbase fcn ID      | -      |
| Set_Parameter | term_idx | fcn_idx & param_idx | value  |
| Set_Weight    | term_idx | -                   | weight |

API response: `{int status, int term_idx}` where status > 0 is success


## Calibration
### IMU Calibration
collect_sensor_cal_data saves files in `data/stationary_calibration_[number].log`.
Copy these to the base computer and run `sensorAnalysis.py` on them to get sensor calibration values.

### Motor Calibration
- To collect motor calibration data, run `python3 runMotorCalibration.py` on the main computer (SBC). 
- Copy that data to the Base computer with `rsync -zic pi@raspberrypi:/home/pi/simplewalker/data/*.motortest ./data/`.
- To run the calibration and get motor parameters, run `uv run calibrate/calibrate_cli.py [file_glob]`
- To validate saved parameters, run `uv run calibrate/model_functions.py <motor_params.json>`
- To run the interactive calibration with a UI, run `uv run --extra interactive_calibration calibrate/interactive_calibrate.py`
- To run the motor calibration as a notebook, run `uv run -w jupyter -w ipympl --extra interactive_calibration jupyter lab`


## CI
Build and Test jobs do what they sound like. They run on Github ARM runners.
- `Build and Test Simplewalker`
- `Build Microcontroller Binaries` uses the pico sdk
- `Test Microcontroller` doesn't use the sdk

`Calibrate Motor` runs motor calibrations on checked-in motor test data.


# Thank you to
Thank you to all contributors to the open-source dependencies of this project.

Also using work from:
- lukstep/raspberry-pi-pico-sdk
- https://github.com/Mad-Scientist-Monkey/sockets-ccpp-rpi
- Silverlock on rpi forums for free heap size
- Ximaz/valgrind-action@v1.2.0


---
Niraj made this

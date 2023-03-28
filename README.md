# Simplewalker
A two-legged walking robot
---
A raspberry pi zero, the Main Computer, calculates state, balance and dynamics. A pi pico is the low level motor controller and sensor reader.

## Download Instructions
Download the repo to a raspberry pi zero (or another SBC).

Requirements:
- C++ 17
- cmake 3.16
- Eigen 3.4.8
- python 3.7
- rapidxml
- googletest 1.10
- wiringPi (`apt-get install libwiringpi-dev` to build on ubuntu)

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

3. Check the pico is connected, then load the built .uf2 file onto the pico

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

collect_sensor_cal_data saves files in `data/stationary_calibration_[number].log`.
Copy these to the base computer and run `sensorAnalysis.py` on them to get sensor calibration values.


---
Niraj made this

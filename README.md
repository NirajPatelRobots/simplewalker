# Simplewalker
Two-legged walking robot

## Download Instructions
Download the repo to a raspberry pi zero / other SBC.

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

Requires python 3.5 or newer.





---
Niraj did this

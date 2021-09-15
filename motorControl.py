# -*- coding: utf-8 -*-
"""
Simplewalker functions for controlling motors

TODO:
    check real motor scales and offsets, store this + use with sensorReader.py
        -store pins too
    investigate normally on H-bridges vs normally off
    replace RPi.GPIO with gpiozero? convenient objects, less control

Created Jun 2021
@author: Niraj
"""

try:
    import RPi.GPIO as GPIO
except:
    print("motorControl not run on Rpi, no functionality")
from numpy import pi

pins = {"Lmot0":  7,
        "Lmot1f":  8,
        "Lmot1r":  10,
        "Lmot2f":  11,
        "Lmot2r":  12,
        
        "Rmot0":  16,
        "Rmot1f":  13,
        "Rmot1r":  15,
        "Rmot2f":  18,
        "Rmot2r":  22,
        }

angle0offsetR = 0.1 # DC from potentiometer_angle=0 to motor_angle=0 for motor 0
angle0offsetL = 0.1 # left


class MotorController:
    """Class for controlling motors"""
    def __init__(self, PWMfreq = 2000):
        """Sets up pi pins and electronics so motors can run."""
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(list(pins.values()), GPIO.OUT)
        self.Lmot0 = GPIO.PWM(pins["Lmot0"], PWMfreq)
        self.Rmot0 = GPIO.PWM(pins["Rmot0"], PWMfreq)
        
        self.Lmot1f = GPIO.PWM(pins["Lmot1f"], PWMfreq) # forward
        self.Rmot1f = GPIO.PWM(pins["Rmot1f"], PWMfreq)
        self.Lmot1r = GPIO.PWM(pins["Lmot1r"], PWMfreq) # reverse
        self.Rmot1r = GPIO.PWM(pins["Rmot1r"], PWMfreq)
        
        self.Lmot2f = GPIO.PWM(pins["Lmot2f"], PWMfreq)
        self.Rmot2f = GPIO.PWM(pins["Rmot2f"], PWMfreq)
        self.Lmot2r = GPIO.PWM(pins["Lmot2r"], PWMfreq)
        self.Rmot2r = GPIO.PWM(pins["Rmot2r"], PWMfreq)
        
        self.fmotors = [self.Rmot1f, self.Rmot2f, self.Lmot1f, self.Lmot2f]
        self.rmotors = [self.Rmot1r, self.Rmot2r, self.Lmot1r, self.Lmot2r]
        
        self.Lmot0.start(angle0offsetL)
        self.Rmot0.start(angle0offsetR)
        for mot in self.fmotors + self.rmotors:
            mot.start(0.0)

    def setMotors(self, rightCommands, leftCommands):
        """Sets the target angle for motor 0 and the duty cycle for motors 1 and 2.
        rightCommands and leftCommands are 3x1 arrays from motionController.
        commands are between -1 and 1 for duty cycles.
        """
        self.Rmot0.ChangeDutyCycle(min(rightCommands[0] / pi + angle0offsetR, 1.0) * 100.0)
        self.Lmot0.ChangeDutyCycle(min(1 + (leftCommands[0] / pi) + angle0offsetL, 1.0) * 100.0)
        
        self.setMotor(rightCommands[1], 0)
        self.setMotor(rightCommands[2], 1)
        self.setMotor(leftCommands[1], 2)
        self.setMotor(leftCommands[2], 3)
        
    def setMotor(self, command, motorNum):
        """set duty cycle for motors 1 or 2.
        command is duty cycle -1 to 1
        motorNum is (0: right motor 1) (1: right motor 2) (2: left motor 1) (3: left motor 2)"""
        fmotor = self.fmotors[motorNum]
        rmotor = self.rmotors[motorNum]
        command = min(max(command, -1.0), 1.0)
        if command > 0.0:
            fmotor.ChangeDutyCycle(100.0) #TODO investigate two brake Hbridge methods
            rmotor.ChangeDutyCycle(100.0 * (1.0 - command))
        else:
            fmotor.ChangeDutyCycle(100.0 * (1.0 + command))
            rmotor.ChangeDutyCycle(100.0)


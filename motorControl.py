# -*- coding: utf-8 -*-
"""
Simplewalker functions for controlling motors

TODO:
    check real motor signs and offsets
    investigate normally on H-bridges vs normally off

Created Jun 2021
@author: Niraj
"""

import RPi.GPIO as GPIO
import from numpy import pi

pins = {"Lmot0":  7,
        "Lmot1f":  8,
        "Lmot1r":  10,
        "Lmot2f":  11,
        "Lmot2r":  12,
        
        "Rmot0":  13,
        "Rmot1f":  15,
        "Rmot1r":  16,
        "Rmot2f":  18,
        "Rmot2r":  19,
        }

angle0offsetR = 0.1 # DC from potentiometer angle=0 to motor angle=0 for angle 0
angle0offsetL = 0.1 # left



class MotorController:
    """Class for controlling motors"""
    def __init__(self, PWMfreq = 2000):
        """Sets up pi pins and electronics so motors can run."""
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pins.values(), GPIO.OUT)
        self.Lmot0 = GPIO.PWM("Lmot0", PWMfreq)
        self.Rmot0 = GPIO.PWM("Rmot0", PWMfreq)
        
        self.Lmot1f = GPIO.PWM("Lmot1f", PWMfreq) # forward
        self.Rmot1f = GPIO.PWM("Rmot1f", PWMfreq)
        self.Lmot1r = GPIO.PWM("Lmot1r", PWMfreq) # reverse
        self.Rmot1r = GPIO.PWM("Rmot1r", PWMfreq)
        
        self.Lmot2f = GPIO.PWM("Lmot2f", PWMfreq)
        self.Rmot2f = GPIO.PWM("Rmot2f", PWMfreq)
        self.Lmot2r = GPIO.PWM("Lmot2r", PWMfreq)
        self.Rmot2r = GPIO.PWM("Rmot2r", PWMfreq)
        
        self.Lmot0.start(0.0)
        self.Rmot0.start(0.0)
        self.Lmot1f.start(0.0)
        self.Rmot1f.start(0.0)
        self.Lmot1r.start(0.0)
        self.Rmot1r.start(0.0)
        self.Lmot2f.start(0.0)
        self.Rmot2f.start(0.0)
        self.Lmot2r.start(0.0)
        self.Rmot2r.start(0.0)
        
        

    def setMotors(self, rightCommands, leftCommands):
        """Sets the target angle for motor 0 and the duty cycle for motors 1 and 2.
        rightCommands and leftCommands are 3x1 arrays from motionController.
        """
        self.Rmot0.ChangeDutyCycle(min(rightCommands[0] / pi + angle0offsetR, 1.0) * 100.0)
        self.Lmot0.ChangeDutyCycle(min(1 + (leftCommands[0] / pi) + angle0offsetL, 1.0) * 100.0)
        
        
        self._setMotor(rightCommands[1], Rmot1f, Rmot1r)
        self._setMotor(rightCommands[2], Rmot2f, Rmot2r)
        self._setMotor(leftCommands[1], Lmot1f, Lmot1r)
        self._setMotor(leftCommands[2], Lmot2f, Lmot2r)
        
    
    def _setMotor(self, command, fmotor, rmotor):
        if command > 0.0:
            self.fmotor.ChangeDutyCycle(1.0) #TODO investigate two brake Hbridge methods
            self.rmotor.ChangeDutyCycle(100.0 * (1.0 - command))
        else:
            self.fmotor.ChangeDutyCycle(100.0 * (1.0 + command))
            self.rmotor.ChangeDutyCycle(1.0)


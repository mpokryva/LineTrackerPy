#!/usr/bin/python
#import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_Stepper
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor

import time
import atexit

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT()

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

myStepper = mh.getStepper(200, 1)  # 200 steps/rev, motor port #1
myStepper.setSpeed(500)             # 30 RPM
myStepper2 = mh.getStepper(200, 2)
myStepper2.setSpeed(10)

print("Single coil 25 steps forward")
time.sleep(2.0)
#myStepper.step(1, Adafruit_MotorHAT.FORWARD,  Adafruit_MotorHAT.SINGLE)
myStepper.step(1, Adafruit_MotorHAT.FORWARD,  Adafruit_MotorHAT.MICROSTEP)
#time.sleep(5.0)
#print("Single coil 25 steps Backward")
#myStepper.step(25, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.SINGLE)
turnOffMotors()

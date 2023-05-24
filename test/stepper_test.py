from ULN2003Pi.ULN2003 import ULN2003
import RPi.GPIO as GPIO
from time import sleep
STEPPER_PINS = ((4, 14, 15, 18), (17, 27, 22, 23))

GPIO.setmode(GPIO.BCM)

stepper = [ULN2003(pins, latency=0.005, half_step=False)
                       for pins in STEPPER_PINS]

while True:
    i = int(input("Enter 0 or 1: "))
    stepper[i].step(-2000)
        
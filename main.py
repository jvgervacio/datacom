"""
MAIN FILE FOR PET FEEDER VENDING MACHINE

"""
from time import sleep
import atexit
import json
import board
import busio
import digitalio
import RPi.GPIO as GPIO

from i2c_lcd import i2c_lcd
from ULN2003Pi.ULN2003 import ULN2003
from adafruit_mcp3xxx import mcp3008, analog_in

from servo import Servo

BUTTONS = (16, 20)
STEPPER_PINS = ((4, 14, 15, 18), (17, 27, 22, 23))
SERVO_PIN = 13

GPIO.setmode(GPIO.BCM)
[GPIO.setup(button_pin, GPIO.IN, GPIO.PUD_DOWN) for button_pin in BUTTONS]

SPI = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
CS = digitalio.DigitalInOut(board.D5)
MCP = mcp3008.MCP3008(SPI, CS)
SENSOR = analog_in.AnalogIn(MCP, mcp3008.P0)

def start():
    """START METHOD FOR PET FEEDER VENDING MACHINE"""
    stepper = [ULN2003(pins, latency=0.005, half_step=False) for pins in STEPPER_PINS]
    servo = Servo(SERVO_PIN)
    # lcd = i2c_lcd.lcd()
    # lcd.backlight_on(True)

    def dispense(type):
        """DISPENSE METHOD FOR PET FEEDER VENDING MACHINE"""
        # print(help(servo))
        # lcd.lcd_display_string("DISPENSING".center(16), 1)
        # lcd.lcd_display_string(f"{type.upper()} FOOD".center(16), 2)
        servo.set_angle(180)
        sleep(1)
        stepper[int(type == "dog")].step(1000)
        servo.set_angle(0)
        sleep(5)
        servo.set_angle(90)
        # lcd.lcd_clear()
    while True:
        if int(SENSOR.voltage) > 0: 
            if GPIO.input(BUTTONS[0]) == GPIO.HIGH:
                dispense("dog")
            elif GPIO.input(BUTTONS[1]) == GPIO.HIGH:
                dispense("cat")
if __name__ == '__main__':
    try:
        start()
    except KeyboardInterrupt:
        GPIO.cleanup()

from ULN2003Pi.ULN2003 import ULN2003
from i2c_lcd import i2c_lcd
from servo import Servo
import RPi.GPIO as GPIO
from time import sleep
import atexit
import json
import board
import busio
import digitalio
from adafruit_mcp3xxx import mcp3008, analog_in

BUTTONS = (16,20)
STEPPER_PINS = ((4,14, 15, 18), (17, 27, 22, 23))
SERVO_PIN = 21

GPIO.setmode(GPIO.BCM)
[GPIO.setup(button_pin, GPIO.IN) for button_pin in BUTTONS]

spi = busio.SPI(clock = board.SCK, MISO=board.MISO, MOSI=board.MOSI)
cs = digitalio.DigitalInOut(board.D5)
mcp = mcp3008.MCP3008(spi, cs)
sensor = analog_in.AnalogIn(mcp, mcp3008.P0)

dispensing = False
 
def start():
    
    stepper = [ULN2003(pins, latency=0.003, half_step=False) for pins in STEPPER_PINS]
    servo = Servo(SERVO_PIN)
    # lcd = i2c_lcd.lcd()
    # lcd.backlight_on(True)

    def dispense(type:str):
        print(type)
        # lcd.lcd_display_string("DISPENSING".center(16),1)
        # lcd.lcd_display_string(f"{type.upper()} FOOD".center(16),2)
        servo.set_angle(180)
        sleep(1)
        stepper[type == "dog"].step(2000)
        servo.set_angle(0)
        sleep(5)
        # lcd.lcd_clear()
            
    while True: 
        print(list(map(GPIO.input, BUTTONS))) 
        # sleep(0.5)  
            
if __name__ == '__main__':
    try:
        start()
    except KeyboardInterrupt:
        GPIO.cleanup()
        
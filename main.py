"""
MAIN FILE FOR PET FEEDER VENDING MACHINE

"""
from time import sleep
import os
import atexit
import json
import board
import busio
import digitalio
import RPi.GPIO as GPIO

from i2c_lcd import i2c_lcd
from ULN2003Pi.ULN2003 import ULN2003
from adafruit_mcp3xxx import mcp3008, analog_in
import cv2 as cv

from servo import Servo
from image_classifier import ImageClassifier


BUTTONS = (16, 20)
STEPPER_PINS = ((4, 14, 15, 18), (17, 27, 22, 23))
SERVO_PIN = 13

GPIO.setmode(GPIO.BCM)
[GPIO.setup(button_pin, GPIO.IN, GPIO.PUD_DOWN) for button_pin in BUTTONS]

SPI = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
CS = digitalio.DigitalInOut(board.D5)
MCP = mcp3008.MCP3008(SPI, CS)
SENSOR = analog_in.AnalogIn(MCP, mcp3008.P0)

classifier = ImageClassifier("can_and_bottle_model_v2.tflite", 0.6, 1)
cam = None

# openjson file

CAN_PTS = 10
BOTTLE_PTS = 5

CAT_CLAIMPOINTS = 15
DOG_CLAIMPOINTS = 20


stepper = [ULN2003(pins, latency=0.005, half_step=False)
                       for pins in STEPPER_PINS]
servo = Servo(SERVO_PIN)
servo.set_angle(90)
lcd = None
def start():
    
    """START METHOD FOR PET FEEDER VENDING MACHINE"""
    global lcd
    lcd = i2c_lcd.lcd()
    lcd.backlight_on(True)
    os.system("clear")
    print("STARTING PET FEEDER VENDING MACHINE...")
    print("RUNNING...")
    def dispense(type):
        """DISPENSE METHOD FOR PET FEEDER VENDING MACHINE"""
        print(f"DISPENSING {type.upper()} FOOD...")
        lcd.lcd_clear()
        lcd.lcd_display_string("DISPENSING".center(16), 1)
        lcd.lcd_display_string(f"{type.upper()} FOOD".center(16), 2)
        add_points(-CAT_CLAIMPOINTS if type == "cat" else -DOG_CLAIMPOINTS)
        stepper[int(type == "dog")].step(1000)
        lcd.lcd_clear()
        print("DONE DISPENSING")

    def add_points(points):
        #overwrites the data.json file
        data = read_data()
        data["points"] += points
        with open("./data.json", "w") as f:
            f.write(json.dumps(data))
    
    def read_data():
        with open("./data.json", "r") as f:
            data = json.loads(f.read())
            return data
        
    def getPoints():
        data = read_data()
        return data["points"]

    def open_door(category):
        """OPEN DOOR METHOD FOR PET FEEDER VENDING MACHINE"""
        points = CAN_PTS if category == "can" else BOTTLE_PTS
        lcd.lcd_display_string(f"{category.upper()} DETECTED!".center(16), 1)
        lcd.lcd_display_string(f"+{points} PTS".center(16), 2)
        servo.set_angle(90)
        sleep(2)
        servo.set_angle(0)
        add_points(points)

    cam = cv.VideoCapture(0)

    while True:
        
        if GPIO.input(BUTTONS[0]):
            if getPoints() >= CAT_CLAIMPOINTS:
                dispense("cat")
            else:
                lcd.lcd_display_string("NOT ENOUGH POINTS".center(16), 1)
                sleep(2)
                lcd.lcd_clear()
        elif GPIO.input(BUTTONS[1]):
            if getPoints() >= DOG_CLAIMPOINTS:
                dispense("dog")
            else:
                lcd.lcd_display_string("NOT ENOUGH POINTS".center(16), 1)
                sleep(2)
                lcd.lcd_clear()
                    
        # print("Sensor value: ", int(SENSOR.voltage))
        if int(SENSOR.voltage) > 0:
            lcd.lcd_clear()
            sleep(0.5)
            lcd.lcd_display_string("SCANNING...".center(16), 1)
            
            success, image = cam.read()
            if success:
                category = classifier.classify(image)
                print("category:",category)
                if category:
                    open_door(category)
        else:
            lcd.lcd_display_string(f"CURRENT POINTS:".center(16), 1)
            lcd.lcd_display_string(f"{getPoints()} PTS".center(16), 2)
                    

def close_program():
    if lcd:
        lcd.lcd_clear()
        lcd.backlight_on(False)
    
    print("Closing program")
    GPIO.cleanup()
    os.system("pidof firefox | xargs kill -9")         
                    
if __name__ == '__main__':
    
    try:
        
        start()
        
    except OSError or IOError as e:
        print("Error:", e)
        print("i2c LCD not detected. Restarting program...")
        start()
    except KeyboardInterrupt or RuntimeError as e:
        print("Closing program:", e)
        GPIO.cleanup()
        os.system("pidof firefox | xargs kill -9")
        
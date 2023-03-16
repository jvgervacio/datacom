import RPi.GPIO as GPIO
import time

LED_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
led = GPIO.PWM(LED_PIN, 50)
led.start(0)
try:
    while True:
        duty = int(input("Enter duty cycle: "))
        led.ChangeDutyCycle(duty)
except KeyboardInterrupt:
    GPIO.cleanup()
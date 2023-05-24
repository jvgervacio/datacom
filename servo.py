import RPi.GPIO as GPIO
import time

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Servo:
    def __init__(self, pin):
        self.gpio = pin
        self.min_angle = 0
        self.max_angle = 180
        self.min_pulse = 0.5
        self.max_pulse = 2.5
        self.pwm_period_ms = 20
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 1000/self.pwm_period_ms)
        self.pwm.start(2.5)

    def set_angle(self, angle):
        if angle < self.min_angle or angle > self.max_angle:
            raise ValueError("Angle must be between 0 and {}".format(self.max_angle))
        pulse = map_range(angle, self.min_angle, self.max_angle, self.min_pulse, self.max_pulse)
        duty = pulse / self.pwm_period_ms * 100
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.15)
        
    def __del__(self): 
        GPIO.cleanup()
        
if __name__ == "__main__":
    try:
        servo = Servo(13)
        while True:
            servo.set_angle(int(input("Enter Angle: ")))
    except KeyboardInterrupt:
        GPIO.cleanup()

import RPi.GPIO as GPIO
import time

HIN = 8
LIN = 10
freq = 500

class Motor:
    def __init__(self, HIN=HIN, LIN=LIN, freq=freq):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(HIN, GPIO.OUT)
        GPIO.setup(LIN, GPIO.OUT)
        
        self.high = GPIO.PWM(HIN, freq) 
        self.low = GPIO.PWM(LIN, freq)
        
        self.low.start(0)

    def setSpeed(self, speed):
        if speed < 0:
            self.high.start(0)
        elif speed > 100:
            self.high.start(100)
        else:
            self.high.start(speed)

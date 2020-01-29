import RPi.GPIO as GPIO
import time

HIN = 8
LIN = 10

GPIO.setmode(GPIO.BOARD)
GPIO.setup(HIN, GPIO.OUT)
GPIO.setup(LIN, GPIO.OUT)

freq = 150

high = GPIO.PWM(HIN, freq) 
low = GPIO.PWM(LIN, freq)

DC = 5

low.start(0)

while DC < 100:
    high.start(DC)
    DC = DC +5
    time.sleep(2)
    print(DC)

try:
    while True:
       
        high.start(DC)
        if DC >= 10:
             DC = DC - 5

        print(DC)
        time.sleep(0.5)
        
except KeyboardInterrupt:
    high.stop()
    low.stop()
    GPIO.cleanup()

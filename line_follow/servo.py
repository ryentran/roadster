import RPi.GPIO as GPIO
import time

left = True
right = False

turningDegrees = 45
rightTurnMax = 8
leftTurnMax = 3
straightTurn = 5.5
turnRange = (rightTurnMax - leftTurnMax) / 2.0
degreesPerCycle = turningDegrees / turnRange

def turn( direction, degrees, servo):
    cycle = degrees / degreesPerCycle
    if (direction):
        cycle *= -1
    cycle += straightTurn
    servo.ChangeDutyCycle(cycle)

servoPIN = 7
GPIO.setmode(GPIO.BOARD)

GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(0) # Initialization
time.sleep(0.5)
try:
    while True:
        turn(right, 14.5, p)
        time.sleep(1)
        turn(left, 34.6, p)
        time.sleep(1)

except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()

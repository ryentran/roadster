import RPi.GPIO as GPIO
import time
import numpy as np

servoPin = 7
turningDegrees = 45.0
rightTurnMax = 10.0
leftTurnMax = 5.0
hertz = 50
threshold = -1

class Servo:
    def __init__(self, servoPin = servoPin, turningDegrees = turningDegrees, leftTurnMax = leftTurnMax, rightTurnMax = rightTurnMax, hertz = hertz, threshold=threshold):
        turnRange = np.divide(np.add(rightTurnMax, -leftTurnMax), 2.0)
      
        self.straightTurn = np.add(leftTurnMax, turnRange)
        self.degreesPerCycle = np.divide(turningDegrees, turnRange)
        self.servoPin = servoPin
      
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servoPin, GPIO.OUT)
        self.servo = GPIO.PWM(self.servoPin, hertz)
        self.servo.start(0)
        self.previousTurn = 0
        self.threshold = threshold

    def turn(self, degrees):
        if abs(np.add(degrees, -self.previousTurn)) > threshold:
            self.previousTurn = degrees
        else:
            return

        if degrees < -45:
            degrees = -45
        elif degrees > 45:
            degrees = 45

        cycle = np.divide(degrees, self.degreesPerCycle)
        cycle += self.straightTurn
        self.servo.ChangeDutyCycle(cycle)

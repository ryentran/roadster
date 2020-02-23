import numpy as np
import cv2
import RPi.GPIO as GPIO
import picamera
import picamera.array
import math
import servo
import motor
import io
import time

PRINT_ANGLE = True

width = 960
height = 720

servo = servo.Servo()
motor = motor.Motor()

# Sort higher to lower coordinates of the box for midpoint formula
def sortCoords(coords):
    xCoords = coords[:, 0].tolist()
    yCoords = coords[:, 1].tolist()
    sortedCoords = np.zeros((4, 2))
    for i in range(len(coords)):
        yMinIndex = yCoords.index(min(yCoords))
        sortedCoords[i] = [xCoords[yMinIndex], yCoords[yMinIndex]]
        xCoords.pop(yMinIndex)
        yCoords.pop(yMinIndex)
    return sortedCoords

try:
    with picamera.PiCamera() as camera:
        with picamera.array.PiRGBArray(camera) as stream:
            while True:
                camera.capture(stream, 'bgr', use_video_port=True) 
                crop_img = stream.array[int(height/4):height, 0:width]
                gray_img = cv2.cvtColor(crop_img, cv2.COLOR_RGB2GRAY)
                corrected_img = cv2.convertScaleAbs(gray_img, alpha=0.5, beta=0)
                blur_img = cv2.GaussianBlur(corrected_img, (5, 5), 0)

                ret, thresh = cv2.threshold(blur_img, 127, 255, 0)
                contours, hierarchy = cv2.findContours(
                    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    cnt = contours[0]
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    boxSorted = sortCoords(box)

                    # Ignore very big or small boxes (very bad edge case)
                    boxW = abs(boxSorted[0][0]-boxSorted[1][0])
                    boxH = abs(boxSorted[0][1]-boxSorted[2][1])

                    # print("Width: {}".format(boxW))
                    # print("Height: {}".format(boxH))

                
                    # if boxW > 40 and boxH > 300:
                    midPoint = [[(boxSorted[0][0]+boxSorted[1][0])/2,
                                (boxSorted[0][1]+boxSorted[1][1])/2]]
                    bottomPoint = [width/2, height]
                    diffOfMedians = [midPoint[0][0] - bottomPoint[0],
                                    abs(midPoint[0][1] - bottomPoint[1])]
                    angle = int(math.degrees(
                        math.atan(diffOfMedians[0] / diffOfMedians[1])))
                    
                    if PRINT_ANGLE:
                        print("{}\n".format(angle))

                    servo.turn(angle)
                    speed = np.add(50, -np.power(abs(angle), 2))
                    if speed < 15:
                        speed = 15
                    motor.setSpeed(speed)

                        # Just for visuals
                        # box = np.int0(box)
                        # midPoint = np.int0(midPoint)
                        # turnLine = [midPoint, [bottomPoint]]
                        # turnLine = np.int0(turnLine)
                        # cv2.drawContours(crop_img, contours, 0, (0, 255, 0), 1)
                        # cv2.drawContours(crop_img, [box], 0, (255, 0, 0), 1)
                        # cv2.drawContours(crop_img, [turnLine], 0, (255, 255, 0), 2)
                        # cv2.drawContours(crop_img, [midPoint], 0, (0, 0, 255), 5)
                        # cv2.imshow("CV", crop_img)

                stream.seek(0)
                stream.truncate()
except KeyboardInterrupt:
    motor.high.stop()
    motor.low.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    camera.close()

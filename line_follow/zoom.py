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

MAX_SPEED = 10
MIN_SPEED = 10
MIN_CONTOUR_DIFF = 10000
MIN_CONTOUR_AREA = 15000

MOTOR = False
TIME = False
PRINT_ANGLE = False
SHOW_VISUAL = True
GPIO.setwarnings(False)
width = 1280
height = 720

hCrop = 0.75

camera = picamera.PiCamera()
camera.resolution = (width, height)
camera.framerate = 90
stream = picamera.array.PiRGBArray(camera, size=(width, height))
time.sleep(0.1)

servo = servo.Servo()
motor = motor.Motor()

def AreaSort(contour):
    return contour[0]

try:
    for frame in camera.capture_continuous(
            stream, format="bgr", use_video_port=True):
        
        if TIME:
            t0 = time.time()
        
        # Image Editing.
        img = frame.array[0:int((2/3) * height), 0:width]
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
        # blur_img = cv2.medianBlur(gray_img,5)
        corrected_img = cv2.convertScaleAbs(blur_img, alpha=1, beta=20)
        ret, thresh = cv2.threshold(corrected_img, 240, 255, cv2.THRESH_BINARY)
        
        # Contour Detection.
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            skip = False
            
            # Sort contours based on area.
            contour_area = np.asarray(
                    [cv2.contourArea(contour) for contour in contours])
            contours_sorted = sorted(
                    zip(contour_area, contours), key=AreaSort, reverse=True)
            
            # Sort contours based on size (# of points)
            contour_size = [len(contour) for contour in contours]
            
            # Skip evaluation if contours are of similar size
            if (len(contours) > 1 and 
                    abs(contours_sorted[0][0] - contours_sorted[1][0]) 
                    < MIN_CONTOUR_DIFF):
                skip = True
                print("skipped")

            cnt = contours[contour_size.index(max(contour_size))]
            
            
            # print(cv2.contourArea(cnt))
            
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
                skip = True
            
            # ellipse
            # ellipse = cv2.fitEllipse(cnt)

            # circle
            # (x, y), radius = cv2.minEnclosingCircle(cnt)

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            boxSorted = box[box[:, 1].argsort()]

            # Ignore very big or small boxes (very bad edge case)
            # boxW = abs(boxSorted[0][0]-boxSorted[1][0])
            # boxH = abs(boxSorted[0][1]-boxSorted[2][1])

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
                
            # Determine new angle and speed.
            if not(skip):
                servo.turn(angle)
                speed = np.add(MAX_SPEED, -np.power(abs(angle), 2))
                if speed < MIN_SPEED:
                    speed = MIN_SPEED
                if MOTOR:
                    motor.setSpeed(speed)
            else:
                if MOTOR:
                    motor.setSpeed(MIN_SPEED)

            if SHOW_VISUAL:
                # Draw Visuals.
                box = np.int0(box)
                midPoint = np.int0(midPoint)
                turnLine = [midPoint, [bottomPoint]]
                turnLine = np.int0(turnLine)
               
                # ellipse
                # cv2.ellipse(img, ellipse, (0, 255, 0), 2)

                # circle
                # cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 1)
                cv2.drawContours(img, [turnLine], -1, (255, 255, 0), 2)
                cv2.drawContours(img, [midPoint], -1, (0, 0, 255), 2)
        
        if SHOW_VISUAL:
            cv2.imshow("CV", img)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        stream.seek(0)
        stream.truncate()

        if TIME:
            print(time.time() - t0)

except KeyboardInterrupt:
    motor.high.stop()
    motor.low.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    camera.close()

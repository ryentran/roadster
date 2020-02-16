import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
# import picamera
# import picamera.array
import time


width=1280
height=720
x=0
y=0
w=1280
h=height

fps = 10

def sortByX(coord):
  return coord[0]

def sortByY(coord):
  return coord[1]

def makeCoord(image, parameters):
    slope, intercept = parameters
    y1 = image.shape[0]
    y2 = int(y1*(3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return ((x1, x2), (y1, y2))


# with picamera.PiCamera() as camera:
#     with picamera.array.PiRGBArray(camera) as stream:
#         camera.resolution = (width, height)

cap = cv2.VideoCapture("natcar_test.mp4")
# while True:
while(cap.isOpened()):
    # camera.capture(stream, 'bgr', use_video_port=True)
    # # stream.array now contains the image data in BGR order
    # crop_img = stream.array[y:y+h, x:x+w]
    _, frame = cap.read()
    gray_img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (5,5), 0)
    canny_img = cv2.Canny(blur_img, 100, 300)
    
    # Image crop
    polygons = np.array([[(0, 540), (0, 1080), (1920, 1080),  (1920, 540)]])
    mask = np.zeros_like(canny_img)
    cv2.fillPoly(mask, polygons, 255)
    masked_img = cv2.bitwise_and(canny_img, mask)
    
    lines = cv2.HoughLinesP(
        masked_img,
        rho=6,
        theta=np.pi / 60,
        threshold=160,
        lines=np.array([]),
        minLineLength=25,
        maxLineGap=10)

    leftLine = []
    rightLine = []
    leftFit = []
    rightFit = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            m = parameters[0]
            b = parameters[1]
            if m < 0:
                leftFit.append((m, b))
            else:
                rightFit.append((m, b))            

        print(leftFit)

        if leftFit:
            leftFitAvg = np.average(leftFit, axis=0)
            mLeft, bLeft = leftFitAvg
            leftLine = makeCoord(frame, leftFitAvg)
            print(leftFitAvg)
        if rightFit:
            rightFitAvg = np.average(rightFit, axis=0)
            mRight, bRight = rightFitAvg
            rightLine = makeCoord(frame, rightFitAvg)



    temp = []
    topScreenMedian = []
    bottomScreenMedian = []

    if (isinstance(lines, np.ndarray)):
        for line in lines:
            x1 = line[0][0]
            y1 = line[0][1]
            x2 = line[0][2]
            y2 = line[0][3]
            temp.append([x1, y1])
            temp.append([x2, y2])

        temp.sort(key=sortByY)
        highYCoords = temp[math.floor(len(temp)/2):]
        lowYCoords = temp[:math.floor(len(temp)/2)]

        highYCoords.sort(key=sortByX)
        lowYCoords.sort(key=sortByX)
        topScreenMedian = highYCoords[math.floor(len(highYCoords)/2)]
        bottomScreenMedian = (960, 1080)
        print(topScreenMedian)

        diffOfMedians = [topScreenMedian[0] - bottomScreenMedian[0],
            topScreenMedian[1] - bottomScreenMedian[1]]
        angle = math.degrees(math.atan(diffOfMedians[1] / diffOfMedians[0]))
        print(angle)
    
    # Shows hough lines
    line_img = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 10)
    
    
    # Shows turn lines
    turn_img = np.zeros_like(frame)
    if topScreenMedian:
        cv2.line(line_img, (topScreenMedian[0], topScreenMedian[1]), (bottomScreenMedian[0], bottomScreenMedian[1]), (0, 255, 0), 10)


    # # Shows best fit lines
    # leftLineImg = []
    # rightLineImg = []
    # if leftLine:
    #     line_img = np.zeros_like(frame)
    #     cv2.line(line_img, leftLine[0], leftLine[1], (0, 255, 0), 10)
    # if rightLine:
    #     line_img = np.zeros_like(frame)
    #     cv2.line(line_img, rightLine[0], rightLine[1], (0, 255, 0), 10)


    # if leftLineImg:
    #     combo_img = cv2.addWeighted(frame, 0.8, leftLineImg, 1, 1)
    #     cv2.imshow("cannied", combo_img)
    # else:
    #     cv2.imshow("cannied", frame)

    combo_img = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
    combo_img = cv2.addWeighted(combo_img, 0.8, turn_img, 1, 1)
    cv2.imshow("cannied", combo_img)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # # reset the stream before the next capture
    # stream.seek(0)
    # stream.truncate()
    
    time.sleep(1/fps)

cap.release()
cv2.destroyAllWindows()
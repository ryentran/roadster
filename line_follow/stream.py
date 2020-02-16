import numpy as np
import cv2
import picamera
import picamera.array
import math

width=1280
height=720
x=0
y=0
w=1280
h=height

def sortByX(coord):
  return coord[0]

def sortByY(coord):
  return coord[1]

with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (width, height)

        while True:
            camera.capture(stream, 'bgr', use_video_port=True)
            # stream.array now contains the image data in BGR order
            crop_img = stream.array[y:y+h, x:x+w]
            gray_img = cv2.cvtColor(crop_img, cv2.COLOR_RGB2GRAY)
            blur_img = cv2.GaussianBlur(gray_img, (5,5), 0)
            canny_img = cv2.Canny(blur_img, 100, 300)

            lines = cv2.HoughLinesP(
                cannyed_img,
                rho=6,
                theta=np.pi / 60,
                threshold=160,
                lines=np.array([]),
                minLineLength=40,
                maxLineGap=25)

            temp = []

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
                bottomScreenMedian = lowYCoords[math.floor(len(lowYCoords)/2)]
                diffOfMedians = [topScreenMedian[0] - bottomScreenMedian[0],
                    topScreenMedian[1] - bottomScreenMedian[1]]
                angle = math.degrees(math.atan(diffOfMedians[1] / diffOfMedians[0]))
                print(angle)

            cv2.imshow("cannied", cannyed_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # reset the stream before the next capture
            stream.seek(0)
            stream.truncate()

        cv2.destroyAllWindows()
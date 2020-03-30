import numpy as np
import cv2
import picamera
import picamera.array
import math

width=1280
height=480
x=0
y=0
w=width
h=height

with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (width, height)

        while True:
            camera.capture(stream, 'bgr', use_video_port=True)
            # stream.array now contains the image data in BGR order
            crop_img = stream.array[y:y+h, x:x+w]
            blur_img = cv2.GaussianBlur(crop_img, (5,5), 0)
            gray_img = cv2.cvtColor(blur_img, cv2.COLOR_RGB2GRAY)
            cannyed_img = cv2.Canny(gray_img, 100, 300)
            lines = cv2.HoughLinesP(
                cannyed_img,
                rho=6,
                theta=np.pi / 60,
                threshold=160,
                lines=np.array([]),
                minLineLength=40,
                maxLineGap=25)

            cv2.imshow("cannied", cannyed_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # reset the stream before the next capture
            stream.seek(0)
            stream.truncate()

        cv2.destroyAllWindows()

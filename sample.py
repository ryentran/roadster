# Write your code here :-)
import numpy as np
import cv2
from time import sleep
from picamera import PiCamera

camera = PiCamera()

x=0
y=720
w=3440
h=720

img = cv2.imread("image.jpg")
crop_img = img[y:y+h, x:x+w]
gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
cannyed_img = cv2.Canny(gray_img, 100, 300)
cv2.imwrite("/home/pi/canny.jpg", cannyed_img)
lines = cv2.HoughLinesP(
    cannyed_img,
    rho=6,
    theta=np.pi / 60,
    threshold=160,
    lines=np.array([]),
    minLineLength=40,
    maxLineGap=25)
print(lines)
cv2.imshow("cannied", cannyed_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
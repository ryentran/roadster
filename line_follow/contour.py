import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import time

fps = 60


cap = cv2.VideoCapture("natcar_test.mp4")

while(cap.isOpened()):
    _, frame = cap.read()
    gray_img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (5,5), 0)
    canny_img = cv2.Canny(blur_img, 100, 300)
    
    # Image crop
    polygons = np.array([[(0, 700), (0, 1080), (1920, 1080),  (1920, 700)]])
    mask = np.zeros_like(canny_img)
    cv2.fillPoly(mask, polygons, 255)

    masked_img = cv2.bitwise_and(canny_img, mask)
    
    # Contour
    ret, thresh = cv2.threshold(masked_img,127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)


    combo_img = cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    cv2.imshow("CV", combo_img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    time.sleep(1/fps)

cap.release()
cv2.destroyAllWindows()
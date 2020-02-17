import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import time

fps = 0
cap = cv2.VideoCapture("natcar_test.mp4")

# Sort higher to lower coordinates of the box for midpoint formula
def sortCoords(coords):
    xCoords = coords[:, 0].tolist()
    yCoords = coords[:, 1].tolist()
    sortedCoords = np.zeros((4, 2))
    for i in range(len(coords)):
        yMaxIndex = yCoords.index(max(yCoords))
        sortedCoords[i] = [xCoords[yMaxIndex], yCoords[yMaxIndex]]
        xCoords.pop(yMaxIndex)
        yCoords.pop(yMaxIndex)
    return sortedCoords


while(cap.isOpened()):
    _, frame = cap.read()
    gray_img = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

    # Image crop with bit mask
    polygons = np.array([[(0, 750), (0, 1080), (1920, 1080),  (1920, 750)]])
    mask = np.zeros_like(blur_img)
    cv2.fillPoly(mask, polygons, 255)
    masked_img = cv2.bitwise_and(blur_img, mask)

    ret, thresh = cv2.threshold(masked_img, 127, 255, 0)
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
        if boxW > 100 or boxW < 50:
            cv2.imshow("CV", frame)
        elif boxH > 350 or boxH < 50:
            cv2.imshow("CV", frame)
        else:
            print("Height: {}".format(int(boxH)))
            print("Width: {}\n".format(int(boxW)))
            line = [((boxSorted[0][0]+boxSorted[1][0])/2, (boxSorted[0][1]+boxSorted[1][1])/2),
                    ((boxSorted[2][0]+boxSorted[3][0])/2, (boxSorted[2][1]+boxSorted[3][1])/2)]
            box = np.int0(box)
            line = np.int0(line)
            cv2.drawContours(frame, contours, 0, (0, 255, 0), 2)
            cv2.drawContours(frame, [box], 0, (255, 0, 0), 2)
            cv2.drawContours(frame, [line], 0, (0, 0, 255), 5)
            cv2.imshow("CV", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if fps > 0:
        time.sleep(1/fps)

cap.release()
cv2.destroyAllWindows()

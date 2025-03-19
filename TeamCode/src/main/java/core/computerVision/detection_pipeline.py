import cv2
import numpy as np
from math import atan2, degrees

def angle(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]

    return degrees(atan2(dx, dy))

def dist(a, b):
    x, y = a
    z, w = b
    return (x - z) ** 2 + (y - w) ** 2

def get_four_corners(contour):
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    box = sorted(box, key=lambda point: (point[0], point[1]))
    tl, tr, br, bl = box
    return [tuple(tl), tuple(tr), tuple(br), tuple(bl)]


def center(contour):
    x,y,w,h = cv2.boundingRect(contour)
    cx = int(x + w / 2)
    cy = int(y + h / 2)
    return (cx, cy)

def cdist(contour):
    x,y,w,h = cv2.boundingRect(contour)
    cx = x + w / 2
    cy = y + h / 2
    return cx ** 2 + cy ** 2

def runPipeline(img, llrobot):

    lower = (20, 50, 100)

    image = img
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (20, 140, 100), (30, 255, 255))

    image = cv2.bitwise_and(image, image, mask=img_threshold)

    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0,0,0]

    for contour in contours:
        if cv2.contourArea(contour) < 4000: continue

        for idx in range(0, len(contour), 2):
            point = contour[idx]
            cv2.circle(image, (point[0][0], point[0][1]), 0, (0, 0, 0), thickness=-1)

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (20, 100, 200), (30, 255, 255))
    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    to_remove = []

    contours = list(contours)

    for idx in range(len(contours)):
        size = cv2.contourArea(contours[idx])
        if size < 4000 or size > 10000: to_remove.append(idx)
        else:
            cv2.circle(image, center(contours[idx]), 10, (255, 0, 0))
    for idx in range(len(to_remove)):
        contours.pop(to_remove[idx] - idx)

    if len(contours) > 0:
        largestContour = max(contours, key=cdist)

        extremeties = get_four_corners(largestContour)
        distances = [dist(extremeties[0], extremeties[x + 1]) for x in range(3)]

        mid = sorted(distances)[1]
        idx = distances.index(mid) + 1

        a = angle(extremeties[idx], extremeties[0]) - 90
        if a < 0: a += 180
        a -= 90
        a *= -1


        for e in extremeties:
            cv2.circle(image, e, 10, (255, 0, 0))

        x,y,w,h = cv2.boundingRect(largestContour)

        lx = x + w / 2
        ly = y + h / 2

        cx = (lx - 120) / 80 * 3
        cy = (ly - 120) / 80 *-3

        cv2.putText(image, f"ANGLE: {int(a)}", [extremeties[0][0], extremeties[0][1] + 30], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, lineType=cv2.LINE_AA, bottomLeftOrigin=False)
        cv2.putText(image, f"POSITION: {round(cx,2)}, {round(cy,2)}", [extremeties[0][0], extremeties[0][1] + 45], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, lineType=cv2.LINE_AA, bottomLeftOrigin=False)
        cv2.line(image, extremeties[0], extremeties[idx], (255, 0, 0), 10)

        llpython = [a,cx,cy]

    return largestContour, image, llpython

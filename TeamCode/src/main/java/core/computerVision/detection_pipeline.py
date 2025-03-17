import cv2
import numpy as np

lx = 0
ly = 0

degration_scale = 10
fail_counter = 0

def cdist(contour):
    x,y,w,h = cv2.boundingRect(contour)
    cx = x + w / 2
    cy = y + h / 2
    return (lx - cx) ** 2 + (ly - cy) ** 2

def runPipeline(img, llrobot):
    global fail_counter
    global degration_scale
    global lx
    global ly

    image = img
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (15[], 180, 120), (30, 255, 255))

    image = cv2.bitwise_and(image, image, mask=img_threshold)

    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largestContour = np.array([[]])
    llpython = [0,0,0,0,0,0,0,0]

    for contour in contours:
        if cv2.contourArea(contour) < 4000: continue

        for idx in range(0, len(contour), 2):
            point = contour[idx]
            cv2.circle(image, (point[0][0], point[0][1]), int(degration_scale), (0, 0, 0), thickness=-1)

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, (20, 160, 100), (60, 255, 255))
    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    to_remove = []

    contours = list(contours)

    for idx in range(len(contours)):
        size = cv2.contourArea(contours[idx])
        if size < 500 or size > 7000: to_remove.append(idx)
    for idx in range(len(to_remove)):
        contours.pop(to_remove[idx] - idx)

    if len(contours) > 0:
        largestContour = max(contours, key=cdist)
        x,y,w,h = cv2.boundingRect(largestContour)
        llpython = [1,x,y,w,h,9,8,7]

        lx = x + w / 2
        ly = y + h / 2

        if degration_scale > 5: degration_scale -= 1

    elif degration_scale < 10 and fail_counter > 5: degration_scale += 11

    if len(contours) == 0: fail_counter += 1
    else: fail_counter = 0

    return largestContour, image, llpython

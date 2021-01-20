#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture('track1.avi')

threshold = 60
width = 640
scan_width, scan_height = 200, 20
lmid = scan_width
rmid = width - scan_width
area_width, area_height = 20, 10
vertical = 430
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_threshold = 0.8 * area_height * area_width

while True:
    ret, frame = cap.read()
    if not ret:
        break
    if cv2.waitKey(1) & 0xFF == 27:
        break
        
    roi = frame[vertical:vertical+scan_height, :]
    frame = cv2.rectangle(frame, (0, vertical), (width, vertical + scan_height), (255, 0, 0), 3)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    bin = cv2.inRange(hsv, lbound, ubound)
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1
    for l in range(0, lmid - area_width):
    #for l in range(area_width, lmid + 1):
        area = bin[row_begin:row_end, l:l + area_width]
        #area = bin[row_begin:row_end, l - area_width:l]
        if cv2.countNonZero(area) > pixel_threshold:
            left = l
            break

    for r in range(width - area_width, rmid + 1, -1):
        area = bin[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > pixel_threshold:
            right = r
            break

    if left != -1:
        lsquare = cv2.rectangle(view, (left, row_begin), (left + area_width, row_end), (0, 255, 0), 3)
    else:
        print("Lost left line")

    if right != -1:
        rsquare = cv2.rectangle(view, (right, row_begin), (right + area_width, row_end), (0, 255, 0), 3)
    else:
        print("Lost right line")

    cv2.imshow('origin', frame)
    cv2.imshow('view', view)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    hsv = cv2.inRange(hsv, lbound, ubound)
    cv2.imshow('hsv', hsv)

    time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()

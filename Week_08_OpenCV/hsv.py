#!/usr/bin/python3

import cv2
import numpy as np

img = cv2.imread('img/cars.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# hsv 범위 지정
lower_white = np.array([0, 0, 150])
upper_white = np.array([179, 255, 255])
# 범위에 포함되는 픽셀들은 흰색으로, 그 외의 픽셀들은 검은색으로 이진화한 grayscale의 이미지를 만들어냄
# 색상 범위: 0 ~ 179
# 채도 범위: 0 ~ 255
# 명도 범위: 150 ~ 255
mask = cv2.inRange(hsv, lower_white, upper_white)
cv2.imshow('line', mask)
cv2.waitKey(0)
cv2.imwrite('img/hsv_result.png', mask)

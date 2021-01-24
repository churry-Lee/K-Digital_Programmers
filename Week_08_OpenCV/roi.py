#!/usr/bin/python3

import cv2

img = cv2.imread('img/cars.png')
cv2.imshow('car', img[120:270, 270:460])
cv2.waitKey(0)

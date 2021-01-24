#!/usr/bin/env python3

import cv2

img = cv2.imread('img/black.png', cv2.IMREAD_COLOR)
img = cv2. rectangle(img, (100, 100), (300, 400), (0, 255, 0), 3)

cv2.imshow('black', img)
cv2.waitKey(0)
cv2.imwrite('img/black_rec.png', img)


#!/usr/bin/env python3

import cv2

img_basic = cv2.imread("img/lenna.bmp", cv2.IMREAD_COLOR)
cv2.imshow('Image Basic', img_basic)
cv2.waitKey(0)
cv2.imwrite('img/result1.png', img_basic)

img_gray = cv2.cvtColor(img_basic, cv2.COLOR_BGR2GRAY)
cv2.imshow('Image Gray', img_gray)
cv2.waitKey(0)
cv2.imwrite('img/result2.png', img_gray)

cv2.destroyAllWindows()

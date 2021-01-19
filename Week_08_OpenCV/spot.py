#!/usr/bin/env python3

import cv2

img = cv2.imread('img/spot.png', cv2.IMREAD_GRAYSCALE)
# 이미지 파일의 모양(크기)을 저장
h = img.shape[0]
w = img.shape[1]
print('This image dimension is {} * {}'.format(h, w))

for i in range(h):
    for j in range(w):
        if img[i, j] == 255:
            print(i, j)

cv2.imshow('spot', img)
cv2.waitKey(0)

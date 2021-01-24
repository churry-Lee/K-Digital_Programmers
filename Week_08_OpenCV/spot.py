#!/usr/bin/env python3

import cv2

img = cv2.imread('img/spot.png', cv2.IMREAD_GRAYSCALE)
# 이미지 파일의 모양(크기)을 저장
h = img.shape[0] # 세로(높이)
w = img.shape[1] # 가로(너비)
print('This image dimension is {} * {}'.format(w, h))
# 흰 점을 찾기 위한 반복문
for i in range(h):
    for j in range(w):
        if img[i, j] == 255:
            print(i, j)
            y = i
            x = j
# (가로, 세로) 좌표 주의
# 찾은 흰 점에 동그라미 표시
img = cv2.circle(img, (x, y), 20, 255, 1)
cv2.imshow('spot', img)
cv2.waitKey(0)
cv2.imwrite('img/spot_find.png', img)

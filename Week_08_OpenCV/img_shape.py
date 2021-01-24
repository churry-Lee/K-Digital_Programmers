#!/usr/bin/python3

import cv2

img = cv2.imread('img/dog.jpeg', cv2.IMREAD_COLOR)
img2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print(type(img))
print("color img shape: ", img.shape)
print("이미지 배열: ", '\n', img)
print("이미지 크기(= 픽셀 열의 크기, 즉, 이미지의 세로 길이): ", len(img))
print("이미지 한 픽셀 열당 행의 크기(즉, 이미지의 가로 길이): ", len(img[0]))
print("한 픽셀의 색깔 정보: ", img[0][0], '\n')
print("grayscale img shape: ", img2.shape)
print("grayscale 한 픽셀의 색깔 정보: ", img2[0][0])

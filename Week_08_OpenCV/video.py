#!/usr/bin/python3

import cv2

#vid = cv2.VideoCapture('img/small.avi')
vid = cv2.VideoCapture(0)

# while True:
while vid.isOpened():
    # 비디오를 프레임 단위로 읽음
    # 프레임을 읽어올 때 ret, frame으로 읽어오며
    # ret에는 bool 값이 저장이 됨
    # ret이 True 이면 frame에 읽어온 이미지 프레임이 저장이됨
    ret, frame = vid.read()
    if not ret:
        break
    # grayscale로 표현
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if ret:
        cv2.imshow('video', frame)
    # 아무 key를 입력하면 종료됨
    if cv2.waitKey(1) > 0:
        break

# 오픈한 비디오나 영상 장치를 닫음
vid.release()
cv2.destroyAllWindows()

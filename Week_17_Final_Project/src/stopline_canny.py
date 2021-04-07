import numpy as np
import cv2, random, math,copy,time 

def detect_stopline(image, stop_min, stop_max, stop_time):
    stop = False 
    
    stopline_image = set_roi_ch(image, 100, 285, 450, 20)
    stopline_image = image_procesing(stopline_image)
    cnt_nzero = cv2.countNonZero(stopline_image)
    # print(cv2.countNonZero(stopline_image))
    if (stop_min < cv2.countNonZero(stopline_image) < stop_max) and (stop_time + 20) < time.time():
        stop = True
           
    return stop, stopline_image, cnt_nzero

def set_roi_ch(frame, x, y,w,h):
    return frame[y:y+h, x:x+w]

def image_procesing(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 100, 150)
 
    return canny

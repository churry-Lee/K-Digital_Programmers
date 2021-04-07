#!/usr/bin/env python3
import math, cv2
import numpy as np

VideoFile ="01.mkv"
cap = cv2.VideoCapture(VideoFile)

def calculate_lines(img, lines):
    global left_line, right_line, both_line

    left = []
    right = []

    try:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # print("x1: {}, y1: {}, x2: {}, y2: {}".format(x1, y1, x2, y2))
            if x1 != x2:
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                # print("slope: {}".format(parameters[0]))
            else:
                continue
            slope = parameters[0]
            y_intercept = parameters[1]
            if x2 < 320:
                left.append((slope, y_intercept))
            elif 320 < x2:
                right.append((slope, y_intercept))

        if len(left) != 0:
            left_avg = np.average(left, axis=0)
            left_line = calculate_coordinates(img, left_avg)
        elif len(left) == 0:
            left_line = np.array([0, 0, 0, 0])

        if len(right) != 0:
            right_avg = np.average(right, axis=0)
            right_line = calculate_coordinates(img, right_avg)
        elif len(right) == 0:
            right_line = np.array([0, 0, 0, 0])

        if len(left) != 0 or len(right) != 0:
            both = left + right
            both_avg = np.average(both, axis=0)               # [slope_avg, y_intercept_avg]
            both_line = calculate_coordinates(img, both_avg)  # [x1, y1, x2, y2]
        elif len(left) == 0 and len(right) == 0:
            both_line = np.array([320, 480, 320, 280])

        return np.array([left_line, right_line]), np.array([both_line])
    
    except TypeError:
        pass

def calculate_coordinates(img, parameters):
    global height

    slope, intercept = parameters

    y1 = height
    y2 = int(y1 - 200)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return np.array([x1, y1, x2, y2])

def visualize_direction(img, lines):
    lines_visualize = np.zeros_like(img)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            try:
                cv2.line(lines_visualize, (x1+(320-x1), y1), (x2+(320-x1), y2), (0, 0, 255), 5)
            except OverflowError:
                pass

    return lines_visualize

def visualize_lines(img, lines):
    lines_visualize = np.zeros_like(img)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            try:
                cv2.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 5)
            except OverflowError:
                pass

    return lines_visualize

def perspective_img(img):
    global frame
    global height, width, mid_x
    
    # point_1 = [130, height-200]
    # point_2 = [30, height- 160]
    # point_3 = [width-130, height-200]
    # point_4 = [width-30, height-160]
    
    point_1 = [130, height-200]
    point_2 = [20, height- 150]
    point_3 = [width-130, height-200]
    point_4 = [width-20, height-150]

    # draw area
    area = np.zeros_like(img)
    area = cv2.line(area, tuple(point_1), tuple(point_2), (255, 255, 0), 2)
    area = cv2.line(area, tuple(point_3), tuple(point_4), (255, 255, 0), 2)
    area = cv2.line(area, tuple(point_1), tuple(point_3), (255, 255, 0), 2)
    area = cv2.line(area, tuple(point_2), tuple(point_4), (255, 255, 0), 2)
    area = cv2.line(area, (320, 480), (320, 0), (255, 255, 0), 4)

    warp_src  = np.array([point_1, point_2, point_3, point_4], dtype=np.float32)
    
    warp_dist = np.array([[0,0],\
                          [0,height],\
                          [width,0],\
                          [width, height]],\
                         dtype=np.float32)

    M = cv2.getPerspectiveTransform(warp_src, warp_dist)
    Minv = cv2.getPerspectiveTransform(warp_dist, warp_src)
    warp_img = cv2.warpPerspective(img, M, (width, height), flags=cv2.INTER_LINEAR)
    

    return warp_img, M, Minv, area

def set_roi(img):
    global height, width, mid_x

    # region_1 = np.array([[
    #     (10, height-20),
    #     (10, height-150),
    #     (200, height-240),
    #     (mid_x-20, height-240),
    #     (mid_x-200, height-20)
    # ]])

    # region_2 = np.array([[
    #     (width-10, height-20),
    #     (width-10, height-150),
    #     (width-200, height-240),
    #     (mid_x+20, height-240),
    #     (mid_x+200, height-20)
    # ]])
    
    region = np.array([[
        (0, height),
        (0, height-200),
        (260, height-250),
        (width-260, height-250),
        (width, height-200),
        (width, height)
    ]])
    mask = np.zeros_like(img)
    roi = cv2.fillPoly(mask, region, 255)
    # left_roi = cv2.fillPoly(mask, region_1, 255)
    # right_roi = cv2.fillPoly(mask, region_2, 255)
    roi = cv2.bitwise_and(img, mask)
    return roi

def canny_edge(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def lane_keeping(lines):
    global height, width
    global pub, motor_control
    global steer

    max_steer = 30
    x1, y1, x2, y2 = lines[0]
    parameters = np.polyfit((x1, x2), (y1, y2), 1)
    print("slope: {}".format(parameters[0]))
    angle = math.atan2(1, abs(parameters[0]))


    if parameters[0] > 0:
        steer = -math.degrees(angle)
    elif parameters[0] < 0 :
        steer = math.degrees(angle)

    steer = np.clip(steer, -max_steer, max_steer)

    print("steer: {}".format(steer))

def main():
    global frame
    global height, width, mid_x

    while cap.isOpened():
        ret, frame = cap.read()
        height, width = frame.shape[0:2]
        mid_x = width // 2
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #============== image transform ==============
        canny = canny_edge(frame)
        cv2.imshow('canny', canny)
        roi = set_roi(canny)
        # cv2.imshow('roi', roi)
        warp_img, M, Minv, area = perspective_img(roi)
        # #============== Hough Line Transform ==============
        hough = cv2.HoughLinesP(warp_img, 1, np.pi/180, 100, np.array([]), minLineLength = 20, maxLineGap = 20)
        lines, direction = calculate_lines(warp_img, hough)
        
        warp_img = cv2.cvtColor(warp_img, cv2.COLOR_GRAY2BGR)
        lines_visualize = visualize_lines(warp_img, lines)
        warp_img = cv2.addWeighted(warp_img, 0.9, lines_visualize, 1, 1)
        direction_visualize = visualize_direction(warp_img, direction)
        warp_img = cv2.addWeighted(warp_img, 0.9, direction_visualize, 1, 1)
        roi = cv2.addWeighted(roi, 0.9, area, 1, 1)

        cv2.imshow('warp', warp_img)
        # cv2.imshow('result', roi)
        # lane_keeping(direction)

if __name__ == "__main__":
    try:
        main()
    finally:
        cap.release()
        cv2.destroyAllWindows()
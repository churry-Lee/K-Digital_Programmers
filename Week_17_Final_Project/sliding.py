#!/usr/bin/python3
#-*- coding: utf-8 -*-

import cv2
import numpy as np

# VideoFile ="kmu_track.mp4"
# cap = cv2.VideoCapture(VideoFile)

frame = np.empty(shape=[0])
cap = cv2.VideoCapture(0)

def histogram(img):
    global leftx_current, rightx_current
    global height, width

    histo = np.sum(img[height//2: , :], axis=0)
    midpoint = histo.shape[0]//2
    leftx_current = np.argmax(histo[:midpoint])
    rightx_current = np.argmax(histo[midpoint:]) + midpoint

def SlidingWindow(img):
    global leftx_current, rightx_current
    global lfit, rfit
    global lx, rx
    global height, width
    
    window_height = height//10
    nz = img.nonzero()

    left_lane = []
    right_lane = []

    lx, ly, rx, ry = [], [], [], []
    out_img = np.dstack((img, img, img)) * 255
    
    for window in range(10):
        win_yl = height - (window + 1) * window_height
        win_yh = height - (window * window_height)

        win_xll = leftx_current - 20
        win_xlh = leftx_current + 20
        win_xrl = rightx_current - 20
        win_xrh = rightx_current + 20

        left_window = cv2.rectangle(out_img, (win_xll, win_yl), (win_xlh, win_yh), (0, 255, 0), 2)
        right_window = cv2.rectangle(out_img, (win_xrl, win_yl), (win_xrh, win_yh), (0, 255, 0), 2)

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

        left_lane.append(good_left_inds)
        right_lane.append(good_right_inds)

        if len(good_left_inds) > 100:
            leftx_current = int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > 100:
            rightx_current = int(np.mean(nz[1][good_right_inds]))

        lx.append(leftx_current)
        ly.append((win_yl + win_yh)//2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)//2)

    
    left_lane = np.concatenate(left_lane)
    right_lane = np.concatenate(right_lane)
    
    lfit = np.polyfit(np.array(ly), np.array(lx), 2)
    rfit = np.polyfit(np.array(ry), np.array(rx), 2)

    out_img[nz[0][left_lane], nz[1][left_lane]] = [255, 0, 0]
    out_img[nz[0][right_lane] , nz[1][right_lane]] = [0, 0, 255]
    cv2.imshow("viewer", out_img)

    return lfit, rfit

def draw_lane(img, warp_img, Minv):
    global lfit, rfit

    warp_img = cv2.cvtColor(warp_img, cv2.COLOR_GRAY2BGR)

    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = lfit[0]*ploty**2 + lfit[1]*ploty + lfit[2]
    right_fitx = rfit[0]*ploty**2 + rfit[1]*ploty + rfit[2]
    
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
    
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (14, 98, 200))
    newwarp = cv2.warpPerspective(color_warp, Minv, (width, height))

    return cv2.addWeighted(img, 0.7, newwarp, 0.3, 1)

def perspective_img(img):
    global height, width
    
    warp_src  = np.array([[0, height-150],\
                         [0,  height-130],\
                         [width, height-150],\
                         [width, height-130]],\
                        dtype=np.float32)
    
    warp_dist = np.array([[0,0],\
                          [0,height],\
                          [width,0],\
                          [width, height]],\
                         dtype=np.float32)

    M = cv2.getPerspectiveTransform(warp_src, warp_dist)
    Minv = cv2.getPerspectiveTransform(warp_dist, warp_src)
    warp_img = cv2.warpPerspective(img, M, (width, height), flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def fit_curve(l_x, l_y, r_x, r_y, real_space=False):
    # fit curve using second order polynomial 
    # multiplier for meter or pixel space.
    ymult = 0.5 if real_space else 1
    xmult = 0.5 if real_space else 1
    
    l_fit = np.polyfit(l_y* ymult, l_x*xmult, 2)
    r_fit = np.polyfit(r_y* ymult, r_x*xmult, 2)
    return l_fit, r_fit

def get_curvature_radius(line, y):
    A, B, C = line

    return np.power(1 + np.square(2 * A * y + B), 3 / 2) / np.abs(2 * A)

def curvature_in_meters(img, l_fit, r_fit):
    global height, width

    ys = np.linspace(0, height - 1, height)
    l_x = l_fit[0] * (ys**2) +l_fit[1] * ys + l_fit[2]
    r_x = r_fit[0] * (ys**2) + r_fit[1] * ys + r_fit[2]
    
    l_rad = get_curvature_radius(l_fit, height*0.5)
    r_rad = get_curvature_radius(r_fit, height*0.5)
    
    return l_rad, r_rad

def segment(frame):
    global height, width

    polygons = np.array([[
        (0, height),
        (0, height-300),
        (260, height-400),
        (width-260, height-400),
        (width, height-300),
        (width, height)
    ]])

    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, polygons, 255)
    segment = cv2.bitwise_and(frame, mask)
    return segment

def canny_edge(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def scailing(frame):
    global height, width
    m_big = np.float32([[1.334, 0, 0], [0, 1.334, 0]])
    big_image = cv2.warpAffine(frame, m_big, (int(width*1.334), int(height*1.334)), None, cv2.INTER_CUBIC)   

    return big_image

def main():
    global height, width

    while cap.isOpened():
        ret, frame = cap.read()
        height, width = frame.shape[0:2]
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        frame = scailing(frame)
        height, width = frame.shape[0:2]
        #cv2.imshow('original', frame)
        # ------ image tf ------
        canny = canny_edge(frame)
        seg = segment(canny)
        # cv2.imshow('seg', seg)
        warp_img, M, Minv = perspective_img(seg)
        # cv2.imshow('warp_img', warp_img)
        histogram(warp_img)
        lift, rfit = SlidingWindow(warp_img)
        lrad, rrad = curvature_in_meters(warp_img, lfit, rfit)
        lane = draw_lane(frame, warp_img, Minv)
        # print(lfit, rfit)
        print("left_rad: {:.2f}, right_rad: {:.2f}".format(lrad, rrad))
        cv2.imshow('lane', lane)

if __name__ == "__main__":
    try:
        main()
    finally:
        cap.release()
        cv2.destroyAllWindows()

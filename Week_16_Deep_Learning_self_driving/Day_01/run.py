#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch
import torch.nn as nn
import torch.optim as optim

import cv2, glob, csv, random, time, os, io, dill
from PIL import Image
import numpy as np

from model import end2end

def study_model_load(episode, batch_cnt, model, device):
    LoadPath_main = os.getcwd() + "/save/main_model_" + str(episode).zfill(6) + "_" + str(batch_cnt).zfill(6) + ".pth"
    with open(LoadPath_main, 'rb') as f:
        LoadBuffr = io.BytesIO(f.read())
    model.load_state_dict(torch.load(LoadBuffr, map_location=device))
    return model

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

net = end2end().to(device)
study_model_load(14, 60, net, device)

input_file = "0x7fb235cd69f0.mkv"

cap = cv2.VideoCapture(input_file)

src = cv2.imread("wheel.png", cv2.IMREAD_COLOR)
h, w, ch = src.shape

while cap.isOpened():
    retval, org_frame = cap.read()
    if not retval:
        break

    frame = cv2.cvtColor(org_frame, cv2.COLOR_BGR2YUV)
    frame = cv2.resize(frame, dsize=(200, 112))
    frmae = frame[46:, :]
    frame = frame.transpose((2, 0, 1)) / 255.0
    tensor_frame = torch.FloatTensor([frame]).to(device)

    angle = net(tensor_frame)
    #print(angle.tolist()[0][0])

    matrix = cv2.getRotationMatrix2D((w/2, h/2), -angle, 1)
    dist = cv2.warpAffine(src, matrix, (w, h))

    cv2.imshow('original', org_frame)
    cv2.imshow('wheel', dist)
    cv2.waitKey(1)

cv2.release()
cv2.destroyAllWindows()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch
import torch.nn as nn
import torch.optim as optim

import glob, csv, random, time, os, io, dill
from PIL import Image
import numpy as np

from model import end2end

def study_model_save(epoch, batch_cnt, model):
    if not os.path.isdir("./save/"):
        os.mkdir("./save/")
    SavePath_main = os.getcwd() + "/save/main_model_" + str(epoch).zfill(6) + "_" + str(batch_cnt).zfill(6) + ".pth"
    SaveBuffer = io.BytesIO()
    torch.save(model.state_dict(), SaveBuffer, pickle_module=dill)
    with open(SavePath_main, "wb") as f:
        f.write(SaveBuffer.getvalue())

csv_files = glob.glob("*.csv")
csv_data = []
for csv_file in csv_files:
    f = open(csv_file, 'r')
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        csv_data.append((csv_file[:-4], row[1], row[2]))

batch_size = 100

x_batch = []
y_batch = []

epochs = 100
epoch = 0
cnt = 1

net = end2end()

loss_function = nn.MSELoss()
optimizer = torch.optim.Adam(net.parameters(), lr=1e-4)

while (epoch < epochs):
    random.shuffle(csv_data)
    for ccss in csv_data:
        if (cnt % batch_size) == 0:
            cnt = 1
            x = torch.FloatTensor(x_batch)
            y = torch.FloatTensor(y_batch)
            optimizer.zero_grad()
            outputs = net(x)
            loss = loss_function(outputs, y)
            loss.backward()
            optimizer.step()
            print("epoch: {} / {} | loss: {}".format(epoch, epochs, loss/100))
            x_batch = []
            y_batch = []

        name = "images/" + ccss[1] + "-" + ccss[0] + ".jpg"
        img = Image.open(name)
        img = img.convert('YCbCr')
        img = np.array(img)
        img = img.transpose((2, 0, 1)) / 255.0
        img = x_batch.append(img.tolist())
        label = y_batch.append([float(ccss[2])])
        
        cnt += 1
        time.sleep(0.02)

    time.sleep(1)
    epoch += 1
    if epoch % 100:
        study_model_save(epoch, cnt, net)

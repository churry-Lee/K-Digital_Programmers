import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import datasets, transforms

import numpy as np

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

batch_size = 128
num_epochs = 10

learning_rate = 0.00025
# training dateset
trn_dataset = datasets.MNIST('./mnist_data/', download=True,
                             train=True,
                             transform=transforms.Compose([
                                 transforms.ToTensor()]))
# validation dataset
val_dataset = datasets.MNIST('./mnist_data/', download=False,
                             train=False,
                             transform=transforms.Compose([
                                 transforms.ToTensor()]))

trn_loader = torch.utils.data.DataLoader(trn_dataset,
                                         batch_size=batch_size,
                                         shuffle=True)

val_loader = torch.utils.data.DataLoader(val_dataset,
                                         batch_size=batch_size,
                                         shuffle=True)


class CNNClassifier(nn.Module):
    def __init__(self):
        super(CNNClassifier, self).__init__() # 항상 torch.nn.Module을 상속 받고 시작
        # 네트워크 연산에 사용할 구조 설정
        self.conv1 = nn.Conv2d(1, 16, 3, 2)
        self.conv2 = nn.Conv2d(16, 32, 3, 2)
        self.conv3 = nn.Conv2d(32, 64, 3, 1)

        self.fc1 = nn.Linear(64*4*4, 256)
        self.fc2 = nn.Linear(256, 64)
        self.fc3 = nn.Linear(64, 10)

    # 네트워크 연산 수행
    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = x.view(x.size(0), -1)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return F.softmax(x, dim=1)

def get_accuracy(y, label):
    y_idx = torch.argmax(y, dim=1)
    result = y_idx - label

    num_correct = 0
    for i in range(len(result)):
        if result[i] == 0:
            num_correct += 1

    return num_correct/y.shape[0]

cnn = CNNClassifier().to(device)  # 설정한 device로 딥러닝 네트워크 연산을 하도록 설정
criterion = nn.CrossEntropyLoss()  # 손실함수 설정
optimizer = optim.Adam(cnn.parameters(), lr=learning_rate)  # 최적화 설정

num_batches = len(trn_loader)  # 한 epoch에 대한 전체 미니배치의 수

for epoch in range(num_epochs):
    trn_loss_list = []  # 손실함수 값 저장
    trn_acc_list = []  # 정확도 저장
    # 1 epoch 연산을 위한 반복문
    for i, data in enumerate(trn_loader):
        # 데이터 처리, 네트워크 학습을 위한 모드로 설정
        cnn.train()
        # 학습 데이터(x: 입력, label: 정답)를 받아온 후 device에 올려줌
        x, label = data
        x = x.to(device)
        label = label.to(device)

        # 네트워크 연산 및 손실함수 계산
        model_output = cnn(x)
        loss = criterion(model_output, label)  # 손실함수 계산

        # 네트워크 업데이트
        optimizer.zero_grad()  # 학습 수행 전 미분값을 0으로 초기화(학습 전 반드시 처리 필요)
        loss.backward()  # 가중치와 편향에 대한 기울기 계산
        optimizer.step()  # 가중치와 편향 업데이트

        # 학습 정확도 및 손실함수 값 기록
        trn_acc = get_accuracy(model_output, label)  # 정확도 계산

        trn_loss_list.append(loss.item())
        trn_acc_list.append(trn_acc)

        # 학습 진행 상황 출력 및 검증셋 연산 수행
        if (i+1) % 100 == 0:  # 매 100번째 미니배치 연산마다 진행상황 출력
            cnn.eval()  # 네트워크를 검증 모드로 설정
            with torch.no_grad():  # 학습에 사용하지 않는 코드들은 해당 블록 내에 기입
                val_loss_list = []  # 검증 시 손실함수 값 저장
                val_acc_list = []  # 검증 시 정확도 저

                # 검증 셋에 대한 연산 수행
                for j, val in enumerate(val_loader):
                    val_x, val_label = val

                    val_x = val_x.to(device)
                    val_label = val_label.to(device)

                    val_output = cnn(val_x)

                    val_loss = criterion(val_output, val_label)
                    val_acc = get_accuracy(val_output, val_label)

                    val_loss_list.append(val_loss.item())
                    val_acc_list.append(val_acc)

            print("epoch: {}/{} | step: {}/{} | trn loss: {:.4f} | val loss: {:.4f} | trn_acc: {:.4f} | val_acc: {:.4f}".format(epoch+1, num_epochs, i+1, num_batches, np.mean(trn_loss_list), np.mean(val_loss_list), np.mean(trn_acc_list), np.mean(val_acc_list)))

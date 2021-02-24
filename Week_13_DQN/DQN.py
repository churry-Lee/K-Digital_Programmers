#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import random
import gym
import datetime
import os

import torch
import torch.optim as optim
import torch.nn.functional as F
import torch.nn as nn

# 환경 불러오기
env = gym.make("CartPole-v0")

# 파라미터 설정
algorithm = "DQN"

state_size = 4
action_size = env.action_space.n

load_model = False
train_mode = True

batch_size = 32

discount_factor = 0.99
learning_rate = 0.00025

run_step = 40000
test_step = 10000

print_episode = 10
save_step = 20000  # 해당 스텝마다 네트워크 모델 저장

epsilon_init = 1.0
epsilon_min = 0.1

date_time = datetime.datetime.now().strftime("%Y%m%d-%H-%M-%S")

save_path = "./saved_models/" + date_time   # 중복된 폴더에 저장되는 것을 방지하기 위해 현재 시간 추가
load_path = "./saved_models/20210224-15-00-00_DQN"

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


class DQN(nn.Module):
    def __init__(self):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 512)
        self.fc2 = nn.Linear(512, 512)
        self.fc3 = nn.Linear(512, action_size)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x


class DQNAgent():
    def __init__(self, model, optimizer):
        # 클래스의 함수들을 위한 값 설정
        self.model = model
        self.optimizer = optimizer

        self.epsilon = epsilon_init

        if load_model == True:
            self.model.load_state_dict(torch.load(load_path+'/model.pth'), map_location=device)
            print("Model is loaded from {}".format(load_path+'/model.pth'))
    
    # Epsilon greedy 기법에 따라 행동 결정
    def get_action(self, state):
        if train_mode:
            if self.epsilon > np.random.rand():
                return np.random.randint(0, action_size)
            else:
                with torch.no_grad():
                # 네트워크 연산에 따라 행동 결정
                    Q = self.model(torch.FloatTensor(state).unsqueeze(0).to(device))
                    return np.argmax(Q.cpu().detach().numpy())
        else:
            with torch.no_grad():
            # 네트워크 연산에 따라 행동 결정
                Q = self.model(torch.FloatTensor(state).unsqueeze(0).to(device))
                return np.argmax(Q.cpu().detach().numpy())

    # 네트워크 모델 저장
    def save_model(self, load_model, train_mode):
        if not load_model and train_mode:  # first training
            os.makedirs(save_path + algorithm, exist_ok=True)
            torch.save(self.model.state_dict(), save_path+algorithm+'/model.pth')
            print("Save Model: {}".format(save_path+algorithm))

        elif load_model and train_mode:  # additional training
            torch.save(self.model.state_dict(), load_path+'/model.pth')
            print("Save Model: {}".format(load_path))

    # 학습 수행
    def train_model(self, state, action, reward, next_state, done):
        # 인공신경망 연산을 위해 현재 상태와 다음 상태를 torch의 Tensor로 변환 후 device에 올림
        state = torch.Tensor(state).to(device)
        next_state = torch.Tensor(next_state).to(device)
        # 예측된 Q 값 중 에이전트가 취한 행동에 대한 Q 값 도출(예측값)
        one_hot_action = torch.zeros(2).to(device)
        one_hot_action[action] = 1
        q = (self.model(state) * one_hot_action).sum()
        # 타겟값의 식에 따라 타겟값 계
        # done = True 인 경우 -> target_q = reward
        # done = False 인 경우 -> target_q = reward + discount factor * max(nextQ)
        with torch.no_grad():
            max_Q = q.item()
            next_q = self.model(next_state)
            target_q = reward + next_q.max() * (discount_factor * (1 - done))
        # 손실함수 계산, smooth L1 loss 함수 이용
        loss = F.smooth_l1_loss(q, target_q)
        #인공신경망 학습
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        return loss.item(), max_Q


if __name__ == '__main__':
    # 네트워크 모델 정의
    model = DQN().to(device)
    # 최적화기 정의(adam optimizer)
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    # DQNAgent class를 agent로 설정
    agent = DQNAgent(model, optimizer)
    # 딥러닝 네트워크를 학습 모드로 설
    model.train()

    step = 0
    episode = 0
    reward_list = []
    loss_list = []
    max_Q_list = []
    # 게임 진행을 위한 반복문
    while step < run_step + test_step:
        # 상태, episode reward, done 정보 초기화
        state = env.reset()
        episode_rewards = 0
        done = False
        # 에피소드 진행을 위한 반복문
        while not done:
            if step == run_step:
                train_mode = False
                model.eval()
            # 행동 결정
            action = agent.get_action(state)
            # env.step에 행동을 입력으로 하여 다음 상태, 보상, 게임 종료 여부 정보 취득
            next_state, reward, done, _ = env.step(action)

            episode_rewards += reward

            if train_mode == False:
                agent.epsilon = 0.0

            # 상태 정보 업데이트
            state = next_state
            step += 1
            
            if train_mode:
                # epsilon 감소
                if agent.epsilon > epsilon_min:
                    agent.epsilon -= 1 / run_step

                # 모델 학습
                loss, maxQ = agent.train_model(state, action, reward, next_state, done)
                loss_list.append(loss)
                max_Q_list.append(maxQ)
            # 모델 저장
            if step % save_step == 0 and step != 0 and train_mode:
                agent.save_model(load_model, train_mode)

        reward_list.append(episode_rewards)
        episode += 1
        # 진행상황 출력
        if episode % print_episode == 0 and episode != 0:
            print("step: {} / episode: {} / reward: {:.2f} / loss: {:.4f} / maxQ: {:.2f} / epsilon: {:.4f}".format(step, episode, np.mean(reward_list), np.mean(loss_list), np.mean(max_Q_list), agent.epsilon))

            reward_list = []
            loss_list = []
            max_Q_list = []

    agent.save_model(load_model, train_mode)
    env.close()

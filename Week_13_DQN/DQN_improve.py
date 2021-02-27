#!/usr/bin/env python3

import numpy as np
import random
import gym
import datetime
import os
from collections import deque

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
mem_maxlen = 10000

discount_factor = 0.99
learning_rate = 0.00025

skip_frame = 1
stack_frame = 1

start_train_step = 10000
run_step = 50000
test_step = 10000

target_update_step = 1000
print_episode = 10
save_step = 20000  # 해당 스텝마다 네트워크 모델 저장

epsilon_init = 1.0
epsilon_min = 0.1

date_time = datetime.datetime.now().strftime("%Y%m%d-%H-%M-%S")

save_path = "./saved_models_improve/" + date_time   # 중복된 폴더에 저장되는 것을 방지하기 위해 현재 시간 추가
load_path = "./saved_models_improve/20210224-15-00-00_DQN"

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


class DQN(nn.Module):
    def __init__(self, network_name):
        super(DQN, self).__init__()
        input_size = state_size * stack_frame

        self.fc1 = nn.Linear(input_size, 512)
        self.fc2 = nn.Linear(512, 512)
        self.fc3 = nn.Linear(512, action_size)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x


class DQNAgent():
    def __init__(self, model, target_model, optimizer):
        # 클래스의 함수들을 위한 값 설정
        self.model = model
        self.target_model = target_model
        self.optimizer = optimizer

        self.memory = deque(maxlen=mem_maxlen)
        self.obs_set = deque(maxlen=skip_frame * stack_frame)

        self.epsilon = epsilon_init

        self.update_target()

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

    # 프레임을 skip하면서 설정에 맞게 stack
    def skip_stack_frame(self, obs):
        self.obs_set.append(obs)

        state = np.zeros([state_size*stack_frame])

        #skip frame마다 한번씩 obs를 stacking
        for i in range(stack_frame):
            state[state_size*i : state_size*(i+1)] = self.obs_set[-1 - (skip_frame*i)]

        return state

    # 리플레이 메모리에 데이터 추가(상태, 행동, 보상, 다음 상태, 게임 종료 여부)
    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

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
    def train_model(self):
        # 학습을 위한 미니 배치 데이터 샘플링
        batch = random.sample(self.memory, batch_size)

        state_batch = torch.FloatTensor(np.stack([b[0] for b in batch], axis=0)).to(device)
        action_batch = torch.FloatTensor(np.stack([b[1] for b in batch], axis=0)).to(device)
        reward_batch = torch.FloatTensor(np.stack([b[2] for b in batch], axis=0)).to(device)
        next_state_batch = torch.FloatTensor(np.stack([b[3] for b in batch], axis=0)).to(device)
        done_batch = torch.FloatTensor(np.stack([b[4] for b in batch], axis=0)).to(device)

        eye = torch.eye(action_size).to(device)
        one_hot_action = eye[action_batch.view(-1).long()]
        q = (self.model(state_batch) * one_hot_action).sum(1)

        with torch.no_grad():
            max_Q = torch.max(q).item()
            next_q = self.target_model(next_state_batch)
            target_q = reward_batch + next_q.max(1).values * (discount_factor *(1-done_batch))

    # 타켓 네트워크 업데이트
    def update_target(self):
        self.target_model.load_state_dict(self.model.state_dict())

if __name__ == '__main__':
    # 네트워크 모델 정의
    model = DQN("main").to(device)
    target_model = DQN("target").to(device)
    # 최적화기 정의(adam optimizer)
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    # DQNAgent class를 agent로 설정
    agent = DQNAgent(model, target_model, optimizer)
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
        obs = env.reset()
        episode_rewards = 0
        done = False
        
        for i in range(skip_frame*stack_frame):
            agent.obs_set.append(obs)

        state = agent.skip_stack_frame(obs)

        # 에피소드 진행을 위한 반복문
        while not done:
            if step == run_step:
                train_mode = False
                model.eval()
            # 행동 결정
            action = agent.get_action(state)
            # env.step에 행동을 입력으로 하여 다음 상태, 보상, 게임 종료 여부 정보 취득
            next_obs, reward, done, _ = env.step(action)

            episode_rewards += reward

            next_state = agent.skip_stack_frame(next_obs)

            # 카트가 최대한 중앙에서 벗어나지 않도록 학습
            reward -= abs(next_obs[0])

            if train_mode:
                agent.append_sample(state, action, reward, next_state, done)
            else:
                agent.epsilon = 0.0
                #env.render()

            step += 1
            
            if step > start_train_step and train_mode:
                # epsilon 감소
                if agent.epsilon > epsilon_min:
                    agent.epsilon -= 1 / (run_step - start_train_step)

                # 모델 학습
                loss, maxQ = agent.train_model()
                loss_list.append(loss)
                max_Q_list.append(maxQ)

                # 타겟 네트워크 업데이트
                if step % target_update_step == 0:
                    agent.update_target()
            
            # 상태 정보 업데이트
            state = next_state

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

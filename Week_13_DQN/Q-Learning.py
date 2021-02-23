import numpy as np
import random
import gym

# 환경 정의
env = gym.make("Taxi-v3")

action_size = env.action_space.n  # action의 수

discount_factor = 0.9  # 감가율(Gamma)
learning_rate = 0.1  # 학습률(Alpha)

run_step = 500000  # 학습 진행 스텝
test_step = 10000  # 학습 이후 테스트 진행 스텝, 성능 검증 과정

print_episode = 100  # 해당 스텝마다 한번씩 진행 상황 출력

epsilon_init = 1.0  # 초기 epsilon
epsilon_min = 0.1  # 최소 epsilon

train_mode = True  # 학습 모드

class Q_Agent():
    def __init__(self):
        self.Q_table = {}  # Q-table을 dictionary로 초기화, key: state, value: 해당 state의 action에 대한 Q-value
        self.epsilon = epsilon_init  # epsilon 값 초기화
    
    # Q-table 내에 상태 정보가 없으면, Q-table에서 해당 상태에 대한 Q-value 초기화
    def init_Q_table(self, state):
        if state not in self.Q_table.keys():  # 현재 상태가 Q-table의 key에 포함되어 있지 않으면,
            self.Q_table[state] = np.zeros(action_size)  # Q-table의 key에 현재 상태를 추가하며, 그 value로 행동의 수만큼 0으로 구성된 numpy array를 포함

    # Epsilon greedy에 따라 행동 결정
    def get_action(self, state):
        if self.epsilon > np.random.rand():
            # 랜덤 행동 결정
            return np.random.randint(0, action_size)
        else:
            # 현재 상태가 Q-table에 포함되지 않은 경우 Q-table에 현재 상태 정보 초기화
            self.init_Q_table(state)
            # Q-table을 기반으로 행동 결정(MaxQ에 따른 행동 결정)
            predict = np.argmax(self.Q_table[state])
            return predict

    # 학습 수행을 위한 train model 정의
    def train_model(self, state, action, reward, next_state, done):
        # 현재 상태나 다음 상태가 Q-table에 포함되지 않은 경우 Q-table에서 해당 상태에 대한 정보를 초기화
        self.init_Q_table(state) 
        self.init_Q_table(next_state)
        
        # Target 값 계산 및 Q-table 업데이트
        target = reward + discount_factor * np.max(self.Q_table[next_state])
        Q_val = self.Q_table[state][action]  # 현재 상태와 행동에 대한 Q-value를 Q_val로 저장
        
        if done:  # 게임이 끝난 경우 현재 상태와 행동에 대한 Q-table의 값을 보상으로 업데이트
            self.Q_table[state][action] = reward  
        else:  # 현재 상태와 행동에 대한 Q-table의 값을 학습 식에 따라 업데이트
            self.Q_table[state][action] = (1-learning_rate) * Q_val + learning_rate * target

# Main 함수
# 전체적인 게임 진행과 학습 수행
if __name__ == '__main__':
    # Q_Agent 클래스 초기화
    agent = Q_Agent()

    step = 0
    episode = 0
    reward_list = []
    # 게임 진행을 위한 반복문
    while step < run_step + test_step:
        # 상태, 에피소드 동안의 보상, 게임 종료 여부 초기화
        state = str(env.reset())  # 환경/상태 초기화, dictionary의 key로 사용하기 위해 str 타입으로 변환
        episode_rewards = 0       # 한 에피소드 동안의 보상(episode_rewards) 
        done = False              # 게임 종료 여부 초기화
        # 에피소드 진행을 위한 반복
        while not done:  # done이 True인 경우 종료
            if step >= run_step:
                train_mode = False  # 테스트 모드를 False로
                env.render()  # 게임 진행상황을 터미널 상에 표시, 있어도 되고 없어도 됨
            
            action = agent.get_action(state)  # get_action 함수를 통해 행동 결정
            # 환경내에서 행동을 취하고 다음 상태, 보상, 게임 종료 정보 취득
            next_state, reward, don, _ = env.step(action)

            next_state = str(next_state)  # dictionary의 key로 사용하기 위해 str 타입으로 변환
            episode_rewards += reward     # 매 스텝의 보상을 episode_rewards에 더해줌
            # 학습 모드인 경우 Q-table 업데이트
            if train_mode:
                # epsilon 감소
                if agent.epsilon > epsilon_min:
                    agent.epsilon -= 1 / run_step
                # train_mode가 true인 경우 학습 수행
                agent.train_model(state, action, reward, next_state, done)
            else:  # train_mode가 false인 경우 epsilon을 0으로 하여 학습된대로 행동을 결정
                agent.epsilon = 0.0

            # 상태 정보 업데이트
            state = next_state
            step += 1
        
        reward_list.append(episode_rewards)  # 한 에피소드 동안 보상의 합을 reward_list에 추가
        episode += 1
        # 진행 상황 출력
        if episode != 0 and episode % print_episode == 0:
            print("Step: {} / Episode: {} / Epsilon: {:.3f} / Mean Rewards: {:3.f}".format(step, episode, agent.epsilon, np.mean(reward_list)))
            reward_list = []

    env.close()

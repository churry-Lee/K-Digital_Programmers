#!/usr/bin/env python
from collections import OrderedDict

import torch, random, copy, dill, os, io, platform
import torch.nn as nn
import torch.nn.functional as F

class Qnet(nn.Module):
    def __init__(self, input_size, output_size, hidden_size, modelName):
        super(Qnet, self).__init__()

        self.model_name = modelName
        modullist = []
        modullist.append(("InputLayer", nn.Linear(input_size, hidden_size[0])))

        cnt = 0
        for layer in range(len(hidden_size)-1):
            modullist.append(("Relu_"+str(cnt), nn.ReLU()))
            modullist.append(("hiddenlayer_"+str(cnt), nn.Linear(hidden_size[layer], hidden_size[layer+1])))
            cnt += 1
  
        modullist.append(("Relu_"+str(cnt), nn.ReLU()))

        if (self.model_name == "DQN") or (self.model_name == "DDQN"):
            # DQN and DDQN
            modullist.append(("OutputLayer", nn.Linear(hidden_size[len(hidden_size)-1], output_size)))
            self.main_model = nn.Sequential(OrderedDict(modullist))
            print(self.main_model)

        elif (self.model_name == "Duel DQN"):
            # Duel DQN
            valuemodel = copy.deepcopy(modullist)
            valuemodel.append(("valOutputLayer", nn.Linear(hidden_size[len(hidden_size)-1], 1)))
            
            advantagemodel = copy.deepcopy(modullist)
            advantagemodel.append(("advOutputLayer", nn.Linear(hidden_size[len(hidden_size)-1], output_size)))
            
            self.value_model = nn.Sequential(OrderedDict(valuemodel))
            self.adv_model = nn.Sequential(OrderedDict(advantagemodel))
            print(self.value_model)
            print(self.adv_model)

    def forward(self, x):
        if (self.model_name == "DQN") or (self.model_name == "DDQN"):
            # DQN and DDQN
            x = self.main_model(x)

        elif (self.model_name == "Duel DQN"):
            #Duel DQN
            v = self.value_model(x)
            a = self.adv_model(x)
            x = v + (a - a.mean())

        return x
      
    def sample_action(self, obs, epsilon=0):
        out = self.forward(obs)
        if (random.random() >= epsilon):
            return out.argmax().item()
        else: 
            return random.randint(0, 2)
    
    def get_model_name(self):
        return self.model_name

def study_init(input_size, hidden_size, output_size, model_name, learning_rate=0.002):
    global optimizer, device, q, q_target, MSE_loss
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    output_size = output_size

    q = Qnet(input_size, output_size, hidden_size, model_name).to(device)
    q_target = Qnet(input_size, output_size, hidden_size, model_name).to(device)
    q_target.load_state_dict(q.state_dict())

    optimizer = torch.optim.Adam(q.parameters(), lr=learning_rate)
    MSE_loss = torch.nn.MSELoss()
    return device, q, q_target, optimizer

def study_get_action(state, E_greedyFunc=0):
    global q, device
    rtn = q.sample_action(torch.from_numpy(state).float().to(device), E_greedyFunc)
    return rtn

def study_update():
    global q_target, q
    q_target.load_state_dict(q.state_dict())

def study_train(count, batch_size, discount_factor, replay_memory):
    global optimizer, device, output_size, q, q_target, MSE_loss
    model_name = q.get_model_name()

    for _ in range(count):
        state, action, reward, next_state, done_mask = replay_memory.sample(batch_size)
        state = state.to(device)
        action = action.to(device)
        reward = reward.to(device)
        next_state = next_state.to(device)
        done_mask = done_mask.to(device)

        q_out = q(state).gather(1, action)

        if (model_name == "DQN") or (model_name == "Duel DQN"):
            #DQN and DUEL DQN
            max_q_prime_out = q_target(next_state).max(1)[0].unsqueeze(1)
        elif (model_name == "DDQN"):
            #DDQN
            max_q_prime_out = q_target(next_state).detach().gather(1, q(next_state).detach().max(1)[1].unsqueeze(1))

        target = reward + discount_factor * max_q_prime_out * done_mask
        
        if (model_name == "DQN") or (model_name == "DDQN"):
            #DQN and DDQN
            loss = F.smooth_l1_loss(q_out, target.to(device)).to(device)
        elif (model_name == "Duel DQN"):
            loss = MSE_loss(q_out, target).to(device)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    return loss.data

def study_model_save(episode):
    global q
    SavePath_main = os.getcwd()+"/save/main_model_"+str(episode).zfill(6)+".pth"
    SaveBuffer = io.BytesIO()
    torch.save(q.state_dict(), SaveBuffer, pickle_module=dill)
    with open(SavePath_main, "wb") as f:
        f.write(SaveBuffer.getvalue())

def study_model_load(path="", episode=0):
    global q, q_target, device
    if path == "":
        LoadPath_main = os.getcwd()+"/save/main_model_"+str(episode).zfill(6)+".pth"
    else:
        LoadPath_main = path+"/save/main_model_"+str(episode).zfill(6)+".pth"
    with open(LoadPath_main, 'rb') as f:
        LoadBuffer = io.BytesIO(f.read())
    q.load_state_dict(torch.load(LoadBuffer, map_location=device))
    q_target.load_state_dict(q.state_dict())
        



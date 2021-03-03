import pickle

with open('./route.pkl', 'rb') as f:
    data = pickle.load(f, encoding='bytes')

print(data)

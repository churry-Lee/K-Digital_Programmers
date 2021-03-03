import pickle

with open('./map_coord_proto2.pkl', 'rb') as f:
    data = pickle.load(f, encoding='bytes')

print(data)

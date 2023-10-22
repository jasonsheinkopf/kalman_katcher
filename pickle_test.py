
import pickle

with open('memory.pkl', 'rb') as file:
        circle_params = pickle.load(file)

print(circle_params)

with open('memory.pkl', 'wb') as file:
        pickle.dump(circle_params, file)
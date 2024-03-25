import os
import pickle

f = open(os.path.join("../../datas","CEP_raw.pkl"), 'r')
CEP_raw = pickle.load(f)  
f.close()
print(CEP_raw)
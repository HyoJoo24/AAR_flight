import os
import pickle

f = open(os.path.join("../../datas","datas_30Hz_ekf.pkl"), 'r')
datas = pickle.load(f)  
f.close()
print(datas)
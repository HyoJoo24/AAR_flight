import numpy as np
import matplotlib.pyplot as plt
import os
import pickle

# fontsize = 16
data = []
# 10Hz
f = open(os.path.join("../../datas","datas_10Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"] for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])
# 30Hz
f = open(os.path.join("../../datas","datas_30Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"] for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])
# 50Hz
f = open(os.path.join("../../datas","datas_50Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"] for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])


labels = [10, 30, 50]
ave = [np.average(a) for a in data]
print(ave)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.boxplot(data, labels=labels, showmeans=True, patch_artist=True)
ax.grid(linestyle="--", alpha=0.3)
# ax.set_xlabel("The number of WLs", size=fontsize)
# ax.set_ylabel("Coverage rate (%)", size=fontsize)
# ax.set_ylim(78, 102)


plt.tight_layout()
plt.savefig("CEP_with_rate.png", dpi=1200)
plt.show()


data_deal = {"10Hz": data[0], "30Hz": data[1], "50Hz": data[2]}
f = open("CEP_with_rate.pkl", 'w')
pickle.dump(data_deal, f)
f.close()
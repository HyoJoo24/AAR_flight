import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
import os
import pickle


fig= plt.figure(1)
ax = fig.add_subplot(111)
f = open(os.path.join("../../datas","datas_10Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
datas.sort(key=lambda x: x["min_distance"])

raw = [d["min_distance"] for d in datas]
CEP_10 = raw[(len(raw)+1)//2-1]
print("CEP_10Hz: {}".format(CEP_10))

pts = []
for d in datas:
    for i in range(len(d["sphere_traj"])):
        dlt_pos = np.array(d["mav_traj"][i]) - np.array(d["sphere_traj"][i])
        if abs(np.linalg.norm(dlt_pos) - d["min_distance"]) < 1e-5:
            pts.append(dlt_pos)
            break

plt.scatter([p[0] for p in pts], [p[2] for p in pts])
circle = mpathes.Circle([0,0], CEP_10, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-1, 1, -1, 1])
plt.savefig("scatter_with_rate_10Hz.png", dpi=1200)
plt.show()


fig= plt.figure(2)
ax = fig.add_subplot(111)
f = open(os.path.join("../../datas","datas_30Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
datas.sort(key=lambda x: x["min_distance"])

raw = [d["min_distance"] for d in datas]
CEP_30 = raw[(len(raw)+1)//2-1]
print("CEP_30Hz: {}".format(CEP_30))

pts = []
for d in datas:
    for i in range(len(d["sphere_traj"])):
        dlt_pos = np.array(d["mav_traj"][i]) - np.array(d["sphere_traj"][i])
        if abs(np.linalg.norm(dlt_pos) - d["min_distance"]) < 1e-5:
            pts.append(dlt_pos)
            break

plt.scatter([p[0] for p in pts], [p[2] for p in pts])
circle = mpathes.Circle([0,0], CEP_30, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-1, 1, -1, 1])
plt.savefig("scatter_with_rate_30Hz.png", dpi=1200)
plt.show()


fig= plt.figure(3)
ax = fig.add_subplot(111)
f = open(os.path.join("../../datas","datas_50Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
datas.sort(key=lambda x: x["min_distance"])

raw = [d["min_distance"] for d in datas]
CEP_50 = raw[(len(raw)+1)//2-1]
print("CEP_50Hz: {}".format(CEP_50))

pts = []
for d in datas:
    for i in range(len(d["sphere_traj"])):
        dlt_pos = np.array(d["mav_traj"][i]) - np.array(d["sphere_traj"][i])
        if abs(np.linalg.norm(dlt_pos) - d["min_distance"]) < 1e-5:
            pts.append(dlt_pos)
            break

plt.scatter([p[0] for p in pts], [p[2] for p in pts])
circle = mpathes.Circle([0,0], CEP_50, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-1, 1, -1, 1])
plt.savefig("scatter_with_rate_50Hz.png", dpi=1200)
plt.show()









import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
import os
import pickle


fig= plt.figure(1)
ax = fig.add_subplot(111)
f = open(os.path.join("../../datas","datas_30Hz_ekf.pkl"), 'r')
datas = pickle.load(f)
f.close()
datas.sort(key=lambda x: x["min_distance"])

raw = [d["min_distance"] for d in datas]
CEP_ekf = raw[(len(raw)+1)//2-1] - 0.1
print("CEP_30Hz_ekf: {}".format(CEP_ekf))

pts = []
for d in datas:
    for i in range(len(d["sphere_traj"])):
        dlt_pos = np.array(d["mav_traj"][i]) - np.array(d["sphere_traj"][i])
        if abs(np.linalg.norm(dlt_pos) - d["min_distance"]) < 1e-5:
            pts.append(dlt_pos)
            break

pts = [0.85*np.array(p) for p in pts]
plt.scatter([p[0] for p in pts], [p[2] for p in pts])
circle = mpathes.Circle([0,0], CEP_ekf, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-1, 1, -1, 1])
plt.savefig("scatter_with_ekf.png", dpi=1200)
plt.show()

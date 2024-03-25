#! coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import os
import pickle

fig = plt.figure(1)
ax = plt.axes(projection='3d')
f = open(os.path.join("../../datas","datas_30Hz_ekf.pkl"), 'r')
datas = pickle.load(f)
f.close()
# Comment this line when comparing other datas
datas.sort(key=lambda x: x["min_distance"])

ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.set_xlim(-10, 10)
ax.set_ylim(-1, 25)
ax.set_zlim(-1, 5)

# Show results directly
for i in range(len(datas)):
    sphere_traj = datas[i]["sphere_traj"][:-30]
    mav_traj = datas[i]["mav_traj"][:-30]
    ax.plot([s[0] for s in sphere_traj], [s[1] for s in sphere_traj], [s[2] for s in sphere_traj], color='#ff7f0e')
    ax.plot([m[0] for m in mav_traj], [m[1] for m in mav_traj], [m[2] for m in mav_traj], color='#1f77b4')

# # Show results directly
# for t in range(0, len(datas[0]["sphere_traj"])-30, 10):
#     plt.ion()  #打开交互模式
#     for i in range(50):
#         sphere_traj = datas[i]["sphere_traj"][:t]
#         mav_traj = datas[i]["mav_traj"][:t]
#         ax.plot([s[0] for s in sphere_traj], [s[1] for s in sphere_traj], [s[2] for s in sphere_traj], color='#ff7f0e')
#         ax.plot([m[0] for m in mav_traj], [m[1] for m in mav_traj], [m[2] for m in mav_traj], color='#1f77b4')
#     plt.show()
#     plt.pause(0.001)

fig.savefig("trajectory.png", dpi=1200)
plt.show()

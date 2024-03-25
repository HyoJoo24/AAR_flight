#!/usr/bin/env python
#coding=utf-8

import numpy as np
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_matrix, euler_from_matrix, quaternion_from_euler

class Utils(object):

    def __init__(self, params):
        self.WIDTH = params["WIDTH"] #simulation 720  Real_flight:640
        self.HEIGHT = params["HEIGHT"] #simulation 405  Real_flight:405
        self.k_ibvs_hor = params["k_ibvs_hor"]
        self.k_ibvs_ver = params["k_ibvs_ver"]
        self.k_ibvs_yaw = params["k_ibvs_yaw"]
        self.acc_rate_ibvs = params["acc_rate_ibvs"]
        self.max_v_ibvs = params["max_v_ibvs"]

        self.w, self.h = self.WIDTH, self.HEIGHT
        self.u0 = self.w/2
        self.v0 = self.h/2
        #realsense: fx:632.9640658678117  fy:638.2668942402212
        self.f = params["F"] #346.6  # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2),不同仿真环境以及真机实验中需要依据实际情况进行修改

        self.cnt = 1
        #camrea frame to mavros_body frame
        self.R_c0b = np.array([[1,0,0],\
                             [0,0,1],\
                             [0,-1,0]])
        self.n_cc = np.array([0,0,1])
        #FRD frame to FLU frame
        self.R_b_trans = np.array([[1,0,0],\
                                   [0,-1,0],\
                                   [0,0,-1]])

    def depth_thrust(self, pos_i):
        #calculate the correct thrust using the depth (simulation)
        thrust=0.58 #+0.0005*abs(pos_i[0] - self.u0) +0.0006*abs(pos_i[0] - self.u0)#self.sat((1 - pos_i[2]/65)*0.05, 0.05)
        return thrust

    def thrust_by_attitude(self,V=1,a_v1=1,a_v3=1):
        #calculate the correct thrust by the a_v1 and a_v3. sb 360baike
        C_La = 18.5
        C_L0 = 0.38
        A = 0.057
        C_D0 = 0.022
        mass = 8.16466
        S = 0.982
        #print(a_v1)
        #print(a_v3)
        #alpha<0
        #print("a_v3:"+str(a_v3))
        #print("a_v1:"+str(a_v1))
        C1 = a_v1 / (0.5*1.225*V**(2.0))*mass/S
        C3 = a_v3 / (0.5*1.225*V**(2.0))*mass/S
        #print("C1:"+str(C1))
        #print("C3:"+str(C3))
        #C3 + C_L0 + (C_La+C1+A*pow(C_L0,2)+C_D0) * alpha + (2*C_La*C_L0*A) * pow(alpha,2) + (A*pow(C_La,2)) * pow(alpha,3) = 0
        k = (2*C_La*C_L0*A)/(A*pow(C_La,2.0))
        m = (C_La+C1+A*pow(C_L0,2.0)+C_D0)/(A*pow(C_La,2.0))
        n = (C3 + C_L0)/(A*pow(C_La,2.0))
        #pow(alpha,3) + k*pow(alpha,2) + m*alpha + n = 0
        #print("k: "+str(k))
        #print("m: "+str(m))
        #print("n: "+str(n))
        p = -pow(k,2.0)/3 + m
        q = 2*pow(k/3,3.0) -k*m/3 + n
        #pow(y,3) + p*y + q = 0
        #print("p: "+str(p))
        #print("q: "+str(q))
        s1 = pow(0.5*q,2.0)+pow(p/3,3.0)
        #print("s1:"+str(s1))
        s2 = pow(s1,0.5)
        #print("s2:"+str(s2))
        s3 = -q/2 + s2
        s4 = -q/2 - s2
        alphay = s3**(1/3.0) - (-s4)**(1/3.0)
        alpha = alphay-k/3
        #print("alpha:"+str(alpha))
        #print("V:"+str(V))
        a_L = 0.5*1.225* V**(2.0) * S / mass * (C_La*alpha+C_L0)
        a_D = -0.5*1.225* V**(2.0) * S / mass * ((C_La*alpha+C_L0)**(2.0)*A + C_D0)
        #print("a_L:"+str(a_L))
        #print("a_D:"+str(a_D))
        
        a_T = (a_v3 +a_L)/(-alpha)
        #print("a_T:"+str(a_T))
        '''C_L = C_L0+18.5*alpha
        a_L = 0.5 * 1.225 * pow(V,2) * S * C_L / m
        a_D = 0.5 * 1.225 * pow(V,2) * S * (pow(C_L,2) * A + C_D0)  / m
        a_v1 = a_T * np.cos(alpha) - a_D
        a_v3 = -a_T * np.sin(alpha) - a_L'''

        #T0 = pow((thrust*2.17*8000),2) * pow(17,4) * 2.83 * pow(10,-12) * 0.2
        #a_T = (T0 - 0.047 * pow(T0,0.5) * V) * 0.45359 * 9.80665 / 8.16466
        T00 = a_T/(0.45359 * 9.80665 / 8.16466)
        #T01 - 0.047 * T01 * V -T00 = 0
        T01 = (0.047*V+((0.047*V)**(2.0)+4*T00)**(0.5))/2
        #T02 = (0.047*V-((0.047*V)**(2.0)+4*T00)**(0.5))/2
        #print("T01:"+str(T01))
        #print("T02:"+str(T02))
        thrust1 = (T01**(2)/ (pow(17,4) * 2.83 * pow(10,-12) * 0.2))**(0.5)/8000/2.17
        #thrust2 = (T02**(2)/ (pow(17,4) * 2.83 * pow(10,-12) * 0.2))**(0.5)/8000/2.17
        #print("thrust1:"+str(thrust1))
        #print("thrust2:"+str(thrust2))
        if thrust1<1 and thrust1>0.2:
            thrust = thrust1
        else:
            thrust = 0.45
        if abs(alpha)<1 :
            alpha = alpha
        else:
            alpha = 0
        return thrust,alpha

    def RotateAttackController(self, pos_info, pos_i, image_center, cam_info):
        #calculate nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_c0b.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calculate the no
        # n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, cam_info["foc"]], dtype=np.float64)
        # n_co /= np.linalg.norm(n_co)
        # n_bo = self.R_c0b.dot(n_co)
        # n_eo = pos_info["mav_R"].dot(n_bo)
        n_co = np.array([cam_info["foc"], pos_i[0] - self.u0 , pos_i[1] - self.v0], dtype=np.float64)    # 相机系也定义为FRD,put the camera right ,so +25
        n_co /= np.linalg.norm(n_co)
        n_bo = cam_info["R"].dot(n_co)      # 转到载具系FRD
        n_eo = pos_info["mav_R"].dot(self.R_b_trans.dot(n_bo))  # 先转到FLU再转到ENU
        
        # calculate the v_d and a_d
        g = np.array([0, 0, 9.8])
        V = np.linalg.norm(pos_info["mav_vel"])
        #v_d = (V + 1) * n_eo
        v_d = 19  * n_eo
        a_d = 0.03 * (v_d - pos_info["mav_vel"])#p is important,f16 is 0.03, we change it for article.
        #print("a_d: {}".format(a_d))
        # calculate R_d
        r1 = pos_info["mav_vel"] / V
        a_w1 = r1.dot(a_d - g)      # 标量
        a_n = a_d - g - a_w1 * r1   # 矢量
        a_w3 = -np.linalg.norm(a_n) # 标量
        r3 = a_n / a_w3
        r2 = np.cross(r3, r1)
        R = np.array([r1 ,r2, r3]).T
        M = np.identity(4)
        M[:3,:3] = R
        # q_array = quaternion_from_matrix(M)
        thrust,alpha = self.thrust_by_attitude(V, a_w1, a_w3)
        euler = euler_from_matrix(M)
        euler_r = [-euler[0], -euler[1]+np.pi/10, euler[2]]
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/7.9, euler[2])     # ly初值打击，target_pos = np.array([1791.3, 5201.8, 17.85])，takeoff_pos=[1305, 4933, 97]
        q_array = quaternion_from_euler(-euler[0], -euler[1]+alpha*10, euler[2])        # ly初值打击+盘旋，target_pos = np.array([1791.3, 2201.8, 17.85])，takeoff_pos=[1305, 1733, 97]  
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/17.5, euler[2])    #-euler[1]+np.pi/10+alpha/3.1415*180
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/30, euler[2])
        print(alpha)
        q = Quaternion()
        # q.w = 0.996
        # q.x = 0.087
        q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
        
        #thrust = self.depth_thrust(pos_i)

        #print("v_d: {}\nv: {}\n".format(v_d, pos_info["mav_vel"]))
        #print("q: {}\nthrust: {}\n".format(q, thrust))
        return [q, thrust]

    def DockingController(self, pos_info, pos_i, image_center, cam_info):
        #calculate nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_c0b.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calculate the no
        # n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, cam_info["foc"]], dtype=np.float64)
        # n_co /= np.linalg.norm(n_co)
        # n_bo = self.R_c0b.dot(n_co)
        # n_eo = pos_info["mav_R"].dot(n_bo)
        n_co = np.array([cam_info["foc"], pos_i[0] - self.u0, pos_i[0] - self.u0], dtype=np.float64)    # 相机系也定义为FRD
        n_co /= np.linalg.norm(n_co)
        n_bo = cam_info["R"].dot(n_co)      # 转到载具系FRD
        n_eo = pos_info["mav_R"].dot(self.R_b_trans.dot(n_bo))  # 先转到FLU再转到ENU
        
        # calculate the v_d and a_d
        g = np.array([0, 0, 9.8])
        #V = np.linalg.norm(pos_info["mav_vel"])
        #v_d = (V + 1) * n_eo
        v_d = 20  * n_eo
        a_d = 0.1 * (v_d - pos_info["mav_vel"])
        #print("a_d: {}".format(a_d))
        # calculate R_d
        r1 = pos_info["mav_vel"] / V
        a_w1 = r1.dot(a_d - g)      # 标量
        a_n = a_d - g - a_w1 * r1   # 矢量
        a_w3 = -np.linalg.norm(a_n) # 标量
        r3 = a_n / a_w3
        r2 = np.cross(r3, r1)
        R = np.array([r1 ,r2, r3]).T
        M = np.identity(4)
        M[:3,:3] = R
        # q_array = quaternion_from_matrix(M)
        thrust,alpha = self.thrust_by_attitude(V, a_w1, a_w3)
        euler = euler_from_matrix(M)
        euler_r = [-euler[0], -euler[1]+np.pi/10, euler[2]]
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/7.9, euler[2])     # ly初值打击，target_pos = np.array([1791.3, 5201.8, 17.85])，takeoff_pos=[1305, 4933, 97]
        q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/10+alpha/3.1415*180, euler[2])        # ly初值打击+盘旋，target_pos = np.array([1791.3, 2201.8, 17.85])，takeoff_pos=[1305, 1733, 97]
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/17.5, euler[2])
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/30, euler[2])
        #print(euler_r)
        q = Quaternion()
        # q.w = 0.996
        # q.x = 0.087
        q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
        
        #thrust = self.depth_thrust(pos_i)

        #print("v_d: {}\nv: {}\n".format(v_d, pos_info["mav_vel"]))
        #print("q: {}\nthrust: {}\n".format(q, thrust))
        return [q, thrust]

    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a
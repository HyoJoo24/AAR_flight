#!/usr/bin/env python
#coding=utf-8

import socket
import cv2
import numpy as np
import struct
import threading
import time
import struct
from geometry_msgs.msg import *
from std_msgs.msg import UInt64
import rospy
from rflysim_ros_pkg.msg import Obj
import math
from geometry_msgs.msg import Quaternion
import sys
from mavros_msgs.msg import AttitudeTarget


def udp_socket():
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def sendUE4Pos(copterID, vehicleType, MotorRPMSMean, PosE, AngEuler, udp, windowID=-1):
    if windowID < 0:
        for i in range(5):
            buf = struct.pack("3i7f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                              PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2])
            socket = udp_socket()
            # socket.sendto(buf, ('192.168.1.98', 20010+i))
            socket.sendto(buf, (udp, 20010+i))
    else:
        buf = struct.pack("3i7f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2])
        socket = udp_socket()
        # socket.sendto(buf, ('192.168.1.98', 20010+windowID))
        socket.sendto(buf, (udp, 20010+windowID))

def sendUE4PosScale(copterID, vehicleType, MotorRPMSMean, PosE, AngEuler, Scale,udp, windowID=-1):
    if windowID < 0:
        for i in range(5):
            buf = struct.pack("3i10f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                              PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2],Scale[0],Scale[1],Scale[2])
            socket = udp_socket()
            # socket.sendto(buf, ('192.168.1.98', 20010+i))
            socket.sendto(buf, (udp, 20010+i))
    else:
        buf = struct.pack("3i10f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2],Scale[0],Scale[1],Scale[2])
        socket = udp_socket()
        # socket.sendto(buf, ('192.168.1.98', 20010+windowID))
        socket.sendto(buf, (udp, 20010+windowID))

def sendUE4PosNew(copterID,vehicleType,PosE,AngEuler, udp, windowID=-1):
    """ send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        # //输出到模拟器的数据
        # struct SOut2SimulatorSimpleTime {
        #     int checkSum; //1234567890
        #     int copterID;  //Vehicle ID
        #     int vehicleType;  //Vehicle type
        #     float PWMs[8];
        #     float VelE[3];
        #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
        #     double PosE[3];   //NED vehicle position in earth frame (m)
        #     double runnedTime; //Current Time stamp (s)
        # };
        #struct.pack 3i14f4d
    """
    # if runnedTime<0:
    #     runnedTime = time.time()-self.startTime
    checkSum=1234567890
    # pack for SOut2SimulatorSimpleTime
    VelE=[0,0,0]
    PWMs=[0]*8
    runnedTime=-1
    buf = struct.pack("3i14f4d",checkSum,copterID,vehicleType,PWMs[0],PWMs[1],PWMs[2],PWMs[3],PWMs[4],PWMs[5],PWMs[6],PWMs[7],VelE[0],VelE[1],VelE[2],AngEuler[0],AngEuler[1],AngEuler[2], PosE[0],PosE[1],PosE[2], runnedTime)
    #print(len(buf))
    if windowID<0:
        for i in range(6):
            socket = udp_socket()
            socket.sendto(buf, (udp, 20010+i))
    else:
        socket = udp_socket()
        socket.sendto(buf, (udp, 20010+windowID)) #ensure this PC can reciver message under specify IP mode



def tank_control(idnum=105,typenum=152,posnum=[-220, 300, 60],sizenum=[5,5,5],angelnum=[0,0,0],is_move=False):
    msg = Obj()

    msg.id = idnum
    msg.type = typenum
    # obj_msg.position.x = sphere_pos_x + 5*np.sin(cnt*1.0/100)
    #sphere_pos = self.sphere_pos_all[0] #+ sphere_vel * 0.02 * is_move 
    #sphere_vel = sphere_vel + sphere_acc * 0.02 * is_move 
    msg.position.x = posnum[0]
    msg.position.y = posnum[1]
    msg.position.z = posnum[2]
    msg.size.x = sizenum[0]
    msg.size.y = sizenum[1]
    msg.size.z = sizenum[2]
    msg.angule.x = angelnum[0]
    msg.angule.y = angelnum[1]
    msg.angule.z = angelnum[2]
    obj_pos = [msg.position.y + param_x, msg.position.x + param_y, - msg.position.z + param_z]
    obj_angle = [msg.angule.x, msg.angule.y, np.pi*2 - msg.angule.z]
    obj_size = [msg.size.x, msg.size.y, msg.size.z]
    sendUE4PosScale(msg.id, msg.type, 0, obj_pos, obj_angle, obj_size, param_ip)
    #sendUE4PosNew(obj_id, obj_type, obj_pos, obj_angle, param_ip)

obj_vel = [0,0,0]
obj_pos = [0, 0, 0]
obj_angle = [0, 0, 0]
obj_size = [0, 0, 0]
obj_type = 0
obj_id = 0
a = 100
hd = 0
#can be used to test receiver control
def tank_cb(msg):
    degree_to_rate = math.pi/180
    global obj_vel, obj_pos, obj_angle,obj_type, obj_id, obj_size,param_x,param_y,param_z,param_ip,a
    obj_id = msg.id
    obj_type = msg.type
    a=a+0.35
    hd = 3.5#3.5
    tank_control(13,520,[-121.44, 302+a, 64.48-hd],[1,1,1],[0,0,0]) #299+a
    tank_control(12,110,[-120, 303+a, 66-hd],[1,1,1],[0,0,0])       #300+a
    tank_control(2,120,[-120,308.5+a,66.5-hd],[1,1,1],[0,0.2,0])
    tank_control(3,120,[-120,313.2+a,67.45-hd],[1,1,1],[0,0.2,0])
    tank_control(4,120,[-120, 318+a, 68.42-hd],[1,1,1],[0,0.2,0])
    tank_control(5,120,[-120, 322.8+a, 69.65-hd],[1,1,1],[0,0.3,0])
    tank_control(6,120,[-120, 327.5+a, 71.1-hd],[1,1,1],[0,0.3,0])
    tank_control(7,120,[-120, 332.1+a, 73.05-hd],[1,1,1],[0,0.5,0])
    tank_control(8,120,[-120, 336.3+a, 75.35-hd],[1,1,1],[0,0.5,0])
    tank_control(9,120,[-120, 340.5+a, 77.65-hd],[1,1,1],[0,0.5,0])
    tank_control(10,120,[-120, 344.5+a, 79.80-hd],[1,1,1],[0,0.5,0])
    tank_control(11,190,[-120, 336+a, 80-hd],[1,1,1],[0,0,0])
    pos_dro = AttitudeTarget()
    q = Quaternion()
    q.x, q.y, q.z, q.w = 0, 0, 0, 0
    pos_dro.orientation = q
    pos_dro.thrust = 302+a
    drogue_pos_pub.publish(pos_dro)
    #obj_vel =[0,20,0]


    
'''
这里进行了结算，mavros到UE4坐标系
mavros          ue4
pos_x          pos_y
pos_y          pos_x
pos_z          - pos_z

sendUE4Pos(CopterID, VehicleType, RotorSpeed, PosM, AngEulerRad, windowsID = 0)  # -8.086
'''

def obj_cb(msg):
    degree_to_rate = math.pi/180
    global obj_vel, obj_pos, obj_angle,obj_type, obj_id, obj_size,param_x,param_y,param_z,param_ip
    obj_id = msg.id
    obj_type = msg.type
    
    
    obj_pos = [msg.position.y + param_x, msg.position.x + param_y, - msg.position.z + param_z]
    obj_angle = [msg.angule.x, msg.angule.y, np.pi*2 - msg.angule.z]
    obj_size = [msg.size.x, msg.size.y, msg.size.z]
    #print("obj_info: {}".format(msg))

    sendUE4PosScale(obj_id, obj_type, 0, obj_pos, obj_angle, obj_size, param_ip)  # -8.086

if __name__ == '__main__':
    rospy.init_node('obj_control', anonymous=True)
    param_x = rospy.get_param("/obj_control/mav_x")
    param_y = rospy.get_param("/obj_control/mav_y")
    param_z = rospy.get_param("/obj_control/mav_z")
    param_ip = rospy.get_param("/obj_control/ip")
    drogue_pos_pub = rospy.Publisher('Drogue_position', AttitudeTarget, queue_size=10)
    #rospy.Subscriber("ue4_ros/obj", Obj, obj_cb)
    rospy.Subscriber("ue4_ros/obj", Obj, tank_cb)
    for i in range(10):
       print("chunshuqiu")
    rospy.spin()


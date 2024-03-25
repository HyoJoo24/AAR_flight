#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import os
import json
import math
import rospy, rospkg
from PX4MavCtrlV4 import EarthModel
from geometry_msgs.msg import TwistStamped, Quaternion
from swarm_msgs.msg import Action
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_matrix, euler_from_matrix, quaternion_from_euler
from rflysim_ros_pkg.msg import Obj


# 无人机控制类
class Px4Controller:
    def __init__(self, drone_id, scene="32s"):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.start_point.pose.position.z = 4
        self.drone_id = drone_id
        self.drone_name = "drone_{}".format(self.drone_id)
        self.scene = scene
        self.rate = rospy.Rate(20)

        self.sphere_pos1 = np.array([-220, 300, 60])
        self.sphere_pos2 = np.array([-228.5,300,66.5])
        self.sphere_pos3 = np.array([-233.2,300,67.45])
        
        self.sphere_pos_all = [self.sphere_pos1,self.sphere_pos2,self.sphere_pos3]

        self.is_initialize_pos = False
        self.mav_yaw = 0
        self.mav_roll = 0
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_vel = np.array([0., 0., 0.])
        self.mav_R = np.identity(3)
        self.uavPosGPSHome = [28.1405220, 112.9825003, 53.220]

        # mavros topics
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        self.global_home_sub = rospy.Subscriber("/mavros/home_position/home", HomePosition, self.mav_home_cb)
        self.local_vel_pub =  rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.local_att_pub =  rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)                     # 发布速度指令（重要）
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)                        # 发布位置指令（初始化飞到厂房前使用）
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)                                              # 解锁服务
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)                                             # 飞行模式服务
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)                                          # 起飞服务
        print("Px4 Controller Initialized with {}".format(self.drone_name))

    # 任务开始前的一些动作，包括解锁、进offboard、飞到厂房前
    def start(self):
        for _ in range(2):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.rate.sleep()

        # 不同场景做不同的准备动作
        if self.scene == "32s":             # 主要
            self.takeoff(h=25)

        if self.scene == "paper":           # 论文使用
            '''self.takeoff(h=6)
            des_pos = np.array([0, 35, 3.5])
            dis = np.linalg.norm(des_pos-self.mav_pos)
            command_vel = TwistStamped()
            while dis > 0.5:
                norm_vel = (des_pos-self.mav_pos)/dis*11
                command_vel.twist.linear.x,command_vel.twist.linear.y,command_vel.twist.linear.z = norm_vel
                self.vel_pub.publish(command_vel)
                dis = np.linalg.norm(des_pos-self.mav_pos)
                self.rate.sleep()
            '''
            # takeoff_pos=[0, 400, 50]    # ENU
            # takeoff_pos=[1305, 4933, 97]    # ENU[-5, 290, 60]
            takeoff_pos=[-5, 290, 60]    # ENU   ue4NED
            self.geo = EarthModel()
            lla = self.geo.enu2lla(takeoff_pos, self.uavPosGPSHome)
            lat, lon, alt = lla[0], lla[1], lla[2]

            print("latitude={}, longitude={}, altitude={}".format(lat, lon, alt))
            self.takeoffService(latitude=lat, longitude=lon, altitude=alt)
            rospy.loginfo("Taking off")
            time.sleep(0.5)
            


            self.sphere_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
            #while not rospy.is_shutdown():
            '''self.sphere_control(12,110,[-220, 300, 66],[1,1,1],[0,0,0])
            self.sphere_control(2,120,[-220,308.5,66.5],[1,1,1],[0,0.2,0])
            self.sphere_control(3,120,[-220,313.2,67.45],[1,1,1],[0,0.2,0])
            self.sphere_control(4,120,[-220, 318, 68.42],[1,1,1],[0,0.2,0])
            self.sphere_control(5,120,[-220, 322.8, 69.65],[1,1,1],[0,0.3,0])
            self.sphere_control(6,120,[-220, 327.5, 71.1],[1,1,1],[0,0.3,0])
            self.sphere_control(7,120,[-220, 332.1, 73.05],[1,1,1],[0,0.5,0])
            self.sphere_control(8,120,[-220, 336.3, 75.35],[1,1,1],[0,0.5,0])
            self.sphere_control(9,120,[-220, 340.5, 77.65],[1,1,1],[0,0.5,0])
            self.sphere_control(10,120,[-220, 344.5, 79.80],[1,1,1],[0,0.5,0])
            self.sphere_control(11,190,[-220, 336, 80],[1,1,1],[0,0,0])'''
            # dis = np.linalg.norm(takeoff_pos - self.mav_pos)
            hd=36
            while abs(self.mav_pos[1] - takeoff_pos[1]) > 50:
                self.sphere_control(12,110,[-120, 300, 66-hd],[1,1,1],[0,0,0])
                self.sphere_control(2,120,[-120,308.5,66.5-hd],[1,1,1],[0,0.2,0])
                self.sphere_control(3,120,[-120,313.2,67.45-hd],[1,1,1],[0,0.2,0])
                self.sphere_control(4,120,[-120, 318, 68.42-hd],[1,1,1],[0,0.2,0])
                self.sphere_control(5,120,[-120, 322.8, 69.65-hd],[1,1,1],[0,0.3,0])
                self.sphere_control(6,120,[-120, 327.5, 71.1-hd],[1,1,1],[0,0.3,0])
                self.sphere_control(7,120,[-120, 332.1, 73.05-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(8,120,[-120, 336.3, 75.35-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(9,120,[-120, 340.5, 77.65-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(10,120,[-120, 344.5, 79.80-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(11,190,[-120, 336, 80-hd],[1,1,1],[0,0,0])
                print("起飞中...")
                print(self.mav_pos)
                # self.takeoffService(latitude=lat, longitude=lon, altitude=alt)
                time.sleep(0.5)
                # dis = np.linalg.norm(takeoff_pos - self.mav_pos)

            # 保持前飞一段
            for i in range(200):
                self.offboard_state = self.offboard()
                # print("开始进入Offboard模式")

                q = Quaternion()
                q.w = 1.
                q.x = 0.
                self.moveByAttitudeThrust(q, 0.75)
                #print("moveByAttitudeThrust")

                # self.moveByBodyRateThrust(0, -0.1, 0, 0.75)  # 可行值：(0.5, 0, 0, 0.75)
                # print("moveByBodyRateThrust")
                self.rate.sleep()

            # 盘旋
            
            # 盘旋：目标位置
            target_pos = np.array([-60, 60, 60])
            circle_R = np.linalg.norm(self.mav_pos[:2] - target_pos[:2])
            circle_H = self.mav_pos[2]
            circle_V = np.linalg.norm(self.mav_vel)
            # 盘旋：微分平坦参数
            Kp = 0.05
            Kv = 0.2

            timeInterval = 0.06
            lastClock=time.time()
            lastTime = time.time()
            a=current_time1=delta_time=0
            for i in range(280):
                
                a=a+0.6
                self.sphere_control(12,110,[-120, 300+a, 66-hd],[1,1,1],[0,0,0])
                self.sphere_control(2,120,[-120,308.5+a,66.5-hd],[1,1,1],[0,0.2,0])
                self.sphere_control(3,120,[-120,313.2+a,67.45-hd],[1,1,1],[0,0.2,0])
                self.sphere_control(4,120,[-120, 318+a, 68.42-hd],[1,1,1],[0,0.2,0])
                self.sphere_control(5,120,[-120, 322.8+a, 69.65-hd],[1,1,1],[0,0.3,0])
                self.sphere_control(6,120,[-120, 327.5+a, 71.1-hd],[1,1,1],[0,0.3,0])
                self.sphere_control(7,120,[-120, 332.1+a, 73.05-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(8,120,[-120, 336.3+a, 75.35-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(9,120,[-120, 340.5+a, 77.65-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(10,120,[-120, 344.5+a, 79.80-hd],[1,1,1],[0,0.5,0])
                self.sphere_control(11,190,[-120, 336+a, 80-hd],[1,1,1],[0,0,0])
                
                
                
                # 微分平坦跟踪圆
                vec_p = self.mav_pos[:2] - target_pos[:2]
                vec_unit_p = vec_p / np.linalg.norm(vec_p)
                mav_pos_d = target_pos + np.array([vec_unit_p[0] * circle_R, vec_unit_p[1] * circle_R, 0])
                # print("mav_pos_d: {}, mav_pos: {}".format(mav_pos_d, self.mav_pos))

                mav_vel_d = circle_V * np.array([-vec_unit_p[1], vec_unit_p[0], 0.])
                if mav_vel_d.dot(self.mav_vel) < 0:
                    mav_vel_d = -mav_vel_d
                # print("mav_vel_d: {}, mav_vel: {}".format(mav_vel_d, self.mav_vel))

                an_norm = circle_V * circle_V / circle_R
                an = np.array([-vec_unit_p[0] * an_norm, -vec_unit_p[1] * an_norm, 0.])
                a_d = an + Kv * (mav_vel_d - self.mav_vel) + Kp * (mav_pos_d - self.mav_pos)
                # print("an: {}, a_d: {}".format(an, a_d))

                # 微分平坦控制
                g = np.array([0, 0, 9.8])
                V = np.linalg.norm(self.mav_vel)
                r1 = self.mav_vel / V
                a_w1 = r1.dot(a_d - g)      # 标量
                a_n = a_d - g - a_w1 * r1   # 矢量
                a_w3 = -np.linalg.norm(a_n) # 标量
                r3 = a_n / a_w3
                r2 = np.cross(r3, r1)
                R = np.array([r1 ,r2, r3]).T
                M = np.identity(4)
                M[:3,:3] = R
                # q_array = quaternion_from_matrix(M)
                euler = euler_from_matrix(M)
                q_array = quaternion_from_euler(-euler[0], -euler[1], euler[2])

                q = Quaternion()
                q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
                thrust = 0.75
                self.moveByAttitudeThrust(q, thrust)
                self.rate.sleep()
                '''current_time1=time.time()
                delta_time=current_time1-lastTime'''

                '''lastTime = lastTime + timeInterval
                sleepTime = lastTime - time.time()
                if sleepTime > 0:
                    time.sleep(sleepTime) # sleep until the desired clock
                else:
                    lastTime = time.time()'''
            
        if self.scene == "jz":           # 拒止项目
            # takeoff_pos=[0, 400, 50]    # ENU
            # takeoff_pos=[1305, 4933, 97]    # ENU[-5, 290, 60]
            takeoff_pos=[0, 180, 60]    # ENU   ue4NED
            self.geo = EarthModel()
            lla = self.geo.enu2lla(takeoff_pos, self.uavPosGPSHome)
            lat, lon, alt = lla[0], lla[1], lla[2]

            print("latitude={}, longitude={}, altitude={}".format(lat, lon, alt))
            self.takeoffService(latitude=lat, longitude=lon, altitude=alt)
            rospy.loginfo("Taking off")
            time.sleep(0.5)
            

            #define the message 
            self.sphere_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
            
            hd = 0
            a=20
            while abs(self.mav_pos[1] - takeoff_pos[1]) > 50:
                #send the command to ue4 in Rflysim_objwithsize.py
                self.sphere_control(12,110,[0, 0, 0],[1,1,1],[0,0,0])
                print("起飞中...")
                print(self.mav_pos)
                # self.takeoffService(latitude=lat, longitude=lon, altitude=alt)
                time.sleep(0.5)
                # dis = np.linalg.norm(takeoff_pos - self.mav_pos)
            timeInterval = 0.06
            lastClock=time.time()
            lastTime = time.time()
            
            # 保持前飞一段
            for i in range(300):
                self.offboard_state = self.offboard()
                # print("开始进入Offboard模式")
                
                self.sphere_control(12,110,[0, 0, 0],[1,1,1],[0,0,0])
                q = Quaternion()
                q.w = 1.
                q.x = 0.
                self.moveByAttitudeThrust(q, 0.79)
                #print("moveByAttitudeThrust")

                # self.moveByBodyRateThrust(0, -0.1, 0, 0.75)  # 可行值：(0.5, 0, 0, 0.75)
                # print("moveByBodyRateThrust")
                self.rate.sleep()

            
               



        if self.scene == "freeflight":      # test for freeflight.py
            if self.drone_id == 1:
                self.start_point.pose.position.x = 0
                self.start_point.pose.position.y = 0
                self.start_point.pose.position.z = 2
            elif self.drone_id == 2:
                self.start_point.pose.position.x = 4
                self.start_point.pose.position.y = 0
                self.start_point.pose.position.z = 2
            elif self.drone_id == 3:
                self.start_point.pose.position.x = 0
                self.start_point.pose.position.y = 4
                self.start_point.pose.position.z = 2

        if self.scene == "attack_in_oldfactory":      # oldfactory场景起点（以后删）
            self.start_point.pose.position.x = 1
            self.start_point.pose.position.y = -12
            self.start_point.pose.position.z = 2.5
            for _ in range(300):
                self.pos_pub.publish(self.start_point)
                self.rate.sleep()

        if self.scene == "freeflight":      # test for tube
            des_pos = np.array([46.55, 36.75, 1.0])
            self.start_point.pose.position.x = des_pos[0]
            self.start_point.pose.position.y = des_pos[1]
            self.start_point.pose.position.z = des_pos[2]
            dis = np.linalg.norm(des_pos-self.mav_pos)
            while dis > 0.5:
                self.pos_pub.publish(self.start_point)
                dis = np.linalg.norm(des_pos-self.mav_pos)
                self.rate.sleep()


    def sphere_control(self, idnum=105,typenum=152,posnum=[-220, 300, 60],sizenum=[5,5,5],angelnum=[0,0,0],is_move=False):
        obj_msg = Obj()

        obj_msg.id = idnum
        obj_msg.type = typenum
        # obj_msg.position.x = sphere_pos_x + 5*np.sin(cnt*1.0/100)
        #sphere_pos = self.sphere_pos_all[0] #+ sphere_vel * 0.02 * is_move 
        #sphere_vel = sphere_vel + sphere_acc * 0.02 * is_move 
        obj_msg.position.x = posnum[0]
        obj_msg.position.y = posnum[1]
        obj_msg.position.z = posnum[2]
        obj_msg.size.x = sizenum[0]
        obj_msg.size.y = sizenum[1]
        obj_msg.size.z = sizenum[2]
        obj_msg.angule.x = angelnum[0]
        obj_msg.angule.y = angelnum[1]
        obj_msg.angule.z = angelnum[2]
        self.sphere_pub.publish(obj_msg)


    # 无人机位置姿态回调函数
    def local_pose_callback(self, msg):
        if not self.is_initialize_pos:
            self.mav_yaw_0 = self.mav_yaw
        self.is_initialize_pos = True
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_roll = np.arctan2(2*(q0*q1 + q2*q3), 1-2*(q2*q2 + q1*q1))
        self.mav_yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
        self.mav_R = np.array([
            [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
        ])

    # 无人机位置速度回调函数
    def local_vel_callback(self, msg):
        self.is_initialize_vel = True
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    
    def mav_home_cb(self, msg):
        self.uavPosGPSHome = [msg.geo.latitude, msg.geo.longitude, msg.geo.altitude]


    # 悬停
    def idle(self):
        print("I'm in idle state!")
        idle_cmd = TwistStamped()
        while not rospy.is_shutdown():
            self.vel_pub.publish(idle_cmd)
            self.rate.sleep()

    # 解锁
    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    # 上锁
    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    # 进offboard模式
    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            print("Vechile Offboard Mode")
            return True
        else:
            print("Vechile Offboard failed")
            return False

    # 起飞
    def takeoff(self, vz=0.8,h=25):
        self.mav_yaw_0 = self.mav_yaw

        self.moveByPosENU(U=h)
        takeoff_done = False
        while not takeoff_done:
            self.moveByPosENU(U=h)
            # command = TwistStamped()
            # command.twist.linear.z = vz
            # self.vel_pub.publish(command)
            rospy.sleep(0.05)
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = (abs(self.mav_pos[2]- h) < 0.05)

    def moveByPosENU(self, E=None, N=None, U=None, mav_yaw=None):
        mav_yaw = mav_yaw if mav_yaw is not None else self.mav_yaw
        E = E if E is not None else self.mav_pos[0]
        N = N if N is not None else self.mav_pos[1]
        U = U if U is not None else self.mav_pos[2]
        command_vel = construct_postarget_ENU(E, N, U, mav_yaw)
        self.local_vel_pub.publish(command_vel)

    def moveByAttitudeThrust(self, q, thrust):
        '''
        q: geometry_msgs/Quaternion type
        thrust: float 0-1
        body frame: FLU, 不能控偏航, 四元数网站: https://quaternions.online/
        '''
        target_raw_attitude = AttitudeTarget()
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE

        target_raw_attitude.orientation = q
        target_raw_attitude.thrust = thrust
        self.local_att_pub.publish(target_raw_attitude)

    def moveByBodyRateThrust(self, roll_rate, pitch_rate, yaw_rate, thrust):
        '''
        body frame: FLU, 不能控偏航
        '''
        target_raw_attitude = AttitudeTarget()
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE

        target_raw_attitude.body_rate.x = roll_rate
        target_raw_attitude.body_rate.y = pitch_rate
        target_raw_attitude.body_rate.z = yaw_rate
        target_raw_attitude.thrust = thrust
        self.local_att_pub.publish(target_raw_attitude)
    
    
def construct_postarget_ENU(E=0, N=0, U=0, yaw=0, yaw_rate = 0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()

    '''
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9

    MAV_FRAME_BODY_FRD
    '''

    target_raw_pose.coordinate_frame = 1

    target_raw_pose.position.x = E
    target_raw_pose.position.y = N
    target_raw_pose.position.z = U

    target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE

    target_raw_pose.yaw = yaw
    target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose




class Assemble:

    def __init__(self, param_id, px4):
        self.px4 = px4
        self.start_time = time.time()
        
        self.pipeline_cmd = TwistStamped()
        self.dj_cmd = TwistStamped()
        self.obs_cmd = TwistStamped()
        self.dj_action = Action()
        self.dro_pos = TwistStamped()
        
        self.Pipeline_cmd_sub = rospy.Subscriber('Pipeline_cmd', TwistStamped, self.Pipeline_cmd_callback)
        self.DJ_cmd_sub = rospy.Subscriber('DJ_cmd', AttitudeTarget, self.DJ_cmd_callback)
        self.Obs_cmd_sub = rospy.Subscriber('Obs_cmd', TwistStamped, self.Obs_cmd_callback)
        self.Expect_action_sub = rospy.Subscriber('expect_action'+str(param_id), Action, self.Expect_action_callback)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.Drogue_pos_sub = rospy.Subscriber('Drogue_position', AttitudeTarget, self.Drogue_pos_callback)

    def Pipeline_cmd_callback(self, msg):
        self.pipeline_cmd = msg

    def DJ_cmd_callback(self, msg):
        self.dj_cmd = msg

    def Obs_cmd_callback(self, msg):
        self.obs_cmd = msg

    def Expect_action_callback(self, msg):
        self.dj_action = msg

    def Drogue_pos_callback(self, msg):
        self.dro_pos = msg

    def TECS_control(self,V=21,V_last=20,H=61,H_last=60,T=1.88,xita=0):
        mass = 8.16466
        V_desired = 20
        H_desired = 60
        delta_T = 0.02
        #tecs parameters
        K_tp = 0.001
        K_ti = 0.005
        K_ep = 0.001
        K_ei = 0.005
        #parameters before the tecs
        K_h = -0.3
        K_v = 0.005
        #calculate the energy rate 
        total_energy = 0.5*mass*V**(2)+mass*9.8*H  #ET
        unit_energy = total_energy/mass/9.8  #E1
        dot_energy = (V-V_last)/delta_T/9.8 + (H-H_last)/delta_T/V  #E dot
        dot_error = (H-H_last)/delta_T/V - (V-V_last)/delta_T/9.8   #L dot

        V_a_dot = (V-V_last)/delta_T #Va dot
        #print("V_a_dot:"+str(V_a_dot))
        X = (H-H_last)/delta_T/V
        #print("X:"+str(X))
        #calculate the Xc and Hc dot according to the H_desired error and speed_desired error
        X_c = K_h*(H_desired - H)/V
        #print("X_c:"+str(X_c))
        V_ac_dot = K_v*(V_desired-V)
        #print("V_ac_dot:"+str(V_ac_dot))
        delta_delta_energy = (V_ac_dot-V_a_dot)/9.8 + (X_c - X) #Ee dot
        delta_delta_allocate = (X_c - X)-(V_ac_dot-V_a_dot) #Le dot
        
        #calculate the Tc and xitac
        delta_thrust = K_tp*dot_energy + K_ti*delta_T*delta_delta_energy#used to change thrust  
        T = T + delta_thrust
        #print("T:"+str(T))
        delta_xita = K_ep*dot_error + K_ei*delta_T*delta_delta_allocate#used to change thrust
        xita = xita + delta_xita
        if xita<0.5 and xita>(-0.5):
            xita = xita
        else:
            xita = 0.1
        
        return T,xita

    def L1_control(self,xi_d=0):
            #line follow
            d0 = self.px4.mav_pos[0] - (-120)-119
            print("d0:"+str(d0))
            line_V = np.linalg.norm(self.px4.mav_vel)
            L1_D = 0.7
            L1_P = 35
            L1 = L1_P * L1_D * line_V / np.pi
            #d1 = self.px4.mav_pos[0] - (-120)-119
            #d_d = (d1-d0) / 0.05
            #print("d0:"+str(d1))
            #d0 = d1
            if d0<0:
                k=-1
            else:
                k=1
            an_norm = 2.*line_V / L1 *(k*self.px4.mav_vel[0] + line_V/L1*abs(d0)) #k andbas(d0)
            print("an_norm:"+str(an_norm))
            unit_V = self.px4.mav_vel/line_V
            print("unit_V:"+str(unit_V))
            sin_n1 = -d0/L1
            if unit_V[0]>sin_n1:
                unit_V = -unit_V                    
            a_d1 = np.array([unit_V[1] * an_norm, -unit_V[0]* an_norm, 0.])
            
            print("a_d1:"+str(a_d1))
            if d0<0:
                phi_d = math.atan(an_norm/9.8)
            else:
                phi_d = math.atan(-an_norm/9.8)
            #
            
            xi_d = 9.8/line_V*math.tan(phi_d)*0.02 +xi_d
            #print("xi_d:"+str(xi_d))
            return phi_d,xi_d
            #circle follow


    def sphere_control(self, idnum=105,typenum=152,posnum=[-220, 300, 60],sizenum=[5,5,5],angelnum=[0,0,0],is_move=False):
        obj_msg = Obj()

        obj_msg.id = idnum
        obj_msg.type = typenum
        # obj_msg.position.x = sphere_pos_x + 5*np.sin(cnt*1.0/100)
        #sphere_pos = self.sphere_pos_all[0] #+ sphere_vel * 0.02 * is_move 
        #sphere_vel = sphere_vel + sphere_acc * 0.02 * is_move 
        obj_msg.position.x = posnum[0]
        obj_msg.position.y = posnum[1]
        obj_msg.position.z = posnum[2]
        obj_msg.size.x = sizenum[0]
        obj_msg.size.y = sizenum[1]
        obj_msg.size.z = sizenum[2]
        obj_msg.angule.x = angelnum[0]
        obj_msg.angule.y = angelnum[1]
        obj_msg.angule.z = angelnum[2]
        self.sphere_pub.publish(obj_msg)

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
        #print("T01:"+str(T01))#last
        #print("T02:"+str(T02))
        thrust1 = (T01**(2)/ (pow(17,4) * 2.83 * pow(10,-12) * 0.2))**(0.5)/8000/2.17
        #thrust2 = (T02**(2)/ (pow(17,4) * 2.83 * pow(10,-12) * 0.2))**(0.5)/8000/2.17
        #print("thrust1:"+str(thrust1))#last
        #print("thrust2:"+str(thrust2))
        if thrust1<1 and thrust1>0.2:
            thrust = thrust1
        else:
            thrust = 0.45
        if abs(alpha)<1:
            alpha = alpha
        else:
            alpha = 0
        return thrust,alpha


    def step3(self):
        #follow the desired altitude and speed with tecs control
        
        V = np.linalg.norm(self.px4.mav_vel)
        H = self.px4.mav_pos[2]
        if self.V_last is not None:
            T,xita =self.TECS_control(V=V,V_last=self.V_last,H=H,H_last=self.H_last,T=self.T_last,xita=self.xita_last)
            phi_d,xi_d =self.L1_control(xi_d=self.xi_last)
        else:
            T,xita =self.TECS_control()
            phi_d,xi_d =self.L1_control()
        q = Quaternion()
        if phi_d<(-0.26):
            phi_d=-0.26
        elif phi_d>0.26:
            phi_d=0.26
        else:
            phi_d=phi_d
        q_array = quaternion_from_euler(phi_d, xita, 0) #(phi_d/3.1415*180, xita/3.1415*180, 0)
        print("phi_d:"+str(phi_d/3.1415*180))
        
        mav_roll = self.px4.mav_roll
        print("roll:"+str(mav_roll/3.1415*180))
        q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
        thrust = (T**(2)/ (pow(17,4) * 2.83 * pow(10,-12) * 0.2))**(0.5)/8000/2.17
        if thrust<1 and thrust>0.2:
            thrust = thrust
        else:
            thrust = 0.45
        #thrust = 0.49
        print(thrust)
        print('\n')
        

        self.px4.moveByAttitudeThrust(q, thrust)
        self.V_last=V
        self.H_last=H
        self.T_last = T
        self.xita_last = xita
        self.xi_last = xi_d
        #while acc==0:
        
        
    
    
    
    







    def step2(self):
        #follow the velocity and the height commands by attitude and thrust control
        desired_H = 50  #f16 is 50, we change it for paper
        d0 = self.px4.mav_pos[0] - (-120)-119
        # control_by_height：微分平坦参数
        Kp = 0.15#0.15   first change v, then change h
        Kv = 0.01
        d2 = desired_H - self.px4.mav_pos[2]
        
        #while acc==0:
        
        # 微分平坦跟踪line
        line_V = np.linalg.norm(self.px4.mav_vel)
        L1_D = 0.7
        L1_P = 35
        L1 = L1_P * L1_D * line_V / np.pi
        #d1 = self.px4.mav_pos[0] - (-120)-119
        #d_d = (d1-d0) / 0.05
        #print("d0:"+str(d1))
        #d0 = d1
        if d0<0:
            k=-1
        else:
            k=1
        an_norm = 2.*line_V / L1 *(k*self.px4.mav_vel[0] + line_V/L1*abs(d0))
        #print("an_norm:"+str(an_norm))
        unit_V = self.px4.mav_vel/line_V
        #print("unit_V:"+str(unit_V))
        sin_n1 = -d0/L1
        if unit_V[0]>sin_n1:
            unit_V = -unit_V                    
        a_d1 = np.array([unit_V[1] * an_norm, -unit_V[0]* an_norm, 0.])
        #a_d = a_d1
        #a_d = Kv * (mav_vel_d - self.mav_vel) + Kp * (mav_pos_d - self.mav_pos) #PD control and feedforward control
        a_d = Kp * np.array([0, 0, d2]) + Kv * np.array([0, 21-self.px4.mav_vel[1], 0]) + a_d1 #-self.px4.mav_vel[2]
        #print("a_d: {}".format(a_d))

        # 微分平坦控制
        g = np.array([0., 0., 9.8])
        V = np.linalg.norm(self.px4.mav_vel)
        r1 = self.px4.mav_vel / V
        a_w1 = r1.dot(a_d - g)      # 标量
        a_n = a_d - g - a_w1 * r1   # 矢量
        a_w3 = -np.linalg.norm(a_n) # 标量
        r3 = a_n / a_w3
        r2 = np.cross(r3, r1)
        R = np.array([r1 ,r2, r3]).T
        M = np.identity(4)
        M[:3,:3] = R
        # q_array = quaternion_from_matrix(M)
        euler = euler_from_matrix(M)
        thrust,alpha = self.thrust_by_attitude(V, a_w1, a_w3) 
        euler_r = [-euler[0], (-euler[1]+alpha), euler[2]]
        q_array = quaternion_from_euler(-euler[0], (-euler[1]+alpha/3.1415*180), euler[2])
        #mav_roll = self.px4.mav_roll
        #print("roll:"+str(mav_roll/3.1415*180))
        #print("euler:"+str(euler_r))
        #print('\n')
        q = Quaternion()
        q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
        #thrust = self.thrust_by_vel()
        
        #print(q)
        #thrust = 0.42
        self.px4.moveByAttitudeThrust(q, thrust)

    

    def begin_task(self):
        rate = rospy.Rate(50)
        self.sphere_pub = rospy.Publisher("ue4_ros/obj", Obj, queue_size=10)
        #self.dj_action.dj=1
        hd=0
        a=80
        attack_false=0
        attack_ing=0
        self.V_last =None
        while not rospy.is_shutdown():
            #change the position of the tanker
            self.sphere_control(12,110,[0, 0, 0],[1,1,1],[0,0,0])
            # different control in three different conditions
            if (self.px4.mav_pos[1]-self.dro_pos.thrust-240)>(-18) and self.dj_cmd.thrust != 0:
                # self.vel_pub.publish(self.dj_cmd)
                #step3: docking by IBVS
                self.px4.moveByAttitudeThrust(self.dj_cmd.orientation, self.dj_cmd.thrust)
                attack_ing = 1

            elif attack_false == 0 and attack_ing ==0:
                #step2: control according to height before docking

                self.step3()
                #print("666")
                #self.vel_pub.publish(self.pipeline_cmd)
            else:
                attack_false = 1
                '''q = Quaternion()
                q.w = 1.
                q.x = 0.
                self.px4.moveByAttitudeThrust(q, 0.4)'''
                #print("777")
                #self.follow_line()
                #self.control_by_height()

                self.step3()
            rate.sleep()
    
    def follow_line(self):
        
        circle_H = self.px4.mav_pos[2]
        # follow_line：微分平坦参数
        #Kp = 0.05
        #Kv = 0.2
        #acc = 0
        d0 = self.px4.mav_pos[0] - (-120)-119
        '''timeInterval = 0.06
        lastClock=time.time()
        lastTime = time.time()
        a=current_time1=delta_time=0'''
        #while acc==0:

        # 微分平坦跟踪line
        line_V = np.linalg.norm(self.px4.mav_vel)
        L1_D = 0.7
        L1_P = 35
        L1 = L1_P * L1_D * line_V / np.pi
        #d1 = self.px4.mav_pos[0] - (-120)-119
        #d_d = (d1-d0) / 0.05
        #print("d0:"+str(d1))
        #d0 = d1
        if d0<0:
            k=-1
        else:
            k=1
        an_norm = 2.*line_V / L1 *(k*self.px4.mav_vel[0] + line_V/L1*abs(d0))
        print("an_norm:"+str(an_norm))
        unit_V = self.px4.mav_vel/line_V
        print("unit_V:"+str(unit_V))
        sin_n1 = -d0/L1
        if unit_V[0]>sin_n1:
            unit_V = -unit_V                    
        a_d = np.array([unit_V[1] * an_norm, -unit_V[0]* an_norm, 0.])
        #a_d = an + Kv * (mav_vel_d - self.mav_vel) + Kp * (mav_pos_d - self.mav_pos) #PD control and qiankui control
        print("a_d: {}".format(a_d))

        # 微分平坦控制
        g = np.array([0., 0., 9.8])
        V = np.linalg.norm(self.px4.mav_vel)
        r1 = self.px4.mav_vel / V
        a_w1 = r1.dot(a_d - g)      # 标量
        a_n = a_d - g - a_w1 * r1   # 矢量
        a_w3 = -np.linalg.norm(a_n) # 标量
        r3 = a_n / a_w3
        r2 = np.cross(r3, r1)
        R = np.array([r1 ,r2, r3]).T
        M = np.identity(4)
        M[:3,:3] = R
        # q_array = quaternion_from_matrix(M)
        euler = euler_from_matrix(M)
        q_array = quaternion_from_euler(-euler[0], -euler[1], euler[2])

        q = Quaternion()
        q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
        thrust = self.thrust_by_vel()
        #thrust = 0.42
        self.px4.moveByAttitudeThrust(q, thrust)
        #if abs(d0) < 2:
            #acc = 1
        #print(acc)
        #self.px4.rate.sleep()



if __name__=="__main__":
    rospy.init_node("assemble", anonymous=True)#初始化
    param_id = rospy.get_param("~drone_id")
    
    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)

    #create_global_function()


    # 飞机初始化，解锁、offboard、飞到厂房前
    px4 = Px4Controller(param_id, setting["SCENE"])
    ass = Assemble(param_id, px4)
    px4.start()
    ass.begin_task()
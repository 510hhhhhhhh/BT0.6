#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Battle import BattleEnv 
from Patrol import Plan
from Following import Follow
from teleop_control import Controller
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW
import time
import math
import numpy as np
import tf

GIMBAL_RELAX = 0
GIMBAL_INIT = 1
GIMBAL_NO_ARTI_INPUT = 2 #无手动控制信息输入模式?
GIMBAL_FOLLOW_ZGYRO = 3  #云台跟随底盘模式
GIMBAL_TRACK_ARMOR = 4   # 	云台追踪装甲，icra 不使用
GIMBAL_PATROL_MODE = 5  #巡逻模式，云台 yaw 周期运动，pitch 不受控制
GIMBAL_SHOOT_BUFF = 6   #打大符模式，icra 不使用
GIMBAL_POSITION_MODE = 7  #云台位置模式，上层控制角度两轴角度

CHASSIS_RELAX = 0
CHASSIS_STOP = 1
MANUAL_SEPARATE_GIMBAL = 2
MANUAL_FOLLOW_GIMBAL = 3
DODGE_MODE = 4             #底盘躲避模式，底盘固定旋转，平移不受控制
AUTO_SEPARATE_GIMBAL = 5   #底盘和云台分离模式，旋转、平移受上层控制查看tf tree
AUTO_FOLLOW_GIMBAL = 6     # 底盘跟随云台，平移受上层控制

#启动控制器和环境类
controller = Controller()
env = BattleEnv()
plan =Plan()
follow =Follow()
tflistener = tf.TransformListener()
#定义控制命令
vel = Twist()
nav_goal = env.navgoal
shoot_cmd = ShootCmd()
mode = ModeSW()
num1= 31
#接受机器人状态消息
rospy.Subscriber('gimbalpos', EnemyPos, env.getGimbalPoseCallback)
rospy.Subscriber('my_pose', Odometry, env.getSelfPoseCallback)
time.sleep(0.1)
rospy.Subscriber('/enemy_pos', EnemyPos, env.getEnemyPoseCallback)
time.sleep(0.1)
print 'orginal my pose is x=%s\ty=%s\t yaw=%s\n' % (env.MyPose['x'], env.MyPose['y'], env.MyPose['theta'])
print 'orginal enemy pose is yaw=%s\tpitch=%s\tdistance=%s\n' % (env.EnemyPose.enemy_yaw, env.EnemyPose.enemy_pitch, env.EnemyPose.enemy_dist)

# env.shooting_stop(shoot_cmd, controller)
# mode.gimbal_mode = GIMBAL_PATROL_MODE  # 巡逻模式，云台 yaw 周期运动，pitch 不受控制
# mode.chassis_mode = AUTO_SEPARATE_GIMBAL  # 底盘和云台分离模式，旋转、平移受上层控制
# controller.mode_switch(mode)  # 发送模式
# print 'return orginal'
# if (env.MyPose['x']!=0.5) and (env.MyPose['y']!=0.5):
#     x = 1
#     y = 1
#     yaw = 0
#     if env.isActionAvaliable(x, y, yaw):#判断目标点是否可行
#         controller.send_goal(env.navgoal)#发送目标
#         time.sleep(1)
#     else:
#         pass
# else:
#     pass

# GIMBAL_PATROL_MODE = 5  #巡逻模式，云台 yaw 周期运动，pitch 不受控制
# GIMBAL_POSITION_MODE = 7  #云台位置模式，上层控制角度两轴角度
# DODGE_MODE = 4             #底盘躲避模式，底盘固定旋转，平移不受控制
# AUTO_SEPARATE_GIMBAL = 5   #底盘和云台分离模式，旋转、平移受上层控制
# AUTO_FOLLOW_GIMBAL = 0     # 底盘跟随云台，平移受上层控制

env.shoot_fric_wheel(shoot_cmd, controller)

# 发布目标点
goal_x = 4
goal_y = 2.5
goal_yaw = -5
print 'sending center_goal successful'
time2 = rospy.Time.now().secs
if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # 判断目标点是否可行
    controller.send_goal(env.navgoal)  # 发送目标
    while (rospy.Time.now().secs - time2 < 12):
        if(env.enemyNew == True):
            env.shooting(shoot_cmd, controller)
            env.enemyNew = False
    env.shooting_stop(shoot_cmd, controller)
else:
    pass

#巡逻路径——发布目标点   
# while True:
#       if (env.EnemyPose.enemy_yaw == 0) and (env.EnemyPose.enemy_pitch == 0) and (env.EnemyPose.enemy_dist == 0):
#             mode.gimbal_mode = GIMBAL_PATROL_MODE # 	巡逻模式，云台 yaw 周期运动，pitch 不受控制
#             mode.chassis_mode = AUTO_SEPARATE_GIMBAL #底盘和云台分离模式，旋转、平移受上层控制
#             controller.mode_switch(mode)#发送模式
#             # plan.patrol_path(env,controller) #发送目标
#             # print 'patrol fixed goal!'
#             plan.random_path(env,controller)
#             print 'patrol random goal!'
#             break


#       elif (env.EnemyPose.enemy_dist <=1) and (env.EnemyPose.enemy_dist >0.1):           
#             #看到就打-定点旋转打
#             print 'at this moment pose is yaw=%s\tpitch=%s\tdistance=%s\n' % (env.EnemyPose.enemy_yaw, env.EnemyPose.enemy_pitch, env.EnemyPose.enemy_dist)
#             mode.gimbal_mode = GIMBAL_POSITION_MODE # #云台位置模式，上层控制角度两轴角度
#             mode.chassis_mode = DODGE_MODE  #
#             controller.mode_switch(mode)#发送模式
#             print 'shoot!'
#             env.shooting(shoot_cmd,controller)#发射子弹
#             follow.axis_trans(env,controller)
#             break 
# else:
#     print 'where?'
# elif (env.EnemyPose.enemy_dist <=1):
#     #看到就打-跟随打-地盘跟随模式
#     print 'enemy pose is yaw=%s\tpitch=%s\tdistance=%s\n' % (env.EnemyPose.enemy_yaw, env.EnemyPose.enemy_pitch, env.EnemyPose.enemy_dist)
#     mode.gimbal_mode = GIMBAL_POSITION_MODE # #云台位置模式，上层控制角度两轴角度
#     mode.chassis_mode = AUTO_FOLLOW_GIMBAL
#     controller.mode_switch(mode)#发送模式
#     print 'shoot!'
#     env.shooting(shoot_cmd,controller)#发射子弹

# elif ():
#     mode.gimbal_mode = GIMBAL_POSITION_MODE # #云台位置模式，上层控制角度两轴角度
#     mode.chassis_mode = AUTO_SEPARATE_GIMBAL #底盘和云台分离模式，旋转、平移受上层控制
#     controller.mode_switch(mode)#发送模式
#     env.shooting(shoot_cmd,controller)#发射子弹


while True:
    # mode.chassis_mode = AUTO_SEPARATE_GIMBAL  # 底盘和云台分离模式，旋转、平移受上层控制
    # mode.gimbal_mode = GIMBAL_POSITION_MODE  # 云台位置模式，上层控制角度两轴角度
    # controller.mode_switch(mode)  # 发送模式
    # env.shooting_stop(shoot_cmd, controller)
    # print '~~~~~~~~~~~~~~~~~~~~~~~~~env.pose.dis = =%s' % env.EnemyPose.enemy_dist
    # print '~~~~~~~~~~~~~~~~~~~~~~~~~env.pose.yaw = =%s' % env.EnemyPose.enemy_yaw
    # print '////////////////sending My_pose !! goal_x=%s\tgoal_y=%s\tgoal_yaw=%s\n' % (env.MyPose['x'], env.MyPose['y'], env.MyPose['theta'])  # 发送目标
    # tflistener = tf.TransformListener()
    # while True:
    #     try:
    #         trans0s, quat0s = tflistener.lookupTransform("/map", "/robot_0/base_link", rospy.Time(0))
    #         env.getSelfPoseCallback(trans0s, quat0s)
    #         break
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    # env.getSelfPoseCallback()
    # print env.MyPose['x']
    # if (env.EnemyPose.enemy_dist != 0):# 没到跳出来
    if (env.enemyNew):
        env.enemyNew = False
        dist = env.EnemyPoseSave.enemy_dist - 0.7
        yaw = (env.EnemyPoseSave.enemy_yaw + env.Gimbal.enemy_yaw) * 3.1416 / 180
        alpha = env.MyPose['theta'] * 3.1416 / 180
        x = env.MyPose['x']
        y = env.MyPose['y']
        dx = dist * math.cos(yaw)
        dy = dist * math.sin(yaw)
        # print 'alpha=%s\tyaw=%s\t\n'%(alpha,yaw)
        Ts0 = np.mat([[math.cos(alpha), -math.sin(alpha), 0, x], [math.sin(alpha), math.cos(alpha), 0, y], [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        Tse = np.mat(
            [[math.cos(yaw), -math.sin(yaw), 0, dx], [math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0], [0, 0, 0, 1]])

        T = Ts0 * Tse
        # print T,Ts0, Tse
        # Ts0 = np.mat([[math.cos(alpha),math.sin(alpha),0,x],[-math.sin(alpha),math.cos(alpha),0,y],[0,0,1,0],[0,0,0,1]])
        # Tse = np.mat([[math.cos(yaw), math.sin(yaw), 0, dx], [-math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0], [0, 0, 0, 1]])
        # T=Ts0*Tse
        goal_x = T[0, 3]
        goal_y = T[1, 3]
        goal_yaw = (np.arctan2(T[1, 0], T[0, 0])) * 180 / 3.1415926
        last_x = goal_x
        last_y = goal_y
        last_yaw = goal_yaw
        env.lastpose = [last_x, last_y, last_yaw]
        # time.sleep(0.5)
        if (env.EnemyPoseSave.enemy_dist >1.5):  # 检测到敌人
            mode.chassis_mode = AUTO_SEPARATE_GIMBAL  # 底盘和云台分离模式，旋转、平移受上层控制
            mode.gimbal_mode = GIMBAL_POSITION_MODE  # 云台位置模式，上层控制角度两轴角度
            controller.mode_switch(mode)  # 发送模式
            env.shooting(shoot_cmd, controller)
            print 'shoot======================================================='
            print 'distance larger than 1.5'
            print 'Enemy pose is yaw=%s\tdistance=%s\n' % (
                yaw, dist + 1)

            # goal_x = dist * math.cos(yaw * 3.1415926/180) + env.MyPose['x']
            # goal_y = dist * math.sin(yaw * 3.1415926/180) + env.MyPose['y']
            # print 'flag1'
            if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # 判断目标点是否可行
                print '?????????????????????????????????????????????????????????????????????????'
                print goal_x, goal_y, goal_yaw
                print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                # print 'flag2'
                # num1 = num1 + 1
                # # controller.send_goal(env.navgoal)
                # # print '////////////////sending following goal !! goal_x=%s\tgoal_y=%s\tgoal_yaw=%s\n' % (goal_x, goal_y, goal_yaw)# 发送目标
                # if (num1 > 30):
                controller.send_goal(env.navgoal)
                #print 'go to position x =%s y=%s'%(env.navgoal.pose.position.x, env.navgoal.pose.position.y)
                #     print '////////////////sending following goal !! goal_x=%s\tgoal_y=%s\tgoal_yaw=%s\n' % (goal_x, goal_y, goal_yaw)  # 发送目标
                #     num1 = 0
                #distnow = (goal_x-env.MyPose['x'])*(goal_x-env.MyPose['x']) +(goal_y-env.MyPose['y'])*(goal_y-env.MyPose['y'])
                t = rospy.Time.now().secs
                while((((goal_x-env.MyPose['x'])*(goal_x-env.MyPose['x']) +(goal_y-env.MyPose['y'])*(goal_y-env.MyPose['y'])) > 0.3 ) and (rospy.Time.now().secs - t) < 2.5):
                    distnow = (goal_x - env.MyPose['x']) * (goal_x - env.MyPose['x']) + (goal_y - env.MyPose['y']) * (goal_y - env.MyPose['y'])
                    rospy.sleep(0.3)
                    env.shooting(shoot_cmd, controller)
                    print np.sqrt(distnow)
                    continue
                # while( env.EnemyPose.enemy_dist > 1.5):
                #      if(env.EnemyPose.enemy_dist!=0):
                #         continue
                #      else:
                #          break
            else:  # 点不可行，就终止当前循环
                pass
            env.shooting_stop(shoot_cmd, controller)
            env.gimbalYawSave = env.Gimbal.enemy_yaw

        if (env.EnemyPoseSave.enemy_dist < 1.5 and env.EnemyPoseSave.enemy_dist > 0):
            mode.chassis_mode = AUTO_SEPARATE_GIMBAL  #
            mode.gimbal_mode = GIMBAL_POSITION_MODE  # 云台位置模式，上层控制角度两轴角度
            controller.mode_switch(mode)  # 发送模式
            print 'distance less than 1.5'
            print 'Enemy pose is yaw=%s\tdistance=%s\n' % (
                env.EnemyPoseSave.enemy_yaw, env.EnemyPoseSave.enemy_dist)
            #goal_yaw = env.MyPose['theta']# * 3.1416/180
            if env.isActionAvaliable(env.MyPose['x'], env.MyPose['y'], goal_yaw):  # 判断目标点是否可行
                controller.send_goal(env.navgoal)
                print 'stay here x =%s y=%s' % (env.navgoal.pose.position.x, env.navgoal.pose.position.y)

            # if abs(dist * math.cos(yaw * 3.1415926/180))<1 and abs(dist * math.sin(yaw * 3.1415926/180))<1:
            env.shooting_plus(shoot_cmd, controller)  # 靠近1米 连发
            #t = rospy.Time.now().secs
            while (env.EnemyPoseSave.enemy_dist < 1.5 and env.EnemyPoseSave.enemy_dist > 0):
                print 'shoot +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++'
            env.shooting_stop(shoot_cmd, controller)
            env.gimbalYawSave = env.Gimbal.enemy_yaw

        # print 'goal_x=%s\tgoal_y=%s\tgoal_yaw=%s\n' % (goal_x, goal_y, goal_yaw)
        # print 'at now:alpha=%s\tgoal_yaw=%s\tyaw=%s\t\n' % (alpha,goal_yaw,180-yaw)

    else:
        # plan.patrol_path(env,controller) #发送目标
        print 'no target times!'
        if(env.enemyNew == False):
            # env.gimbalYawSave = env.Gimbal.enemy_yaw
            mode.gimbal_mode = GIMBAL_PATROL_MODE  # 巡逻模式，云台 yaw 周期运动，pitch 不受控制
            mode.chassis_mode = AUTO_SEPARATE_GIMBAL  # 底盘和云台分离模式，旋转、平移受上层控制
            controller.mode_switch(mode)  # 发送模式
            print '=========================patrol random goal!================================'
            plan.random_path(env,controller)


















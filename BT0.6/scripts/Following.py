# -*- coding: utf-8 -*-
import rospy
import random
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion,Twist, PoseStamped
from nav_msgs.msg import Odometry
from teleop_controller.msg import ShootCmd, EnemyPos
import time
import math
import tf
from teleop_control import Controller
from Battle import BattleEnv 


class Follow():
    def __init__(self):
        pass

    def axis_trans(self,env,controller):
        pass



    def shoot_dodge(self,env,controller):
        pass 
    def shoot_follow(self,env,controller):
        while (True):
            if (env.EnemyPose.enemy_dist != 0):  # 没到跳出来
                if (env.EnemyPose.enemy_dist > 1):  # 检测到敌人
                    mode.chassis_mode = AUTO_SEPARATE_GIMBAL  # 底盘和云台分离模式，旋转、平移受上层控制
                    mode.gimbal_mode = GIMBAL_POSITION_MODE  # 云台位置模式，上层控制角度两轴角度
                    controller.mode_switch(mode)  # 发送模式
                    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, env.getSelfPoseCallback)
                    dist = env.EnemyPose.enemy_dist - 0.5
                    x = env.MyPose['x']
                    y = env.MyPose['y']
                    alpha = env.MyPose['theta'] * 3.1416 / 180
                    yaw = env.EnemyPose.enemy_yaw * 3.1416 / 180
                    dx = dist * math.cos(yaw)
                    dy = dist * math.sin(yaw)
                    # print 'alpha=%s\tyaw=%s\t\n'%(alpha,yaw)
                    Ts0 = np.mat([[math.cos(alpha), -math.sin(alpha), 0, x], [math.sin(alpha), math.cos(alpha), 0, y],
                                  [0, 0, 1, 0], [0, 0, 0, 1]])
                    Tse = np.mat(
                        [[math.cos(yaw), -math.sin(yaw), 0, dx], [math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0],
                         [0, 0, 0, 1]])

                    T = Ts0 * Tse
                    print T, Ts0, Tse
                    # Ts0 = np.mat([[math.cos(alpha),math.sin(alpha),0,x],[-math.sin(alpha),math.cos(alpha),0,y],[0,0,1,0],[0,0,0,1]])
                    # Tse = np.mat([[math.cos(yaw), math.sin(yaw), 0, dx], [-math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0], [0, 0, 0, 1]])
                    # T=Ts0*Tse
                    goal_x = T[0, 3]
                    goal_y = T[1, 3]
                    # print T[0,1]
                    # print T[1,1]
                    goal_yaw = (np.arctan2(T[1, 0], T[0, 0])) * 180 / 3.1415926
                    print goal_yaw
                    # goal_x = dist * math.cos(yaw * 3.1415926/180) + env.MyPose['x']
                    # goal_y = dist * math.sin(yaw * 3.1415926/180) + env.MyPose['y']
                    if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # 判断目标点是否可行
                        controller.send_goal(env.navgoal)  # 发送目标
                        time.sleep(1.5)
                        print 'sending shoot goal'
                    else:  # 点不可行，就终止当前循环
                        pass

                    env.shooting(shoot_cmd, controller)
                else:

                    mode.chassis_mode = AUTO_FOLLOW_GIMBAL  # 底盘和云台分离模式，旋转、平移受上层控制
                    mode.gimbal_mode = GIMBAL_POSITION_MODE  # 云台位置模式，上层控制角度两轴角度
                    controller.mode_switch(mode)  # 发送模式
                    if abs(dist * math.cos(yaw * 3.1415926 / 180)) < 0.5 and abs(
                            dist * math.sin(yaw * 3.1415926 / 180)) < 0.5:
                        env.shooting_plus(shoot_cmd, controller)  # 靠近1米 连发
                    else:
                        pass
                    print 'shoot ++'

                print 'goal_x=%s\tgoal_y=%s\tgoal_yaw=%s\n' % (goal_x, goal_y, goal_yaw)
                print 'at now:alpha=%s\tgoal_yaw=%s\tyaw=%s\t\n' % (alpha, goal_yaw, 180 - yaw)




if __name__ == '__main__':
    follow =Follow()
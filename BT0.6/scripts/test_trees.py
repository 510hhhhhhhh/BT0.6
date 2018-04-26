#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Battle import BattleEnv
from teleop_control import Controller
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW
import time
import math
import numpy as np
import tf
from pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *

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
tflistener = tf.TransformListener()
#定义控制命令
vel = Twist()
nav_goal = env.navgoal
shoot_cmd = ShootCmd()
mode = ModeSW()

controller.shoot_cmd.fric_wheel_spd = 1215
controller.shoot_cmd.fric_wheel_run = 1
controller.shoot_cmd.c_shoot_cmd = 0
controller.shoot_cmd.shoot_cmd =0
controller.shoot(controller.shoot_cmd)

#接受机器人状态消息
rospy.Subscriber('gimbalpos', EnemyPos, env.getGimbalPoseCallback)
rospy.Subscriber('my_pose', Odometry, env.getSelfPoseCallback)
time.sleep(0.1)
rospy.Subscriber('/enemy_pos', EnemyPos, env.getEnemyPoseCallback)
time.sleep(0.1)

class BiuldTree():
    def __init__(self):
        # rospy.init_node('simple battle tree', anonymous=False)
        rate = rospy.Rate(10)
        BAHAVE = Sequence("BEHAVE")

        GET_BUFF = Selector("GET_BUFF")
        BATTLE = Selector("BATTLE")

        BAHAVE.add_child(GET_BUFF)
        BAHAVE.add_child(BATTLE)

        ISENEMY = CheckEnemy("isEnemy", blackboard=env)
        CHECKBUFF = CheckBuff("hasBuff", blackboard=env)
        GOTOCENTER = GoToCenterTask("GOTOCENTER", controller=controller, blackboard=env)

        GET_BUFF.add_child(ISENEMY)
        GET_BUFF.add_child(CHECKBUFF)
        GET_BUFF.add_child(GOTOCENTER)

        CLOSEATTACK = Sequence("FASTSHOOT")
        FARATTACK = Sequence("CLOSESHOOT")
        PATROL = Sequence("PATROL")

        BATTLE.add_child(CLOSEATTACK)
        BATTLE.add_child(FARATTACK)
        BATTLE.add_child(PATROL)

        ISCLOSE = IsClose("ISCLOSE", controller=controller, blackboard=env)
        FASTSHOOT = FastShoot("FASTSHOOT", controller=controller)
        STOPCHASSIS = StopChassis("STOPCHASSIS", controller=controller, blackboard=env)

        ISFAR = IsFar("ISFAR", controller=controller, blackboard=env)
        SINGLESHOOT = SingleShoot("FASTSHOOT", controller=controller)
        GETCLOSE = GetClose("CETCLOSE", controller=controller, blackboard=env)

        CLOSEATTACK.add_child(ISCLOSE)
        CLOSEATTACK.add_child(FASTSHOOT)
        CLOSEATTACK.add_child(STOPCHASSIS)

        FARATTACK.add_child(ISFAR)
        FARATTACK.add_child(SINGLESHOOT)
        FARATTACK.add_child(GETCLOSE)

        CONFORMENEMY =ConformEnemy("CONFORMENEMY", controller=controller, blackboard=env)
        TURNAROUND = TurnAround("TURNAROUND", controller=controller, blackboard=env)
        RANDOMPOINT = RandomPoint("RANDOMPOINT", controller=controller, blackboard=env)

        PATROL.add_child(CONFORMENEMY)
        PATROL.add_child(TURNAROUND)
        PATROL.add_child(RANDOMPOINT)

        print "Bahave Tree"
        print_tree(BAHAVE)

        while not rospy.is_shutdown():
            BAHAVE.status = BAHAVE.run()
            rate.sleep()

class CheckEnemy(Task):
    def __init__(self, name, blackboard=None):
        super(CheckEnemy, self).__init__(name)
        print "init!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        self.name = name
        self.blackboard = blackboard
    def run(self):
        if self.blackboard.enemyNew:
            env.sended = False
            print "SEE ENEMY"
            return TaskStatus.SUCCESS
        else:
            print "NO ENEMY"
            return TaskStatus.FAILURE

    def reset(self):
        self.status = None

class GoToCenterTask(Task):
    def __init__(self, name, controller, blackboard):
        super(GoToCenterTask, self).__init__(name)

        self.name = name
        self.controller = controller
        self.blackboard = blackboard

    def run(self):
        goal_x = 4.0
        goal_y = 2.5
        goal_yaw = -5

        time2 = rospy.Time.now().secs
        if (self.blackboard.getbuff == False):
            if env.sended == False:
                print 'sending center_goal successful'
                env.isActionAvaliable(goal_x, goal_y, goal_yaw)
                self.controller.send_goal(env.navgoal)  # 发送目标
                env.sended = True
            print "On the way to get buff"
            return TaskStatus.RUNNING
        else:
            print "get buff"
            env.sended = False
            return TaskStatus.SUCCESS

class CheckBuff(Task):
    def __init__(self, name, blackboard):
        super(CheckBuff, self).__init__(name)

        self.name = name
        self.blackboard = blackboard
        self.flag = 0
        self.timestart = rospy.Time.now().secs
    def run(self):
        if (pow(self.blackboard.MyPose['x'] - 4, 2) + pow(self.blackboard.MyPose['y'] - 2.5, 2)) < 0.04 or env.getbuff == True:
            if self.flag == 0:
                self.timestart = rospy.Time.now().secs
                self.flag = 1
            if (rospy.Time.now().secs - self.timestart > 5) or env.getbuff == True:
                env.getbuff = True
                print "get buff!!!"
                return TaskStatus.SUCCESS
            else:
                print "Wait for %s seconds for buff" % (rospy.Time.now().secs - self.timestart)
                return TaskStatus.FAILURE
        print "no buff"
        return TaskStatus.FAILURE

class IsClose(Task):
    def __init__(self, name, controller, blackboard):
        super(IsClose, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.controller = controller
    def run(self):
        if self.blackboard.EnemyPoseSave.enemy_dist < 1.5 and self.blackboard.EnemyPoseSave.enemy_dist > 0:
            print "enemy distance is less than 1.5 meter"
            return TaskStatus.SUCCESS
        else:
            self.controller.shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
            self.controller.shoot_cmd.shoot_cmd = 0  # 单发命令
            self.controller.shoot_cmd.c_shoot_cmd = 0  # 连发命令
            self.controller.shoot(self.controller.shoot_cmd)
            print "no close enemy, Stop SHOOTING"
            return TaskStatus.FAILURE

class ConformEnemy(Task):
    def __init__(self, name, controller, blackboard):
        super(ConformEnemy, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.controller = controller
    def run(self):
        if self.blackboard.enemyNew == False:
            print "no enemy, i am patroling"
            return TaskStatus.SUCCESS
        else:
            # self.controller.shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
            # self.controller.shoot_cmd.shoot_cmd = 0  # 单发命令
            # self.controller.shoot_cmd.c_shoot_cmd = 0  # 连发命令
            # self.controller.shoot(self.controller.shoot_cmd)
            print "have enemy, SHOOTING"
            return TaskStatus.FAILURE

class FastShoot(Task):
    def __init__(self, name, controller):
        super(FastShoot, self).__init__(name)
        self.name = name
        self.controller = controller
    def run(self):
        print "SHOOTING++SHOOTING+++SHOOTING++SHOOTING++SHOOTING+++++++++++++++++++"
        self.controller.shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        self.controller.shoot_cmd.shoot_cmd = 0  # 单发命令
        self.controller.shoot_cmd.c_shoot_cmd = 1  # 连发命令
        self.controller.shoot(self.controller.shoot_cmd)
        return TaskStatus.SUCCESS

class SingleShoot(Task):
    def __init__(self, name, controller):
        super(SingleShoot, self).__init__(name)
        self.name = name
        self.controller = controller
    def run(self):
        print "SHOOTING++++++++++++++++++++++++"
        self.controller.shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        self.controller.shoot_cmd.shoot_cmd = 1  # 单发命令
        self.controller.shoot_cmd.c_shoot_cmd = 0  # 连发命令
        self.controller.shoot(self.controller.shoot_cmd)
        return TaskStatus.SUCCESS

class IsFar(Task):
    def __init__(self, name, controller, blackboard):
        super(IsFar, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.controller = controller
    def run(self):
        if self.blackboard.EnemyPoseSave.enemy_dist > 1.5:
            print "enemy distance is more than 1.5 meter"
            return TaskStatus.SUCCESS
        else:
            self.controller.shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
            self.controller.shoot_cmd.shoot_cmd = 0  # 单发命令
            self.controller.shoot_cmd.c_shoot_cmd = 0  # 连发命令
            self.controller.shoot(self.controller.shoot_cmd)
            print "no far enemy, Stop SHOOTING"
            return TaskStatus.FAILURE

class GetClose(Task):
    def __init__(self, name, controller, blackboard):
        super(GetClose, self).__init__(name)

        self.name = name
        self.controller = controller
        self.blackboard = blackboard
        self.sended = False

    def run(self):
        # dist = self.blackboard.EnemyPoseSave.enemy_dist - 1.0
        # yaw = (self.blackboard.EnemyPoseSave.enemy_yaw + self.blackboard.Gimbal.enemy_yaw) * 3.1416 / 180
        # alpha = self.blackboard.MyPose['theta'] * 3.1416 / 180
        # x = self.blackboard.MyPose['x']
        # y = self.blackboard.MyPose['y']
        #
        # dx = dist * math.cos(yaw)
        # dy = dist * math.sin(yaw)
        # Ts0 = np.mat([[math.cos(alpha), -math.sin(alpha), 0, x], [math.sin(alpha), math.cos(alpha), 0, y], [0, 0, 1, 0],
        #               [0, 0, 0, 1]])
        # Tse = np.mat(
        #     [[math.cos(yaw), -math.sin(yaw), 0, dx], [math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0], [0, 0, 0, 1]])
        #
        # T = Ts0 * Tse
        # goal_x = T[0, 3]
        # goal_y = T[1, 3]
        # goal_yaw = (np.arctan2(T[1, 0], T[0, 0])) * 180 / 3.1415926
        # self.blackboard.goal_yawsave = goal_yaw
        if self.blackboard.getbuff == False:
            return TaskStatus.FAILURE
        if ((pow(self.blackboard.MyPose['x'] - self.blackboard.goal_x, 2) + pow(self.blackboard.MyPose['y'] - self.blackboard.goal_y, 2)) > 0.3):
            if env.sended == False:
                print 'sending center_goal successful'
                if env.isActionAvaliable(self.blackboard.goal_x, self.blackboard.goal_y, self.blackboard.goal_yaw):
                    print 'sending goal successful'
                    self.controller.send_goal(env.navgoal)  # 发送目标
                    env.sended = True
            print "On the way to get close, distance is %s" % np.sqrt(pow(self.blackboard.MyPose['x'] - self.blackboard.goal_x, 2) + pow(self.blackboard.MyPose['y'] - self.blackboard.goal_y, 2))
            return TaskStatus.RUNNING
        else:
            print "get closed"
            env.sended = False
            return TaskStatus.SUCCESS

class StopChassis(Task):
    def __init__(self, name, controller, blackboard):
        super(StopChassis, self).__init__(name)

        self.name = name
        self.controller = controller
        self.blackboard = blackboard
        self.lastcount = 0
    def run(self):
        if env.sended == False:
            self.blackboard.lastgoal[2] = self.blackboard.goal_yaw
        # print 'MyPose %s' % (env.MyPose['theta'])
        # print 'count %s' % (count)
        if np.abs(self.blackboard.MyPose['theta'] - self.blackboard.lastgoal[2]) > 30:
            if env.sended == False:
                if env.isActionAvaliable(env.MyPose['x'], env.MyPose['y'], self.blackboard.lastgoal[2]):  # 共转1圈
                    self.controller.send_goal(env.navgoal)
                    print 'send self pose'
                    env.sended = True
            return TaskStatus.RUNNING
        else:
            print "Stop at a close position"
            env.sended = False
            return TaskStatus.SUCCESS

class TurnAround(Task):
    def __init__(self, name, controller, blackboard):
        super(TurnAround, self).__init__(name)

        self.name = name
        self.controller = controller
        self.blackboard = blackboard
        self.lastcount = 0
    def run(self):
        if env.isturned == True:
            return TaskStatus.SUCCESS
        if self.blackboard.MyPose['theta'] > 0:
            count = self.blackboard.MyPose['theta'] + 180
        else:
            count = self.blackboard.MyPose['theta'] - 180
            # if count > 180:
            # count = count - 360
        if count > 180:
            count = -360 + count
        if count < -180:
            count = 360 + count
        if env.sended == False:
            self.blackboard.lastgoal[2] = count
        # print 'MyPose %s' % (env.MyPose['theta'])
        # print 'count %s' % (count)
        if np.abs(self.blackboard.MyPose['theta'] - self.blackboard.lastgoal[2])> 30:
            if env.sended == False:
                if env.isActionAvaliable(env.MyPose['x'], env.MyPose['y'], self.blackboard.lastgoal[2]):  # 共转1圈
                    self.controller.send_goal(env.navgoal)
                    print 'twist----twist----twist----twist----twist----twist----'
                    env.sended = True
            print "i am turning around, the rest angle is %s degree" % (self.blackboard.MyPose['theta'] - self.blackboard.lastgoal[2])
            env.isturned = False
            return TaskStatus.RUNNING
        else:
            print "get turned"
            env.isturned = True
            env.sended = False
            return TaskStatus.SUCCESS

class RandomPoint(Task):
    def __init__(self, name, controller, blackboard):
        super(RandomPoint, self).__init__(name)

        self.name = name
        self.controller = controller
        self.blackboard = blackboard

        self.lastgoal_x = 0
        self.lastgoal_y = 0
        self.lastgoal_yaw = 0
    def run(self):
        up_x =env.lastpose[0] + 1.5
        down_x =env.lastpose[0] -1.5
        up_y =  env.lastpose[1] + 1.5
        down_y =env.lastpose[1] -1.5
        goal_x = np.random.uniform(down_x, up_x)
        goal_y = np.random.uniform(down_y, up_y)
        goal_yaw = 0
        if env.sended == False:
            self.blackboard.lastgoal[0] = goal_x
            self.blackboard.lastgoal[1] = goal_y
            self.blackboard.lastgoal[2] = goal_yaw
        # print env.sended
        if ((pow(self.blackboard.MyPose['x'] - self.blackboard.lastgoal[0], 2) + pow(self.blackboard.MyPose['y'] - self.blackboard.lastgoal[1], 2)) > 0.1):
            if env.sended == False:
                if env.isActionAvaliable(self.blackboard.lastgoal[0], self.blackboard.lastgoal[1], self.blackboard.lastgoal[2]):
                    print 'sending patrol goal successful'
                    self.controller.send_goal(env.navgoal)  # 发送目标
                    env.sended = True
            print "On the way to get patrol point, distance is %s" % np.sqrt(pow(self.blackboard.MyPose['x'] - self.blackboard.lastgoal[0], 2) + pow(self.blackboard.MyPose['y'] - self.blackboard.lastgoal[1], 2))
            env.goingtopoint = True
            return TaskStatus.RUNNING
        else:
            print "reach a point"
            env.goingtopoint = False
            env.isturned = False
            env.sended = False
            return TaskStatus.SUCCESS

if __name__ == '__main__':
    tree = BiuldTree()


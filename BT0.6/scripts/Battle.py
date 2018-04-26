# -*- coding: utf-8 -*-
import sys
import numpy as np
from PIL import Image
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from teleop_controller.msg import ShootCmd, EnemyPos
import time
import math
import tf
from teleop_control import Controller



class BattleEnv():
    def __init__(self):
        self.action_space = ['N', 'E', 'W', 'S', 'NE', 'NW', 'SE', 'SW', 'shoot']
        self.map = np.array(Image.open("icra2.pgm"))
        self.shoot_pub = rospy.Publisher('shoot_cmd', ShootCmd, queue_size=1)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        self.count= 0
        self.navgoal = PoseStamped()
        self.navgoal.header.frame_id = 'map'
        #self.navgoal.pose.orientation.w = 1.0
        self.shoot = ShootCmd()
        self.MyPose = {'x': 0, 'y': 0, 'theta': 0}
        self.EnemyPose = EnemyPos()
        self.Gimbal = EnemyPos()
        self.enemyNew = False
        self.EnemyPoseSave = EnemyPos()
        self.totalhurt = [0, 0]
        self.lastpose = [4, 2.5, 0]
        self.lastgoal = [4, 2.5, 0]
        self.goal_x = 4
        self.goal_y = 2.5
        self.goal_yaw = 0
        self.goingtopoint = False
        self.isturned = False
        self.goal_yawsave = 0
        self.num = 0
        self.getbuff = False
        self.sended = False
        self.gimbalYawSave = 0
        rospy.init_node('BattleSim')

    def reset(self):
        # 重置仿真环境
        initialstates = np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0])  # [xs,ys,dirs,enemy_yaw,enemy_pitch,enemy_dist,isTarget,isTargeted]
        self.totalhurt = [0, 0]
        return initialstates

    # def gettransform(self):

    def getSelfPoseCallback(self, data):
        #data = PoseWithCovarianceStamped()
        self.MyPose['x'] = data.pose.pose.position.x
        self.MyPose['y'] = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        # tflistener = tf.TransformListener()
        # while True:
        #     try:
        #         t, q = tflistener.lookupTransform("/map", "/robot_0/base_link", rospy.Time(0))
        #         self.MyPose['x'] = t[0]
        #         self.MyPose['y'] = t[1]
        #         qx = q[0]
        #         qy = q[1]
        #         qz = q[2]
        #         qw = q[3]
        #         angle = math.atan2(2 * (qx * qy + qz * qw), qw * qw + qx * qx - qy * qy - qz * qz) * 180 / 3.1415926
        #
        #         self.MyPose['theta'] = angle
        #         break
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         continue
        # self.MyPose['x'] = t[0]
        # self.MyPose['y'] = t[1]
        # qx = q[0]
        # qy = q[1]
        # qz = q[2]
        # qw = q[3]
        angle = math.atan2(2 * (qx * qy + qz * qw), qw * qw + qx * qx - qy * qy - qz * qz)* 180 / 3.1415926
        self.MyPose['theta'] = angle
        # print('UPDATE POSE++++++++++++++++++++++++++++++++++++++++++++++++')
        # print(qx, qy, qz, qw, angle)

    def getEnemyPoseCallback(self, data):
        # data = EnemyPos()
        self.EnemyPose.enemy_yaw = - data.enemy_yaw
        self.EnemyPose.enemy_pitch = data.enemy_pitch
        self.EnemyPose.enemy_dist = data.enemy_dist / 1000
        if self.EnemyPose.enemy_dist != 0:
            self.num = 0
            self.enemyNew = True
            self.EnemyPoseSave.enemy_yaw = - data.enemy_yaw
            self.EnemyPoseSave.enemy_pitch = data.enemy_pitch
            self.EnemyPoseSave.enemy_dist = data.enemy_dist / 1000
        else:
            self.num = self.num + 1
        if self.num > 100:
            self.enemyNew = False
            self.EnemyPoseSave.enemy_yaw = 0
            self.EnemyPoseSave.enemy_pitch = 0
            self.EnemyPoseSave.enemy_dist = 0
            self.num = 0

        dist = self.EnemyPoseSave.enemy_dist - 1.0
        yaw = (self.EnemyPoseSave.enemy_yaw + self.Gimbal.enemy_yaw) * 3.1416 / 180
        alpha = self.MyPose['theta'] * 3.1416 / 180
        x = self.MyPose['x']
        y = self.MyPose['y']

        dx = dist * math.cos(yaw)
        dy = dist * math.sin(yaw)
        Ts0 = np.mat([[math.cos(alpha), -math.sin(alpha), 0, x], [math.sin(alpha), math.cos(alpha), 0, y], [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        Tse = np.mat(
            [[math.cos(yaw), -math.sin(yaw), 0, dx], [math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0], [0, 0, 0, 1]])

        T = Ts0 * Tse
        self.goal_x = T[0, 3]
        self.goal_y = T[1, 3]
        self.goal_yaw = (np.arctan2(T[1, 0], T[0, 0])) * 180 / 3.1415926


    def getGimbalPoseCallback(self, data):
        # data = EnemyPos()
        self.Gimbal.enemy_yaw = data.enemy_yaw
        self.Gimbal.enemy_pitch = data.enemy_pitch
        self.Gimbal.enemy_dist = data.enemy_dist
        # if self.EnemyPose.enemy_dist != 0:
        #     self.enemyNew = True
        # print('Get enemy pose')

    '''
    仿真步进，根据状态动作输出回报
    '''
    '''def step(self, action, observation):
        # 判断目标的点是否可达
        if self.isActionAvaliable(action, observation):
            # take move or shoot
            self.shoot_pub.publish(self.shoot)
            self.goal_pub.publish(self.navgoal)

            observationnew = self.calcAttackArea(observation)  # 计算攻击区域

            isshoot, ishurt = self.calcShootProb(observationnew, action)  # 计算命中率

            if observationnew[6] == 1:  # target enemy
                movereward = 1
                if observationnew[7] == 0:  # not targeted
                    movereward = 1.5
            elif observationnew[7] == 0:  # both side not target
                movereward = -1
            elif observationnew[7] == 1:  # only be targeted
                movereward = -1.5
            else:
                movereward = 0

            if isshoot == 1:  # shoot enemy
                shootreward = 2
            else:
                shootreward = -1

            if ishurt == 1:  # be shooted by enemy
                hurtreward = -2
            else:
                hurtreward = 0
        else:
            # take shoot or nothing
            self.shoot_pub.publish(self.shoot)
            observationnew = self.calcAttackArea(observation)  # 计算攻击区域
            isshoot, ishurt = self.calcShootProb(observationnew, action)
            movereward = -1

            if isshoot == 1:  # shoot enemy
                shootreward = 2
            else:
                shootreward = -1

            if ishurt == 1:  # be shooted by enemy
                hurtreward = -2
            else:
                hurtreward = 0

        reward = movereward + shootreward + hurtreward
        done = False
        if self.totalhurt[0] > 1500 or self.totalhurt[1] > 1500:
            done = True
        return observationnew, reward, done'''

    def isActionAvaliable(self, px, py, theta):
        #orin = Quaternion()
        #goal = PoseStamped()
        ok = False
        x_goal = int(px / 0.05)
        y_goal = int(py / 0.05)
        quat = tf.transformations.quaternion_from_euler(0, 0, theta*3.1416/180)
        if (y_goal<=15)  or (y_goal >= 107) or (x_goal >= 165) or (x_goal <= 15):
            print('out of map!!!!!')
            pass
        elif self.map[101 - y_goal +10, x_goal + 10] == 255:  # 是否在地图可行区域
            self.navgoal.pose.position.x = x_goal * 0.05  # 101X161地图栅格
            self.navgoal.pose.position.y = y_goal * 0.05
            self.navgoal.pose.orientation.x = quat[0]
            self.navgoal.pose.orientation.y = quat[1]
            self.navgoal.pose.orientation.z = quat[2]
            self.navgoal.pose.orientation.w = quat[3]
            self.navgoal.header.stamp = rospy.Time().now()
            ok = True
        else:
            print 'cant move!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            pass
        '''if shoot == 1:  # Shoot
            self.shoot.shoot_cmd = 1
        else:
            pass'''
        return ok
        # return states


    '''
    计算攻击区域范围，通过位置状态判断敌人是否进入攻击区域，返回攻击所造成伤害
    '''

    def calcAttackArea(self, observation):
        observation[0] = self.MyPose['x']
        observation[1] = self.MyPose['y']
        observation[2] = self.MyPose['theta']
        observation[3] = self.MyPose['x'] + np.cos((self.MyPose['theta'] + self.EnemyPose.enemy_yaw) * 180 / 3.1416)
        observation[4] = self.MyPose['y'] + np.sin((self.MyPose['theta'] + self.EnemyPose.enemy_yaw) * 180 / 3.1416)
        observation[5] = self.EnemyPose.enemy_dist
        if self.enemyNew and (
                self.EnemyPose.enemy_yaw < 90 or self.EnemyPose.enemy_yaw > -90) and self.EnemyPose.enemy_dist < 3:
            observation[6] = 1  # targeted
            observation[7] = 1
        else:
            observation[6] = 0
            observation[7] = 0
        return observation

    '''
    计算射击命中率
    '''

    def calcShootProb(self, observation, action):
        p = 0
        if observation[6] == 0 or action[4] == 0:  # 未瞄或未射击
            p = 0  # binomial distribution
        elif observation[5] > 3:
            p = 0.4
        elif observation[5] > 2 and observation[5] < 3:
            p = 0.6
        elif observation[5] > 1 and observation[5] < 2:
            p = 0.8
        elif observation[5] > 0 and observation[5] < 1:
            p = 0.9
        if observation[6] == 1 and action[4] == 0:
            pp = 0.25
        else:
            pp = p
        isshoot = np.random.binomial(1, p)
        ishurt = np.random.binomial(1, pp)
        shootreward = isshoot * 50
        hurt = ishurt * 50
        self.totalhurt[0] = self.totalhurt[0] + shootreward
        self.totalhurt[1] = self.totalhurt[1] + hurt
        return isshoot, ishurt

    '''计算收到伤害'''

    def getDamage(self):
        pass

    def render(self):
        pass

    def shoot_fric_wheel(self,shoot_cmd,controller):
        shoot_cmd.fric_wheel_spd = 1215  # 摩擦轮转速设置
    #发弹命令
    def shooting(self,shoot_cmd,controller):
        shoot_cmd.fric_wheel_run = 1#开关摩擦轮
        shoot_cmd.shoot_cmd = 1#单发命令
        shoot_cmd.c_shoot_cmd =0#连发命令
        controller.shoot(shoot_cmd)#发布射击命令

    def shooting_plus(self, shoot_cmd, controller):
      #  shoot_cmd.fric_wheel_spd = 1215  # 摩擦轮转速设置
        shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        shoot_cmd.shoot_cmd = 0  # 单发命令
        shoot_cmd.c_shoot_cmd = 1  # 连发命令
        controller.shoot(shoot_cmd)  # 发布射击命令
    def shooting_stop(self, shoot_cmd, controller):
      #  shoot_cmd.fric_wheel_spd = 1215# 摩擦轮转速设置
        shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        shoot_cmd.shoot_cmd = 0  # 单发命令
        shoot_cmd.c_shoot_cmd = 0  # 连发命令
        controller.shoot(shoot_cmd)  # 发布射击命令


if __name__ == '__main__':
    env = BattleEnv()
    initialobservation = env.reset()
    action = [1, 0, 0, 0, 0]
    #env.step(action, initialobservation)
    # Image._show(env.map)
    print(env.map)

# 模式３　发布目标点 ,引入检测环节　if判断(是否到达目标点or是否检测到目标)
mode.chassis_mode = AUTO_FOLLOW_GIMBAL   # 底盘跟随云台，平移受上层控制
mode.gimbal_mode = GIMBAL_POSITION_MODE  # 云台位置模式，上层控制角度两轴角度
controller.mode_switch(mode)  # 发送模式

# 发布目标点
goal_x = 4
goal_y = 2.5
goal_yaw = 0
if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # 判断目标点是否可行
    controller.send_goal(env.navgoal)  # 发送目标
    print 'sending goal successful'
else:
    pass
error_x = abs(goal_x-env.MyPose['x'])# 设定误差范围
error_y = abs(goal_y-env.MyPose['y'])
error_yaw = abs(goal_yaw-env.MyPose['theta'])
error_all = [error_x,error_y,error_yaw]
# my_pose = [env.MyPose['x'], env.MyPose['y'], env.MyPose['theta']]
while error_all > [0.3,0.3,1]: #(没有到达目标点并且没有发现目标)
    if env.EnemyPose.enemy_dist != 0:
        break
    error_x = abs(goal_x - env.MyPose['x'])
    error_y = abs(goal_y - env.MyPose['y'])
    error_yaw = abs(goal_yaw - env.MyPose['theta'])
    error_all = [error_x, error_y, error_yaw]

if error_all < [0.3,0.3,1] or env.EnemyPose.enemy_dist != 0:  # 判断是否到达目标点或检测到目标
    if error_all < [0.3,0.3,1]:  # 到达目标点
        if (env.EnemyPose.enemy_dist == 0): #是否检测到敌人
            time.sleep(5)

            #random_goal_pose1 = [0.5,0.5,0.5]
            #goal_pose = [goal_x, goal_y, goal_yaw]
            # goal_pose = [0, 0, 0]
            # goal_pose = random_goal_pose1
        while ( env.EnemyPose.enemy_dist == 0): #如没检测到目标点，一直巡逻
            goal_x = 0.5
            goal_y = 0.5
            goal_yaw = 0
            for i in range(2):
                if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # 判断目标点是否可行
                    controller.send_goal(env.navgoal)  # 发送目标
                    print 'sending goal successful'
                else:                                  #点不可行，就终止当前循环
                    continue

                error_x = abs(goal_x - env.MyPose['x'])  # 设定误差范围
                error_y = abs(goal_y - env.MyPose['y'])
                error_yaw = abs(goal_yaw - env.MyPose['theta'])
                error_all = [error_x, error_y, error_yaw]

                while (error_all > [0.3,0.3,1]):  #行进过程中检测目标点
                    if ( env.EnemyPose.enemy_dist != 0):
                        break
                    else:
                        error_x = abs(goal_x - env.MyPose['x'])  # 设定误差范围
                        error_y = abs(goal_y - env.MyPose['y'])
                        error_yaw = abs(goal_yaw - env.MyPose['theta'])
                        error_all = [error_x, error_y, error_yaw]
                if (env.EnemyPose.enemy_dist != 0):#跳出最外围的while的循环
                    break
                else:
                    goal_x = 4
                    goal_y = 2.5
                    goal_yaw = 0
            if (env.EnemyPose.enemy_dist != 0):  # 跳出最外围的while的循环
                break



    if (env.EnemyPose.enemy_dist != 0):
        while (env.EnemyPose.enemy_dist < 2):  #检测到敌人
            goal_x = env.EnemyPose.enemy_dist * cos(env.EnemyPose.enemy_pitch) *cos(env.EnemyPose.enemy_yaw) + env.MyPose['x']
            goal_y = env.EnemyPose.enemy_dist * cos(env.EnemyPose.enemy_pitch) *sin(env.EnemyPose.enemy_yaw) + env.MyPose['x']
            goal_yaw = env.EnemyPose.enemy_yaw
            if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # 判断目标点是否可行
                controller.send_goal(env.navgoal)  # 发送目标
                print 'sending goal successful'
            else:  # 点不可行，就终止当前循环
                continue

            error_x = abs(goal_x - env.MyPose['x'])
            error_y = abs(goal_y - env.MyPose['y'])
            error_yaw = abs(goal_yaw - env.MyPose['theta'])
            error_all = [error_x, error_y, error_yaw]
            if (error_all < [0.3,0.3,1]):
                shoot_cmd.fric_wheel_spd = 1500  # 摩擦轮转速设置
                shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
                shoot_cmd.shoot_cmd = 1  # 单发命令
                shoot_cmd.c_shoot_cmd = 0  # 连发命令
                controller.shoot(shoot_cmd)  # 发布射击命令
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Vector3
from sensor_msgs.msg import Imu
import math
import numpy as np
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import GetModelState

class MultiTurtleBotController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('multi_turtlebot_controller', anonymous=True)

        # 定义两个机器人的命名空间
        self.robot_namespaces = ['tb3_1', 'tb3_2']

        # 创建发布者列表
        self.vel_publishers = []
        for ns in self.robot_namespaces:
            pub = rospy.Publisher(ns + '/cmd_vel', Twist, queue_size=10)
            self.vel_publishers.append(pub)

        # 创建IMU订阅者
        self.imu_subscribers = []
        for ns in self.robot_namespaces:
            sub = rospy.Subscriber(ns + '/imu', Imu, self.imu_callback, callback_args=ns)
            self.imu_subscribers.append(sub)

        # 位置和朝向初始化
        self.positions = { 'tb3_1': None, 'tb3_2': None }  # 记录两台机器人的位置
        self.orientations = { 'tb3_1': None, 'tb3_2': None }  # 记录机器人的朝向

        # 初始化Twist消息
        self.cmd_vel_msgs = [Twist(), Twist()]
        # 设置圆周运动的线速度和角速度
        self.cmd_vel_msgs[0].linear.x = 0.2  # 机器人1的线速度
        self.cmd_vel_msgs[0].angular.z = 0.1  # 机器人1的角速度

        self.cmd_vel_msgs[1].linear.x = 0.2  # 机器人2的线速度
        self.cmd_vel_msgs[1].angular.z = -0.1  # 机器人2的角速度（反方向转动）

        # 创建 Marker 发布器
        self.marker_publishers = {}
        for ns in self.robot_namespaces:
            self.marker_publishers[ns] = rospy.Publisher(ns + '/visualization_marker', Marker, queue_size=10)

        # 等待 Gazebo 服务
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        rospy.loginfo("Multi TurtleBot Controller Initialized.")

    def imu_callback(self, msg, namespace):
        # 提取IMU的线加速度和角加速度
        linear_acc = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        angular_acc = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

    def update_positions_from_gazebo(self):
        for ns in self.robot_namespaces:
            try:
                # 获取 Gazebo 中的模型状态
                resp = self.get_model_state_service(ns, '')
                position = resp.pose.position
                orientation = resp.pose.orientation

                # 更新位置和朝向
                self.positions[ns] = (position.x, position.y, position.z)
                self.orientations[ns] = (orientation.x, orientation.y, orientation.z, orientation.w)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to get model state for %s: %s" % (ns, e))

        # 如果两个机器人的位置和朝向都已获取，计算它们之间的距离和相对方向向量
        if self.positions['tb3_1'] and self.positions['tb3_2']:
            x1, y1, z1 = self.positions['tb3_1']
            x2, y2, z2 = self.positions['tb3_2']

            # 计算两个机器人之间的三维距离
            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

            # 计算相对方向向量
            direction_vector = np.array([x2 - x1, y2 - y1, z2 - z1])

            # 获取机器人朝向的旋转矩阵
            q1 = self.orientations['tb3_1']
            q2 = self.orientations['tb3_2']
            R1 = quaternion_matrix(q1)[:3, :3]
            R2 = quaternion_matrix(q2)[:3, :3]

            # 将方向向量转换到每个机器人的坐标系
            direction_robot1 = np.dot(R1.T, direction_vector)
            direction_robot2 = np.dot(R2.T, -direction_vector)

            # 日志记录距离和方向向量
            # rospy.loginfo("Distance between TB3_1 and TB3_2: %.2f", distance)
            # rospy.loginfo("Relative direction vector in TB3_1 frame: [%.2f, %.2f, %.2f]", *direction_robot1)
            # rospy.loginfo("Relative direction vector in TB3_2 frame: [%.2f, %.2f, %.2f]", *direction_robot2)

            # 发布方向向量 Marker
            self.publish_marker('tb3_1', direction_robot1)
            self.publish_marker('tb3_2', direction_robot2)

    def publish_marker(self, ns, direction_vector):
        marker = Marker()
        marker.header.frame_id = ns + "/base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 线宽
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # 设置起点和终点
        origin = Point(0, 0, 0)
        end_point = Point(*direction_vector)
        marker.points.append(origin)
        marker.points.append(end_point)

        self.marker_publishers[ns].publish(marker)

    def run(self):
        rate = rospy.Rate(60)  # 60Hz
        while not rospy.is_shutdown():
            
            # 发布速度命令
            for pub, cmd in zip(self.vel_publishers, self.cmd_vel_msgs):
                pub.publish(cmd)
            
            # 更新两个机器人的位置真值
            self.update_positions_from_gazebo()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = MultiTurtleBotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from my_package.msg import direct

# 定义全局变量来存储数据
x_data, y_data, z_data = [], [], []


def dirct_callback(msg):
    global x_data, y_data, z_data
    # 更新数据
    x_data.append(msg.x)
    y_data.append(msg.y)
    z_data.append(msg.z)


def update_plot(frame):
    ax.clear()
    # 绘制球面
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x_sphere = np.outer(np.cos(u), np.sin(v))
    y_sphere = np.outer(np.sin(u), np.sin(v))
    z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x_sphere, y_sphere, z_sphere, color='b', alpha=0.1, edgecolor='w')

    # 绘制散点图
    ax.scatter(x_data, y_data, z_data, c='r', marker='o')


def image_listener():
    global ax
    rospy.init_node('image_listener', anonymous=True)

    # 创建一个新的 3D 图形对象
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制球面
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x_sphere = np.outer(np.cos(u), np.sin(v))
    y_sphere = np.outer(np.sin(u), np.sin(v))
    z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x_sphere, y_sphere, z_sphere, color='b', alpha=0.1, edgecolor='w')

    # 订阅话题
    rospy.Subscriber('direct_topic', direct, dirct_callback)

    # 设置动画
    ani = FuncAnimation(fig, update_plot, interval=100)

    plt.show()
    rospy.spin()

    # 关闭所有OpenCV窗口
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        image_listener()
    except rospy.ROSInterruptException:
        pass

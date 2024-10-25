import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# 定义时间参数
dt = 0.01  # 时间步长
t_end = 10  # 模拟结束时间
t = np.arange(0, t_end, dt)  # 时间数组

# 物体1的圆周运动参数
radius1 = 2  # 圆的半径
angular_velocity1 = 2 * np.pi / 2  # 角速度（圆周一圈所需时间为10秒）
angular_acceleration1 = 0  # 圆周运动中角加速度为零

# 计算物体1的圆周运动坐标
x1 = radius1 * np.cos(angular_velocity1 * t)
y1 = radius1 * np.sin(angular_velocity1 * t)
z1 = np.zeros_like(t)  # 在xy平面上运动，所以z分量为零

# 计算物体1的速度和加速度
vx1 = -radius1 * angular_velocity1 * np.sin(angular_velocity1 * t)
vy1 = radius1 * angular_velocity1 * np.cos(angular_velocity1 * t)
vz1 = np.zeros_like(t)  # 速度在z轴上为零

ax1 = -radius1 * angular_velocity1**2 * np.cos(angular_velocity1 * t)
ay1 = -radius1 * angular_velocity1**2 * np.sin(angular_velocity1 * t)
az1 = np.zeros_like(t)  # 加速度在z轴上为零

# 物体2的圆周运动参数
radius2 = 1  # 圆的半径
angular_velocity2 = 2 * np.pi / 5  # 角速度（圆周一圈所需时间为10秒）
angular_acceleration2 = 0  # 圆周运动中角加速度为零

# 计算物体2的圆周运动坐标
x2 = radius2 * np.cos(angular_velocity2 * t)
y2 = radius2 * np.sin(angular_velocity2 * t)
z2 = np.zeros_like(t)  # 在xy平面上运动，所以z分量为零

# 计算物体2的速度和加速度
vx2 = -radius2 * angular_velocity2 * np.sin(angular_velocity2 * t)
vy2 = radius2 * angular_velocity2 * np.cos(angular_velocity2 * t)
vz2 = np.zeros_like(t)  # 速度在z轴上为零

ax2 = -radius2 * angular_velocity2**2 * np.cos(angular_velocity2 * t)
ay2 = -radius2 * angular_velocity2**2 * np.sin(angular_velocity2 * t)
az2 = np.zeros_like(t)  # 加速度在z轴上为零

# 计算加速度的大小
acceleration_magnitude1 = np.sqrt(ax1**2 + ay1**2 + az1**2)
acceleration_magnitude2 = np.sqrt(ax2**2 + ay2**2 + az2**2)

# 计算三轴角加速度（在圆周运动中，角加速度在x轴、y轴和z轴上的分量为零）
angular_acceleration_x1 = np.zeros_like(t)
angular_acceleration_y1 = np.zeros_like(t)
angular_acceleration_z1 = np.full_like(t, angular_acceleration1)

angular_acceleration_x2 = np.zeros_like(t)
angular_acceleration_y2 = np.zeros_like(t)
angular_acceleration_z2 = np.full_like(t, angular_acceleration2)

# 计算两个物体之间的距离
distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)

# 绘制结果
fig = plt.figure(figsize=(18, 12))

# 位置图
ax_1 = fig.add_subplot(231, projection='3d')
ax_1.plot(x1, y1, z1, label='Object 1')
ax_1.plot(x2, y2, z2, label='Object 2')
ax_1.set_xlabel('X (m)')
ax_1.set_ylabel('Y (m)')
ax_1.set_zlabel('Z (m)')
ax_1.set_title('Trajectories')
ax_1.legend()

# 加速度图：三轴随时间变化
ax_2 = fig.add_subplot(232)
ax_2.plot(t, ax1, label='Object 1 Acceleration X', color='r')
ax_2.plot(t, ay1, label='Object 1 Acceleration Y', color='g')
ax_2.plot(t, az1, label='Object 1 Acceleration Z', color='b')
ax_2.plot(t, ax2, label='Object 2 Acceleration X', linestyle='--', color='r')
ax_2.plot(t, ay2, label='Object 2 Acceleration Y', linestyle='--', color='g')
ax_2.plot(t, az2, label='Object 2 Acceleration Z', linestyle='--', color='b')
ax_2.set_xlabel('Time (s)')
ax_2.set_ylabel('Acceleration (m/s²)')
ax_2.set_title('Accelerations (X, Y, Z)')
ax_2.legend()

# 角加速度图
ax_3 = fig.add_subplot(233)
ax_3.plot(t, angular_acceleration_x1, label='Object 1 Angular Acceleration X', color='r')
ax_3.plot(t, angular_acceleration_y1, label='Object 1 Angular Acceleration Y', color='g')
ax_3.plot(t, angular_acceleration_z1, label='Object 1 Angular Acceleration Z', color='b')
ax_3.plot(t, angular_acceleration_x2, label='Object 2 Angular Acceleration X', linestyle='--', color='r')
ax_3.plot(t, angular_acceleration_y2, label='Object 2 Angular Acceleration Y', linestyle='--', color='g')
ax_3.plot(t, angular_acceleration_z2, label='Object 2 Angular Acceleration Z', linestyle='--', color='b')
ax_3.set_xlabel('Time (s)')
ax_3.set_ylabel('Angular Acceleration (rad/s²)')
ax_3.set_title('Angular Accelerations (X, Y, Z)')
ax_3.legend()

# 距离图
ax_4 = fig.add_subplot(234)
ax_4.plot(t, distance, label='Distance between Objects')
ax_4.set_xlabel('Time (s)')
ax_4.set_ylabel('Distance (m)')
ax_4.set_title('Distance between Objects')
ax_4.legend()

plt.tight_layout()
plt.show()
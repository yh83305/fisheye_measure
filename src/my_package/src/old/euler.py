import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# Euler angles in radians
euler_angles = [0.8, 0.8, 0.8]  # [roll, pitch, yaw]
direct = [1, 1, 1]
direct = direct/np.linalg.norm(direct)
print(direct)

# Create a rotation object using the provided Euler angles (XYZ convention)
rotation = R.from_euler('xyz', euler_angles)

# Define the original coordinate system axes
x_axis = np.array([1, 0, 0])
y_axis = np.array([0, 1, 0])
z_axis = np.array([0, 0, 1])

# Rotate the axes using the rotation matrix
rotated_x_axis = rotation.apply(x_axis)
rotated_y_axis = rotation.apply(y_axis)
rotated_z_axis = rotation.apply(z_axis)
rotated_direct = rotation.apply(direct)

print(rotated_x_axis, rotated_y_axis, rotated_z_axis)

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制 x 轴
ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X-axis')  # 红色 x 轴
# 绘制 y 轴
ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y-axis')  # 绿色 y 轴
# 绘制 z 轴
ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z-axis')  # 蓝色 z 轴

# Plot the rotated axes
ax.quiver(0, 0, 0, rotated_x_axis[0], rotated_x_axis[1], rotated_x_axis[2], color='r', linestyle='dashed', label='Rotated X-axis')
ax.quiver(0, 0, 0, rotated_y_axis[0], rotated_y_axis[1], rotated_y_axis[2], color='g', linestyle='dashed', label='Rotated Y-axis')
ax.quiver(0, 0, 0, rotated_z_axis[0], rotated_z_axis[1], rotated_z_axis[2], color='b', linestyle='dashed', label='Rotated Z-axis')
ax.quiver(0, 0, 0, rotated_direct[0], rotated_direct[1], rotated_direct[2], color='y', linestyle='dashed', label='Rotated direct')

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set the plot limits
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

gravity_vector = np.array([0, 0, -1])  # 重力方向，假设指向负 z 轴
ax.quiver(0, 0, 0, gravity_vector[0], gravity_vector[1], gravity_vector[2], color='k', linestyle='dashed', label='gravity_vector')

# 重力方向应与 z 轴方向相反
gravity_vector = gravity_vector / np.linalg.norm(gravity_vector)  # 确保重力向量归一化
# 计算旋转轴和角度
axis = np.cross(rotated_z_axis, -gravity_vector)
axis = axis / np.linalg.norm(axis)  # 归一化旋转轴
angle = np.arccos(np.dot(rotated_z_axis, -gravity_vector))  # 计算夹角
# 构造旋转矩阵，将 z 轴对准重力方向
if np.linalg.norm(axis) > 1e-6:  # 防止零向量的情况
    adjustment_rotation = R.from_rotvec(axis * angle)
    adjusted_x = adjustment_rotation.apply(rotated_x_axis)
    adjusted_y = adjustment_rotation.apply(rotated_y_axis)
    adjusted_z = -gravity_vector
    adjusted_direct = adjustment_rotation.apply(rotated_direct)
else:
    adjusted_x = rotated_x_axis
    adjusted_y = rotated_y_axis
    adjusted_z = -gravity_vector
    adjusted_direct = rotated_direct

ax.quiver(0, 0, 0, adjusted_x[0], adjusted_x[1], adjusted_x[2], color='r', linestyle='dotted', label='adjusted_x')
ax.quiver(0, 0, 0, adjusted_y[0], adjusted_y[1], adjusted_y[2], color='g', linestyle='dotted', label='adjusted_y')
ax.quiver(0, 0, 0, adjusted_z[0], adjusted_z[1], adjusted_z[2], color='b', linestyle='dotted', label='adjusted_z')
ax.quiver(0, 0, 0, adjusted_direct[0], adjusted_direct[1], adjusted_direct[2], color='y', linestyle='dotted', label='adjusted_direct')

plt.show()
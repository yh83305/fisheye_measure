import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

class RelativeOrientation:
    def __init__(self, euler_angles, direct):
        # 从陀螺仪得到测量姿态
        self.euler_angles = euler_angles
        # 测算方向向量
        self.direct = direct / np.linalg.norm(direct)
        # 测量姿态欧拉角对应旋转矩阵
        self.rotation = R.from_euler('xyz', euler_angles)
        self.original_axes = {
            'x': np.array([1, 0, 0]),
            'y': np.array([0, 1, 0]),
            'z': np.array([0, 0, 1])
        }
        self.rotated_axes = self._rotate_axes()
        self.rotated_direct = self.rotation.apply(self.direct)
        self.gravity_vector = np.array([0, 0, -1])

        # Z-Y-X中的roll轴和pitch轴旋转，从测量姿态得到中间帧
        self.adjustment_rotation = None
        # 中间帧姿态
        self.adjusted_axes = None
        # 中间帧的方向向量
        self.adjusted_direct = None

        self._adjust_for_gravity()
        self.plot()

        # 投影角
        self.angle = 0
        self._project_and_plot()

    def _rotate_axes(self):
        return {
            'x': self.rotation.apply(self.original_axes['x']),
            'y': self.rotation.apply(self.original_axes['y']),
            'z': self.rotation.apply(self.original_axes['z'])
        }

    def _adjust_for_gravity(self):
        self.gravity_vector = self.gravity_vector / np.linalg.norm(self.gravity_vector)
        axis = np.cross(self.rotated_axes['z'], -self.gravity_vector)
        axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-6 else axis
        angle = np.arccos(np.dot(self.rotated_axes['z'], -self.gravity_vector))
        if np.linalg.norm(axis) > 1e-6:
            self.adjustment_rotation = R.from_rotvec(axis * angle)
            self.adjusted_axes = {
                'x': self.adjustment_rotation.apply(self.rotated_axes['x']),
                'y': self.adjustment_rotation.apply(self.rotated_axes['y']),
                'z': -self.gravity_vector
            }
            self.adjusted_direct = self.adjustment_rotation.apply(self.rotated_direct)
        else:
            self.adjusted_axes = self.rotated_axes
            self.adjusted_axes['z'] = -self.gravity_vector
            self.adjusted_direct = self.rotated_direct

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 绘制原始坐标轴
        ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X-axis', linestyle='-')
        ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y-axis', linestyle='-')
        ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z-axis', linestyle='-')

        # 绘制旋转后的坐标轴
        for axis_name, axis in self.rotated_axes.items():
            ax.quiver(0, 0, 0, axis[0], axis[1], axis[2],
                      color='r' if axis_name == 'x' else 'g' if axis_name == 'y' else 'b',
                      linestyle='dashed', label=f'Rotated {axis_name.upper()}-axis')

        # 绘制旋转后的方向
        ax.quiver(0, 0, 0, self.rotated_direct[0], self.rotated_direct[1], self.rotated_direct[2],
                  color='y', linestyle='dashed', label='Rotated direct')

        # 绘制重力方向
        ax.quiver(0, 0, 0, self.gravity_vector[0], self.gravity_vector[1], self.gravity_vector[2],
                  color='k', linestyle='dashed', label='Gravity vector')

        # 绘制调整后的坐标轴
        for axis_name, axis in self.adjusted_axes.items():
            ax.quiver(0, 0, 0, axis[0], axis[1], axis[2],
                      color='r' if axis_name == 'x' else 'g' if axis_name == 'y' else 'b',
                      linestyle='dotted', label=f'Adjusted {axis_name.upper()}-axis')

        # 绘制调整后的方向
        ax.quiver(0, 0, 0, self.adjusted_direct[0], self.adjusted_direct[1], self.adjusted_direct[2],
                  color='y', linestyle='dotted', label='Adjusted direct')

        # 设置轴标签和范围
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])

        # 避免重复图例
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        plt.show()

    def _project_and_plot(self):

        # 投影到 xoy 平面
        projected_direct = np.array([self.adjusted_direct[0], self.adjusted_direct[1], 0])

        # 计算投影向量与 x 轴的夹角
        x_axis_2d = np.array([1, 0])
        projected_direct_2d = np.array([projected_direct[0], projected_direct[1]])
        self.angle = np.arccos(
            np.dot(projected_direct_2d, x_axis_2d) / (np.linalg.norm(projected_direct_2d) * np.linalg.norm(x_axis_2d)))
        angle_degrees = np.degrees(self.angle)

        # 绘图
        fig = plt.figure()
        ax = fig.add_subplot(111)

        # 绘制 x 轴
        ax.quiver(0, 0, 1, 0, angles='xy', scale_units='xy', scale=1, color='r', label='X-axis')
        # 绘制 y 轴
        ax.quiver(0, 0, 0, 1, angles='xy', scale_units='xy', scale=1, color='g', label='Y-axis')

        # 绘制投影向量
        ax.quiver(0, 0, projected_direct[0], projected_direct[1], angles='xy', scale_units='xy', scale=1, color='m',
                  label='Projected direct')

        # 添加角度标注
        ax.text(projected_direct[0] / 2, projected_direct[1] / 2, f'{angle_degrees:.2f}°', color='m', fontsize=12)

        # 设置坐标轴标签和范围
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])

        # 避免重复图例
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        plt.grid(True)
        plt.show()


# 使用类
euler_angles1 = [0.8, 0.8, 0.8]
direct1 = [1, 1, 1]
euler_angles2 = [1.8, 1.8, 1.8]
direct2 = [1, 1, 1]
RO1 = RelativeOrientation(euler_angles1, direct1)
RO2 = RelativeOrientation(euler_angles2, direct2)
relative_yaw = RO1.angle - RO2.angle + np.pi
print(relative_yaw)
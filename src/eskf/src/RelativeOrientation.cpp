#include "eskf/RelativeOrientation.h"
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

RelativeOrientation::RelativeOrientation(Vector3d euler_angles, Vector3d direct) {
    // 从陀螺仪得到测量姿态
    this->euler_angles = euler_angles;
    // 测算方向向量，归一化
    this->direct = direct.normalized();
    // 计算旋转矩阵
    this->rotation = AngleAxisd(euler_angles[2], Vector3d::UnitZ())
                     * AngleAxisd(euler_angles[1], Vector3d::UnitY())
                     * AngleAxisd(euler_angles[0], Vector3d::UnitX());
    // 定义初始坐标轴
    this->original_axes['x'] = Vector3d(1, 0, 0);
    this->original_axes['y'] = Vector3d(0, 1, 0);
    this->original_axes['z'] = Vector3d(0, 0, 1);

    // 旋转坐标轴
    rotate_axes();
    this->gravity_vector = Vector3d(0, 0, -1);
    this->adjustment_rotation = Matrix3d::Identity();
    this->adjusted_axes = this->rotated_axes;
    this->adjusted_direct = this->rotated_direct;

    // Z-Y-X中的roll轴和pitch轴旋转，从测量姿态得到中间帧
    adjust_for_gravity();

    // 投影角
    this->angle = 0;

    project();
    save_values_to_file("values.txt");

}

double RelativeOrientation::getAngle() {
    return this->angle;
}

void RelativeOrientation::rotate_axes() {
    
    this->rotated_axes['x'] = this->rotation * this->original_axes['x'];
    this->rotated_axes['y'] = this->rotation * this->original_axes['y'];
    this->rotated_axes['z'] = this->rotation * this->original_axes['z'];
    this->rotated_direct = this->rotation * this->direct;
}

void RelativeOrientation::adjust_for_gravity() {
    this->gravity_vector.normalize();
    Vector3d axis = this->rotated_axes['z'].cross(-this->gravity_vector);
    axis.normalize();
    double angle = acos(this->rotated_axes['z'].dot(-this->gravity_vector));
    if (axis.norm() > 1e-6) {
        this->adjustment_rotation = AngleAxisd(angle, axis).toRotationMatrix();
        this->adjusted_axes['x'] = this->adjustment_rotation * this->rotated_axes['x'];
        this->adjusted_axes['y'] = this->adjustment_rotation * this->rotated_axes['y'];
        this->adjusted_axes['z'] = -this->gravity_vector;
        this->adjusted_direct = this->adjustment_rotation * this->rotated_direct;
    } else {
        this->adjusted_axes = this->rotated_axes;
        this->adjusted_axes['z'] = -this->gravity_vector;
        this->adjusted_direct = this->rotated_direct;
    }
}

void RelativeOrientation::project() {
    Vector3d projected_direct = Vector3d(this->adjusted_direct[0], this->adjusted_direct[1], 0);
    Vector2d x_axis_2d(1, 0);
    Vector2d projected_direct_2d(projected_direct[0], projected_direct[1]);
    this->angle = acos(projected_direct_2d.dot(x_axis_2d) / (projected_direct_2d.norm() * x_axis_2d.norm()));
    double angle_degrees = this->angle * 180.0 / M_PI;
    cout << "Projected angle in degrees: " << angle_degrees << endl;
}

void RelativeOrientation::save_values_to_file(const std::string& filename) {
    ofstream file(filename);

    file << "Original Axes:\n";
    for (const auto& axis : original_axes) {
        file << axis.first << ": " << axis.second.transpose() << "\n";
    }

    file << "Direct Vector: " << this->direct.transpose() << "\n";

    file << "Rotated Axes:\n";
    for (const auto& axis : rotated_axes) {
        file << axis.first << ": " << axis.second.transpose() << "\n";
    }

    file << "Rotated Direct: " << this->rotated_direct.transpose() << "\n";

    file << "Adjusted Axes:\n";
    for (const auto& axis : adjusted_axes) {
        file << axis.first << ": " << axis.second.transpose() << "\n";
    }

    file << "Adjusted Direct: " << this->adjusted_direct.transpose() << "\n";

    file.close();
}
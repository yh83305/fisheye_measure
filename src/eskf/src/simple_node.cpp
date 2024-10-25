
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include "eskf/RelativeOrientation.h"
#include <Eigen/Dense>

using namespace Eigen;

Matrix3d angle_to_rotation_matrix(double angle, const Vector3d& axis) {
    // 归一化旋转轴
    Vector3d normalized_axis = axis.normalized();
    // 创建角轴表示
    AngleAxisd angle_axis(angle, normalized_axis);
    // 转换为旋转矩阵
    Matrix3d rotation_matrix = angle_axis.toRotationMatrix();
    return rotation_matrix;
}


int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;

    // 创建一个发布者对象
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("chatter", 1000);

    // 设置循环的频率
    ros::Rate rate(10); // 10 Hz

    Vector3d euler_angles1(0.8, 0.8, 0.8);
    Vector3d direct1(1, 1, 1);
    Vector3d euler_angles2(1.8, 1.8, 1.8);
    Vector3d direct2(1, 1, 1);

    RelativeOrientation RO1(euler_angles1, direct1);
    RelativeOrientation RO2(euler_angles2, direct2);

    double relative_yaw = RO1.getAngle() - RO2.getAngle() + M_PI;
    std::cout << "Relative yaw: " << relative_yaw << std::endl;

    Vector3d axis(0, 0, 1);  // 绕z轴旋转
    Matrix3d rotation_matrix = angle_to_rotation_matrix(relative_yaw, axis);
    
    std::cout << "Rotation Matrix:\n" << rotation_matrix << std::endl;

    while (ros::ok())
    {
        // 创建并填充消息
        std_msgs::Float64 msg;
        msg.data = relative_yaw;

        // 发布消息
        pub.publish(msg);

        // 输出日志信息
        ROS_INFO("Published: %f", msg.data);

        // 等待直到下一个循环
        rate.sleep();
    }

    return 0;
}




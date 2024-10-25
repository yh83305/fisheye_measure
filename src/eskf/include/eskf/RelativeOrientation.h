#ifndef RELATIVE_ORIENTATION_H
#define RELATIVE_ORIENTATION_H

#include <Eigen/Dense>
#include <map>
#include <fstream>

class RelativeOrientation {
public:
    RelativeOrientation(Eigen::Vector3d euler_angles, Eigen::Vector3d direct);

    Eigen::Vector3d euler_angles;
    Eigen::Vector3d direct;
    Eigen::Matrix3d rotation;
    std::map<char, Eigen::Vector3d> original_axes;

    std::map<char, Eigen::Vector3d> rotated_axes;
    Eigen::Vector3d rotated_direct;

    Eigen::Vector3d gravity_vector;
    Eigen::Matrix3d adjustment_rotation;
    std::map<char, Eigen::Vector3d> adjusted_axes;
    Eigen::Vector3d adjusted_direct;

    double angle;
    double getAngle();

private:
    
    void rotate_axes();
    void adjust_for_gravity();
    void project();
    void save_values_to_file(const std::string& filename);
};

#endif // RELATIVE_ORIENTATION_H

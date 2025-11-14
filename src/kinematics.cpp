#include "kinematics.hpp"
#include <Eigen/Dense>
#include "robotModel.hpp"

namespace robot {

// Rotation matrix for an axis + angle (radians)
inline Eigen::Matrix3d rotationMatrix(RotationAxis axis, double angle) {
    double c = std::cos(angle);
    double s = std::sin(angle);

    switch (axis) {
    case RotationAxis::X:
        return (Eigen::Matrix3d() <<
            1, 0, 0,
            0, c,-s,
            0, s, c).finished();

    case RotationAxis::Y:
        return (Eigen::Matrix3d() <<
             c, 0, s,
             0, 1, 0,
            -s, 0, c).finished();

    case RotationAxis::Z:
        return (Eigen::Matrix3d() <<
            c,-s, 0,
            s, c, 0,
            0, 0, 1).finished();
    }

    return Eigen::Matrix3d::Identity(); // fallback
}

// Composition of transforms: C = A ∘ B
inline Transform operator*(const Transform& A, const Transform& B) {
    Transform C;
    C.R = A.R * B.R;
    C.p = A.R * B.p + A.p;
    return C;
}

// --------------------------
// Forward Kinematics
// --------------------------

Eigen::Vector3d forwardKinematics(const Robot& robot) {
    Transform T;  
    T.p = Eigen::Vector3d(0, 0, 0); // start at origin

    for (const auto& link : robot.links) {
        // 1. Rotation for this joint
        Eigen::Matrix3d R = rotationMatrix(link.axis, link.angle);

        // 2. Constant translation along local Z (or your chosen direction)
        Eigen::Vector3d p(0, 0, link.length);

        Transform Ti{R, p};

        // 3. Compose
        T = T * Ti;
    }

    return T.p;  // end-effector position in world frame
}

} // namespace robot

